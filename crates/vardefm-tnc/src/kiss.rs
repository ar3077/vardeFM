//! KISS over TCP: the host side of Unproto (unconnected broadcast) bursts.
//!
//! Like VARA FM's KISS port: frames are `FEND <type byte> <data, escaped> FEND`.
//! The type byte's low nibble is the KISS command and its high nibble the port.
//! The whole frame, type byte included, crosses the air unchanged and is handed to
//! every KISS client of every station that decodes it.
//!
//! Data frames are command 0 on any port, and command 1 with more than one byte of
//! data: VarAC and VARA FM use type 1 for frames with 7-character AX.25 addresses
//! (a CQ call is `C0 01 "CQ" … 03 F0 C0`), while a real TXDELAY command carries
//! exactly one parameter byte. All other commands (TXDELAY, P, SlotTime, TXtail,
//! FullDuplex, SetHardware, Return) are accepted and ignored: VardeFM uses its own
//! leader and channel access.
//!
//! Any number of clients may connect at once. Each gets every received frame, and
//! any of them may send.

use crate::server::HostIn;
use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Sender;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

pub const FEND: u8 = 0xc0;
pub const FESC: u8 = 0xdb;
pub const TFEND: u8 = 0xdc;
pub const TFESC: u8 = 0xdd;

/// Longest frame accepted from a client (longer ones are dropped while reading).
const MAX_FRAME: usize = 64 * 1024;

/// `frame` (type byte first) as it goes on the wire: FEND, escaped bytes, FEND.
pub fn encode(frame: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(frame.len() + 4);
    out.push(FEND);
    for &b in frame {
        match b {
            FEND => out.extend_from_slice(&[FESC, TFEND]),
            FESC => out.extend_from_slice(&[FESC, TFESC]),
            _ => out.push(b),
        }
    }
    out.push(FEND);
    out
}

/// True if `frame` (type byte first) carries data to transmit; false for
/// parameter commands, which are ignored.
pub fn is_data(frame: &[u8]) -> bool {
    match frame.first().map(|t| t & 0x0f) {
        Some(0) => frame.len() > 1,
        Some(1) => frame.len() > 2,
        _ => false,
    }
}

/// Streaming KISS decoder: feed it bytes as they arrive, get whole frames back.
#[derive(Default)]
pub struct Decoder {
    buf: Vec<u8>,
    /// Inside a frame (a FEND has been seen).
    open: bool,
    escaped: bool,
    /// The current frame grew too long and is being skipped.
    overflow: bool,
}

impl Decoder {
    /// Frames completed by `bytes`, unescaped, type byte first. Empty frames
    /// (back-to-back FENDs) are skipped, and so are bytes before the first FEND.
    pub fn push(&mut self, bytes: &[u8]) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        for &b in bytes {
            if b == FEND {
                if self.open && !self.buf.is_empty() && !self.overflow {
                    out.push(std::mem::take(&mut self.buf));
                }
                self.buf.clear();
                (self.open, self.escaped, self.overflow) = (true, false, false);
                continue;
            }
            if !self.open {
                continue;
            }
            let b = if self.escaped {
                self.escaped = false;
                match b {
                    TFEND => FEND,
                    TFESC => FESC,
                    other => other,
                }
            } else if b == FESC {
                self.escaped = true;
                continue;
            } else {
                b
            };
            if self.buf.len() >= MAX_FRAME {
                self.overflow = true;
                self.buf.clear();
            }
            if !self.overflow {
                self.buf.push(b);
            }
        }
        out
    }
}

type Clients = Arc<Mutex<Vec<TcpStream>>>;

/// A KISS TCP listener. Data frames from clients arrive in the host inbox as
/// [`HostIn::Kiss`].
pub struct KissPort {
    pub addr: SocketAddr,
    clients: Clients,
    stop: Arc<AtomicBool>,
}

impl KissPort {
    /// Listen on `host:port` (port 0 picks a free port).
    pub fn bind(host: &str, port: u16, inbox: Sender<HostIn>, stop: Arc<AtomicBool>) -> io::Result<Self> {
        let listener = TcpListener::bind((host, port))?;
        let addr = listener.local_addr()?;
        listener.set_nonblocking(true)?;
        let clients = Clients::default();
        let (c, s) = (clients.clone(), stop.clone());
        thread::spawn(move || {
            while !s.load(Ordering::Relaxed) {
                match listener.accept() {
                    Ok((stream, _)) => {
                        let _ = stream.set_nonblocking(false);
                        let _ = stream.set_nodelay(true);
                        let _ = stream.set_write_timeout(Some(Duration::from_secs(5)));
                        let Ok(reader) = stream.try_clone() else { continue };
                        c.lock().unwrap().push(stream);
                        let inbox = inbox.clone();
                        thread::spawn(move || read_loop(reader, inbox));
                    }
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock => thread::sleep(Duration::from_millis(20)),
                    Err(_) => thread::sleep(Duration::from_millis(100)),
                }
            }
        });
        Ok(Self { addr, clients, stop })
    }

    /// Send one frame (type byte first) to every client.
    pub fn send(&self, frame: &[u8]) {
        let wire = encode(frame);
        self.clients.lock().unwrap().retain_mut(|s| s.write_all(&wire).is_ok());
    }

    pub fn shutdown(&self) {
        self.stop.store(true, Ordering::Relaxed);
        for s in self.clients.lock().unwrap().drain(..) {
            let _ = s.shutdown(std::net::Shutdown::Both);
        }
    }
}

fn read_loop(mut stream: TcpStream, inbox: Sender<HostIn>) {
    let mut dec = Decoder::default();
    let mut buf = [0u8; 4096];
    loop {
        let n = match stream.read(&mut buf) {
            Ok(0) | Err(_) => break,
            Ok(n) => n,
        };
        for f in dec.push(&buf[..n]) {
            if is_data(&f) && inbox.send(HostIn::Kiss(f)).is_err() {
                return;
            }
        }
    }
    let _ = stream.shutdown(std::net::Shutdown::Both);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn escaping_roundtrip_all_byte_values() {
        let frame: Vec<u8> = std::iter::once(0x00).chain(0..=255u8).chain([FEND, FESC, FEND]).collect();
        let wire = encode(&frame);
        assert_eq!(wire.iter().filter(|&&b| b == FEND).count(), 2, "only the delimiters are FEND");
        assert_eq!(Decoder::default().push(&wire), vec![frame]);
    }

    #[test]
    fn split_reads_and_several_frames_per_read() {
        let frames = [vec![0x00, 1, 2, FEND, 3], vec![0x10, FESC], vec![0x01; 20]];
        let wire: Vec<u8> = frames.iter().flat_map(|f| encode(f)).collect();
        for chunk in [1, 2, 3, 7, wire.len()] {
            let mut d = Decoder::default();
            let got: Vec<_> = wire.chunks(chunk).flat_map(|c| d.push(c)).collect();
            assert_eq!(got, frames, "chunk {chunk}");
        }
    }

    #[test]
    fn junk_before_first_fend_and_empty_frames_are_skipped() {
        let mut d = Decoder::default();
        let got = d.push(&[1, 2, 3, FEND, FEND, FEND, 0x00, 0xaa, FEND, FEND]);
        assert_eq!(got, vec![vec![0x00, 0xaa]]);
    }

    #[test]
    fn data_frames_and_commands() {
        assert!(is_data(&[0x00, 0x82]));
        assert!(is_data(&[0x50, 0x82])); // data on KISS port 5
        assert!(is_data(&[0x01, 0x86, 0xa0, 0x40, 0x40, 0x40])); // VarAC 7-char addressing
        assert!(!is_data(&[0x00])); // empty data frame
        assert!(!is_data(&[0x01, 50])); // TXDELAY 500 ms
        for cmd in [0x02, 0x03, 0x04, 0x05, 0x06, 0x16, 0xff] {
            assert!(!is_data(&[cmd, 1]), "{cmd:#x}");
        }
    }

    #[test]
    fn oversized_frame_is_dropped_and_the_next_one_kept() {
        let mut d = Decoder::default();
        let mut wire = vec![FEND];
        wire.resize(MAX_FRAME + 10, 0x41);
        wire.extend_from_slice(&encode(&[0x00, 7]));
        assert_eq!(d.push(&wire), vec![vec![0x00, 7]]);
    }
}
