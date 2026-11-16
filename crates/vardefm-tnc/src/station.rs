//! One VardeFM station: link + modulator + receiver, speaking the VARA host protocol.
//!
//! The station is driven from outside: feed it host input and received audio, ask
//! it what to transmit, and collect its host output. It has no threads or clock.

use crate::host::{parse, Command};
use std::sync::Arc;
use vardefm_link::{Link, LinkConfig, LinkEvent, TxInfo};
use vardefm_modem::{Header, Modem, Receiver, RxBurst};

/// Output for the host application.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HostOut {
    /// A command-port line (without the CR).
    Line(String),
    /// Bytes for the data port.
    Data(Vec<u8>),
    /// A frame for the KISS port (type byte first, unescaped).
    Kiss(Vec<u8>),
}

/// A burst that has just been keyed up.
pub struct TxStart {
    pub audio: Vec<f32>,
    pub header: Header,
    pub info: TxInfo,
    pub seconds: f32,
}

const IAMALIVE_S: f64 = 60.0;

pub struct Station {
    pub name: String,
    modem: Arc<Modem>,
    pub link: Link,
    rx: Receiver,
    out: Vec<HostOut>,
    transmitting: bool,
    busy_reported: bool,
    last_alive: f64,
    bandwidth: &'static str,
    /// `CHAT ON`: report the S/N of every burst from the session peer (`SN <dB>`).
    chat: bool,
    /// `UNPROTO SN ON`: report the S/N of every received Unproto burst, with the
    /// CRC-32 of each frame it carried (`UNPROTO SN <dB> <crc> …`).
    unproto_sn: bool,
}

impl Station {
    pub fn new(name: &str, modem: Arc<Modem>, cfg: LinkConfig, seed: u32) -> Self {
        let bandwidth = if modem.ofdm.p.name == "wide" { "WIDE" } else { "NARROW" };
        Self {
            name: name.to_string(),
            link: Link::new(modem.clone(), cfg, seed),
            rx: Receiver::new(modem.clone()),
            modem,
            out: Vec::new(),
            transmitting: false,
            busy_reported: false,
            last_alive: 0.0,
            bandwidth,
            chat: false,
            unproto_sn: false,
        }
    }

    pub fn modem(&self) -> &Arc<Modem> {
        &self.modem
    }

    fn line(&mut self, s: impl Into<String>) {
        self.out.push(HostOut::Line(s.into()));
    }

    /// One line from the command port.
    pub fn host_line(&mut self, now: f64, line: &str) {
        let Ok(cmd) = parse(line) else {
            self.line("WRONG");
            return;
        };
        match cmd {
            Command::MyCall(calls) => {
                self.link.set_mycalls(calls);
                self.line("OK");
            }
            Command::Listen(on) => {
                self.link.set_listen(on);
                self.line("OK");
            }
            Command::Connect { source, target } => match self.link.connect(now, &source, &target) {
                Ok(()) => self.line("OK"),
                Err(()) => self.line("WRONG"),
            },
            Command::Disconnect => {
                self.line("OK");
                self.link.disconnect(now);
            }
            Command::Abort => {
                self.line("OK");
                self.link.abort(now);
            }
            Command::Chat(on) => {
                self.chat = on;
                self.line("OK");
            }
            Command::UnprotoSn(on) => {
                self.unproto_sn = on;
                self.line("OK");
            }
            Command::Version => {
                self.line("OK");
                self.line(format!("VERSION {}", crate::VERSION));
            }
            Command::Ignored => self.line("OK"),
        }
    }

    /// Bytes from the data port (dropped when not connected).
    pub fn host_data(&mut self, data: &[u8]) {
        self.link.send_data(data);
    }

    /// A data frame from the KISS port (type byte first): queued for an Unproto
    /// burst. Frames too long for one burst are dropped (counted in the link stats).
    pub fn kiss_in(&mut self, now: f64, frame: &[u8]) {
        self.link.send_unproto(now, frame);
    }

    /// The host closed the command port: drop any session, and forget its settings
    /// that only matter to it (`CHAT`, `UNPROTO SN`), so the next host starts from
    /// the defaults.
    pub fn host_closed(&mut self, now: f64) {
        self.chat = false;
        self.unproto_sn = false;
        self.link.abort(now);
    }

    /// Periodic housekeeping (IAMALIVE).
    pub fn tick(&mut self, now: f64) {
        if now - self.last_alive >= IAMALIVE_S {
            self.last_alive = now;
            self.line("IAMALIVE");
        }
    }

    /// If the link wants to transmit, key up and return the audio.
    /// Call only while not transmitting.
    pub fn poll_tx(&mut self, now: f64) -> Option<TxStart> {
        debug_assert!(!self.transmitting);
        let burst = self.link.poll(now, self.rx.busy())?;
        let audio = self.modem.modulate(&burst);
        self.transmitting = true;
        self.rx.reset();
        self.line("PTT ON");
        Some(TxStart {
            seconds: audio.len() as f32 / self.modem.fs(),
            audio,
            header: burst.header,
            info: self.link.last_tx().clone(),
        })
    }

    /// The burst returned by `poll_tx` has been fully transmitted.
    pub fn tx_done(&mut self, now: f64) {
        self.transmitting = false;
        self.line("PTT OFF");
        self.link.tx_done(now);
    }

    pub fn transmitting(&self) -> bool {
        self.transmitting
    }

    /// Received audio. Returns any bursts decoded (for logging).
    pub fn rx_audio(&mut self, now: f64, samples: &[f32]) -> Vec<RxBurst> {
        if self.transmitting {
            return Vec::new();
        }
        let bursts = self.rx.push(samples);
        for b in &bursts {
            self.link.on_burst(now, b);
        }
        let busy = self.rx.busy();
        if busy != self.busy_reported {
            self.busy_reported = busy;
            self.line(if busy { "BUSY ON" } else { "BUSY OFF" });
        }
        bursts
    }

    /// Everything for the host since the last call, in order.
    pub fn take_output(&mut self) -> Vec<HostOut> {
        // CRC-32 of each Unproto frame handed to the KISS port in this batch, for the
        // `UNPROTO SN` line that follows a burst's frames.
        let mut crcs: Vec<u32> = Vec::new();
        for e in self.link.take_events() {
            let o = match e {
                LinkEvent::PeerSnr(db) if self.chat => HostOut::Line(sn_line(db)),
                LinkEvent::PeerSnr(_) => continue,
                LinkEvent::UnprotoSnr { snr_db, frames } if self.unproto_sn => {
                    HostOut::Line(unproto_sn_line(snr_db, &crcs[crcs.len().saturating_sub(frames)..]))
                }
                LinkEvent::UnprotoSnr { .. } => continue,
                LinkEvent::Unproto(f) => {
                    crcs.push(frame_crc(&f));
                    HostOut::Kiss(f)
                }
                LinkEvent::Pending => HostOut::Line("PENDING".into()),
                LinkEvent::CancelPending => HostOut::Line("CANCELPENDING".into()),
                LinkEvent::Connected { source, destination } => {
                    HostOut::Line(format!("CONNECTED {source} {destination} {}", self.bandwidth))
                }
                LinkEvent::Disconnected => HostOut::Line("DISCONNECTED".into()),
                LinkEvent::Buffer(n) => HostOut::Line(format!("BUFFER {n}")),
                LinkEvent::Data(d) => HostOut::Data(d),
            };
            self.out.push(o);
        }
        std::mem::take(&mut self.out)
    }
}

/// `SN <dB>` with one decimal, as VARA sends it (e.g. `SN 12.3`, `SN -2.0`).
pub fn sn_line(snr_db: f32) -> String {
    let v = (snr_db * 10.0).round() / 10.0;
    let v = if v == 0.0 || !v.is_finite() { 0.0 } else { v }; // no "-0.0"
    format!("SN {v:.1}")
}

/// CRC-32 (IEEE 802.3, the one zlib, zip and Node's `zlib.crc32` compute) of a
/// KISS frame exactly as KISS clients get it: unescaped, type byte first.
pub fn frame_crc(frame: &[u8]) -> u32 {
    vardefm_fec::crc::crc32(frame)
}

/// `UNPROTO SN <dB> <crc> …`: the S/N of an Unproto burst (as in [`sn_line`]) and
/// the [`frame_crc`] of each frame it carried, in order, as 8 upper-case hex digits.
pub fn unproto_sn_line(snr_db: f32, crcs: &[u32]) -> String {
    let mut s = format!("UNPROTO {}", sn_line(snr_db));
    for c in crcs {
        s.push_str(&format!(" {c:08X}"));
    }
    s
}

#[cfg(test)]
mod tests {
    use super::{frame_crc, sn_line, unproto_sn_line};

    #[test]
    fn sn_format() {
        assert_eq!(sn_line(12.34), "SN 12.3");
        assert_eq!(sn_line(12.35), "SN 12.4");
        assert_eq!(sn_line(-2.0), "SN -2.0");
        assert_eq!(sn_line(-0.04), "SN 0.0");
        assert_eq!(sn_line(60.0), "SN 60.0");
    }

    #[test]
    fn unproto_sn_format() {
        assert_eq!(frame_crc(b"123456789"), 0xCBF4_3926);
        // Ortoo's CQ frame (type byte 0x01 first), see tests/unproto.rs.
        let cq = [
            0x01, 0x86, 0xa2, 0x40, 0x40, 0x40, 0x40, 0x40, 0x60, 0x9c, 0x66, 0x98, 0x9e, 0x40, 0x40, 0x40, 0x69, 0x03,
            0xf0,
        ];
        assert_eq!(
            unproto_sn_line(9.04, &[frame_crc(&cq), 0x0000_00ab]),
            format!("UNPROTO SN 9.0 {:08X} 000000AB", frame_crc(&cq))
        );
        assert_eq!(unproto_sn_line(-3.0, &[]), "UNPROTO SN -3.0");
    }
}
