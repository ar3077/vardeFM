//! Unconnected broadcasts (Unproto bursts) end to end, with the frames Ortoo
//! actually sends: AX.25 UI frames written to the KISS port.
//!
//! - Through `sim-pair`: frames written to station A's KISS port must come out of
//!   station B's KISS port byte for byte, in order.
//! - With three stations on one simulated channel: a broadcast from a third
//!   station reaches both ends of a session that is in progress, and the session
//!   carries on undisturbed.
//!
//! Set VARDEFM_SIM_VERBOSE=1 to print every transmission.

use std::io::{Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::sync::mpsc::{channel, Receiver};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use vardefm_link::LinkConfig;
use vardefm_modem::{Kind, Modem, OfdmParams};
use vardefm_sim::{transmit, FmChannelConfig};
use vardefm_tnc::kiss;
use vardefm_tnc::simpair::{SimPair, SimPairConfig};
use vardefm_tnc::station::{frame_crc, sn_line, HostOut, Station};

const TIMEOUT: Duration = Duration::from_secs(120);

// ------------------------------------------------- Ortoo's frames (server.js)

/// `ax25Address()`: callsign shifted left one bit, padded to `width`, then the SSID
/// byte `0x60 | ssid << 1 | last`. Ortoo never sets the command bit.
fn ax25_address(call: &str, last: bool, width: usize) -> Vec<u8> {
    let (base, ssid) = match call.split_once('-') {
        Some((b, s)) => (b, s.parse::<u8>().unwrap()),
        None => (call, 0),
    };
    let mut out: Vec<u8> = format!("{base:<width$}").bytes().map(|b| b << 1).collect();
    out.push(0x60 | ssid << 1 | last as u8);
    out
}

/// `encodeUI()`: dest | src | digis | control 0x03 (UI) | PID 0xF0 | info.
/// Digipeater addresses are always 6 characters wide.
fn encode_ui(dest: &str, src: &str, info: &[u8], digis: &[&str], width: usize) -> Vec<u8> {
    let mut f = ax25_address(dest, false, width);
    f.extend(ax25_address(src, digis.is_empty(), width));
    for (i, d) in digis.iter().enumerate() {
        f.extend(ax25_address(d, i == digis.len() - 1, 6));
    }
    f.extend([0x03, 0xf0]);
    f.extend_from_slice(info);
    f
}

const MYCALL: &str = "N3LO-4";

/// POST /api/cq with the KISS port up: UI frame to "CQ", no info, KISS type 1
/// (VarAC's 7-character addressing). The KISS frame, type byte first.
fn cq_frame() -> Vec<u8> {
    [vec![0x01], encode_ui("CQ", MYCALL, &[], &[], 7)].concat()
}

/// POST /api/broadcast without a link: one UI frame to "ALL" per line (at most 77
/// bytes of text each), KISS type 0, digipeater path from the Broadcast Vias.
fn broadcast_frame(text: &str, vias: &[&str]) -> Vec<u8> {
    assert!(text.len() <= 77, "Ortoo splits lines at UI_MAX_INFO = 77 bytes");
    [vec![0x00], encode_ui("ALL", MYCALL, text.as_bytes(), vias, 6)].concat()
}

/// A line of exactly 77 bytes (Ortoo's largest packet).
fn line77(n: usize) -> String {
    let s = format!("Line {n:02} of the net bulletin: {}", "abcdefghijklmnopqrstuvwxyz0123456789");
    format!("{s:.<77}")[..77].to_string()
}

/// 10 text frames with a [PAGE] marker frame between them (19 frames).
fn ten_line_broadcast() -> Vec<Vec<u8>> {
    let mut frames = Vec::new();
    for n in 1..=10 {
        if n > 1 {
            frames.push(broadcast_frame("[PAGE]", &[]));
        }
        frames.push(broadcast_frame(&line77(n), &[]));
    }
    frames
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| format!("{x:02x}")).collect()
}

/// The frame builders above against bytes produced by Ortoo's own
/// `kissFrame(encodeUI(...))` (server.js), for MYCALL N3LO-4.
#[test]
fn frames_match_ortoo_server_js() {
    assert_eq!(hex(&kiss::encode(&cq_frame())), "c00186a24040404040609c66989e4040406903f0c0");
    assert_eq!(
        hex(&kiss::encode(&broadcast_frame("[PAGE]", &[]))),
        "c000829898404040609c66989e40406903f05b504147455dc0"
    );
    assert_eq!(
        hex(&kiss::encode(&broadcast_frame(&"x".repeat(77), &[]))),
        format!("c000829898404040609c66989e40406903f0{}c0", "78".repeat(77))
    );
    assert_eq!(
        hex(&kiss::encode(&broadcast_frame("Net control: check-ins now, please", &["WIDE1-1", "K1ABC-7"]))),
        "c000829898404040609c66989e404068ae92888a6240629662828486406f03f0\
         4e657420636f6e74726f6c3a20636865636b2d696e73206e6f772c20706c65617365c0"
    );
}

// ------------------------------------------------------------ host helpers

/// A KISS client: writes frames, and records the raw bytes and decoded frames it receives.
struct KissClient {
    name: &'static str,
    s: TcpStream,
    raw: Arc<Mutex<Vec<u8>>>,
    frames: Arc<Mutex<Vec<Vec<u8>>>>,
}

impl KissClient {
    fn connect(name: &'static str, addr: SocketAddr) -> Self {
        let s = TcpStream::connect(addr).unwrap();
        let (raw, frames) = (Arc::new(Mutex::new(Vec::new())), Arc::new(Mutex::new(Vec::new())));
        let (raw2, frames2, mut r) = (raw.clone(), frames.clone(), s.try_clone().unwrap());
        thread::spawn(move || {
            let mut dec = kiss::Decoder::default();
            let mut buf = [0u8; 4096];
            while let Ok(n) = r.read(&mut buf) {
                if n == 0 {
                    break;
                }
                raw2.lock().unwrap().extend_from_slice(&buf[..n]);
                frames2.lock().unwrap().extend(dec.push(&buf[..n]));
            }
        });
        thread::sleep(Duration::from_millis(100));
        Self { name, s, raw, frames }
    }

    /// Like Ortoo: every frame written back to back. Returns the bytes written.
    fn send(&mut self, frames: &[Vec<u8>]) -> Vec<u8> {
        let wire: Vec<u8> = frames.iter().flat_map(|f| kiss::encode(f)).collect();
        for f in frames {
            self.s.write_all(&kiss::encode(f)).unwrap();
        }
        wire
    }

    fn frames(&self) -> Vec<Vec<u8>> {
        self.frames.lock().unwrap().clone()
    }

    /// Wait for `n` frames; returns (frames, raw bytes).
    fn wait(&self, n: usize) -> (Vec<Vec<u8>>, Vec<u8>) {
        let deadline = Instant::now() + TIMEOUT;
        loop {
            let got = self.frames();
            if got.len() >= n {
                thread::sleep(Duration::from_millis(200)); // anything extra would show up now
                return (self.frames(), self.raw.lock().unwrap().clone());
            }
            assert!(Instant::now() < deadline, "{}: got only {} of {n} frames", self.name, got.len());
            thread::sleep(Duration::from_millis(20));
        }
    }
}

/// A minimal VARA host on the command and data ports.
struct VaraHost {
    name: &'static str,
    cmd: TcpStream,
    data: TcpStream,
    lines: Receiver<String>,
    seen: Vec<String>,
    rx: Arc<Mutex<Vec<u8>>>,
}

impl VaraHost {
    fn connect(name: &'static str, (cmd_addr, data_addr): (SocketAddr, SocketAddr)) -> Self {
        let cmd = TcpStream::connect(cmd_addr).unwrap();
        let data = TcpStream::connect(data_addr).unwrap();
        let (tx, lines) = channel();
        let mut r = cmd.try_clone().unwrap();
        thread::spawn(move || {
            let (mut buf, mut line) = ([0u8; 1024], Vec::new());
            while let Ok(n) = r.read(&mut buf) {
                if n == 0 {
                    break;
                }
                for &b in &buf[..n] {
                    if b == b'\r' {
                        let _ = tx.send(String::from_utf8_lossy(&line).into_owned());
                        line.clear();
                    } else {
                        line.push(b);
                    }
                }
            }
        });
        let rx = Arc::new(Mutex::new(Vec::new()));
        let (rx2, mut d) = (rx.clone(), data.try_clone().unwrap());
        thread::spawn(move || {
            let mut buf = [0u8; 4096];
            while let Ok(n) = d.read(&mut buf) {
                if n == 0 {
                    break;
                }
                rx2.lock().unwrap().extend_from_slice(&buf[..n]);
            }
        });
        thread::sleep(Duration::from_millis(100));
        Self { name, cmd, data, lines, seen: Vec::new(), rx }
    }

    fn expect(&mut self, prefix: &str) -> String {
        let deadline = Instant::now() + TIMEOUT;
        loop {
            match self.lines.recv_timeout(deadline.saturating_duration_since(Instant::now())) {
                Ok(l) => {
                    self.seen.push(l.clone());
                    if l.starts_with(prefix) {
                        return l;
                    }
                }
                Err(_) => panic!("{}: no '{prefix}' line; saw {:?}", self.name, self.seen),
            }
        }
    }

    /// Record every line that has arrived so far without waiting for one.
    fn drain(&mut self) {
        while let Ok(l) = self.lines.try_recv() {
            self.seen.push(l);
        }
    }

    fn command(&mut self, line: &str) {
        self.cmd.write_all(format!("{line}\r").as_bytes()).unwrap();
        loop {
            let l = self.expect("");
            if l == "OK" || l == "WRONG" {
                assert_eq!(l, "OK", "{}: {line}", self.name);
                return;
            }
        }
    }

    fn wait_rx(&self, n: usize) -> Vec<u8> {
        let deadline = Instant::now() + TIMEOUT;
        loop {
            let got = self.rx.lock().unwrap().clone();
            if got.len() >= n {
                return got;
            }
            assert!(Instant::now() < deadline, "{}: received only {} of {n} bytes", self.name, got.len());
            thread::sleep(Duration::from_millis(20));
        }
    }
}

fn sim(mode: &str, cnr: f32, seed: u64) -> SimPair {
    SimPair::start(SimPairConfig {
        mode: mode.into(),
        cnr_db: Some(cnr),
        seed,
        speed: 20.0,
        a_port: 0,
        b_port: 0,
        a_kiss_port: Some(0),
        b_kiss_port: Some(0),
        verbose: std::env::var("VARDEFM_SIM_VERBOSE").is_ok(),
        ..SimPairConfig::default()
    })
    .unwrap()
}

/// Transmissions in the sim-pair log: (time, line).
fn tx_log(sp: &SimPair, kind: &str) -> Vec<(f64, String)> {
    sp.stats()
        .log
        .into_iter()
        .filter(|l| l.contains("->") && l.contains(kind))
        .map(|l| (l.trim_start().split('s').next().unwrap().trim().parse().unwrap(), l))
        .collect()
}

/// Send `frames` into A's KISS port and check that B's KISS port delivers the same
/// bytes in the same order, and that A's own client gets nothing back.
fn broadcast(mode: &str, frames: &[Vec<u8>], seed: u64) -> usize {
    let sp = sim(mode, 20.0, seed);
    let mut a = KissClient::connect("A kiss", sp.kiss_addrs[0].unwrap());
    let b = KissClient::connect("B kiss", sp.kiss_addrs[1].unwrap());
    let wire = a.send(frames);
    let (got, raw) = b.wait(frames.len());
    assert_eq!(got.len(), frames.len(), "B got extra frames");
    for (i, (g, f)) in got.iter().zip(frames).enumerate() {
        assert_eq!(hex(g), hex(f), "frame {i} differs");
    }
    assert_eq!(raw, wire, "B's KISS byte stream differs from what A's client wrote");
    assert!(a.frames().is_empty(), "A's own frames came back to A");
    let bursts = tx_log(&sp, "Unproto");
    for (_, l) in &bursts {
        println!("{l}");
    }
    sp.stop();
    bursts.len()
}

// ------------------------------------------------------------ sim-pair tests

#[test]
fn cq_frame_arrives_byte_identical() {
    assert_eq!(broadcast("narrow", &[cq_frame()], 11), 1);
}

#[test]
fn ten_line_broadcast_with_page_markers_keeps_order() {
    let frames = ten_line_broadcast();
    assert_eq!(frames.len(), 19);
    let bytes: usize = frames.iter().map(|f| f.len() + 2).sum();
    let bursts = broadcast("narrow", &frames, 12);
    // Batched: every burst but the last is full (level 0 narrow: 291 bytes for frames).
    let cap = vardefm_link::unproto_capacity(&Modem::new(OfdmParams::narrow(), 0.1), 0);
    println!("{} frames, {bytes} bytes with length prefixes, in {bursts} bursts (capacity {cap})", frames.len());
    assert!(bursts <= bytes.div_ceil(cap) + 1, "{bursts} bursts for {bytes} bytes: frames were not batched");
}

#[test]
fn ten_line_broadcast_wide() {
    let frames = ten_line_broadcast();
    let bursts = broadcast("wide", &frames, 13);
    assert!(bursts <= 2, "{bursts} bursts");
}

#[test]
fn broadcast_with_two_vias() {
    let frames: Vec<_> = (1..=3)
        .map(|n| broadcast_frame(&line77(n), &["WIDE1-1", "K1ABC-7"]))
        .chain([broadcast_frame("[PAGE]", &["WIDE1-1", "K1ABC-7"])])
        .collect();
    assert_eq!(frames[0].len(), 1 + 4 * 7 + 2 + 77);
    broadcast("narrow", &frames, 14);
}

/// A broadcast written while A is in a session is held until the session has ended.
#[test]
fn broadcast_waits_for_the_session_to_end() {
    let sp = sim("narrow", 20.0, 15);
    let mut a = VaraHost::connect("A", sp.addrs[0]);
    let mut b = VaraHost::connect("B", sp.addrs[1]);
    let mut ak = KissClient::connect("A kiss", sp.kiss_addrs[0].unwrap());
    let bk = KissClient::connect("B kiss", sp.kiss_addrs[1].unwrap());
    b.command("MYCALL LA2BBB");
    b.command("LISTEN ON");
    a.command("MYCALL LA1AAA");
    a.command("CONNECT LA1AAA LA2BBB");
    a.expect("CONNECTED");
    b.expect("CONNECTED");

    let data: Vec<u8> = (0..3000u32).map(|i| (i * 7) as u8).collect();
    a.data.write_all(&data).unwrap();
    let frames = vec![broadcast_frame(&line77(1), &[]), broadcast_frame("[PAGE]", &[]), cq_frame()];
    ak.send(&frames);
    assert_eq!(b.wait_rx(data.len()), data);
    // The session is still up and idle: nothing may have gone out yet.
    thread::sleep(Duration::from_millis(300)); // 6 s of simulated time
    assert!(bk.frames().is_empty(), "broadcast sent during the session");
    assert!(tx_log(&sp, "Unproto").is_empty());

    a.command("DISCONNECT");
    a.expect("DISCONNECTED");
    b.expect("DISCONNECTED");
    let (got, _) = bk.wait(frames.len());
    assert_eq!(got, frames);
    let disc_end = tx_log(&sp, "DiscAck").last().unwrap().0;
    let first = tx_log(&sp, "Unproto")[0].0;
    assert!(first > disc_end, "Unproto at {first} s, session ended at {disc_end} s");
    sp.stop();
}

// ----------------------------------------------- three stations, one channel

/// Stations that all hear each other through the FM channel simulator.
struct Air {
    st: Vec<Station>,
    out: Vec<Vec<HostOut>>,
    incoming: Vec<Vec<(u64, Vec<f32>)>>,
    tx_end: Vec<Option<u64>>,
    channel: FmChannelConfig,
    t: u64,
    n_tx: u64,
    log: Vec<String>,
    /// Bursts each station decoded: (kind, SNR dB).
    rx: Vec<Vec<(Kind, f32)>>,
}

const STEP: usize = 480;

impl Air {
    fn new(n: usize) -> Self {
        let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
        let names = ["A", "B", "C"];
        Self {
            st: (0..n).map(|i| Station::new(names[i], modem.clone(), LinkConfig::default(), 31 + i as u32)).collect(),
            out: vec![Vec::new(); n],
            incoming: vec![Vec::new(); n],
            tx_end: vec![None; n],
            channel: FmChannelConfig {
                cnr_db: Some(25.0),
                keyup_delay_s: 0.05,
                lead_silence_s: 0.0,
                ..FmChannelConfig::by_name("narrow").unwrap()
            },
            t: 0,
            n_tx: 0,
            log: Vec::new(),
            rx: vec![Vec::new(); n],
        }
    }

    fn now(&self) -> f64 {
        self.t as f64 / 48_000.0
    }

    fn step(&mut self) {
        let now = self.now();
        let n = self.st.len();
        for i in 0..n {
            if self.tx_end[i].is_some_and(|e| e <= self.t) {
                self.tx_end[i] = None;
                self.st[i].tx_done(now);
            }
        }
        for i in 0..n {
            if self.tx_end[i].is_none() {
                if let Some(tx) = self.st[i].poll_tx(now) {
                    self.log
                        .push(format!("{now:7.2}s {} tx {:?} cw {}", self.st[i].name, tx.header.kind, tx.header.ncw));
                    for j in (0..n).filter(|&j| j != i) {
                        self.n_tx += 1;
                        let ch = FmChannelConfig { seed: 1000 + self.n_tx, ..self.channel.clone() };
                        self.incoming[j].push((self.t, transmit(&ch, &tx.audio)));
                    }
                    self.tx_end[i] = Some(self.t + tx.audio.len() as u64);
                }
            }
        }
        for i in 0..n {
            let (a, b) = (self.t, self.t + STEP as u64);
            let mut chunk = vec![0f32; STEP];
            for (s, audio) in &self.incoming[i] {
                for k in a.max(*s)..b.min(s + audio.len() as u64) {
                    chunk[(k - a) as usize] += audio[(k - s) as usize];
                }
            }
            self.incoming[i].retain(|(s, audio)| s + audio.len() as u64 > b);
            for b in self.st[i].rx_audio(now, &chunk) {
                self.rx[i].push((b.header.kind, b.snr_db));
            }
            let o = self.st[i].take_output();
            self.out[i].extend(o);
        }
        self.t += STEP as u64;
    }

    fn run_until(&mut self, limit_s: f64, mut done: impl FnMut(&Self) -> bool) {
        let end = self.now() + limit_s;
        while !done(self) {
            assert!(self.now() < end, "timed out at {:.1} s; log:\n{}", self.now(), self.log.join("\n"));
            self.step();
        }
    }

    fn has_line(&self, i: usize, prefix: &str) -> bool {
        self.out[i].iter().any(|o| matches!(o, HostOut::Line(l) if l.starts_with(prefix)))
    }

    fn data(&self, i: usize) -> Vec<u8> {
        self.out[i].iter().flat_map(|o| if let HostOut::Data(d) = o { d.clone() } else { Vec::new() }).collect()
    }

    fn lines(&self, i: usize, prefix: &str) -> Vec<String> {
        self.out[i]
            .iter()
            .filter_map(|o| match o {
                HostOut::Line(l) if l.starts_with(prefix) => Some(l.clone()),
                _ => None,
            })
            .collect()
    }

    fn kiss(&self, i: usize) -> Vec<Vec<u8>> {
        self.out[i].iter().filter_map(|o| if let HostOut::Kiss(f) = o { Some(f.clone()) } else { None }).collect()
    }
}

/// A and B are in a session; C broadcasts. Both A and B deliver C's frames to their
/// KISS ports while their session is up, and the session's data arrives intact.
#[test]
fn broadcast_from_a_third_station_reaches_both_ends_of_a_session() {
    let mut air = Air::new(3);
    air.st[1].host_line(0.0, "MYCALL LA2BBB");
    air.st[1].host_line(0.0, "LISTEN ON");
    air.st[0].host_line(0.0, "MYCALL LA1AAA");
    air.st[0].host_line(0.0, "CONNECT LA1AAA LA2BBB");
    air.run_until(30.0, |a| a.has_line(0, "CONNECTED") && a.has_line(1, "CONNECTED"));

    // C's host writes a broadcast while A is sending data to B.
    let first: Vec<u8> = (0..4000u32).map(|i| (i * 13 + 5) as u8).collect();
    air.st[0].host_data(&first);
    let frames = vec![broadcast_frame(&line77(1), &[]), broadcast_frame("[PAGE]", &[]), cq_frame()];
    let now = air.now();
    for f in &frames {
        air.st[2].kiss_in(now, f);
    }
    air.run_until(120.0, |a| a.kiss(0).len() >= frames.len() && a.kiss(1).len() >= frames.len());
    assert!(air.st[0].link.is_connected() && air.st[1].link.is_connected(), "the session dropped");
    assert_eq!(air.kiss(0), frames, "A's KISS port");
    assert_eq!(air.kiss(1), frames, "B's KISS port");
    assert!(air.kiss(2).is_empty());

    // The session is unaffected: the data arrives, and more data after the broadcast.
    air.run_until(120.0, |a| a.data(1).len() >= first.len());
    let more: Vec<u8> = (0..1000u32).map(|i| (i * 3) as u8).collect();
    air.st[0].host_data(&more);
    air.run_until(120.0, |a| a.data(1).len() >= first.len() + more.len());
    assert_eq!(air.data(1), [first, more].concat());
    let now = air.now();
    air.st[0].host_line(now, "DISCONNECT");
    air.run_until(60.0, |a| a.has_line(0, "DISCONNECTED") && a.has_line(1, "DISCONNECTED"));
    println!("{}", air.log.join("\n"));
}

// ------------------------------------------------------------- UNPROTO SN

/// `UNPROTO SN <dB> <crc>…` parsed: (dB, CRCs).
fn parse_unproto_sn(l: &str) -> (f32, Vec<u32>) {
    let mut w = l.strip_prefix("UNPROTO SN ").expect(l).split(' ');
    let db = w.next().unwrap();
    assert_eq!(db.split_once('.').unwrap().1.len(), 1, "{l}: not one decimal");
    let crcs = w
        .map(|c| {
            assert!(c.len() == 8 && c.chars().all(|h| h.is_ascii_hexdigit() && !h.is_ascii_lowercase()), "{l}");
            u32::from_str_radix(c, 16).unwrap()
        })
        .collect();
    (db.parse().unwrap(), crcs)
}

/// A and B are in a session (both with CHAT ON, A also with UNPROTO SN ON); C
/// broadcasts, then sends the same CQ twice more in separate bursts. A's UNPROTO SN
/// lines list exactly the frames A delivered to its KISS port, with the S/N of the
/// burst that carried them, each line after its burst's frames. The broadcasts do
/// not show up as session SN lines, and B (UNPROTO SN off) gets no UNPROTO line.
#[test]
fn unproto_sn_names_the_frames_of_each_burst() {
    let mut air = Air::new(3);
    air.st[1].host_line(0.0, "MYCALL LA2BBB");
    air.st[1].host_line(0.0, "LISTEN ON");
    air.st[0].host_line(0.0, "MYCALL LA1AAA");
    for st in &mut air.st[..2] {
        st.host_line(0.0, "CHAT ON");
    }
    air.st[0].host_line(0.0, "UNPROTO SN ON");
    air.st[2].host_line(0.0, "UNPROTO SN ON"); // C never hears its own frames
    air.st[0].host_line(0.0, "CONNECT LA1AAA LA2BBB");
    air.run_until(30.0, |a| a.has_line(0, "CONNECTED") && a.has_line(1, "CONNECTED"));

    let frames = vec![broadcast_frame(&line77(1), &[]), broadcast_frame("[PAGE]", &[]), cq_frame()];
    let now = air.now();
    for f in &frames {
        air.st[2].kiss_in(now, f);
    }
    air.run_until(60.0, |a| a.kiss(0).len() >= 3 && a.kiss(1).len() >= 3);
    for k in 0..2 {
        let now = air.now();
        air.st[2].kiss_in(now, &cq_frame());
        air.run_until(60.0, |a| a.kiss(0).len() > 3 + k && a.kiss(1).len() > 3 + k);
    }
    let until = air.now() + 2.0;
    air.run_until(10.0, |a| a.now() >= until);

    let got = air.kiss(0);
    assert_eq!(got.len(), 5);
    let lines: Vec<(f32, Vec<u32>)> = air.lines(0, "UNPROTO ").iter().map(|l| parse_unproto_sn(l)).collect();
    println!("A: {:?}", air.lines(0, "UNPROTO "));
    // Every frame named once, in order, by the CRC-32 of its bytes.
    let named: Vec<u32> = lines.iter().flat_map(|(_, c)| c.clone()).collect();
    assert_eq!(named, got.iter().map(|f| frame_crc(f)).collect::<Vec<_>>());
    // The repeated CQ: same CRC, in separate lines.
    assert!(lines.len() >= 3, "the three CQs went in at least three bursts");
    assert_eq!(lines[lines.len() - 1].1, vec![frame_crc(&cq_frame())]);
    assert_eq!(lines[lines.len() - 2].1, vec![frame_crc(&cq_frame())]);
    // The S/N is the one the receiver measured on that Unproto burst.
    let unproto_snr: Vec<String> =
        air.rx[0].iter().filter(|(k, _)| *k == Kind::Unproto).map(|&(_, db)| sn_line(db)).collect();
    let reported: Vec<String> = lines.iter().map(|(db, _)| sn_line(*db)).collect();
    assert_eq!(reported, unproto_snr);
    // In the modem's output each line follows the frames it names.
    let mut seen_frames = 0;
    for o in &air.out[0] {
        match o {
            HostOut::Kiss(_) => seen_frames += 1,
            HostOut::Line(l) if l.starts_with("UNPROTO ") => {
                let (_, c) = parse_unproto_sn(l);
                let before: Vec<u32> = got[..seen_frames].iter().map(|f| frame_crc(f)).collect();
                assert!(before.ends_with(&c), "{l} before its frames");
            }
            _ => {}
        }
    }
    // Session SN lines: one per session burst heard, none for the broadcasts.
    let session_bursts = air.rx[0].iter().filter(|(k, _)| *k != Kind::Unproto).count();
    assert_eq!(air.lines(0, "SN ").len(), session_bursts);
    assert!(!air.lines(1, "SN ").is_empty());
    // B delivers the frames but did not ask for UNPROTO SN; C hears none of its own.
    assert_eq!(air.kiss(1).len(), 5);
    assert!(air.lines(1, "UNPROTO ").is_empty());
    assert!(air.lines(2, "UNPROTO ").is_empty());
}

/// Through sim-pair over TCP, the way a host such as Ortoo would use it: B's host
/// asks for UNPROTO SN and attributes each KISS frame to a line by its CRC-32,
/// whatever order the two sockets deliver them in. Every one of the 19 frames of a
/// ten-line broadcast gets exactly one S/N, and a second KISS client on B (like
/// VarAC) still gets the bytes A's client wrote, unchanged.
#[test]
fn unproto_sn_lets_the_host_attribute_every_frame() {
    let sp = sim("narrow", 20.0, 21);
    let mut ah = VaraHost::connect("A", sp.addrs[0]);
    let mut bh = VaraHost::connect("B", sp.addrs[1]);
    ah.command("UNPROTO SN ON");
    bh.command("UNPROTO SN ON");
    let mut a = KissClient::connect("A kiss", sp.kiss_addrs[0].unwrap());
    let b = KissClient::connect("B kiss (Ortoo)", sp.kiss_addrs[1].unwrap());
    let other = KissClient::connect("B kiss (VarAC)", sp.kiss_addrs[1].unwrap());
    let frames = ten_line_broadcast();
    let wire = a.send(&frames);
    let (got, raw) = b.wait(frames.len());
    let (_, raw2) = other.wait(frames.len());
    assert_eq!(raw, wire);
    assert_eq!(raw2, wire, "the second KISS client's bytes differ");

    // Host side: collect the named CRCs, then match each frame against the oldest
    // unmatched entry with its CRC.
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut pending: Vec<(u32, f32)>;
    loop {
        bh.drain();
        pending = bh
            .seen
            .iter()
            .filter(|l| l.starts_with("UNPROTO SN "))
            .flat_map(|l| {
                let (db, c) = parse_unproto_sn(l);
                c.into_iter().map(move |x| (x, db))
            })
            .collect();
        if pending.len() >= frames.len() || Instant::now() > deadline {
            break;
        }
        thread::sleep(Duration::from_millis(20));
    }
    let mut attributed = Vec::new();
    for f in &got {
        let crc = frame_crc(f);
        let k = pending.iter().position(|(c, _)| *c == crc).unwrap_or_else(|| panic!("no S/N for frame {}", hex(f)));
        attributed.push(pending.remove(k).1);
    }
    assert!(pending.is_empty(), "lines name frames that never arrived: {pending:?}");
    let lines: Vec<&String> = bh.seen.iter().filter(|l| l.starts_with("UNPROTO ")).collect();
    println!("{} frames attributed from {} lines: {lines:?}", attributed.len(), lines.len());
    assert!(attributed.iter().all(|db| (10.0..40.0).contains(db)), "{attributed:?}");
    // A sent them; it hears none of its own.
    thread::sleep(Duration::from_millis(200));
    ah.drain();
    assert!(!ah.seen.iter().any(|l| l.starts_with("UNPROTO ")), "{:?}", ah.seen);
    sp.stop();
}
