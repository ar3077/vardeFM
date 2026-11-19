//! One VardeFM station: link + modulator + receiver, speaking the VARA host protocol.
//!
//! The station is driven from outside: feed it host input and received audio, ask
//! it what to transmit, and collect its host output. It has no threads or clock.

use crate::host::{parse, Command};
use crate::level::{LevelMeter, HOST_LEVEL_WINDOW_S};
use std::sync::Arc;
use vardefm_link::{Link, LinkConfig, LinkEvent, Node, TxInfo};
use vardefm_modem::{AudioLevel, Header, Modem, Receiver, RxBurst};

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
    /// A digipeater repeat of another station's burst.
    pub repeat: bool,
}

const IAMALIVE_S: f64 = 60.0;

pub struct Station {
    pub name: String,
    modem: Arc<Modem>,
    /// The link and, if set, the digipeater.
    node: Node,
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
    /// `LEVELS ON`: report the receive level every [`HOST_LEVEL_WINDOW_S`] while
    /// not transmitting (`LEVEL <rms> <peak> <raw>`).
    levels: bool,
    /// The meter behind `LEVEL`, fed only while `levels` is on.
    level_meter: LevelMeter,
}

impl Station {
    pub fn new(name: &str, modem: Arc<Modem>, cfg: LinkConfig, seed: u32) -> Self {
        let bandwidth = if modem.ofdm.p.name == "wide" { "WIDE" } else { "NARROW" };
        Self {
            name: name.to_string(),
            node: Node::new(Link::new(modem.clone(), cfg, seed)),
            rx: Receiver::new(modem.clone()),
            level_meter: LevelMeter::new((HOST_LEVEL_WINDOW_S * modem.fs()) as usize),
            levels: false,
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

    pub fn link_config(&self) -> &LinkConfig {
        self.node.link.config()
    }

    /// The link, read only (state, statistics).
    pub fn link(&self) -> &Link {
        &self.node.link
    }

    /// Repeat the bursts of sessions routed through any of `calls` (none: off).
    /// Works without a host and alongside our own sessions.
    pub fn set_digi(&mut self, calls: &[String]) {
        self.node.set_digi(calls);
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
                self.node.link.set_mycalls(calls);
                self.line("OK");
            }
            Command::Listen(on) => {
                self.node.link.set_listen(on);
                self.line("OK");
            }
            Command::Connect { source, target, via } => match self.node.link.connect(now, &source, &target, &via) {
                Ok(()) => self.line("OK"),
                Err(()) => self.line("WRONG"),
            },
            Command::Disconnect => {
                self.line("OK");
                self.node.link.disconnect(now);
            }
            Command::Abort => {
                self.line("OK");
                self.node.link.abort(now);
            }
            Command::Chat(on) => {
                self.chat = on;
                self.line("OK");
            }
            Command::UnprotoSn(on) => {
                self.unproto_sn = on;
                self.line("OK");
            }
            Command::Levels(on) => {
                self.levels = on;
                // The meter is not fed while off: drop what it held from before.
                self.level_meter.reset();
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
        self.node.link.send_data(data);
    }

    /// A data frame from the KISS port (type byte first): queued for an Unproto
    /// burst. Frames too long for one burst are dropped (counted in the link stats).
    pub fn kiss_in(&mut self, now: f64, frame: &[u8]) {
        self.node.link.send_unproto(now, frame);
    }

    /// The host closed the command port: drop any session, and forget its settings
    /// that only matter to it (`CHAT`, `UNPROTO SN`, `LEVELS`), so the next host
    /// starts from the defaults.
    pub fn host_closed(&mut self, now: f64) {
        self.chat = false;
        self.unproto_sn = false;
        self.levels = false;
        self.node.link.abort(now);
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
        let (burst, repeat) = self.node.poll(now, self.rx.busy())?;
        let audio = self.modem.modulate(&burst);
        self.transmitting = true;
        self.rx.reset();
        self.level_meter.reset();
        self.line("PTT ON");
        Some(TxStart {
            seconds: audio.len() as f32 / self.modem.fs(),
            audio,
            info: if repeat {
                TxInfo {
                    kind: burst.header.kind,
                    level: burst.header.level,
                    ncw: burst.payload.len(),
                    ..TxInfo::default()
                }
            } else {
                self.node.link.last_tx().clone()
            },
            header: burst.header,
            repeat,
        })
    }

    /// The burst returned by `poll_tx` has been fully transmitted.
    pub fn tx_done(&mut self, now: f64) {
        self.transmitting = false;
        self.line("PTT OFF");
        self.node.tx_done(now);
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
            self.node.on_burst(now, b);
        }
        let busy = self.rx.busy();
        if busy != self.busy_reported {
            self.busy_reported = busy;
            self.line(if busy { "BUSY ON" } else { "BUSY OFF" });
        }
        if self.levels {
            if let Some(level) = self.level_meter.push(samples) {
                // No input gain yet: the raw peak is the peak.
                self.line(level_line(level, level.peak_dbfs));
            }
        }
        bursts
    }

    /// Everything for the host since the last call, in order.
    pub fn take_output(&mut self) -> Vec<HostOut> {
        // CRC-32 of each Unproto frame handed to the KISS port in this batch, for the
        // `UNPROTO SN` line that follows a burst's frames.
        let mut crcs: Vec<u32> = Vec::new();
        for e in self.node.link.take_events() {
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

/// `v` rounded to one decimal for a host line: never "-0.0", and 0 if not finite.
fn tenths(v: f32) -> f32 {
    let v = (v * 10.0).round() / 10.0;
    if v == 0.0 || !v.is_finite() {
        0.0
    } else {
        v
    }
}

/// `SN <dB>` with one decimal, as VARA sends it (e.g. `SN 12.3`, `SN -2.0`).
pub fn sn_line(snr_db: f32) -> String {
    format!("SN {:.1}", tenths(snr_db))
}

/// `LEVEL <rms> <peak> <raw>`: the receive level in dBFS with one decimal each.
/// `rms` and `peak` are of the audio the receiver gets; `raw` is the peak the sound
/// card delivered, before any input gain (only that one shows sound-card clipping).
pub fn level_line(level: AudioLevel, raw_peak_dbfs: f32) -> String {
    format!("LEVEL {:.1} {:.1} {:.1}", tenths(level.rms_dbfs), tenths(level.peak_dbfs), tenths(raw_peak_dbfs))
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
    use super::{frame_crc, level_line, sn_line, unproto_sn_line, HostOut, Station};
    use std::sync::Arc;
    use vardefm_link::LinkConfig;
    use vardefm_modem::{AudioLevel, Modem, OfdmParams};

    /// 10 ms at 48 kHz, as a sound card usually delivers it.
    const CHUNK: usize = 480;

    fn station() -> Station {
        Station::new("A", Arc::new(Modem::new(OfdmParams::narrow(), 0.1)), LinkConfig::default(), 7)
    }

    /// Quiet noise-like audio (about -31 dBFS rms), never loud enough to read busy.
    fn hiss() -> Vec<f32> {
        (0..CHUNK).map(|i| 0.05 * ((i * 7919) % 200) as f32 / 100.0 - 0.05).collect()
    }

    /// Command-port lines since the last call.
    fn lines(st: &mut Station) -> Vec<String> {
        st.take_output()
            .into_iter()
            .filter_map(|o| match o {
                HostOut::Line(l) => Some(l),
                _ => None,
            })
            .collect()
    }

    /// Feeds `chunks` chunks of hiss from time `t`; returns the `LEVEL` lines and the new time.
    fn listen(st: &mut Station, t: f64, chunks: usize) -> (Vec<String>, f64) {
        let mut t = t;
        let mut got = Vec::new();
        for _ in 0..chunks {
            t += CHUNK as f64 / 48_000.0;
            st.rx_audio(t, &hiss());
            got.extend(lines(st).into_iter().filter(|l| l.starts_with("LEVEL ")));
        }
        (got, t)
    }

    #[test]
    fn level_format() {
        let l = AudioLevel { rms_dbfs: -27.34, peak_dbfs: -18.35 };
        assert_eq!(level_line(l, l.peak_dbfs), "LEVEL -27.3 -18.4 -18.4");
        assert_eq!(level_line(l, -6.04), "LEVEL -27.3 -18.4 -6.0");
        let zero = AudioLevel { rms_dbfs: -0.04, peak_dbfs: 0.0 };
        assert_eq!(level_line(zero, 0.04), "LEVEL 0.0 0.0 0.0", "no -0.0");
        assert_eq!(level_line(AudioLevel::of(&[0.0; 10]), -120.0), "LEVEL -120.0 -120.0 -120.0");
        // With --rx-gain the gained peak can exceed full scale.
        assert_eq!(level_line(AudioLevel { rms_dbfs: -3.0, peak_dbfs: 4.25 }, -7.75), "LEVEL -3.0 4.3 -7.8");
    }

    #[test]
    fn level_lines_only_while_on() {
        let mut st = station();
        let (got, t) = listen(&mut st, 0.0, 100);
        assert!(got.is_empty(), "off by default: {got:?}");

        st.host_line(t, "LEVELS ON");
        assert_eq!(lines(&mut st), ["OK"]);
        // 1 s of audio in 10 ms chunks: one line per 0.25 s.
        let (got, t) = listen(&mut st, t, 100);
        assert_eq!(got.len(), 4, "{got:?}");
        for l in &got {
            let v: Vec<f32> = l["LEVEL ".len()..].split(' ').map(|w| w.parse().unwrap()).collect();
            assert_eq!(v.len(), 3, "{l}");
            assert!((v[0] + 30.8).abs() < 0.5, "rms of the hiss: {l}");
            assert!(v[1] > v[0] && v[1] < 0.0, "{l}");
            assert_eq!(v[2], v[1], "raw = peak until there is an input gain: {l}");
        }

        st.host_line(t, "LEVELS OFF");
        assert_eq!(lines(&mut st), ["OK"]);
        let (got, _) = listen(&mut st, t, 100);
        assert!(got.is_empty(), "{got:?}");
    }

    #[test]
    fn levels_off_again_when_the_host_goes_away() {
        let mut st = station();
        st.host_line(0.0, "LEVELS ON");
        let (got, t) = listen(&mut st, 0.0, 25);
        assert_eq!(got.len(), 1, "{got:?}");
        st.host_closed(t);
        let (got, _) = listen(&mut st, t, 100);
        assert!(got.is_empty(), "the next host starts with LEVELS off: {got:?}");
    }

    #[test]
    fn levels_on_starts_a_fresh_window() {
        let mut st = station();
        st.host_line(0.0, "LEVELS ON");
        let (_, t) = listen(&mut st, 0.0, 20);
        st.host_line(t, "LEVELS OFF");
        st.host_line(t, "LEVELS ON");
        lines(&mut st);
        let (got, t) = listen(&mut st, t, 24);
        assert!(got.is_empty(), "the 0.2 s from before OFF were counted: {got:?}");
        let (got, _) = listen(&mut st, t, 1);
        assert_eq!(got.len(), 1);
    }

    #[test]
    fn no_level_lines_while_transmitting() {
        let mut st = station();
        st.host_line(0.0, "MYCALL LA1ABC");
        st.host_line(0.0, "LEVELS ON");
        // 0.2 s of a window before key-up: thrown away with the key-up.
        let (_, mut t) = listen(&mut st, 0.0, 20);
        st.host_line(t, "CONNECT LA1ABC LA2XYZ");
        let tx = loop {
            t += 0.01;
            if let Some(tx) = st.poll_tx(t) {
                break tx;
            }
            assert!(t < 10.0, "the connect request never went out");
        };
        assert!(lines(&mut st).contains(&"PTT ON".to_string()));
        let (got, t) = listen(&mut st, t, (tx.seconds * 100.0) as usize + 50);
        assert!(got.is_empty(), "LEVEL while transmitting: {got:?}");
        st.tx_done(t);
        lines(&mut st);
        let (got, t) = listen(&mut st, t, 24);
        assert!(got.is_empty(), "the partial window from before key-up was kept: {got:?}");
        let (got, _) = listen(&mut st, t, 1);
        assert_eq!(got.len(), 1, "{got:?}");
    }

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
