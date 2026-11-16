//! `SN` reports (S/N of the session peer, VARA's `CHAT ON` behaviour), with
//! several stations on one simulated FM channel: two sessions side by side and
//! a station that only listens. Every station hears every burst, but a station
//! reports the S/N only of bursts of its own session, and only with `CHAT ON`.
//!
//! Set VARDEFM_SIM_VERBOSE=1 to print every transmission.

use std::collections::BTreeSet;
use std::sync::Arc;
use vardefm_link::LinkConfig;
use vardefm_modem::{Kind, Modem, OfdmParams};
use vardefm_sim::{transmit, FmChannelConfig};
use vardefm_tnc::station::{sn_line, HostOut, Station};

const STEP: usize = 480;

/// Stations that all hear each other through the FM channel simulator.
struct Air {
    st: Vec<Station>,
    out: Vec<Vec<HostOut>>,
    incoming: Vec<Vec<(u64, Vec<f32>)>>,
    tx_end: Vec<Option<u64>>,
    channel: FmChannelConfig,
    t: u64,
    n_tx: u64,
    /// Session ids each station has transmitted with.
    tx_sessions: Vec<BTreeSet<u8>>,
    /// Bursts each station decoded: (session id, kind, SNR dB).
    rx: Vec<Vec<(u8, Kind, f32)>>,
    log: Vec<String>,
}

impl Air {
    fn new(names: &[&str], cnr_db: f32) -> Self {
        let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
        let n = names.len();
        Self {
            st: names
                .iter()
                .enumerate()
                .map(|(i, name)| Station::new(name, modem.clone(), LinkConfig::default(), 71 + i as u32))
                .collect(),
            out: vec![Vec::new(); n],
            incoming: vec![Vec::new(); n],
            tx_end: vec![None; n],
            channel: FmChannelConfig {
                cnr_db: Some(cnr_db),
                keyup_delay_s: 0.05,
                lead_silence_s: 0.0,
                ..FmChannelConfig::by_name("narrow").unwrap()
            },
            t: 0,
            n_tx: 0,
            tx_sessions: vec![BTreeSet::new(); n],
            rx: vec![Vec::new(); n],
            log: Vec::new(),
        }
    }

    fn now(&self) -> f64 {
        self.t as f64 / 48_000.0
    }

    fn cmd(&mut self, i: usize, line: &str) {
        let now = self.now();
        self.st[i].host_line(now, line);
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
                    let h = tx.header;
                    self.log.push(format!("{now:7.2}s {} tx {:?} session {}", self.st[i].name, h.kind, h.session));
                    if std::env::var("VARDEFM_SIM_VERBOSE").is_ok() {
                        println!("{}", self.log.last().unwrap());
                    }
                    self.tx_sessions[i].insert(h.session);
                    for j in (0..n).filter(|&j| j != i) {
                        self.n_tx += 1;
                        let ch = FmChannelConfig { seed: 5000 + self.n_tx, ..self.channel.clone() };
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
            for burst in self.st[i].rx_audio(now, &chunk) {
                self.rx[i].push((burst.header.session, burst.header.kind, burst.snr_db));
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

    fn lines(&self, i: usize, prefix: &str) -> Vec<String> {
        self.out[i]
            .iter()
            .filter_map(|o| match o {
                HostOut::Line(l) if l.starts_with(prefix) => Some(l.clone()),
                _ => None,
            })
            .collect()
    }

    fn data_len(&self, i: usize) -> usize {
        self.out[i].iter().map(|o| if let HostOut::Data(d) = o { d.len() } else { 0 }).sum()
    }

    /// The one session id station `i` has transmitted with.
    fn session(&self, i: usize) -> u8 {
        assert_eq!(self.tx_sessions[i].len(), 1, "{}: sessions {:?}", self.st[i].name, self.tx_sessions[i]);
        *self.tx_sessions[i].first().unwrap()
    }
}

#[test]
fn sn_only_for_our_own_session_and_only_with_chat_on() {
    const A: usize = 0;
    const B: usize = 1;
    const C: usize = 2;
    const D: usize = 3;
    const E: usize = 4;
    let mut air = Air::new(&["A", "B", "C", "D", "E"], 25.0);
    for (i, call) in [(A, "LA1AAA"), (B, "LA2BBB"), (C, "LA3CCC"), (D, "LA4DDD"), (E, "LA5EEE")] {
        air.cmd(i, &format!("MYCALL {call}"));
        air.cmd(i, "LISTEN ON");
    }
    // A, C, D and E want SN reports; B does not (CHAT is off by default, and is set
    // off explicitly here).
    for i in [A, C, D, E] {
        air.cmd(i, "CHAT ON");
    }
    air.cmd(B, "CHAT OFF");

    // Session 1: A calls B and sends data.
    air.cmd(A, "CONNECT LA1AAA LA2BBB");
    air.run_until(30.0, |a| !a.lines(A, "CONNECTED").is_empty() && !a.lines(B, "CONNECTED").is_empty());
    air.st[A].host_data(&vec![0x55; 6000]);
    air.run_until(90.0, |a| a.data_len(B) >= 6000);

    // Session 2 on the same frequency while session 1 is still up: C calls D.
    air.cmd(C, "CONNECT LA3CCC LA4DDD");
    air.run_until(30.0, |a| !a.lines(C, "CONNECTED").is_empty() && !a.lines(D, "CONNECTED").is_empty());
    air.st[D].host_data(&vec![0xaa; 6000]);
    air.run_until(90.0, |a| a.data_len(C) >= 6000);
    // Let the reply traffic and session 1's idle period play out a little.
    let until = air.now() + 3.0;
    air.run_until(10.0, |a| a.now() >= until);
    assert!(air.st[A].link.is_connected() && air.st[C].link.is_connected(), "a session dropped");

    let (s1, s2) = (air.session(A), air.session(C));
    assert_eq!(air.session(B), s1);
    assert_eq!(air.session(D), s2);
    assert_ne!(s1, s2, "the two sessions picked the same id; change the seeds");

    for (i, own, chat) in
        [(A, Some(s1), true), (B, Some(s1), false), (C, Some(s2), true), (D, Some(s2), true), (E, None, true)]
    {
        let name = air.st[i].name.clone();
        let heard = &air.rx[i];
        let others = heard.iter().filter(|(s, _, _)| Some(*s) != own).count();
        let expected: Vec<String> = if chat {
            heard.iter().filter(|(s, _, _)| Some(*s) == own).map(|&(_, _, db)| sn_line(db)).collect()
        } else {
            Vec::new()
        };
        let got = air.lines(i, "SN ");
        println!(
            "{name}: decoded {} bursts ({others} of other sessions), SN lines {}: {:?}",
            heard.len(),
            got.len(),
            &got[..got.len().min(6)]
        );
        assert!(others > 0, "{name} heard no other session's bursts, so the test proves nothing");
        assert_eq!(got, expected, "{name}");
        if chat && own.is_some() {
            assert!(got.len() >= 3, "{name}: only {} SN lines", got.len());
        }
    }
    // The S/N of the peer comes after CONNECTED, never before it.
    for i in [A, C, D] {
        let pos = |p: &str| air.out[i].iter().position(|o| matches!(o, HostOut::Line(l) if l.starts_with(p)));
        assert!(pos("CONNECTED") < pos("SN "), "{}", air.st[i].name);
    }
}

/// A new command-port client starts with CHAT OFF, whatever the last one set.
#[test]
fn chat_is_reset_when_the_host_goes_away() {
    let mut air = Air::new(&["A", "B"], 25.0);
    air.cmd(0, "MYCALL LA1AAA");
    air.cmd(1, "MYCALL LA2BBB");
    air.cmd(1, "LISTEN ON");
    air.cmd(0, "CHAT ON");
    air.cmd(1, "CHAT ON");
    air.cmd(0, "CONNECT LA1AAA LA2BBB");
    air.run_until(30.0, |a| !a.lines(0, "SN ").is_empty() && !a.lines(1, "SN ").is_empty());
    // B's host goes away and comes back without CHAT ON.
    let now = air.now();
    air.st[1].host_closed(now);
    air.out[1].clear();
    air.cmd(1, "LISTEN ON");
    air.run_until(30.0, |a| !a.lines(0, "DISCONNECTED").is_empty());
    air.cmd(0, "CONNECT LA1AAA LA2BBB");
    air.run_until(60.0, |a| a.lines(0, "CONNECTED").len() >= 2 && !a.lines(1, "CONNECTED").is_empty());
    air.st[0].host_data(&[1; 500]);
    air.run_until(60.0, |a| a.data_len(1) >= 500);
    assert!(air.lines(1, "SN ").is_empty(), "{:?}", air.lines(1, "SN "));
}
