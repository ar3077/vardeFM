//! `LEVELS ON/OFF` over a simulated FM channel: `LEVEL` lines every 0.25 s while
//! not transmitting, one `RXPEAK` per decoded burst of any kind, the replay of the
//! last burst after `LEVELS ON`, and nothing after `LEVELS OFF` or when the host
//! goes away. A station whose host never sends `LEVELS` behaves exactly as before.
//!
//! Set VARDEFM_SIM_VERBOSE=1 to print every transmission.

use std::sync::Arc;
use vardefm_link::LinkConfig;
use vardefm_modem::{AudioLevel, Kind, Modem, OfdmParams};
use vardefm_sim::{transmit, FmChannelConfig};
use vardefm_tnc::station::{rxpeak_line, HostOut, Station};

const STEP: usize = 480;
/// Samples per `LEVEL` line (0.25 s at 48 kHz).
const WINDOW: usize = 12_000;
const A: usize = 0;
const B: usize = 1;
const C: usize = 2;

/// Stations that all hear each other through the FM channel simulator.
struct Air {
    st: Vec<Station>,
    out: Vec<Vec<HostOut>>,
    incoming: Vec<Vec<(u64, Vec<f32>)>>,
    tx_end: Vec<Option<u64>>,
    channel: FmChannelConfig,
    t: u64,
    n_tx: u64,
    /// Bursts each station decoded: (time passed to `rx_audio`, kind, level).
    rx: Vec<Vec<(f64, Kind, AudioLevel)>>,
    /// Samples each station was given while not transmitting.
    rx_samples: Vec<usize>,
    /// Transmissions each station keyed up.
    tx_count: Vec<usize>,
    log: Vec<String>,
}

impl Air {
    fn new(n: usize) -> Self {
        let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
        Self {
            st: (0..n)
                .map(|i| Station::new(["A", "B", "C"][i], modem.clone(), LinkConfig::default(), 31 + i as u32))
                .collect(),
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
            rx: vec![Vec::new(); n],
            rx_samples: vec![0; n],
            tx_count: vec![0; n],
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
                    self.tx_count[i] += 1;
                    for j in (0..n).filter(|&j| j != i) {
                        self.n_tx += 1;
                        let ch = FmChannelConfig { seed: 7000 + self.n_tx, ..self.channel.clone() };
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
            if !self.st[i].transmitting() {
                self.rx_samples[i] += STEP;
            }
            for burst in self.st[i].rx_audio(now, &chunk) {
                self.rx[i].push((now, burst.header.kind, burst.level));
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

    fn run_for(&mut self, seconds: f64) {
        let until = self.now() + seconds;
        self.run_until(seconds + 1.0, |a| a.now() >= until);
    }

    fn has_line(&self, i: usize, prefix: &str) -> bool {
        self.out[i].iter().any(|o| matches!(o, HostOut::Line(l) if l.starts_with(prefix)))
    }

    fn data_len(&self, i: usize) -> usize {
        self.out[i].iter().map(|o| if let HostOut::Data(d) = o { d.len() } else { 0 }).sum()
    }
}

/// Command-port lines of `out` starting with `prefix`.
fn lines<'a>(out: &'a [HostOut], prefix: &str) -> Vec<&'a str> {
    out.iter()
        .filter_map(|o| match o {
            HostOut::Line(l) if l.starts_with(prefix) => Some(l.as_str()),
            _ => None,
        })
        .collect()
}

/// Where each phase of [`scenario`] starts: indices into A's output and burst list,
/// and A's sample and transmission counters at that point.
#[derive(Clone, Copy, Default)]
struct Mark {
    out: usize,
    rx: usize,
    rx_samples: usize,
    tx_count: usize,
    now: f64,
}

fn mark(air: &Air) -> Mark {
    Mark {
        out: air.out[A].len(),
        rx: air.rx[A].len(),
        rx_samples: air.rx_samples[A],
        tx_count: air.tx_count[A],
        now: air.now(),
    }
}

/// C broadcasts, A calls B and sends data, B answers with data, then A's host goes
/// away. With `levels`, A's host turns `LEVELS` on at the start, off before B's data
/// and on again after it. Returns the air and the marks at: LEVELS OFF, the second
/// LEVELS ON, the host leaving (all `Default` without `levels`).
fn scenario(levels: bool) -> (Air, [Mark; 3]) {
    let mut air = Air::new(3);
    let mut marks = [Mark::default(); 3];
    air.cmd(A, "MYCALL LA1AAA");
    air.cmd(B, "MYCALL LA2BBB");
    air.cmd(B, "LISTEN ON");
    air.cmd(C, "MYCALL LA3CCC");
    if levels {
        air.cmd(A, "LEVELS ON");
    }
    let now = air.now();
    air.st[C].kiss_in(now, b"\x00level test broadcast");
    air.run_until(20.0, |a| a.out[A].iter().any(|o| matches!(o, HostOut::Kiss(_))));

    air.cmd(A, "CONNECT LA1AAA LA2BBB");
    air.run_until(30.0, |a| a.has_line(A, "CONNECTED") && a.has_line(B, "CONNECTED"));
    air.st[A].host_data(&[0x55; 3000]);
    air.run_until(60.0, |a| a.data_len(B) >= 3000);

    if levels {
        marks[0] = mark(&air);
        air.cmd(A, "LEVELS OFF");
    }
    air.st[B].host_data(&[0xaa; 1000]);
    air.run_until(60.0, |a| a.data_len(A) >= 1000);
    air.run_for(1.5);

    if levels {
        marks[1] = mark(&air);
        air.cmd(A, "LEVELS ON");
    }
    air.run_for(3.0);

    let now = air.now();
    if levels {
        marks[2] = mark(&air);
    }
    air.st[A].host_closed(now);
    air.run_for(3.0);
    (air, marks)
}

#[test]
fn level_reports_over_the_air() {
    let (air, [off, on_again, closed]) = scenario(true);
    let out = &air.out[A];

    // Phase 1, LEVELS on from the start: one LEVEL per full 0.25 s of received audio.
    // A key-up drops the partial window, so each transmission can cost one line.
    let phase1 = &out[..off.out];
    let n_level = lines(phase1, "LEVEL ").len();
    let windows = off.rx_samples / WINDOW;
    println!("phase 1: {n_level} LEVEL lines, {windows} full windows heard, {} transmissions", off.tx_count);
    assert!(off.tx_count >= 2, "A hardly transmitted, so the test proves little");
    assert!(n_level <= windows && n_level + off.tx_count + 1 >= windows, "{n_level} LEVEL lines for {windows} windows");
    for l in lines(phase1, "LEVEL ") {
        let v: Vec<f32> = l["LEVEL ".len()..].split(' ').map(|w| w.parse().unwrap()).collect();
        assert!(v.len() == 3 && v[1] >= v[0] && v[2] == v[1], "{l}");
    }

    // Every burst A decoded gives one live RXPEAK, in order, with the burst's level.
    let heard = &air.rx[A][..off.rx];
    let kinds: Vec<Kind> = heard.iter().map(|&(_, k, _)| k).collect();
    assert!(
        kinds.contains(&Kind::Unproto) && kinds.iter().any(|&k| k != Kind::Unproto),
        "an Unproto and a session burst: {kinds:?}"
    );
    let expected: Vec<String> = heard.iter().map(|&(_, _, level)| rxpeak_line(level, 0)).collect();
    assert_eq!(lines(phase1, "RXPEAK "), expected);

    // Phase 2, LEVELS off: A keeps decoding, but reports nothing.
    let phase2 = &out[off.out..on_again.out];
    assert_eq!(lines(phase2, "OK"), ["OK"], "the answer to LEVELS OFF");
    assert!(on_again.rx > off.rx, "A decoded nothing while LEVELS was off");
    assert!(lines(phase2, "LEVEL ").is_empty() && lines(phase2, "RXPEAK ").is_empty(), "{phase2:?}");

    // Phase 3, LEVELS on again: OK, then the last burst with its age, then live reports.
    let phase3 = &out[on_again.out..closed.out];
    let (at, _, level) = air.rx[A][on_again.rx - 1];
    let replay = rxpeak_line(level, (on_again.now - at) as u64);
    assert_eq!(phase3[..2], [HostOut::Line("OK".into()), HostOut::Line(replay.clone())], "{phase3:?}");
    println!("replay after LEVELS ON: {replay}");
    let live: Vec<String> = air.rx[A][on_again.rx..closed.rx].iter().map(|&(_, _, l)| rxpeak_line(l, 0)).collect();
    assert_eq!(lines(&phase3[2..], "RXPEAK "), live);
    assert!(!lines(phase3, "LEVEL ").is_empty());

    // After the host went away: nothing more, though A still hears B.
    let after = &out[closed.out..];
    assert!(air.rx[A].len() > closed.rx, "A decoded nothing after its host left");
    assert!(lines(after, "LEVEL ").is_empty() && lines(after, "RXPEAK ").is_empty(), "{after:?}");

    // Never a LEVEL while transmitting (between PTT ON and PTT OFF).
    let mut keyed = false;
    for o in out {
        match o {
            HostOut::Line(l) if l == "PTT ON" => keyed = true,
            HostOut::Line(l) if l == "PTT OFF" => keyed = false,
            HostOut::Line(l) if l.starts_with("LEVEL ") => assert!(!keyed, "{l} while transmitting"),
            _ => {}
        }
    }

    // B and C never asked.
    for i in [B, C] {
        assert!(lines(&air.out[i], "LEVEL ").is_empty() && lines(&air.out[i], "RXPEAK ").is_empty());
    }
}

/// The same scenario with and without the LEVELS commands: apart from the LEVEL and
/// RXPEAK lines and the OK answers, every station's host output and every
/// transmission are identical.
#[test]
fn a_host_that_never_sends_levels_sees_no_change() {
    let (plain, _) = scenario(false);
    let (metered, _) = scenario(true);
    assert_eq!(plain.log, metered.log, "the transmissions differ");
    let from_levels = |o: &HostOut| match o {
        HostOut::Line(l) => l == "OK" || l.starts_with("LEVEL ") || l.starts_with("RXPEAK "),
        _ => false,
    };
    let strip = |out: &[HostOut]| -> Vec<HostOut> { out.iter().filter(|o| !from_levels(o)).cloned().collect() };
    assert!(plain.out[A].iter().any(|o| matches!(o, HostOut::Data(_) | HostOut::Kiss(_))));
    assert!(lines(&plain.out[A], "LEVEL ").is_empty() && lines(&plain.out[A], "RXPEAK ").is_empty());
    assert_eq!(strip(&plain.out[A]), strip(&metered.out[A]), "A");
    for i in [B, C] {
        assert_eq!(plain.out[i], metered.out[i], "{}", plain.st[i].name);
    }
}
