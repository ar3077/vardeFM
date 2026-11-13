//! Link tests over a fake channel: bursts take their real airtime, collide when
//! both stations transmit, and can lose whole bursts or single codewords.

use super::*;
use vardefm_modem::OfdmParams;

const DT: f64 = 0.01;

struct Node {
    link: Link,
    /// (start, end, burst) while transmitting.
    tx: Option<(f64, f64, Burst)>,
    host_rx: Vec<u8>,
    events: Vec<LinkEvent>,
    /// Stop receiving and transmitting (station switched off).
    dead: bool,
}

struct Net {
    nodes: [Node; 2],
    modem: Arc<Modem>,
    now: f64,
    /// Bursts on their way: (deliver_at, to, burst, start, end).
    air: Vec<(f64, usize, Burst, f64, f64)>,
    rng: Prng,
    burst_loss: f64,
    cw_loss: f64,
    airtime: f64,
}

impl Net {
    fn new(seed: u32, burst_loss: f64, cw_loss: f64) -> Self {
        let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
        let node = |s| Node {
            link: Link::new(modem.clone(), LinkConfig::default(), s),
            tx: None,
            host_rx: Vec::new(),
            events: Vec::new(),
            dead: false,
        };
        let mut n = Self {
            nodes: [node(seed), node(seed + 1)],
            modem,
            now: 0.0,
            air: Vec::new(),
            rng: Prng::new(seed + 99),
            burst_loss,
            cw_loss,
            airtime: 0.0,
        };
        n.nodes[0].link.set_mycalls(vec!["LA1AAA".into()]);
        n.nodes[1].link.set_mycalls(vec!["LA2BBB-1".into()]);
        n.nodes[1].link.set_listen(true);
        n
    }

    fn chance(&mut self, p: f64) -> bool {
        (self.rng.next_u32() as f64 / u32::MAX as f64) < p
    }

    fn transmitting(&self, i: usize, a: f64, b: f64) -> bool {
        self.nodes[i].tx.as_ref().is_some_and(|(s, e, _)| *s < b && *e > a)
            || self.air.iter().any(|(_, to, _, s, e)| *to != i && *s < b && *e > a)
    }

    fn busy(&self, i: usize) -> bool {
        // Hear the other station from key-up until the squelch tail ends.
        let other = &self.nodes[1 - i];
        other.tx.as_ref().is_some_and(|(s, _, _)| self.now >= s + 0.1)
            || self.air.iter().any(|(_, to, _, _, e)| *to == i && self.now < e + 0.25)
    }

    fn step(&mut self) {
        self.now += DT;
        let now = self.now;
        // Finish transmissions and deliver.
        for i in 0..2 {
            if let Some((start, end, _)) = &self.nodes[i].tx {
                if now >= *end {
                    let (start, end) = (*start, *end);
                    let (_, _, burst) = self.nodes[i].tx.take().unwrap();
                    self.nodes[i].link.tx_done(now);
                    self.airtime += end - start;
                    self.air.push((end + 0.02, 1 - i, burst, start, end));
                }
            }
        }
        let mut arrived = Vec::new();
        self.air.retain(|a| {
            if a.0 <= now {
                arrived.push(a.clone());
                false
            } else {
                true
            }
        });
        for (_, to, burst, start, end) in arrived {
            if self.nodes[to].dead || self.transmitting(to, start, end) || self.chance(self.burst_loss) {
                if std::env::var("LINK_TRACE").is_ok() {
                    eprintln!("{now:7.2} lost burst to {}", ["A", "B"][to]);
                }
                continue;
            }
            let payload = burst
                .payload
                .iter()
                .map(|cw| if self.chance(self.cw_loss) { None } else { Some(cw.clone()) })
                .collect();
            let rx = RxBurst { header: burst.header, payload, snr_db: 30.0, end_sample: 0, level: Default::default() };
            self.nodes[to].link.on_burst(now, &rx);
        }
        // Poll for new transmissions.
        for i in 0..2 {
            if self.nodes[i].dead || self.nodes[i].tx.is_some() {
                continue;
            }
            let busy = self.busy(i);
            if let Some(b) = self.nodes[i].link.poll(now, busy) {
                let dur = self.modem.burst_seconds(b.header.level, b.header.ncw as usize) as f64;
                if std::env::var("LINK_TRACE").is_ok() {
                    eprintln!(
                        "{now:7.2} {} TX {:?} ncw={} lvl={} id={} ack={}/{} bm={:b} {:.2}s",
                        ["A", "B"][i],
                        b.header.kind,
                        b.header.ncw,
                        b.header.level,
                        b.header.burst_id,
                        b.header.ack_burst,
                        b.header.ack_offset,
                        b.header.bitmap,
                        dur
                    );
                }
                self.nodes[i].tx = Some((now, now + dur, b));
            }
        }
        for n in &mut self.nodes {
            for e in n.link.take_events() {
                if let LinkEvent::Data(d) = &e {
                    n.host_rx.extend_from_slice(d);
                }
                n.events.push(e);
            }
        }
    }

    /// Run until `done` or `limit` seconds pass. Returns the time taken.
    fn run_until(&mut self, limit: f64, mut done: impl FnMut(&Net) -> bool) -> f64 {
        let start = self.now;
        while !done(self) {
            assert!(self.now - start < limit, "timed out after {limit} s at t={:.1}", self.now);
            self.step();
        }
        self.now - start
    }

    fn has(&self, i: usize, e: &LinkEvent) -> bool {
        self.nodes[i].events.contains(e)
    }
}

fn data(n: usize, seed: u32) -> Vec<u8> {
    // Every byte value, including CR and 0x00, in pseudo-random order.
    let mut rng = Prng::new(seed);
    (0..n).map(|i| if i < 256 { i as u8 } else { rng.next_u32() as u8 }).collect()
}

fn session(burst_loss: f64, cw_loss: f64, seed: u32) {
    let mut net = Net::new(seed, burst_loss, cw_loss);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1").unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_connected() && n.nodes[1].link.is_connected());
    assert!(net.has(0, &LinkEvent::Connected { source: "LA1AAA".into(), destination: "LA2BBB-1".into() }));
    assert!(net.has(1, &LinkEvent::Connected { source: "LA1AAA".into(), destination: "LA2BBB-1".into() }));
    assert!(net.has(1, &LinkEvent::Pending));

    let (a, b) = (data(10_000, seed), data(10_000, seed + 7));
    net.nodes[1].link.send_data(&b[..300]); // banner first
    let t = net.run_until(600.0, |n| n.nodes[0].host_rx.len() >= 300);
    assert!(t < 60.0);
    net.nodes[1].link.send_data(&b[300..]);
    net.nodes[0].link.send_data(&a);
    let t = net.run_until(900.0, |n| n.nodes[0].host_rx.len() >= b.len() && n.nodes[1].host_rx.len() >= a.len());
    assert_eq!(net.nodes[0].host_rx, b);
    assert_eq!(net.nodes[1].host_rx, a);
    net.run_until(60.0, |n| n.has(0, &LinkEvent::Buffer(0)) && n.has(1, &LinkEvent::Buffer(0)));

    net.nodes[0].link.disconnect(net.now);
    net.run_until(60.0, |n| n.nodes[0].link.is_idle() && n.nodes[1].link.is_idle());
    for node in &net.nodes {
        assert_eq!(node.events.iter().filter(|e| **e == LinkEvent::Disconnected).count(), 1);
    }
    println!(
        "loss {burst_loss}/{cw_loss}: 20 kB in {t:.0} s, airtime {:.0} s, retries {}/{}, resent {}/{} bytes",
        net.airtime,
        net.nodes[0].link.stats.retries,
        net.nodes[1].link.stats.retries,
        net.nodes[0].link.stats.bytes_resent,
        net.nodes[1].link.stats.bytes_resent
    );
}

#[test]
fn clean_session() {
    session(0.0, 0.0, 1);
}

#[test]
fn lossy_session() {
    session(0.15, 0.15, 2);
    session(0.3, 0.1, 3);
}

#[test]
fn connect_to_absent_station_gives_up() {
    let mut net = Net::new(4, 0.0, 0.0);
    net.nodes[1].dead = true;
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1").unwrap();
    let t = net.run_until(60.0, |n| n.nodes[0].link.is_idle());
    assert!(t < 25.0, "took {t} s");
    assert_eq!(net.nodes[0].events, vec![LinkEvent::Disconnected]);
}

#[test]
fn not_listening_or_wrong_call_is_ignored() {
    let mut net = Net::new(5, 0.0, 0.0);
    net.nodes[1].link.set_listen(false);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1").unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_idle());
    assert!(net.nodes[1].events.is_empty());

    let mut net = Net::new(6, 0.0, 0.0);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA9ZZZ").unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_idle());
    assert!(net.nodes[1].events.is_empty());
}

#[test]
fn abort_tells_the_peer() {
    let mut net = Net::new(7, 0.0, 0.0);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1").unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_connected() && n.nodes[1].link.is_connected());
    net.nodes[0].link.abort(net.now);
    assert!(net.nodes[0].link.is_idle());
    let t = net.run_until(10.0, |n| n.nodes[1].link.is_idle());
    assert!(t < 5.0);
}

#[test]
fn dead_peer_is_detected() {
    let mut net = Net::new(8, 0.0, 0.0);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1").unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_connected() && n.nodes[1].link.is_connected());
    net.nodes[1].dead = true;
    net.nodes[0].link.send_data(b"hello?");
    let t = net.run_until(120.0, |n| n.nodes[0].link.is_idle());
    assert!(t < 60.0, "took {t} s");
    assert!(net.has(0, &LinkEvent::Disconnected));
}

#[test]
fn idle_link_is_kept_alive_then_dropped_when_peer_dies() {
    let mut net = Net::new(9, 0.0, 0.0);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1").unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_connected() && n.nodes[1].link.is_connected());
    // Quiet for two minutes: keepalives keep both ends connected.
    net.run_until(130.0, |n| n.now > 120.0);
    assert!(net.nodes[0].link.is_connected() && net.nodes[1].link.is_connected());
    // Then the caller disappears without a word; the called side notices.
    net.nodes[0].dead = true;
    net.run_until(200.0, |n| n.nodes[1].link.is_idle());
}

#[test]
fn rate_choice_accounts_for_drive_per_level() {
    for params in [OfdmParams::narrow(), OfdmParams::wide()] {
        let modem = Modem::new(params, 0.1);
        let name = modem.ofdm.p.name;
        let top = LEVELS.len() as u8 - 1;
        for l in 0..=top {
            // Just above the threshold, measured on a burst at that level.
            let at = SNR_THRESHOLDS_DB[l as usize] + SNR_MARGIN_DB + 0.05;
            assert!(level_by_snr(&modem, at, l) >= l, "{name} level {l}");
            // The same channel measured on a louder level-0 burst (the model is invertible
            // below the ceilings).
            let at0 = modem.predict_snr_db(at, l, 0);
            assert!(level_by_snr(&modem, at0, 0) >= l, "{name} level {l}: {at:.2} dB -> {at0:.2} dB at level 0");
            // A level never predicts above its own ceiling, nor above the measurement
            // plus the drive difference.
            for to in 0..=top {
                let pred = modem.predict_snr_db(at, l, to);
                assert!(pred <= modem.ofdm.p.level_snr_ceiling_db[to as usize] + 0.01);
                assert!(pred <= at + modem.level_gain_db(l, to).max(0.0) + 0.01, "{name} {l}->{to}: {pred}");
            }
        }
        // Well below the ceilings, a loud burst's SNR overstates what a softer, faster
        // level would get by about the drive difference.
        let gain = modem.level_gain_db(top, 0);
        assert!(gain > 0.0);
        let pred = modem.predict_snr_db(5.0, 0, top);
        assert!((pred - (5.0 - gain)).abs() < 0.5, "{name}: {pred} vs {}", 5.0 - gain);
        // At the ceiling the measurement says nothing about the channel.
        let ceiling0 = modem.ofdm.p.level_snr_ceiling_db[0];
        assert_eq!(level_by_snr(&modem, ceiling0 + 1.0, 0), top);
        assert_eq!(level_by_snr(&modem, -10.0, top), 0);
        assert_eq!(level_by_snr(&modem, 60.0, 0), top);
    }
}

#[test]
fn predicted_snr_matches_the_simulator() {
    // Measured with the `level_table` benchmark (wide mode): at 24 dB CNR a level-4
    // burst reports 18.0 dB and a level-5 burst 16.9 dB; at 12 dB CNR a level-0 burst
    // reports 8.3 dB and a level-5 burst 4.7 dB.
    let modem = Modem::new(OfdmParams::wide(), 0.1);
    let p = modem.predict_snr_db(18.0, 4, 5);
    assert!((p - 16.9).abs() < 0.7, "L4 -> L5 at 24 dB CNR: {p:.1}");
    let p = modem.predict_snr_db(8.3, 0, 5);
    assert!((p - 4.7).abs() < 1.0, "L0 -> L5 at 12 dB CNR: {p:.1}");
    // Narrow: at 14 dB CNR a level-4 burst reports 15.9 dB and a level-5 burst 15.3 dB.
    let modem = Modem::new(OfdmParams::narrow(), 0.1);
    let p = modem.predict_snr_db(15.9, 4, 5);
    assert!((p - 15.3).abs() < 1.0, "narrow L4 -> L5 at 14 dB CNR: {p:.1}");
}

// ------------------------------------------------------------------ Unproto

fn lone_link(seed: u32) -> Link {
    Link::new(Arc::new(Modem::new(OfdmParams::narrow(), 0.1)), LinkConfig::default(), seed)
}

/// Poll a lone link (quiet channel unless `busy` says otherwise) from `from` until it
/// transmits or `until` passes. Returns (time, burst).
fn poll_until(link: &mut Link, from: f64, until: f64, busy: impl Fn(f64) -> bool) -> Option<(f64, Burst)> {
    let mut t = from;
    while t < until {
        if let Some(b) = link.poll(t, busy(t)) {
            return Some((t, b));
        }
        t += DT;
    }
    None
}

fn rx_of(b: &Burst) -> RxBurst {
    let payload = b.payload.iter().cloned().map(Some).collect();
    RxBurst { header: b.header, payload, snr_db: 20.0, end_sample: 0, level: Default::default() }
}

fn unproto_events(l: &mut Link) -> Vec<Vec<u8>> {
    l.take_events().into_iter().filter_map(|e| if let LinkEvent::Unproto(f) = e { Some(f) } else { None }).collect()
}

#[test]
fn unproto_pack_parse_roundtrip_and_failed_codewords() {
    let frames: Vec<Vec<u8>> = (0..5).map(|i| data(40 + 30 * i, i as u32)).collect();
    let cws = pack_unproto(0, &frames);
    assert!(cws.iter().all(|c| c.len() == Modem::payload_bytes(0)));
    assert_eq!(cws.len(), 4, "1 + 5 x 2 + 500 = 511 bytes in 146-byte codewords");
    let ok: Vec<_> = cws.iter().cloned().map(Some).collect();
    assert_eq!(parse_unproto(&ok), frames);
    // A failed codeword loses the frames it touches and everything after it.
    let mut bad = ok.clone();
    bad[1] = None;
    let got = parse_unproto(&bad);
    let in_first: usize = frames
        .iter()
        .scan(1, |pos, f| {
            *pos += 2 + f.len();
            Some(*pos)
        })
        .take_while(|&e| e <= 146)
        .count();
    assert_eq!(got, frames[..in_first]);
    bad[0] = None;
    assert!(parse_unproto(&bad).is_empty());
    // Unknown format version: nothing.
    let mut other = ok;
    other[0].as_mut().unwrap()[0] = 2;
    assert!(parse_unproto(&other).is_empty());
}

#[test]
fn unproto_capacity_per_mode() {
    for (mode, l0, l1) in [("narrow", 291, 729), ("wide", 875, 1897)] {
        let m = Modem::new(OfdmParams::by_name(mode).unwrap(), 0.1);
        assert_eq!((unproto_capacity(&m, 0), unproto_capacity(&m, 1)), (l0, l1), "{mode}");
        assert_eq!(unproto_max_frame(&m), l1 - 2);
    }
}

#[test]
fn unproto_frames_within_the_batch_window_share_a_burst_in_order() {
    let mut l = lone_link(1);
    let f: Vec<Vec<u8>> = (0..4).map(|i| vec![0x00, i as u8, 0xc0, 0xdb]).collect();
    assert!(l.send_unproto(5.0, &f[0]));
    assert!(l.send_unproto(5.2, &f[1])); // 0.2 s later: same batch
    assert!(l.send_unproto(5.45, &f[2])); // 0.25 s after the last: still the same batch
    let (t, b) = poll_until(&mut l, 0.0, 10.0, |_| false).expect("sent");
    assert!(t >= 5.45 + UNPROTO_BATCH_S - 1e-9, "sent at {t} before the batch window closed");
    assert_eq!((b.header.kind, b.header.level, b.header.ncw), (Kind::Unproto, 0, 1));
    assert_eq!(parse_unproto(&rx_of(&b).payload), f[..3]);
    l.tx_done(t + 1.0);
    // A frame after the burst goes in a burst of its own, after a fresh clear wait.
    l.send_unproto(t + 1.0, &f[3]);
    let (t2, b2) = poll_until(&mut l, t + 1.0, t + 10.0, |_| false).expect("sent");
    assert!(t2 >= t + 1.0 + UNPROTO_CLEAR_S - 1e-9);
    assert_eq!(parse_unproto(&rx_of(&b2).payload), f[3..]);
    assert_eq!(l.stats.unproto_sent, 4);
}

#[test]
fn unproto_waits_for_a_clear_channel() {
    for seed in 0..20 {
        let mut l = lone_link(seed);
        l.send_unproto(0.0, &[0x00, 1, 2, 3]);
        // Busy until 3 s: nothing before 3.5 s, and it goes by 4.0 s.
        let (t, _) = poll_until(&mut l, 0.0, 10.0, |t| t < 3.0).expect("sent");
        assert!(
            (3.0 + UNPROTO_CLEAR_S..=3.0 + UNPROTO_CLEAR_S + UNPROTO_CLEAR_RANDOM_S + 2.0 * DT).contains(&t),
            "seed {seed}: sent at {t}"
        );
    }
}

#[test]
fn unproto_long_frame_uses_level_1_and_too_long_is_dropped() {
    let mut l = lone_link(2);
    let m = Modem::new(OfdmParams::narrow(), 0.1);
    let big = vec![0x00; unproto_capacity(&m, 0)]; // does not fit level 0 with its length
    assert!(l.send_unproto(0.0, &big));
    assert!(l.send_unproto(0.0, &[0x00, 9]));
    let (t, b) = poll_until(&mut l, 0.0, 10.0, |_| false).unwrap();
    assert_eq!(b.header.level, 1);
    assert_eq!(parse_unproto(&rx_of(&b).payload), vec![big, vec![0x00, 9]]);
    l.tx_done(t + 4.0);
    assert!(!l.send_unproto(t, &vec![0x00; unproto_max_frame(&m) + 1]));
    assert!(!l.send_unproto(t, &[]));
    assert_eq!(l.stats.unproto_dropped, 2);
}

#[test]
fn unproto_is_held_during_a_session_and_received_in_any_state() {
    let mut net = Net::new(40, 0.0, 0.0);
    net.nodes[0].link.connect(0.0, "LA1AAA", "LA2BBB-1").unwrap();
    net.run_until(60.0, |n| n.nodes[0].link.is_connected() && n.nodes[1].link.is_connected());
    net.nodes[0].link.send_unproto(net.now, &[0x00, 0xaa]);
    net.nodes[0].link.send_data(&data(2000, 3));
    net.run_until(300.0, |n| n.nodes[1].host_rx.len() >= 2000);
    net.run_until(60.0, |n| n.has(0, &LinkEvent::Buffer(0)));
    for _ in 0..1000 {
        net.step(); // 10 s with the session up and quiet
    }
    assert_eq!(net.nodes[0].link.unproto_queued(), 1, "sent during the session");
    assert_eq!(net.nodes[0].link.stats.unproto_sent, 0);

    // A burst from a third station reaches B mid-session and leaves the session alone.
    let mut c = lone_link(77);
    c.send_unproto(0.0, &[0x01, 1, 2, 3]);
    let (_, burst) = poll_until(&mut c, 0.0, 10.0, |_| false).unwrap();
    let before = (net.nodes[1].link.stats.bursts_sent, net.nodes[1].link.is_connected());
    net.nodes[1].link.on_burst(net.now, &rx_of(&burst));
    assert_eq!(unproto_events(&mut net.nodes[1].link), vec![vec![0x01, 1, 2, 3]]);
    for _ in 0..100 {
        net.step();
    }
    assert_eq!((net.nodes[1].link.stats.bursts_sent, net.nodes[1].link.is_connected()), before, "B answered it");

    net.nodes[0].link.disconnect(net.now);
    net.run_until(60.0, |n| n.nodes[0].link.is_idle() && n.nodes[1].link.is_idle());
    net.run_until(10.0, |n| n.nodes[0].link.stats.unproto_sent == 1);
    // And B, idle now, received it.
    net.run_until(10.0, |n| n.nodes[1].events.contains(&LinkEvent::Unproto(vec![0x00, 0xaa])));
}
