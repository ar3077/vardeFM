//! A station's link and its digipeater, sharing one radio.
//!
//! The radio's side of a [`Link`] (bursts heard, bursts to send) can only be reached
//! through a [`Node`], so a station cannot drive its link past the digipeater.

use crate::{Digipeater, Link};
use vardefm_modem::{Burst, RxBurst};

pub struct Node {
    /// The host's side: commands, data and events.
    pub link: Link,
    digi: Option<Digipeater>,
    /// The burst on the air is a repeat, not the link's own.
    repeating: bool,
}

impl Node {
    pub fn new(link: Link) -> Self {
        Self { link, digi: None, repeating: false }
    }

    /// Repeat the bursts of sessions routed through any of `calls` (none: off).
    /// Works alongside the link's own sessions.
    pub fn set_digi(&mut self, calls: &[String]) {
        let turnaround_s = self.link.config().turnaround_s;
        self.digi = (!calls.is_empty()).then(|| Digipeater::new(calls, turnaround_s));
    }

    /// Bursts repeated so far.
    pub fn repeats(&self) -> u64 {
        self.digi.as_ref().map_or(0, |d| d.repeated())
    }

    /// Handle a decoded burst.
    pub fn on_burst(&mut self, now: f64, rx: &RxBurst) {
        self.link.on_burst(now, rx);
        if let Some(d) = &mut self.digi {
            d.on_burst(now, rx);
        }
    }

    /// Called regularly while not transmitting. Returns a burst to transmit now, and
    /// whether it is a repeat.
    pub fn poll(&mut self, now: f64, busy: bool) -> Option<(Burst, bool)> {
        if let Some(d) = &mut self.digi {
            // Repeats answer a burst just heard, so they go first.
            if let Some(b) = d.poll(now) {
                self.repeating = true;
                return Some((b, true));
            }
            // A repeat is waiting out its turnaround. A burst of our own started now
            // would hold it back until that burst ends, past what the end stations
            // wait for.
            if d.pending() {
                return None;
            }
        }
        Some((self.link.poll(now, busy)?, false))
    }

    /// Called when the burst returned by `poll` has finished transmitting.
    pub fn tx_done(&mut self, now: f64) {
        // A repeat is not the link's burst: its timers must not restart. That includes
        // the held-back timer, as the link's own burst is still waiting for the channel.
        // But the channel was in use, so an Unproto burst waits for it to be clear again:
        // the next station's answer comes 0.2 s after the repeat.
        if std::mem::take(&mut self.repeating) {
            self.link.channel_used(now);
        } else {
            self.link.tx_done(now);
        }
    }
}

#[cfg(test)]
impl Node {
    /// Replace the digipeater, e.g. with a slow one.
    pub(crate) fn set_digipeater(&mut self, d: Digipeater) {
        self.digi = Some(d);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{calls_payload, pack_unproto, LinkConfig, LinkEvent, UNPROTO_CLEAR_S};
    use std::sync::Arc;
    use vardefm_modem::{Header, Kind, Modem, OfdmParams, FLAG_UP};

    const DIGI: &str = "LA3DDD";

    /// A digipeater for `DIGI` whose own link is calling LA5X directly.
    fn calling_digi(seed: u32) -> Node {
        let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
        let mut node = Node::new(Link::new(modem, LinkConfig::default(), seed));
        node.set_digi(&[DIGI.to_string()]);
        node.link.connect(0.0, "LA3DDD", "LA5X", &[]).unwrap();
        node
    }

    /// Another station's routed request through `DIGI`, as its caller sends it.
    fn routed_request() -> RxBurst {
        let mut header = Header { kind: Kind::ConnectReq, session: 7, ncw: 1, ..Header::default() };
        header.set_hop(0);
        let payload = vec![Some(calls_payload("LA1AAA", "LA2BBB", &[DIGI.to_string()]))];
        RxBurst { header, payload, snr_db: 30.0, end_sample: 0, level: Default::default() }
    }

    #[test]
    fn repeat_goes_before_our_own_burst() {
        let mut node = calling_digi(1);
        node.on_burst(0.0, &routed_request());
        // The repeat is due 0.2 s after the burst. Our own request is ready at once,
        // but sending it now would hold the repeat back for the whole burst.
        assert!(node.poll(0.1, false).is_none(), "own burst sent while a repeat was waiting");
        let (burst, repeat) = node.poll(0.2, false).expect("the repeat is due");
        assert!(repeat);
        assert_eq!((burst.header.session, burst.header.hop()), (7, 1));
        node.tx_done(2.0);
        let (burst, repeat) = node.poll(2.0, false).expect("then our own request");
        assert!(!repeat);
        assert_eq!(burst.header.kind, Kind::ConnectReq);
        assert_eq!(node.repeats(), 1);
    }

    /// Steps `node` from `from` until its link sends a burst; returns the time.
    fn next_own_burst(node: &mut Node, from: f64) -> f64 {
        let mut t = from;
        loop {
            assert!(t < from + 60.0, "no burst");
            if let Some((_, repeat)) = node.poll(t, false) {
                assert!(!repeat);
                return t;
            }
            t += 0.01;
        }
    }

    #[test]
    fn repeat_does_not_restart_the_links_timers() {
        // Two stations with the same seed send their first request together. One then
        // repeats a burst while it waits for the answer.
        let (mut quiet, mut busy) = (calling_digi(2), calling_digi(2));
        for node in [&mut quiet, &mut busy] {
            assert!(node.poll(0.0, false).is_some_and(|(_, repeat)| !repeat));
            node.tx_done(1.9);
        }
        busy.on_burst(2.0, &routed_request());
        assert!(busy.poll(2.2, false).is_some_and(|(_, repeat)| repeat));
        busy.tx_done(2.9);
        // Its own retry still goes out when it would have without the repeat.
        let (a, b) = (next_own_burst(&mut quiet, 2.9), next_own_burst(&mut busy, 2.9));
        assert!(a > 2.9 + 0.1, "retry at {a}: not waiting for an answer");
        assert_eq!(a, b);
    }

    /// A digipeater for `DIGI` whose own link is idle.
    fn idle_digi(seed: u32) -> Node {
        let modem = Arc::new(Modem::new(OfdmParams::narrow(), 0.1));
        let mut node = Node::new(Link::new(modem, LinkConfig::default(), seed));
        node.set_digi(&[DIGI.to_string()]);
        node
    }

    #[test]
    fn unproto_waits_for_a_repeat_and_a_clear_channel_after_it() {
        for seed in 0..10 {
            let mut node = idle_digi(seed);
            assert!(node.link.send_unproto(0.0, &[0x00, 1, 2, 3]));
            // Busy until 4.5 s, so the broadcast is due from 5.0 s.
            let mut t = 0.0;
            while t < 4.9 {
                assert!(node.poll(t, t < 4.5).is_none(), "seed {seed}: sent at {t}");
                t += 0.01;
            }
            // A routed request arrives; its repeat is due 0.2 s later. Our broadcast,
            // due meanwhile, must not hold it back.
            node.on_burst(4.9, &routed_request());
            while t < 5.09 {
                assert!(node.poll(t, false).is_none(), "seed {seed}: own burst at {t} before the repeat");
                t += 0.01;
            }
            let (burst, repeat) = node.poll(5.11, false).expect("the repeat is due");
            assert!(repeat);
            assert_eq!(burst.header.kind, Kind::ConnectReq);
            node.tx_done(7.0);
            // The next station answers 0.2 s after the repeat ends: the broadcast waits
            // for a clear channel again, counted from the end of the repeat.
            let mut t = 7.0;
            let sent = loop {
                assert!(t < 12.0, "seed {seed}: broadcast never sent");
                if let Some((b, repeat)) = node.poll(t, false) {
                    assert!(!repeat);
                    assert_eq!(b.header.kind, Kind::Unproto);
                    break t;
                }
                t += 0.01;
            };
            assert!(sent >= 7.0 + UNPROTO_CLEAR_S - 1e-9, "seed {seed}: broadcast {sent:.2} s, right after the repeat");
        }
    }

    #[test]
    fn unproto_with_route_bits_is_not_repeated() {
        let mut node = idle_digi(4);
        node.on_burst(0.0, &routed_request());
        assert!(node.poll(0.3, false).is_some_and(|(_, repeat)| repeat));
        node.tx_done(2.0);
        node.link.take_events();
        // A kind-5 burst that, against the rules, carries that session's id, UP and a
        // route field: as the called station's answer it would be ours to repeat.
        let frame = vec![0x00, 7, 8, 9];
        let payload: Vec<_> = pack_unproto(0, std::slice::from_ref(&frame)).into_iter().map(Some).collect();
        let mut header =
            Header { kind: Kind::Unproto, session: 7, ncw: payload.len() as u8, flags: FLAG_UP, ..Header::default() };
        header.set_hop(0);
        node.on_burst(3.0, &RxBurst { header, payload, snr_db: 30.0, end_sample: 0, level: Default::default() });
        for t in [3.2, 3.5, 5.0] {
            assert!(node.poll(t, false).is_none(), "repeated at {t}");
        }
        assert_eq!(node.repeats(), 1);
        // The link still takes it as a broadcast.
        assert!(node.link.take_events().contains(&LinkEvent::Unproto(frame)));
    }

    #[test]
    fn repeat_does_not_restart_the_links_held_back_timer() {
        let mut node = calling_digi(3);
        assert!(node.poll(0.0, false).is_some_and(|(_, repeat)| !repeat));
        node.tx_done(1.9);
        assert!(node.poll(2.0, true).is_none());
        assert_eq!(node.link.held_since, Some(2.0));
        node.on_burst(2.0, &routed_request());
        // The repeat is due 0.2 s after the burst, busy channel or not. It gets us on
        // the air, but our own request is still held back.
        assert!(node.poll(2.2, true).is_some_and(|(_, repeat)| repeat));
        node.tx_done(2.9);
        assert_eq!(node.link.held_since, Some(2.0));
    }
}
