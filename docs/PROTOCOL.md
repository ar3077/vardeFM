# VardeFM Protocol Specification

**Status: draft v0.** Everything here may change until v1.0.

Contents: 1. Physical layer · 2. Bursts and error correction · 3. Link layer ·
4. Host interface (VARA-compatible TCP, KISS).

## 1. Physical layer

### 1.1 Common
- Real-valued audio, 48 000 samples/s.
- OFDM with a cyclic prefix. Carrier *k* (0-based) sits at FFT bin `first_bin + k`.
  The time signal is the real part of a Hermitian-symmetric IFFT, scaled by 1/√N.
- The TX applies a spectral tilt: carrier gain = `10^(tilt · log2(f_k / f_0) / 20)`.
- Each burst is scaled to the TX RMS of its speed level (table below; header-only bursts
  use level 0's), measured over the whole burst including the leader. Robust levels go
  out louder: they gain more from the extra deviation than they lose to clipping.
- Peaks are then limited to ±1.0 (full scale = peak deviation) by clip and filter:
  1. A 4th-order Butterworth low-pass at 1.15 × the top carrier frequency removes the
     sidelobes of the symbol edges.
  2. Twice: what clipping at ±1.0 would remove is low-passed forwards and backwards
     (zero phase) with the same filter and subtracted.
  3. A final clamp at ±1.0.

  Power above 1.3 × the top carrier stays more than 30 dB below the in-band power even
  at RMS 0.55 (plain clipping: about 21 dB).

| Parameter | narrow | wide |
|---|---|---|
| FFT size N | 1024 | 512 |
| Cyclic prefix | 128 (2.67 ms) | 64 (1.33 ms) |
| Carrier spacing | 46.875 Hz | 93.75 Hz |
| first_bin / carriers | 9 / 49 | 3 / 62 |
| Band | 421.9–2671.9 Hz | 281.3–6000 Hz |
| Pilot spacing | 6 | 6 |
| Pilots / data carriers | 9 / 40 | 12 / 50 |
| Symbol rate | 41.67 /s | 83.33 /s |
| TX tilt | 0 dB/oct | +3 dB/oct |
| TX RMS, levels 0–2 / 3–4 / 5–6 | 0.30 / 0.30 / 0.25 | 0.50 / 0.40 / 0.30 |
| TX low-pass | 3073 Hz | 6900 Hz |

### 1.2 Pseudo-random sequences
All known sequences come from xorshift32 (`x ^= x<<13; x ^= x>>17; x ^= x<<5`). Each
output bit is the most significant bit of the new state; bit 0 maps to +1, bit 1 to −1.

| Use | Seed |
|---|---|
| Pilot table (32 symbols × pilots, row by row) | 0x5EED0001 |
| Preamble A | 0x5EED00A1 |
| Preamble B | 0x5EED00B2 |
| Leader (I then Q per carrier) | 0x5EED00C3 |

### 1.3 Frame
```
[leader × n][preamble A][preamble B][data 0] … [data nsym-1]
```
- **Leader:** `ceil(txdelay / symbol duration)` symbols of pseudo-random QPSK on every
  carrier. It covers the radio's key-up time and is not used by the receiver.
- **Preamble A:** BPSK × √2 on carriers whose FFT bin is even, zero elsewhere. One value
  is drawn per carrier, odd-bin carriers included, so the sequence stays aligned. The time
  signal repeats every N/2 samples, for Schmidl-Cox detection.
- **Preamble B:** BPSK on every carrier. Used for exact timing and the channel estimate.
- **Data symbols:** pilots on carriers where `k mod pilot_spacing == 0` and on the last
  carrier. The pilot value is `table[sym mod 32][pilot index]`. Data fills the remaining
  carriers in ascending order.

### 1.4 Constellations
Gray-coded, unit average power. BPSK uses the I axis only. For square QAM, the first half
of each symbol's bits selects the I level and the second half the Q level, MSB first. The
Gray word *g* maps to level index *i* = gray⁻¹(*g*), amplitude `(2i − (L−1)) · scale`.
So BPSK maps bit 0 to −1 and bit 1 to +1.

### 1.5 Receiver (informative)
The reference receiver:
1. High-passes the audio (8th order, 420 Hz narrow, 340 Hz wide) to keep CTCSS tones out,
   low-passes it too in wide mode (2500 Hz), and runs a Schmidl-Cox autocorrelation.
2. Confirms the frame by cross-correlating the high-passed audio with preamble B, filtered
   the same way.
3. Places the FFT window mid cyclic prefix, which absorbs clock drift of ±ncp/2 samples
   over the frame.
4. Estimates the channel from preamble B, smoothed across frequency, and decodes the
   header with it. Once the header has decoded, it demodulates the frame again with the
   header symbols (now known) averaged into the channel estimate, each weighted by the
   inverse of its noise: the header decodes through a click on one of its symbols, and
   that symbol must not corrupt the estimate.
5. Tracks it with pilots averaged over ±4 symbols and interpolated in magnitude and phase.
6. Estimates the noise per carrier and, from the pilots, a noise scale per symbol, so
   that a symbol hit by an FM click or a squelch pop gets low-confidence LLRs.

Because the window never moves, a frame should stay shorter than
`ncp / 2 / (clock error)`. At 100 ppm that is 13 s (narrow) or 6.7 s (wide).

## 2. Bursts and error correction

### 2.1 Burst layout
One transmission is one burst, sent as one OFDM frame (section 1.3):
```
[leader][preamble A][preamble B][header: 400 coded bits, BPSK][payload: ncw × 2400 coded bits]
```
The header fills 10 symbols in narrow mode (40 data carriers) and 8 in wide mode (50).
The payload uses the header's speed level for every codeword. A burst carries at most
24 codewords and at most 4.0 s of payload, so clock drift stays inside the cyclic prefix
(section 1.5).

### 2.2 Speed levels
Every payload codeword is 2400 bits. *k* is the LDPC information length; each codeword
carries `k/8 − 4` data bytes followed by a big-endian CRC-32.

| Level | Modulation | Code rate | k | Bytes/codeword | Net rate narrow | Net rate wide |
|---|---|---|---|---|---|---|
| 0 | BPSK | 1/2 | 1200 | 146 | 0.7 kbit/s | 1.9 kbit/s |
| 1 | QPSK | 1/2 | 1200 | 146 | 1.5 kbit/s | 3.8 kbit/s |
| 2 | QPSK | 3/4 | 1800 | 221 | 2.2 kbit/s | 5.8 kbit/s |
| 3 | 16QAM | 1/2 | 1200 | 146 | 2.9 kbit/s | 7.6 kbit/s |
| 4 | 16QAM | 3/4 | 1800 | 221 | 4.5 kbit/s | 11.5 kbit/s |
| 5 | 64QAM | 2/3 | 1600 | 196 | 5.9 kbit/s | 14.9 kbit/s |
| 6 | 64QAM | 5/6 | 2000 | 246 | 7.4 kbit/s | 18.7 kbit/s |

Net rate = data bytes of a full burst ÷ its airtime (leader, preambles and header
included), before link-layer overhead. For each mode and level, the carrier-to-noise
ratio in the FM channel simulator at which 10 % of codewords fail, decoded from the
known burst position with the header taken as known, is recorded in
`crates/vardefm-modem/perf/levels.toml` and checked by the `level_gate` test. That file
is the reference; roughly, it runs from 3 dB (level 0) to 16 dB (level 6) in narrow
mode and from 5 dB to 24 dB in wide mode. Error-free decoding begins about half a dB
higher at the low levels and about 1 dB higher at levels 5 and 6. These are payload
figures. The header, decoded from preamble B alone, is recorded separately in the same
file, once per drive level, since a burst goes out at its level's TX RMS (§1.1), header
included. The header's fields vary from burst to burst, and with them how much of it is
clipped, so its points are averaged over random field values. That one point per drive
covers every level and burst length at that drive is an estimate, not a measured header
error rate: on the noiseless transmitted header, the useful level less the clipping
distortion, counted as noise at a 3 dB reference SNR, agrees within 0.1 dB
(`header_level_depends_only_on_drive`). At level 0's drive the header reaches 10 %
header errors at about 3.7 dB (narrow) and 5.9 dB (wide), above level 0's point, and its
error rate falls much more slowly than the payload's, so at the low levels the header
limits a burst. At the lower drives of the high levels it needs up to about 0.5 dB
(narrow) and 1.4 dB (wide) more, still far below those levels' own points. Detection is
not part of either. Below about 8 dB (narrow) and 11 dB (wide) some level 1 bursts are
not detected at all.

### 2.3 Header (96 bits before coding)
80 bits of fields, MSB first, then a CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over
the 10 field bytes.

| Bits | Field |
|---|---|
| 3 | kind: 0 ConnectReq, 1 ConnectAck, 2 Data, 3 Disc, 4 DiscAck (sessions, §3.1–3.4), 5 Unproto (§3.5); 6 and 7 unassigned |
| 8 | session id (random, chosen by the caller) |
| 3 | payload speed level |
| 5 | ncw: payload codewords (0–24) |
| 3 | req_level: level the sender asks the peer to use next |
| 3 | burst_id: sender's burst counter, mod 8 |
| 3 | ack_burst: burst_id of the peer burst being acknowledged |
| 24 | ack_offset: next byte offset expected from the peer, mod 2²⁴ |
| 24 | bitmap: bit *i* (LSB = codeword 0) set if codeword *i* of burst `ack_burst` decoded |
| 4 | flags, see below |

| Flag bits | Name | Meaning |
|---|---|---|
| 0 | POLL | The peer must answer, even with nothing to send (§3.2). |
| 1 | UP | Sent by the called station. Routed sessions only; 0 on a direct session (§3.4). |
| 2–3 | route field | 0 on a direct session, as every version sends it; on a routed session 1 + the digipeater repeats so far, so 1–3 (§3.4). |

Each value is used once: the kinds 0–4 are sessions (direct or routed, told apart by the
flags), kind 5 is Unproto, and the flags mean something only on kinds 0–4. An Unproto
burst is sent with flags 0 and a receiver ignores its flags (§3.5).

A receiver drops a header whose kind it does not know exactly as it drops one that failed
its CRC: no burst is reported and the payload is not decoded. Every version back to 0.1.0
does this, stock 0.3.2 included, which is why kind 5 could be added without a minor version
bump. A burst like
that counts as channel activity only through the level detector, which takes a steady
sound as background within about 2 s. So a station older than kind 5 reads an Unproto
burst as busy for about its first 1.7 s only (measured with v0.3.1 and with master
before kind 5). A narrow level-0 burst of one codeword (about 1.9 s on the air) is
covered almost entirely. In a longer one, an older station that has something to send
may start transmitting in the second half.

### 2.4 LDPC codes
Systematic IRA codes: `H = [Hu | Hp]`, where `Hp` is a staircase (check *i* holds parity
bits *i* and *i−1*). Encoding is a running XOR: `p_i = p_{i−1} ⊕ (Hu row i · u)`. The
codeword is the information bits followed by the parity bits. `Hu` is built by
progressive edge growth (PEG) from `(n, k, seed)` (`Ldpc::new` in
crates/vardefm-fec/src/ldpc.rs). The construction is normative: both ends must build
identical graphs.
1. **Degrees.** `m = n − k`. Of the *k* information bits, the last
   `h = min(⌊(4k + 5m)/20⌋, ⌊k/2⌋)` have degree 8 and the first `k − h` have degree
   `max(3, ⌈3m/2k⌉)`. Both degrees are capped at *m*.
2. **Starting graph.** The Tanner graph starts with the staircase edges only: parity
   bit *p* is in checks *p* and *p+1*, and the last parity bit (*m−1*) only in check
   *m−1*. A check's degree counts all its edges, parity edges included, so check 0
   starts at degree 1 and every other check at degree 2.
3. **Edge growth.** Let `rng` be xorshift32 (section 1.2) seeded with `seed`. For
   *j* = 0, 1, …, *k−1* in turn, add the *d_j* edges of information bit *j* one at a time.
   For each edge:
   - Compute the distance (number of edges) from bit *j* to every check in the
     current graph. The current graph holds the staircase, every edge added so far
     and the earlier edges of bit *j* itself.
   - If some checks cannot be reached from bit *j*, the candidates are those checks.
     This always applies to a bit's first edge, when all *m* checks are candidates.
     Otherwise the candidates are the checks at the greatest distance.
   - Keep the candidates with the lowest current degree, in ascending check order.
     Connect bit *j* to the one at index `rng.next() mod (number kept)`. Draw exactly
     one `next()` per edge, also when only one candidate is kept, and add 1 to the
     chosen check's degree.

| Code | n | k | Degree-8 bits | Other bits | Seed |
|---|---|---|---|---|---|
| Header | 400 | 96 | 48 | 48 × degree 5 | 0x1D9C1000 |
| Payload | 2400 | 1200 | 540 | 660 × degree 3 | 0x1D9C04B0 |
| Payload | 2400 | 1600 | 520 | 1080 × degree 3 | 0x1D9C0640 |
| Payload | 2400 | 1800 | 510 | 1290 × degree 3 | 0x1D9C0708 |
| Payload | 2400 | 2000 | 500 | 1500 × degree 3 | 0x1D9C07D0 |

Payload seeds are 0x1D9C0000 + *k*. A hash of each of these graphs is pinned by the
`construction_is_stable` test in ldpc.rs.

Informative: none of the graphs has a 4-cycle. The header and rate-1/2 graphs have
girth 8; the denser high-rate graphs have girth 6. Check degrees differ by at most 2
within a code. The reference decoder is layered offset min-sum: every
check-to-variable magnitude is `max(0, min − 0.5)`, with no scaling, for at most 50
iterations, and input LLRs are clamped to ±40. The Eb/N0 at which each code reaches 1 %
frame error rate with it on BPSK/AWGN is recorded in `crates/vardefm-fec/perf/ldpc.toml`
and checked by the `fer_gate` test. That file is the reference; roughly, the points are
1.5 dB (rate 1/2), 2.1 dB (2/3), 2.6 dB (3/4), 3.3 dB (5/6) and 2.2 dB (header).

### 2.5 Scrambling and interleaving
- Information bits are XORed with xorshift32 bits before encoding. The seed is
  0x5C4AFFFF for the header and 0x5C4A0000 + codeword index for the payload.
- Coded bits are permuted by Fisher-Yates over the whole header (seed 0x17E40002) or the
  whole payload of the burst (all codewords together, seed 0x17E40001). Coded bit *j*
  goes to slot `perm[j]`. Slots fill OFDM symbols in order, carrier by carrier, with
  each carrier's constellation bits together.

## 3. Link layer

### 3.1 Sessions
- **Connect:** the caller sends `ConnectReq` (level 0, one codeword) with payload
  `"VFM1" | len | source call | len | destination call`, zero padded, and a random
  non-zero session id. The caller avoids ids it has heard in any burst in the last 10
  minutes, or three keepalive intervals if that is longer (any id if it has heard all of
  them), so it does not share an id with a session it can hear. This is local policy:
  peers need nothing for it. A session it cannot hear may still share its id. Through digipeaters the payload is
  `"VFM2" | len | source | len | destination | count | (len | digipeater) × count`
  (section 3.4). A station that has `LISTEN ON` and the destination among its
  `MYCALL`s answers `ConnectAck` with the same payload and session id. The caller retries
  5 times, then gives up and reports DISCONNECTED (about 17 s in narrow mode, 11 s in
  wide).
- **Answer:** the caller connects only on a `ConnectAck` with its session id whose payload
  names its request: source, destination and digipeaters. Other bursts carry no call
  signs, and one with its session id may belong to another session that uses the same id
  despite the avoidance above: one the caller had not heard yet when it picked the id.
  A called station that has heard no burst of the session from the caller since its
  `ConnectAck` resends the `ConnectAck` instead of Data when a burst goes unanswered, for
  as long as the caller could still be retrying its `ConnectReq` (`connect_attempts` ×
  (connect burst + wait + relay time through the digipeaters + 0.5 s jitter + longest
  burst + 0.6 s)): the last two cover a burst of its own that holds the caller's retry
  back while the channel is busy, rechecked every 0.3 s. It also answers a repeated
  `ConnectReq` with its `ConnectAck`, but only one that names its caller, itself and its
  path: another is a different caller's that happens to use the same session id.
- **Mixed versions:** a 0.3.x station as the caller connects on the called station's
  first Data burst too. As the called station it does not resend its `ConnectAck`
  unprompted, so a caller of this version that missed it recovers only if a later
  `ConnectReq` gets through, and gives up more often on a lossy channel.
- **Turns:** the called station may send first right after `ConnectAck`; the caller
  waits one response period for it.
- Bursts with a different session id are ignored.

### 3.2 Data transfer
- Each direction is a byte stream numbered by offset. A payload codeword carries one
  segment: `offset mod 2²⁴ (3 bytes) | length (1 byte) | data | zero padding`.
- **Acknowledgements:** every burst carries `ack_offset` and the bitmap for the peer's
  last burst. The sender drops acknowledged bytes and resends the rest, re-split to fit
  the current level. Data is delivered to the host in order, exactly once. A receiver
  must keep every byte of a codeword whose bitmap bit it sets: a resend may start at the
  offset of a segment it already holds ahead of a gap, and carry more bytes.
- **Turn-taking:** a station that receives a burst with data, or with POLL, answers after
  0.2 s. The answer holds acks plus any data of its own. If neither side has data, the
  link goes quiet. Either station may then start a burst once the channel has been clear
  for a short random time.
- **Rate adaptation:** `req_level` is the highest level whose SNR threshold plus 1 dB is
  at or below the predicted SNR at that level. The SNR is measured on a burst sent at the
  burst's own level. Another level's SNR is predicted from the drive difference and each
  level's SNR ceiling (the clipping floor of its drive, measured in the simulator):
  `1/snr = 1/(channel · drive²) + 1/ceiling`. Thresholds (estimated SNR on a burst at that
  level, levels 0–6): −2, 1.5, 4.5, 7, 11, 15, 18 dB (level 0's is never used). After a
  failed codeword the request drops at least one level below the burst's level. Each
  unanswered burst lowers our own level by one. The called station starts at the level
  it measured on the connect request.
- **Retries:** an unanswered burst is resent after 1.2 s (plus a random back-off of
  0–0.4 s, and longer while the channel is busy). A retry not yet sent gives way when the
  channel turns busy: that is most likely the peer's own retry, which is answered instead.
  After 8 consecutive failures the link is lost.
- **Keepalive:** a quiet link is probed with a POLL burst after 30 s (caller) or 45 s
  (called station).

### 3.3 Disconnect
- **Graceful:** once all of its data is acknowledged, the closing station sends `Disc`;
  the peer answers `DiscAck`. Both report DISCONNECTED. `Disc` is retried up to 4 times;
  a station that already closed still answers a repeated `Disc`.
- **ABORT:** the station sends one `Disc` without waiting and closes at once.

### 3.4 Digipeating
A session may run through up to two digipeaters, named in order from the caller
(`CONNECT … VIA`). A digipeater repeats whole bursts; the ARQ, acknowledgements and rate
adaptation stay end to end between the two stations. This is VardeFM's own scheme and is
not on-air compatible with VARA FM digipeaters.

- **Path:** [caller, D1, …, Dn, called station], n = 1–2. The ConnectReq and ConnectAck
  carry it in their `VFM2` payload. A station that predates digipeating ignores `VFM2`, so
  it never answers a routed request it hears directly.
- **Flags:** every burst of a routed session, from either end and of every kind, carries
  UP (0 from the caller, 1 from the called station) and the route field: 1 as an end
  station sends it, plus one for each digipeater repeat, so 1–3. Its *hop* below is the
  route field minus one, the number of repeats so far. Direct sessions leave UP and the
  route field zero, as every version sends them, and ignore a burst that has any of them
  set: it is a routed session's burst heard directly, or a digipeater's copy. The field's
  two bits are what limit a path to two digipeaters.
- **Repeating:** a digipeater never repeats a burst with route field 0, and such a burst
  does not change what it knows: a direct session cannot be mistaken for a routed one.
  It learns a session from a routed ConnectReq or ConnectAck that names one of its call
  signs, and keeps it by session id until 10 minutes pass without a burst it repeats.
  Once it has repeated a DiscAck the session is over: it repeats only a resent Disc or
  DiscAck, forgets the session 60 s after the last of them, and takes a connect burst
  under that id as a new session. Until it has repeated the called station's answer
  (a ConnectAck or a burst with UP = 1) it keeps the session only 90 s, renewed by each
  request, and repeats nothing else from the caller's side, so an unanswered request does
  not linger or repeat the bursts of another routed session that reuses the id and whose
  connect bursts it did not decode. A routed ConnectReq or ConnectAck that does not name
  it (through other digipeaters) means the session id is in use by another session: it
  forgets any session it had under that id. The digipeater at position *i* (1 = next to
  the caller) repeats a burst with UP = 0 and hop *i* − 1, or with UP = 1 and hop
  *n* − *i*.
  It sends the burst again 0.2 s after it ends, at the same level, with the route field
  plus one and every other field unchanged, whether or not the channel is busy. A
  digipeater with a session of its own sends repeats first, and starts no burst of its
  own while a repeat is waiting for its 0.2 s. A path
  may name one station more than once (the same call twice, or two calls of one
  station). It never hears its own repeat, so when it holds the next positions in a row
  it passes them all in one repeat, adding one to the route field for each.
- **Failed codewords:** a codeword the digipeater could not decode goes out as a *void*
  segment: segment length byte 0xFF (a real segment holds at most 242 bytes), the rest
  zero. The receiving station treats a void like a codeword that failed: its bitmap bit
  stays clear, it counts as a failure for rate adaptation, and the sender resends the
  data. A ConnectReq or ConnectAck whose codeword failed is not repeated.
- **Accepting:** an endpoint of a routed session accepts only bursts from the other end
  (UP set if we are the caller, clear if we are the called station) with hop *n* (route
  field *n* + 1). Copies heard directly or part way along the path, repeats of our own
  bursts and direct bursts (route field 0) are ignored. The called station accepts a
  routed (`VFM2`) ConnectReq only with UP = 0 and hop *n*, and a direct (`VFM1`) one only
  with UP = 0 and route field 0.
- **After ConnectAck:** the called station may send first, but on a routed session only
  after `n · (connect burst + 0.2 s + hop margin)`, the time the digipeaters take to
  repeat its ConnectAck. Sooner, its burst would collide with those repeats.
- **Timing:** the far end's answer comes back after every digipeater has repeated our
  burst and then the answer, and the stations at the ends may hear none of the middle
  repeats. A station therefore waits
  `1.2 s + n · (our burst + longest possible burst + 2 · 0.2 s + hop margin)` before it
  resends. The hop margin (default 0.5 s, `--hop-margin`, at most 10 s) covers a
  digipeater's decoding time and the delay before its transmitter is up. An answer passes
  each digipeater twice, so it has to be about twice a digipeater's extra delay, and both
  ends need the same value. It does not cover a VOX hang: the neighbours still answer
  0.2 s after a repeat ends. The wait is about 11 s and 20.5 s for 1 and 2
  digipeaters in narrow mode after a full burst (10 and 19 s wide). Connect
  retries are spaced the same way with the connect bursts' own length. Retries are spread
  by a random 0–0.4 s times (1 + 2n), and give way while the channel is busy: that is
  most likely the other end's burst (its own retry, or coming through a digipeater),
  which is then answered instead.
- **Called station waits longer:** the two ends do not hear each other, so if both start
  a burst at about the same time, both can be lost at a digipeater, and with equal timers
  every retry would meet there again. So before resending a burst it started itself (not
  an answer to the caller), and before every further retry, the called station waits
  until the caller's longest retry period has passed since the start of its own burst
  (`longest burst + the wait above after it + 0.4 s · (1 + 2n)`), plus
  `n · (longest burst + 0.2 s + hop margin)`, the time the caller's retry takes to reach
  it. After each such collision its next burst therefore starts at least that much later,
  relative to the caller's, than the last one did, so within a few retries it hears the
  caller's retry and answers it. The wait is about 21 s and 36.5 s for 1 and 2
  digipeaters in narrow mode after a short burst (17 and 32.5 s after a full burst;
  16–34 s wide). The caller's waits, direct sessions and `Disc` retries are unchanged
  (a digipeater forgets the session 60 s after the DiscAck). A quiet routed link is probed
  after the larger of the usual keepalive and twice the called station's longest wait.
- **Limits:** `req_level` is measured on the last hop only. A weak first hop still lowers
  the level through failed (void) codewords, but the level can settle above what that hop
  supports best. Each digipeater adds a full copy of every burst to the channel, so with
  *n* digipeaters on one frequency the throughput falls to about 1/(*n* + 1). Session ids
  are 8 bits: two routed sessions through one digipeater with the same id would confuse
  it (a direct session with the same id cannot). A digipeater's own session shares the
  channel with the sessions it repeats, and its peer usually cannot hear the stations on
  the other side, nor they the peer, so their bursts collide at the digipeater. Right
  after a repeat the digipeater may also start a burst of its own over the answer, which
  it cannot hear yet. Both sessions then retry a lot.

### 3.5 Unproto (unconnected broadcast)
An `Unproto` burst (kind 5) carries frames from the host's KISS port (§4.5) to every
station that hears it. There is no session, no acknowledgement and no retry, like an AX.25
UI frame.

- **Header:** kind 5, `level` and `ncw` as for any burst. Every other field is 0, flags
  included (no POLL, UP or route field), and receivers ignore them all, flags included:
  a burst of kind 5 is an Unproto burst whatever its flags say. Session id 0 never matches
  a session (callers pick 1–255).
- **Never digipeated:** a digipeater repeats only bursts with a non-zero route field of
  sessions it has learned (§3.4), and an Unproto burst has neither. A broadcast reaches only
  the stations that hear the sender directly, even one that sets route bits on it. A
  digipeater's own Unproto bursts wait while a repeat is due, like its other bursts of its
  own, and after a repeat the clear-channel wait below starts again from the end of the
  repeat, since the next station answers it 0.2 s later.
- **Payload:** the data bytes of the `ncw` codewords, joined in order:
  `0x01 (format) | { length (u16, big-endian, ≥ 1) | frame }* | zero padding`. A frame is a
  whole KISS frame after unescaping, type byte first, carried unchanged. `ncw` is the
  smallest number of codewords that holds it all. A receiver reads frames until the
  first codeword that failed its CRC. After that point no length can be trusted, so the
  frames that touch that codeword or come after it are lost. A payload with another
  format byte is dropped.
- **Level:** 0, or 1 if the first frame waiting does not fit at level 0. There is no
  feedback, so no rate adaptation. One burst holds at most `max_ncw(level) × 146 − 1`
  bytes of frames plus lengths:

  | Mode | Level 0 | Level 1 | Longest frame |
  |---|---|---|---|
  | narrow | 291 | 729 | 727 |
  | wide | 875 | 1897 | 1895 |

  A longer frame is dropped.
- **Batching:** frames that reach the modem within 0.3 s of each other share a burst, as
  far as they fit, in the order they arrived; frames are never reordered. The first frame
  waits at most 2 s for others. Frames that do not fit go in the next burst.
- **Channel access:** an Unproto burst is sent only while the station has no session and is
  not connecting, so frames written during a session wait until it ends. The channel must
  also have been clear for 0.5 s plus a random 0–0.5 s, measured again before each burst.
  That is longer than a session's 0.2 s turnaround, so a broadcast does not step into
  the gap between two other stations' turns. Up to 64 frames or 32 KB wait; beyond that
  the oldest are dropped.
- **Reception:** every station delivers the frames of every Unproto burst it decodes to its
  KISS clients, whatever its state, including during a session of its own. They do not
  affect the session.

No encryption is used, and there is no compression yet.

## 4. Host interface

VardeFM offers VARA's TCP interface. The command port (default 8300) carries ASCII lines
ending in CR; LF is also accepted as a terminator. The data port (command port + 1)
carries raw bytes. Each port takes one client; a new connection replaces the old one.
Verified against EA5HVK's "VARA Protocol Native TNC Commands" (February 2022) and Pat's
VARA driver (github.com/n8jja/Pat-Vara).

### 4.1 Host → modem
Each command gets `OK`, or `WRONG` if it is invalid.

| Command | Behaviour |
|---|---|
| `MYCALL c1 [c2 … c5]` | Up to 5 call signs: 3–7 characters A–Z/0–9, optional SSID −1…−15, −T or −R. |
| `LISTEN ON` / `LISTEN OFF` | Accept incoming connections or not. |
| `CONNECT <mycall> <target> [VIA <d1> [<d2>]]` | `WRONG` if a session is already active. `VIA` (any case) routes the session through up to two digipeaters, in order from us (section 3.4). |
| `DISCONNECT` | Graceful: closes once everything queued is acknowledged. |
| `ABORT` | Immediate close. |
| `VERSION` | `OK`, then `VERSION VardeFM <version>`. |
| `CHAT ON` / `CHAT OFF` | Send `SN` reports (§4.2) or not. Off by default, and off again when the command-port client disconnects. VARA's other `CHAT ON` effects (an implied `LISTEN ON`, keyboard-to-keyboard handover timing) are not copied. |
| `UNPROTO SN ON` / `UNPROTO SN OFF` | VardeFM only: send `UNPROTO SN` reports (§4.2, §4.5) or not. Off by default, and off again when the command-port client disconnects. Real VARA answers `WRONG`, which tells a host the reports are not available. |
| `LEVELS ON` / `LEVELS OFF` | VardeFM only: send `LEVEL` and `RXPEAK` reports of the receive audio level (§4.2, "Receive level") or not. `LEVELS ON` is answered with `OK` and then, if any burst has been decoded since the modem started, one `RXPEAK` for the last of them. Off by default, and off again when the command-port client disconnects. Real VARA and VardeFM before 0.3.2-ortoo.2 answer `WRONG`, which tells a host the reports are not available. |
| `PUBLIC ON/OFF`, `COMPRESSION OFF/TEXT/FILES`, `CWID ON/OFF`, `BW500/BW2300/BW2750`, `WINLINK SESSION`, `P2P SESSION` | Accepted (`OK`), no effect. |

### 4.2 Modem → host
| Line | When |
|---|---|
| `CONNECTED <source> <destination> NARROW` (or `WIDE`) | Session up. `source` is the calling station. |
| `DISCONNECTED` | Session closed for any reason: DISCONNECT, ABORT, peer closed, link lost, or connect failed. |
| `PENDING` | Called side: a connect request for us arrived (followed by `CONNECTED`). |
| `CANCELPENDING` | Reserved; not sent yet. |
| `BUFFER <n>` | Bytes queued and not yet acknowledged. Sent after every read from the data port (even if *n* is unchanged), and whenever acknowledgements change *n*, down to `BUFFER 0`. A report that follows a write therefore already counts that write. |
| `PTT ON` / `PTT OFF` | Transmitter keyed / released. |
| `BUSY ON` / `BUSY OFF` | Channel activity heard (audio in the carrier band at least 6 dB above the background level and above −40 dBFS, or a frame being received). The band is 422–2672 Hz in narrow mode and 400–6000 Hz in wide mode (filter edges, 6–9 dB down); sound outside it, such as a CTCSS tone, mains hum or hiss above the band, does not count. A steady open-squelch hiss becomes the background within about 2 s. After the modem's own transmission nothing is measured until the received audio comes back (at most 1 s), nor for 20 ms after that, and a busy hold from before the transmission ends. |
| `SN <dB>` | With `CHAT ON` only: after every burst received from the session peer, its S/N with one decimal, e.g. `SN 12.3` or `SN -1.5`. That covers the connect request or answer that opens the session and every burst of the session after it. Bursts with another session id (other stations' sessions on the frequency) and bursts heard while not in a session are not reported. Sent after `CONNECTED`, never before it. On a routed session (§3.4) it is the S/N of the last hop: the peer's burst as the last digipeater repeated it, not the peer's own signal at that digipeater. See the note below the table. |
| `UNPROTO SN <dB> <crc> …` | VardeFM only, with `UNPROTO SN ON`: after every received Unproto burst (§3.5) that delivered at least one frame to the KISS port, the burst's S/N (formatted as for `SN`) and then, for each of those frames in order, the CRC-32 of the frame as 8 upper-case hex digits. Sent in any state, including during a session. How a host uses it: §4.5. |
| `LEVEL <rms> <peak> <raw>` | VardeFM only, with `LEVELS ON`: about every 0.25 s while not transmitting, in any state, including during a session. The receive level in dBFS, one decimal each, e.g. `LEVEL -27.3 -18.9 -18.9`. See "Receive level" below. |
| `RXPEAK <peak> <rms> <age>` | VardeFM only, with `LEVELS ON`: after every decoded burst, its peak and RMS level in dBFS (one decimal each) and `0`; and once straight after the `OK` to every `LEVELS ON`, the last burst decoded since the modem started, with its age in whole seconds, e.g. `RXPEAK -6.2 -15.8 42`. See "Receive level" below. |
| `IAMALIVE` | Every 60 s. |

**What `SN` measures.** It is the receiver's own estimate, from the pilots, of the SNR per
symbol (Es/N0) of the demodulated audio, in dB: the figure the modem logs for each burst
and uses for rate adaptation (§3.2). It is not the RF carrier-to-noise ratio. Measured in
`sim-pair` (narrow mode, test `sn_reports_track_the_cnr`, ranges over several runs):

| CNR | Highest `SN` | Median `SN` |
|---|---|---|
| 6 dB | 5.9–8.0 | 5.6–5.9 |
| 12 dB | 14.6–14.7 | 14.2 |
| 18 dB | 19.2–19.3 | 18.5–18.9 |
| 24 dB | 23.4–23.5 | 21.3–22.1 |

Above the FM threshold the reading is within about 2.5 dB of the CNR (9 dB CNR still read
11.5). At 6 dB, below the threshold, the discriminator's clicks dominate and the reading
stays around the CNR. The highest reading is limited by the burst's own drive level: about
21 dB on level-0 bursts (connect, ack and poll bursts) and about 25 dB on level-6 data
bursts, so a strong signal reads a few dB lower on a short control burst than on a data
burst; that is why the median falls behind the highest reading at 24 dB. The estimate is
capped at 60 dB. Hosts that show a meter may want to smooth it.

The VARA document lists `SN value` as "S/N of the received frame", sent only with
`CHAT ON`. Pat ignores the line.

#### Receive level (`LEVELS`)
For a host's level meter, for example a dialog the user opens while setting the sound
card's recording level or the radio's volume. The host sends `LEVELS ON` when the meter
opens and `LEVELS OFF` when it closes. It is off by default and off again when the
command-port client disconnects, so the next host starts without it. A host finds out
whether the modem has it by sending `LEVELS ON`: real VARA and VardeFM before
0.3.2-ortoo.2 answer `WRONG`.

**`LEVEL <rms> <peak> <raw>`**, about every 0.25 s:

    LEVEL -27.3 -18.9 -18.9

- The audio measured is the received audio as the sound card delivers it (the chosen
  channel, resampled to 48 kHz if the card runs at another rate), before any filtering:
  the modem has no AGC. 0 dBFS is full scale.
- Each line covers the 0.25 s since the last one (blocks follow each other without
  overlap): `rms` = 10·log₁₀ of the mean of the squared samples, `peak` = 20·log₁₀ of the
  largest absolute sample. Silence reads `-120.0`. A NaN or infinite sample from the sound
  card is skipped.
- `rms` and `peak` are of the audio the receiver gets. `raw` is the peak before any input
  gain, as the sound card delivered it, so only `raw` shows clipping in the sound card
  (`raw` at or near 0 dBFS). Today the modem applies no input gain and `raw` always equals
  `peak`; a later input gain option will make them differ. With such a gain, `rms` and
  `peak` can be above 0 dBFS without anything clipping, so a host must accept values
  above 0.
- No `LEVEL` lines are sent while the modem transmits (between `PTT ON` and `PTT OFF`).
  The block in progress at key-up is dropped, so the first line comes 0.25 s after
  reception resumes. That first block may include the radio's own audio returning after
  the transmission, such as a squelch opening.
- Received bursts should peak around -6 dBFS (docs/RADIO.md). With the squelch open and
  nothing on the channel, `LEVEL` shows the hiss.

**`RXPEAK <peak> <rms> <age>`**, the level of a decoded burst, to tell a real signal
apart from open-squelch hiss:

    RXPEAK -6.2 -15.8 0

- Sent after every burst whose header decoded, of any kind and from any station: the
  session peer, other stations' sessions, digipeater repeats, Unproto broadcasts and
  control bursts (acknowledgements, polls). A burst counts even if some or all of its
  payload codewords failed. A burst whose header did not decode gives no line.
- The span measured runs from the start of the burst's preamble to the end of its last
  data symbol (the last header symbol for a burst without payload). It leaves out the
  txdelay leader before the preamble and any squelch tail after the burst. `peak` and
  `rms` are in dBFS with one decimal, measured like `LEVEL`'s `peak` and `rms`.
- On the command port the line comes just before that burst's `SN` or `UNPROTO SN` line,
  if it has one.
- `age` is in whole seconds, rounded down: `0` for a burst just decoded. Straight after
  the `OK` to **every** `LEVELS ON`, even when already on, the modem sends one `RXPEAK`
  for the last burst it decoded since it started, with its real age. That burst may be
  from before the host connected or from while `LEVELS` was off, so a meter just opened
  can show the last real signal at once. If no burst has been decoded yet, nothing
  follows the `OK`. Age is counted on the modem's clock, which is the input sound card's
  sample count since the modem started, so it does not advance while the sound card
  delivers no audio.

**Traffic.** While on, the reports add about 4 lines a second, about 75 bytes a second,
to the command port only (the data port is not affected). Measuring costs the modem
next to nothing, and it does not change the modem's timing, so the meter can stay open
during a session. The host must keep reading the command port while `LEVELS` is on, as it
must for any VARA host. It should send `LEVELS OFF` when its meter closes.

### 4.3 Data port
While connected, bytes written are sent and received bytes are delivered. Any byte value
is allowed. Bytes written while not connected are dropped. Data written after
`DISCONNECT` is still sent, as in VARA, until the closing `Disc` goes out.

**Ordering across the two sockets.** Every byte written to the data port before
`DISCONNECT` was sent on the command port is transmitted, whether or not a `BUFFER`
report counted it yet. The modem holds `DISCONNECT` back until its data-port reader has
found the socket empty (at most 0.5 s). `DISCONNECT` and `ABORT` arriving after the link
has already ended get `OK` and have no effect. In particular, no second `DISCONNECTED`
is sent.

**Advice for hosts (also for real VARA, which has the same two-socket design):** before
`DISCONNECT`, wait for a `BUFFER 0` that arrives *after* your last data write. A
`BUFFER 0` read just after a write may be stale. After `DISCONNECTED`, keep reading the
data port briefly (about 0.5 s), because received data and the `DISCONNECTED` line
travel on different sockets.

### 4.4 Differences from VARA
- `CONNECT … VIA` takes up to two digipeaters, as VARA FM's document allows. The
  digipeaters must be VardeFM stations running with `--digi` (section 3.4). A link
  through digipeaters is slower, and a lost burst takes longer to recover, so a host's
  own connect or session timeouts may need raising, most of all for two digipeaters.
- `CONNECTED` always ends with `NARROW` or `WIDE`, like VARA FM's bandwidth field
  (the VARA document's example is `CONNECTED EA5HVK-7 EB1MS-3 WIDE`).
- `LISTEN ON/OFF` does not drop an active session (VARA's document says it does).
- After `ABORT`, `DISCONNECTED` is always sent (Pat notes that VARA sometimes omits it).
- `DISCONNECT` or `ABORT` with no session gives `OK` and nothing else.
- `VERSION` gets `OK` before the `VERSION …` line. Whether real VARA sends `OK` here is
  unverified, so hosts should accept either.
- If the command-port client disconnects, any session is aborted.
- Not implemented: `CQFRAME`, `REGISTERED`/`LINK REGISTERED`, encryption, compression.

### 4.5 KISS port
A separate TCP port (`modem --kiss-port`, default 8100, `off` to disable; the same default
as VARA FM's KISS port) carries unconnected broadcasts (§3.5). Frames are
`FEND (0xC0) | type byte | data | FEND`, with FESC (0xDB) escaping: FEND inside a frame is
sent as `DB DC`, and FESC as `DB DD`.

- **Host → modem:** data frames are queued for an Unproto burst. A data frame is command 0
  on any port (`type & 0x0F == 0`), or command 1 with more than one data byte. VarAC and
  VARA FM use type 1 for frames with 7-character AX.25 addresses: Ortoo's CQ is
  `C0 01 "CQ" … 03 F0 C0`. A real TXDELAY command carries exactly one byte. The other
  commands are accepted and ignored: TXDELAY, P, SlotTime, TXtail, FullDuplex,
  SetHardware and Return (`0xFF`). VardeFM uses its own leader and channel access.
  Empty frames and bytes before the first FEND are ignored.
- **Modem → host:** each received frame, exactly as the sender's host wrote it, type byte
  included: `C0 <type> <escaped data> C0`. A station's own frames are not echoed to it.
- VardeFM does not parse the frames. AX.25 UI frames (APRS, VarAC, Ortoo) pass through
  unchanged, and so would any other bytes.
- Any number of clients may connect at once. Each gets every received frame, and any of
  them may send.

`sim-pair` gives each station a KISS port: `--a-kiss-port` (default 8100) and
`--b-kiss-port` (default 8110), each a port number or `off`.

#### S/N of received broadcasts (`UNPROTO SN`)
A host that wants to know how well it hears the station behind a CQ, broadcast or beacon
sends `UNPROTO SN ON` on the command port. After each received Unproto burst the modem
then sends one line on the command port:

    UNPROTO SN 17.4 6C447797 F86971FE 3FBDD9F7

That is the burst's S/N (the same measurement and format as `SN`, §4.2), then one CRC-32
per frame the burst delivered to the KISS port, in the order they were delivered. The CRC
is the standard CRC-32 (IEEE 802.3, reflected polynomial `0xEDB88320`, as computed by
zlib, zip and Node's `zlib.crc32`; `CRC32("123456789") = CBF43926`). It is taken over the
frame exactly as KISS clients get it after unescaping, type byte first, without the FENDs.
Frames lost to a failed codeword (§3.5) are not delivered, so they are not named either.
A burst that delivers no frame gives no line. A station never hears its own broadcasts,
so it never reports them.

**Attributing the S/N to frames.** For each frame from the KISS port, compute its CRC-32
and take the oldest not-yet-used entry with that CRC from the `UNPROTO SN` lines received
in the last few seconds. If none has arrived yet, keep the frame for a moment (say 2 s) and
match it when the line comes. Match the other way round as well: a line may arrive before
its frames.

**Why it is built this way.**
- *Nothing is added to the KISS port.* Other KISS clients (VarAC, APRS programs) must keep
  getting the frames byte for byte, and every KISS client gets everything the port sends.
  So the S/N cannot ride inside the frames, nor in extra KISS frames such as a vendor
  command. It goes on the command port, which has one client and which that client asked
  for.
- *The two sockets are not ordered.* The modem writes a burst's frames to the KISS port
  first and the `UNPROTO SN` line to the command port right after, in the same step. But
  a host reads the two sockets separately, so it may see the line first, or after the next
  burst's frames. "The next *n* frames" or "the frames since the last line" would then be
  wrong. Naming each frame by the CRC of its own bytes does not depend on arrival order at
  all.
- *Identical CRCs are harmless.* Two frames with the same CRC are, apart from a 2⁻³² chance,
  the same bytes, including the AX.25 source address, so they came from the same station.
  That happens when a station repeats a CQ or a `[PAGE]` line. Matching the oldest entry
  first then gives each copy the S/N of one of the bursts that carried it. Either answer is
  a correct reading of that station.
- *One line per burst, not per frame.* All frames of a burst share one measurement.
- *A burst counter was not chosen.* It would have to be carried in the KISS stream to be
  of any use, which rules it out (see the first point).

The longest line comes from a level-1 burst full of one-byte frames: 243 CRCs in narrow
mode, 632 in wide mode, about 5.7 KB. Hosts that cap line length should allow for that.
