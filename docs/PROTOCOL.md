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
1. Low-pass filters the audio (wide mode only) and runs a Schmidl-Cox autocorrelation.
2. Confirms the frame by cross-correlating with preamble B.
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
| 3 | kind: 0 ConnectReq, 1 ConnectAck, 2 Data, 3 Disc, 4 DiscAck, 5 Unproto (§3.4); 6 and 7 unassigned |
| 8 | session id (random, chosen by the caller) |
| 3 | payload speed level |
| 5 | ncw: payload codewords (0–24) |
| 3 | req_level: level the sender asks the peer to use next |
| 3 | burst_id: sender's burst counter, mod 8 |
| 3 | ack_burst: burst_id of the peer burst being acknowledged |
| 24 | ack_offset: next byte offset expected from the peer, mod 2²⁴ |
| 24 | bitmap: bit *i* (LSB = codeword 0) set if codeword *i* of burst `ack_burst` decoded |
| 4 | flags: bit 0 = POLL (the peer must answer) |

A receiver drops a header whose kind it does not know exactly as it drops one that failed
its CRC: no burst is reported and the payload is not decoded. Every version back to 0.1.0
does this, which is why kind 5 could be added without a minor version bump. A burst like
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
  non-zero session id. A station that has `LISTEN ON` and the destination among its
  `MYCALL`s answers `ConnectAck` with the same payload and session id. The caller retries
  5 times, then gives up and reports DISCONNECTED (about 17 s in narrow mode, 11 s in
  wide).
- **Turns:** the called station may send first right after `ConnectAck`; the caller
  waits one response period for it.
- Bursts with a different session id are ignored.

### 3.2 Data transfer
- Each direction is a byte stream numbered by offset. A payload codeword carries one
  segment: `offset mod 2²⁴ (3 bytes) | length (1 byte) | data | zero padding`.
- **Acknowledgements:** every burst carries `ack_offset` and the bitmap for the peer's
  last burst. The sender drops acknowledged bytes and resends the rest, re-split to fit
  the current level. Data is delivered to the host in order, exactly once.
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
- **Retries:** an unanswered burst is resent after 1.2 s (plus a random back-off, and
  longer while the channel is busy). After 8 consecutive failures the link is lost.
- **Keepalive:** a quiet link is probed with a POLL burst after 30 s (caller) or 45 s
  (called station).

### 3.3 Disconnect
- **Graceful:** once all of its data is acknowledged, the closing station sends `Disc`;
  the peer answers `DiscAck`. Both report DISCONNECTED. `Disc` is retried up to 4 times;
  a station that already closed still answers a repeated `Disc`.
- **ABORT:** the station sends one `Disc` without waiting and closes at once.

### 3.4 Unproto (unconnected broadcast)
An `Unproto` burst (kind 5) carries frames from the host's KISS port (§4.5) to every
station that hears it. There is no session, no acknowledgement and no retry, like an AX.25
UI frame.

- **Header:** kind 5, `level` and `ncw` as for any burst. Every other field is 0, and
  receivers ignore it. Session id 0 never matches a session (callers pick 1–255).
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
| `CONNECT <mycall> <target>` | `WRONG` if a session is already active. |
| `DISCONNECT` | Graceful: closes once everything queued is acknowledged. |
| `ABORT` | Immediate close. |
| `VERSION` | `OK`, then `VERSION VardeFM <version>`. |
| `CHAT ON` / `CHAT OFF` | Send `SN` reports (§4.2) or not. Off by default, and off again when the command-port client disconnects. VARA's other `CHAT ON` effects (an implied `LISTEN ON`, keyboard-to-keyboard handover timing) are not copied. |
| `UNPROTO SN ON` / `UNPROTO SN OFF` | VardeFM only: send `UNPROTO SN` reports (§4.2, §4.5) or not. Off by default, and off again when the command-port client disconnects. Real VARA answers `WRONG`, which tells a host the reports are not available. |
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
| `SN <dB>` | With `CHAT ON` only: after every burst received from the session peer, its S/N with one decimal, e.g. `SN 12.3` or `SN -1.5`. That covers the connect request or answer that opens the session and every burst of the session after it. Bursts with another session id (other stations' sessions on the frequency) and bursts heard while not in a session are not reported. Sent after `CONNECTED`, never before it. See the note below the table. |
| `UNPROTO SN <dB> <crc> …` | VardeFM only, with `UNPROTO SN ON`: after every received Unproto burst (§3.4) that delivered at least one frame to the KISS port, the burst's S/N (formatted as for `SN`) and then, for each of those frames in order, the CRC-32 of the frame as 8 upper-case hex digits. Sent in any state, including during a session. How a host uses it: §4.5. |
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
- `CONNECT … via <digis>` (VARA FM digipeating) is not supported (`WRONG`).
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
as VARA FM's KISS port) carries unconnected broadcasts (§3.4). Frames are
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
Frames lost to a failed codeword (§3.4) are not delivered, so they are not named either.
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
