# Plan: unconnected broadcast bursts (Unproto) and a KISS port

Status: **implemented** on branch `unproto-kiss` (version 0.3.1-ortoo.1). docs/PROTOCOL.md
§2.3, §3.5 (§3.4 before the merge with 0.3.2) and §4.5 are the reference. This file is kept as the design record. Where the
implementation differs from the plan below:

- Each frame on the air is the whole KISS frame, **type byte included**, and it is
  delivered back exactly. Data frames are accepted on any port nibble. Command 1 with more
  than one byte also counts as data: VarAC and Ortoo send CQs as type 1, where a real
  TXDELAY has a single byte.
- **Batching:** frames that arrive within 0.3 s of each other share one burst, in order.
  A burst waits at most 2 s for more frames.
- Per-burst capacity is `max_ncw × 146 − 1` bytes (narrow 291/729, wide 875/1897). The
  table in §1 below used − 3.
- Queue limits are 64 frames / 32 KB.
- §5 below has a correction: old stations' busy detection covers only the first ~1.7 s
  of an Unproto burst.

Based on `master` at e982ca6. Master is on-air identical to v0.3.1: modulated audio
is bit-identical, and recordings cross-decode both ways.

## 1. Burst kind 5 = Unproto

### Header (`crates/vardefm-modem/src/header.rs`, PROTOCOL.md §2.3)
The 3-bit `kind` field has three free values (5, 6, 7). Add `Kind::Unproto = 5` and
extend `Kind::from_u8`. The fields are used like this:

| Field | Unproto use |
|---|---|
| kind | 5 |
| session | 0. Real session ids are 1..255 (`rng % 255 + 1`), so 0 can never match a session. |
| level | payload speed level, 0 or 1 (see below) |
| ncw | codewords actually needed, 1..max_ncw(level) |
| burst_id | sender's unproto counter mod 8 (informative only, for logs) |
| req_level, ack_burst, ack_offset, bitmap, flags | 0. Receivers ignore them. |

`Header::unpack` needs no other change: its range checks (level < 7, ncw ≤ 24) already
hold.

### Payload format
Join the data bytes of the burst's codewords in order (146 B each at level 0 or 1) into
one byte string:

```
0x01 (format version) | { len: u16 BE (1..) | frame: len opaque host bytes }* | 0x00 0x00 | zero padding
```

- Frames are opaque. VardeFM never parses AX.25.
- A frame may cross a codeword boundary. The receiver delivers a frame only if every
  codeword it touches passed its CRC-32. Rule: walk the frames until the first failed codeword, then stop. After that
  point no `len` field can be trusted, so the frames after it are lost too.
- An unknown format version means the burst is dropped, which leaves room for later
  changes.

### Level and size
There is no feedback, so no rate adaptation. Use **level 0** (BPSK 1/2) because every
station that can hear the header can decode it. If a single frame doesn't fit at level
0, use level 1. If it doesn't fit at level 1 either, drop it and log it.

Capacity per burst = `max_ncw(level) × 146 − 3` bytes (version byte plus terminator),
less 2 per frame:

| Mode | Level | max_ncw | Payload bytes | Largest single frame | Airtime of a full burst* |
|---|---|---|---|---|---|
| narrow | 0 | 2 | 292 | 287 | ≈ 3.3 s |
| narrow | 1 | 5 | 730 | 725 | ≈ 4.0 s |
| wide | 0 | 6 | 876 | 871 | ≈ 3.7 s |
| wide | 1 | 13 | 1898 | 1893 | ≈ 4.2 s |

\*With txdelay 0.15 s. These numbers come from `max_ncw` (the 4.0 s payload limit) and
the §2.2 table. Check them with a unit test.

An AX.25 UI frame with a 256-byte info field and no digipeaters is 272 bytes (2 digis:
286). That fits one narrow level-0 burst. Longer digi paths fall back to level 1. One
burst may pack several queued frames.

## 2. Sending (`crates/vardefm-link/src/lib.rs`)

- `Link` gets `unproto: VecDeque<Vec<u8>>` (capped at about 32 frames or 16 KB; when full,
  drop the oldest and log it), `send_unproto(frame)`, `unproto_at: Option<f64>`, and
  `clear_since: f64`, which `poll` updates from `busy`.
- Only in `State::Idle` with no `oneshot` pending. So nothing goes out while
  `Connecting` or `Connected`; frames stay queued until the session ends.
- CSMA, modelled on the ConnectReq path (`poll`, State::Connecting: on busy,
  `next_tx = max(next_tx, now + 0.3)`):
  - The channel must have been clear for at least **0.5 s + random 0–0.5 s** before the
    burst starts. That is longer than ConnectReq's 0.3 s on purpose: other stations in an
    ARQ session leave only `turnaround_s` = 0.2 s (plus decode time) between turns, and an
    Unproto must not step into that gap.
  - If `busy` turns true before the start time, give way and re-arm the timer.
- Build the burst: take frames in FIFO order while they fit at level 0. If the first
  frame alone doesn't fit, send it alone at level 1. Use `ncw = ceil(bytes / 146)`, not
  the maximum.
- `tx_done` in `State::Idle`: if more frames are queued, re-arm the CSMA timer. There is
  no retry and no ack; Unproto is fire-and-forget like an AX.25 UI frame. (A future
  option is a `--unproto-repeat N` setting.)
- `TxInfo{kind: Unproto, …}` and `stats.bursts_sent` are updated as for other bursts, so
  the sim-pair and modem logs show it.
- A ConnectReq from the host while frames are queued takes priority. The frames wait.

## 3. Receiving (every station)

- `Receiver` needs no change. A kind-5 header decodes like any other, and the payload is
  decoded by `decode_payload`.
- `Link::on_burst`: handle `Kind::Unproto` **first**, before the `match &mut self.state`.
  Otherwise `State::Connected` returns early on `h.session != s.id`, and `Connecting`
  does the same. Parse the payload and push one `LinkEvent::Unproto(Vec<u8>)` per good
  frame, in every state, including during our own session. Unproto must not touch
  session state (`last_heard`, acks, `req_level`).
- `Station::take_output` maps it to a new `HostOut::Kiss(Vec<u8>)`. The sim-pair and radio
  loops write it to the KISS port. The VARA command and data ports see nothing new: no new
  lines, so existing VARA hosts are unaffected.
- Every station delivers every Unproto burst it decodes. There is no address filtering,
  same as a KISS TNC; the host (an APRS or packet client) filters.

## 4. KISS-over-TCP host port

- New `crates/vardefm-tnc/src/kiss.rs` with a codec: FEND 0xC0, FESC 0xDB, TFEND 0xDC,
  TFESC 0xDD. The decoder is streaming, so it handles split TCP reads and many frames per
  read.
- Host → modem: the command byte's low nibble 0 means a data frame, and the payload goes
  to `Link::send_unproto`. The high nibble (KISS port) must be 0; frames for other ports
  are dropped. Commands 1–6 (TXDELAY, P, SlotTime, TXtail, FullDuplex, SetHardware) and
  0xFF are accepted and ignored, because VardeFM's own leader and CSMA apply. Empty frames
  are dropped.
- Modem → host: `C0 00 <escaped frame> C0` for each received frame.
- Compatibility with VARA FM's KISS port (TCP 8100): VARA FM sends and receives standard
  KISS data frames holding AX.25 frames on port 0, and APRS and packet clients (YAAC,
  Xastir, APRSIS32, Dire Wolf's `kissutil`, BPQ "KISS over TCP") connect to it unchanged.
  VardeFM carries the bytes opaquely, so AX.25 UI frames round-trip exactly. Connected-mode
  AX.25 over it would also pass, but with no extra reliability, as with any KISS TNC.
- Clients: accept several at once, like Dire Wolf. Every client gets every received frame,
  and any client may send. This is a superset of VARA's one-client behaviour, so it costs
  no compatibility. Our own transmissions are not echoed back.
- Implementation: a `KissPort` in `server.rs` (or `kiss.rs`) with an acceptor thread and a
  reader thread per client, feeding a new `HostIn::Kiss(Vec<u8>)` into the same inbox
  channel, plus `send_kiss(&[u8])`. It reuses the existing `HostPort` patterns. It is a
  separate listener, so the VARA ports' cross-socket DISCONNECT ordering is unaffected.
- CLI:
  - `vardefm modem --kiss-port <N|off>`, default 8100. `off` disables it. (`0` keeps the
    existing meaning of "pick a free port", which tests need.)
  - `vardefm sim-pair --a-kiss-port <N|off> --b-kiss-port <N|off>`, defaults 8100 and
    8110. Both sides have their own port, matching the 8300/8310 pattern.
  - The startup banner prints the KISS address next to the command and data ports.
- Note: VARA FM itself also defaults to 8100. Running both on one PC needs
  `--kiss-port` changed on one of them.

## 5. Do older VardeFM versions safely ignore kind 5?

**Yes. I checked the decode path in every tag from v0.1.0 through v0.3.1 and in master.**
They all have the same code:

1. `Receiver::step` (`crates/vardefm-modem/src/receiver.rs`) detects preamble A/B, then
   calls `Modem::decode_header` (`lib.rs`). The LDPC header decode and CRC-16 succeed, and
   it calls `Header::unpack`.
2. `Header::unpack` runs `kind: Kind::from_u8(vals[0] as u8)?`, and `from_u8` has
   `_ => return None`. So `unpack` returns `None` **after** the CRC check passes, and
   `decode_header` returns `None`.
3. Back in `step`, the `match header { None => self.search = b, … }` arm runs. This is
   exactly the path taken by a header that failed its CRC. No `RxBurst` is produced,
   `pending` is not set, and the link layer never sees the burst. Nothing is logged and
   the host sees nothing.
4. Detection resumes at preamble B's index `b`. Preamble A lies before `b`, so the same
   burst is not detected again, and there is no loop. The search then scans the payload
   symbols, as it already does after any corrupt header. A false preamble hit there costs
   only a failed header decode.
5. Channel busy: because `pending` stays `None`, `busy()` relies on the energy detector
   (`BusyDetector` in master; the `quiet_samples` hold in v0.1/v0.2). **Correction after
   measurement:** that detector takes a steady sound as background within about 2 s.
   Fed recorded Unproto bursts, the v0.3.1 receiver and stock master's receiver both
   report busy for only the first ~1.7 s. A narrow 1-codeword burst is covered (one 60 ms
   gap at its end). In a narrow 2-codeword burst (3.3 s) about 1.5 s reads as clear, and
   in a full wide burst about 2 s. An old station ignores the content safely, but it
   respects only the first ~1.7 s of the airtime.

Remaining risk: this is a convention, not a guarantee. The header is not
versioned, so a future upstream release could assign kind 5 to something else.
Coordinate with upstream (notepid) before using 5, or ask them to reserve it in
PROTOCOL.md.

## 6. Testing

Unit tests:
- `header.rs`: an Unproto round-trip through `pack` and `unpack`. Also edit the field bits
  of a packed header to kind 6, recompute the CRC-16, and assert `unpack` returns `None`
  (the "unknown kind is ignored" contract).
- `link/src/tests.rs`: frames queued while Connected are held, then sent after
  DISCONNECTED; nothing goes out while `busy` or before the 0.5 s clear time; packing and
  splitting across codewords; level-1 fallback; oversize frames dropped; a failed
  codeword loses its frames and those after it, never the ones before; received Unproto is delivered in the Idle, Connecting
  and Connected states without changing the session.
- `kiss.rs`: escaping round-trip with all 256 byte values, split reads, back-to-back
  FENDs, non-data commands ignored.

Integration (`crates/vardefm-tnc/tests/simpair.rs`, ports 0 so nothing fixed is bound):
- A KISS → B KISS: a 272-byte AX.25 UI frame plus several small ones, at noiseless and
  at 10 dB CNR, narrow and wide. Assert byte-exact delivery.
- Broadcast during a session: A↔B in an ARQ session; frames queued on A go out only
  after the session ends, and the session's 10 kB transfer stays intact.
- Both directions at once, so CSMA collisions are rare; this exercises the random wait.
- Old-receiver check: run `sim-pair --record <dir>` with the new build, then decode the
  recorded `*_rx.wav` with a **v0.3.1 binary built from the tag in a separate worktree**
  (`vardefm decode --in …`). Expect the Unproto burst to print nothing while the Data and
  ConnectReq bursts in the same recording still decode. (This uses a separate build, not
  C:\VardeFM.)

Manual sim-pair (ports chosen away from 8300/8301/8310/8311 and VARA's 8100):
```
target\release\vardefm.exe sim-pair --cnr 15 --a-port 8400 --b-port 8410 ^
    --a-kiss-port 8402 --b-kiss-port 8412
```
Then point a KISS client at each side, such as Dire Wolf `kissutil -h 127.0.0.1 -p 8402`
or YAAC/Xastir "KISS over TCP". Send an APRS beacon from A and watch it arrive at B.

## 7. Files and size

| File | Change | ≈ lines |
|---|---|---|
| `crates/vardefm-modem/src/header.rs` | `Kind::Unproto`, `from_u8`, tests | 25 |
| `crates/vardefm-modem/src/lib.rs` | `unproto_capacity(level)` helper | 10 |
| `crates/vardefm-link/src/lib.rs` | queue, CSMA in `poll` Idle, `tx_done`, `on_burst` dispatch, pack/parse, `LinkEvent::Unproto` | 130 |
| `crates/vardefm-link/src/tests.rs` | link tests above | 150 |
| `crates/vardefm-tnc/src/kiss.rs` (new) | codec + `KissPort` listener, tests | 220 |
| `crates/vardefm-tnc/src/server.rs` | `HostIn::Kiss` variant | 5 |
| `crates/vardefm-tnc/src/station.rs` | `HostOut::Kiss`, `kiss_in` | 15 |
| `crates/vardefm-tnc/src/simpair.rs` | two KISS ports, plumbing, `addrs` for tests | 40 |
| `crates/vardefm-tnc/src/radio.rs` | KISS port, `HostIn::Kiss` arm in `RadioEngine::step` | 25 |
| `crates/vardefm-cli/src/main.rs` | `--kiss-port`, `--a-/--b-kiss-port`, banner | 30 |
| `crates/vardefm-tnc/tests/simpair.rs` | integration tests | 120 |
| `docs/PROTOCOL.md` | §2.3 kind 5, new §3.4 Unproto, new §4.5 KISS port | 60 |
| `README.md`, `CHANGELOG.md` | short notes | 15 |

In total that is about 300 lines of production code, 400 of tests and 75 of docs. The
modem, receiver, DSP and FEC crates get no logic changes, so the performance gates
(`level_gate`, `fer_gate`) aren't affected.
