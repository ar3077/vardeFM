# Embedding VardeFM

VardeFM can be linked into another application as a static library. The app owns
audio and PTT; the library is stepped from the app's audio callbacks and has no
worker thread of its own (decode may use scoped threads inside a receive step).

## Crates

| Crate | Role |
|---|---|
| `vardefm-engine` | Station, host protocol, `RadioEngine`, `AudioSink` / `Ptt` traits |
| `vardefm-ffi` | C ABI (`libvardefm.a`) and header in `include/CVardeFM/` |

`vardefm-ffi` depends only on the DSP/modem/link/engine stack. It does not pull
in sound-card, serial, HID, TCP or WAV recording crates.

## C API

Header: [`crates/vardefm-ffi/include/CVardeFM/vardefm.h`](../crates/vardefm-ffi/include/CVardeFM/vardefm.h).
Regenerate after changing the Rust FFI surface:

```sh
cbindgen -c crates/vardefm-ffi/cbindgen.toml -o crates/vardefm-ffi/include/CVardeFM/vardefm.h crates/vardefm-ffi
```

```c
VfmConfig cfg;
vfm_config_default(&cfg);
cfg.decode_threads = 1;   /* decode on the calling thread */

VfmCallbacks cbs = {
    .tx_audio = on_tx_audio,   /* one burst, 48 kHz mono float */
    .tx_flush = on_tx_flush,
    .ptt = on_ptt,             /* may block */
    .host_line = on_host_line, /* host protocol lines (docs/PROTOCOL.md) */
    .host_data = on_host_data,
    .log = on_log,             /* optional */
    .tx_report = on_tx_report, /* optional structured TX stats */
    .rx_report = on_rx_report, /* optional structured RX stats */
};

VfmModem *m = vfm_new(&cfg, &cbs, ctx);
vfm_command(m, "MYCALL N0CALL");
vfm_command(m, "LISTEN ON");

/* Each audio callback: */
vfm_rx_audio(m, samples, n);          /* also advances the sample clock */
vfm_tx_played(m, total_played);       /* after draining the TX queue */

/* While keyed with no RX audio from the radio: */
vfm_advance(m, n);                    /* keep the clock running */

vfm_free(m);
```

### Contract

- **One call at a time.** Callbacks fire on the calling thread before the call returns.
- **Re-entrancy.** `vfm_command` and `vfm_send` may be called from inside a callback;
  they are queued and drained by the outer call. Any other re-entrant entry point is
  ignored and logged.
- **Sample clock.** Time advances only by samples given to `vfm_rx_audio` plus
  `vfm_advance`. The PTT tail and watchdog use this clock. The caller must keep it
  running while keyed even when the radio returns no RX audio (common on half-duplex
  network paths). Feeding silence via `vfm_rx_audio` is equivalent; `vfm_advance` is
  the explicit form.
- **`vfm_tx_played`.** Reports how many TX samples have left the host's play-out so
  far (absolute total, clamped to what was queued). Un-key happens on the next
  `vfm_rx_audio` / `vfm_advance` after the burst has drained plus `txtail_s`.
- **PTT may block.** `ptt(ctx, false)` may wait for the radio to confirm un-key. That
  only pauses the sample clock; it is safe.
- **Panic.** A Rust panic is caught at the boundary: the modem flushes, calls
  `ptt(false)`, logs, and ignores later calls. It never unwinds into the host.

## Apple XCFramework

```sh
./packaging/apple/build-xcframework.sh
```

Produces `packaging/apple/build/VardeFM.xcframework` with slices for
`ios-arm64`, `ios-arm64_x86_64-simulator` and `macos-arm64_x86_64`, plus a zip and
SwiftPM checksum. The local Swift package is [`packaging/apple/Package.swift`](../packaging/apple/Package.swift)
(`CVardeFM` binary target). Build the XCFramework before `swift test` or opening
an Xcode project that depends on it:

```sh
(cd packaging/apple && swift test)
```

Release workflows upload `vardefm_<version>_apple_xcframework.zip` alongside the
desktop archives.
