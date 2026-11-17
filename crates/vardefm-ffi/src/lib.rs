//! C ABI for embedding the VardeFM radio engine.
//!
//! The modem is thread-free apart from scoped decode workers inside a receive
//! step. Call one function at a time; callbacks fire on the calling thread before
//! the call returns. The sample clock is the samples given to [`vfm_rx_audio`]
//! plus [`vfm_advance`]: the caller must keep it running while keyed.
//!
//! See `docs/EMBEDDING.md` and the header in `include/CVardeFM/vardefm.h`.
//!
//! # Safety
//!
//! Every `extern "C"` entry point takes raw pointers from the host. Unless noted
//! otherwise, `cfg` / `cbs` / `line` / audio buffers must be valid for the call,
//! `m` must be a pointer from [`vfm_new`] (or null, which is ignored), and the
//! host must not call in from two threads at once.

#![allow(clippy::missing_safety_doc)]

use std::ffi::{c_char, c_void, CStr, CString};
use std::io;
use std::mem;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use vardefm_engine::{AudioSink, EngineConfig, HostIn, HostOut, Ptt, RadioEngine, Report, RATE};
use vardefm_modem::{Header, Modem, OfdmParams, LEVELS};

/// Narrow (mic/speaker) mode.
pub const VFM_MODE_NARROW: u8 = 0;
/// Wide (9600 data port) mode.
pub const VFM_MODE_WIDE: u8 = 1;

/// Configuration for [`vfm_new`].
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VfmConfig {
    /// [`VFM_MODE_NARROW`] or [`VFM_MODE_WIDE`].
    pub mode: u8,
    pub txdelay_s: f32,
    pub txtail_s: f32,
    pub tx_gain: f32,
    /// Highest speed level to send or request (0–6).
    pub max_level: u8,
    /// 0 = one decode thread per CPU core; 1 = decode on the calling thread.
    pub decode_threads: u8,
    /// 0 = pick a random seed; any other value is used as-is.
    pub seed: u32,
}

/// Per-burst transmit statistics.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VfmTxReport {
    pub time_s: f64,
    pub repeat: bool,
    pub kind: u8,
    pub level: u8,
    pub codewords: u8,
    pub bytes: u32,
    pub resent_bytes: u32,
    pub retry: u32,
    pub air_s: f32,
    pub net_bps: f32,
}

/// Per-burst receive statistics.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct VfmRxReport {
    pub time_s: f64,
    pub kind: u8,
    pub level: u8,
    pub codewords: u8,
    pub codewords_ok: u8,
    pub snr_db: f32,
    pub peak_dbfs: f32,
    pub rms_dbfs: f32,
    pub net_bps: f32,
}

/// Host-supplied callbacks. Optional pointers may be null.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct VfmCallbacks {
    /// One transmitted burst, 48 kHz mono float samples. Required.
    pub tx_audio: Option<extern "C" fn(ctx: *mut c_void, samples: *const f32, n: usize)>,
    /// Drop any queued transmit audio. Required.
    pub tx_flush: Option<extern "C" fn(ctx: *mut c_void)>,
    /// Key or un-key the radio. May block. Required.
    pub ptt: Option<extern "C" fn(ctx: *mut c_void, on: bool)>,
    /// One host command-port line (no CR). Required.
    pub host_line: Option<extern "C" fn(ctx: *mut c_void, line: *const c_char)>,
    /// Bytes for the host data port. Required.
    pub host_data: Option<extern "C" fn(ctx: *mut c_void, data: *const u8, n: usize)>,
    /// Diagnostic log line. Optional.
    pub log: Option<extern "C" fn(ctx: *mut c_void, line: *const c_char)>,
    /// Structured TX burst report. Optional.
    pub tx_report: Option<extern "C" fn(ctx: *mut c_void, report: *const VfmTxReport)>,
    /// Structured RX burst report. Optional.
    pub rx_report: Option<extern "C" fn(ctx: *mut c_void, report: *const VfmRxReport)>,
}

/// Opaque modem handle.
pub struct VfmModem {
    engine: RadioEngine<CallbackSink>,
    cbs: VfmCallbacks,
    ctx: *mut c_void,
    /// True while a public entry point is running: nested command/send are queued.
    busy: bool,
    pending: Vec<HostIn>,
    /// After a caught panic the modem ignores further calls.
    dead: bool,
    clock: Arc<TxClock>,
}

// The host is responsible for synchronizing access; we only promise one call at a time.
unsafe impl Send for VfmModem {}

struct TxClock {
    queued: AtomicU64,
    played: AtomicU64,
}

struct CallbackSink {
    cbs: VfmCallbacks,
    ctx: *mut c_void,
    clock: Arc<TxClock>,
}

unsafe impl Send for CallbackSink {}

impl AudioSink for CallbackSink {
    fn play(&mut self, samples: &[f32]) -> u64 {
        if let Some(f) = self.cbs.tx_audio {
            f(self.ctx, samples.as_ptr(), samples.len());
        }
        self.clock.queued.fetch_add(samples.len() as u64, Ordering::AcqRel) + samples.len() as u64
    }
    fn consumed(&self) -> u64 {
        let queued = self.clock.queued.load(Ordering::Acquire);
        self.clock.played.load(Ordering::Acquire).min(queued)
    }
    fn flush(&mut self) {
        if let Some(f) = self.cbs.tx_flush {
            f(self.ctx);
        }
        let queued = self.clock.queued.load(Ordering::Acquire);
        self.clock.played.store(queued, Ordering::Release);
    }
}

struct CallbackPtt {
    cbs: VfmCallbacks,
    ctx: *mut c_void,
}

unsafe impl Send for CallbackPtt {}

impl Ptt for CallbackPtt {
    fn set(&mut self, on: bool) -> io::Result<()> {
        if let Some(f) = self.cbs.ptt {
            f(self.ctx, on);
        }
        Ok(())
    }
}

fn kind_u8(h: &Header) -> u8 {
    h.kind as u8
}

/// Fill `cfg` with the defaults used by the desktop TNC.
#[no_mangle]
pub unsafe extern "C" fn vfm_config_default(cfg: *mut VfmConfig) {
    if cfg.is_null() {
        return;
    }
    let e = EngineConfig::default();
    *cfg = VfmConfig {
        mode: VFM_MODE_NARROW,
        txdelay_s: e.txdelay_s,
        txtail_s: e.txtail_s,
        tx_gain: e.tx_gain,
        max_level: e.max_level,
        decode_threads: 0,
        seed: 0,
    };
}

/// Create a modem. Returns null if `cfg` or `cbs` is refused (missing callbacks,
/// bad mode, etc.).
#[no_mangle]
pub unsafe extern "C" fn vfm_new(cfg: *const VfmConfig, cbs: *const VfmCallbacks, ctx: *mut c_void) -> *mut VfmModem {
    if cfg.is_null() || cbs.is_null() {
        return ptr::null_mut();
    }
    let cfg = *cfg;
    let cbs = *cbs;
    if cbs.tx_audio.is_none()
        || cbs.tx_flush.is_none()
        || cbs.ptt.is_none()
        || cbs.host_line.is_none()
        || cbs.host_data.is_none()
    {
        return ptr::null_mut();
    }
    match catch_unwind(AssertUnwindSafe(|| new_modem(cfg, cbs, ctx))) {
        Ok(Ok(m)) => Box::into_raw(m),
        _ => ptr::null_mut(),
    }
}

fn new_modem(cfg: VfmConfig, cbs: VfmCallbacks, ctx: *mut c_void) -> Result<Box<VfmModem>, io::Error> {
    let mode = match cfg.mode {
        VFM_MODE_NARROW => "narrow",
        VFM_MODE_WIDE => "wide",
        _ => return Err(io::Error::new(io::ErrorKind::InvalidInput, "mode must be narrow or wide")),
    };
    let eng_cfg = EngineConfig {
        mode: mode.into(),
        txdelay_s: cfg.txdelay_s,
        txtail_s: cfg.txtail_s,
        tx_gain: cfg.tx_gain,
        max_level: cfg.max_level,
        show_levels: false,
        digi_calls: Vec::new(),
        hop_margin_s: EngineConfig::default().hop_margin_s,
    };
    eng_cfg.check()?;
    let params = OfdmParams::by_name(mode).ok_or_else(|| io::Error::other("mode must be narrow or wide"))?;
    let mut modem = Modem::new(params, cfg.txdelay_s);
    if cfg.decode_threads > 0 {
        modem.decode_threads = cfg.decode_threads as usize;
    }
    let modem = Arc::new(modem);
    let seed = if cfg.seed == 0 { vardefm_engine::random_seed() } else { cfg.seed };
    let clock = Arc::new(TxClock { queued: AtomicU64::new(0), played: AtomicU64::new(0) });
    let engine = RadioEngine::from_config(
        &eng_cfg,
        modem,
        Box::new(CallbackPtt { cbs, ctx }),
        CallbackSink { cbs, ctx, clock: clock.clone() },
        seed,
    )?;
    Ok(Box::new(VfmModem { engine, cbs, ctx, busy: false, pending: Vec::new(), dead: false, clock }))
}

/// Flush audio, un-key, and free the modem.
#[no_mangle]
pub unsafe extern "C" fn vfm_free(m: *mut VfmModem) {
    if m.is_null() {
        return;
    }
    let mut m = Box::from_raw(m);
    let _ = catch_unwind(AssertUnwindSafe(|| {
        m.engine.unkey();
    }));
}

/// Submit one host command line (without CR). May be called from inside a callback.
#[no_mangle]
pub unsafe extern "C" fn vfm_command(m: *mut VfmModem, line: *const c_char) {
    if m.is_null() || line.is_null() {
        return;
    }
    let line = match CStr::from_ptr(line).to_str() {
        Ok(s) => s.to_owned(),
        Err(_) => return,
    };
    with_modem(m, |modem| {
        if modem.busy {
            modem.pending.push(HostIn::Line(line));
        } else {
            run_host(modem, vec![HostIn::Line(line)]);
        }
    });
}

/// Bytes for the data port. May be called from inside a callback.
#[no_mangle]
pub unsafe extern "C" fn vfm_send(m: *mut VfmModem, data: *const u8, n: usize) {
    if m.is_null() || (n > 0 && data.is_null()) {
        return;
    }
    let bytes = if n == 0 { Vec::new() } else { std::slice::from_raw_parts(data, n).to_vec() };
    with_modem(m, |modem| {
        if modem.busy {
            modem.pending.push(HostIn::Data(bytes));
        } else {
            run_host(modem, vec![HostIn::Data(bytes)]);
        }
    });
}

/// Feed received audio (48 kHz mono float) and advance the sample clock by `n`.
#[no_mangle]
pub unsafe extern "C" fn vfm_rx_audio(m: *mut VfmModem, samples: *const f32, n: usize) {
    if m.is_null() || (n > 0 && samples.is_null()) {
        return;
    }
    let chunk = if n == 0 { &[][..] } else { std::slice::from_raw_parts(samples, n) };
    with_modem(m, |modem| {
        if modem.busy {
            emit_log(modem, "vfm_rx_audio ignored: re-entrant call");
            return;
        }
        modem.busy = true;
        let out = modem.engine.step(Some(chunk), Vec::new());
        emit_outputs(modem, out);
        drain_pending(modem);
        modem.busy = false;
    });
}

/// Advance the sample clock by `samples` without feeding audio content.
///
/// Required while keyed when the radio is not delivering RX audio, so the PTT
/// tail and watchdog can fire.
#[no_mangle]
pub unsafe extern "C" fn vfm_advance(m: *mut VfmModem, samples: u64) {
    if m.is_null() {
        return;
    }
    with_modem(m, |modem| {
        if modem.busy {
            emit_log(modem, "vfm_advance ignored: re-entrant call");
            return;
        }
        modem.busy = true;
        let out = modem.engine.advance(samples, Vec::new());
        emit_outputs(modem, out);
        drain_pending(modem);
        modem.busy = false;
    });
}

/// Report how many transmit samples have been played so far (absolute total).
/// Clamped to what has been queued. Un-key happens on the next rx/advance call.
#[no_mangle]
pub unsafe extern "C" fn vfm_tx_played(m: *mut VfmModem, total_samples: u64) {
    if m.is_null() {
        return;
    }
    with_modem(m, |modem| {
        let queued = modem.clock.queued.load(Ordering::Acquire);
        modem.clock.played.store(total_samples.min(queued), Ordering::Release);
    });
}

/// Library version string, e.g. `"VardeFM 0.3.2"`.
#[no_mangle]
pub unsafe extern "C" fn vfm_version() -> *const c_char {
    static mut VERSION: *const c_char = ptr::null();
    // SAFETY: called under the one-call-at-a-time contract; initialized once.
    if VERSION.is_null() {
        let s = CString::new(vardefm_engine::VERSION).unwrap();
        VERSION = s.into_raw();
    }
    VERSION
}

/// Sample rate expected by the engine (Hz).
#[no_mangle]
pub static VFM_RATE: u32 = RATE;

/// Number of speed levels.
#[no_mangle]
pub static VFM_LEVEL_COUNT: u32 = LEVELS.len() as u32;

fn with_modem(m: *mut VfmModem, f: impl FnOnce(&mut VfmModem)) {
    let modem = unsafe { &mut *m };
    if modem.dead {
        return;
    }
    match catch_unwind(AssertUnwindSafe(|| f(modem))) {
        Ok(()) => {}
        Err(_) => {
            modem.dead = true;
            let _ = catch_unwind(AssertUnwindSafe(|| {
                modem.engine.unkey();
                emit_log(modem, "modem panicked; further calls ignored");
            }));
        }
    }
}

fn run_host(modem: &mut VfmModem, host_in: Vec<HostIn>) {
    modem.busy = true;
    let out = modem.engine.step(None, host_in);
    emit_outputs(modem, out);
    drain_pending(modem);
    modem.busy = false;
}

fn drain_pending(modem: &mut VfmModem) {
    while !modem.pending.is_empty() {
        let more = mem::take(&mut modem.pending);
        let out = modem.engine.step(None, more);
        emit_outputs(modem, out);
    }
}

fn emit_outputs(modem: &mut VfmModem, out: Vec<HostOut>) {
    for o in out {
        match o {
            HostOut::Line(l) => emit_line(modem, &l),
            HostOut::Data(d) => {
                if let Some(f) = modem.cbs.host_data {
                    f(modem.ctx, d.as_ptr(), d.len());
                }
            }
        }
    }
    let reports = mem::take(&mut modem.engine.reports);
    let logs = mem::take(&mut modem.engine.log);
    modem.engine.notices.clear();
    for r in reports {
        match r {
            Report::Tx { time_s, repeat, header, info, air_s } => {
                if let Some(f) = modem.cbs.tx_report {
                    let net_bps =
                        if header.ncw > 0 { modem.engine.station.modem().net_rate(header.level) } else { 0.0 };
                    let rep = VfmTxReport {
                        time_s,
                        repeat,
                        kind: kind_u8(&header),
                        level: header.level,
                        codewords: header.ncw,
                        bytes: info.bytes as u32,
                        resent_bytes: info.resent_bytes as u32,
                        retry: info.retry,
                        air_s,
                        net_bps,
                    };
                    f(modem.ctx, &rep);
                }
            }
            Report::Rx { time_s, header, codewords_ok, snr_db, level } => {
                if let Some(f) = modem.cbs.rx_report {
                    let net_bps =
                        if header.ncw > 0 { modem.engine.station.modem().net_rate(header.level) } else { 0.0 };
                    let rep = VfmRxReport {
                        time_s,
                        kind: kind_u8(&header),
                        level: header.level,
                        codewords: header.ncw,
                        codewords_ok: codewords_ok.min(255) as u8,
                        snr_db,
                        peak_dbfs: level.peak_dbfs,
                        rms_dbfs: level.rms_dbfs,
                        net_bps,
                    };
                    f(modem.ctx, &rep);
                }
            }
        }
    }
    for l in logs {
        emit_log(modem, &l);
    }
}

fn emit_line(modem: &mut VfmModem, line: &str) {
    if let Some(f) = modem.cbs.host_line {
        if let Ok(c) = CString::new(line) {
            f(modem.ctx, c.as_ptr());
        }
    }
}

fn emit_log(modem: &mut VfmModem, line: &str) {
    if let Some(f) = modem.cbs.log {
        if let Ok(c) = CString::new(line) {
            f(modem.ctx, c.as_ptr());
        }
    }
}
