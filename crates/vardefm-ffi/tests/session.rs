//! Two modems driven only through the C ABI complete a session.
//! The keyed station advances its clock with `vfm_advance` only.

use std::collections::VecDeque;
use std::ffi::{c_char, c_void, CStr, CString};
use std::sync::{Arc, Mutex};
use vardefm::{
    vfm_advance, vfm_command, vfm_config_default, vfm_free, vfm_new, vfm_rx_audio, vfm_send, vfm_tx_played,
    VfmCallbacks, VfmConfig, VfmModem, VfmRxReport, VfmTxReport,
};

const CHUNK: usize = 480;
const KEYUP: usize = 5;

#[derive(Default)]
struct Side {
    queue: VecDeque<f32>,
    queued: u64,
    played: u64,
    ptt: bool,
    keyed_for: usize,
    /// True if every `tx_audio` so far arrived while PTT was already on.
    ptt_before_audio: bool,
    audio_bursts: usize,
    lines: Vec<String>,
    data: Vec<u8>,
    tx_reports: usize,
    rx_reports: usize,
}

type Shared = Arc<Mutex<Side>>;

extern "C" fn on_tx_audio(ctx: *mut c_void, samples: *const f32, n: usize) {
    let side = unsafe { &*(ctx as *const Shared) };
    let mut s = side.lock().unwrap();
    if s.audio_bursts == 0 {
        s.ptt_before_audio = s.ptt;
    } else {
        s.ptt_before_audio &= s.ptt;
    }
    s.audio_bursts += 1;
    let slice = unsafe { std::slice::from_raw_parts(samples, n) };
    s.queue.extend(slice.iter().copied());
    s.queued += n as u64;
}

extern "C" fn on_tx_flush(ctx: *mut c_void) {
    let side = unsafe { &*(ctx as *const Shared) };
    let mut s = side.lock().unwrap();
    s.played = s.queued;
    s.queue.clear();
}

extern "C" fn on_ptt(ctx: *mut c_void, on: bool) {
    let side = unsafe { &*(ctx as *const Shared) };
    let mut s = side.lock().unwrap();
    if on && !s.ptt {
        s.keyed_for = 0;
    }
    s.ptt = on;
}

extern "C" fn on_host_line(ctx: *mut c_void, line: *const c_char) {
    let side = unsafe { &*(ctx as *const Shared) };
    let text = unsafe { CStr::from_ptr(line) }.to_string_lossy().into_owned();
    side.lock().unwrap().lines.push(text);
}

extern "C" fn on_host_data(ctx: *mut c_void, data: *const u8, n: usize) {
    let side = unsafe { &*(ctx as *const Shared) };
    let bytes = unsafe { std::slice::from_raw_parts(data, n) };
    side.lock().unwrap().data.extend_from_slice(bytes);
}

extern "C" fn on_tx_report(ctx: *mut c_void, _r: *const VfmTxReport) {
    let side = unsafe { &*(ctx as *const Shared) };
    side.lock().unwrap().tx_reports += 1;
}

extern "C" fn on_rx_report(ctx: *mut c_void, _r: *const VfmRxReport) {
    let side = unsafe { &*(ctx as *const Shared) };
    side.lock().unwrap().rx_reports += 1;
}

fn cbs() -> VfmCallbacks {
    VfmCallbacks {
        tx_audio: Some(on_tx_audio),
        tx_flush: Some(on_tx_flush),
        ptt: Some(on_ptt),
        host_line: Some(on_host_line),
        host_data: Some(on_host_data),
        log: None,
        tx_report: Some(on_tx_report),
        rx_report: Some(on_rx_report),
    }
}

fn default_cfg(seed: u32) -> VfmConfig {
    let mut cfg = unsafe {
        let mut d = std::mem::zeroed();
        vfm_config_default(&mut d);
        d
    };
    cfg.seed = seed;
    cfg.decode_threads = 1;
    cfg
}

fn open(side: &Shared, seed: u32) -> *mut VfmModem {
    let cfg = default_cfg(seed);
    let c = cbs();
    let m = unsafe { vfm_new(&cfg, &c, side as *const Shared as *mut c_void) };
    assert!(!m.is_null());
    m
}

fn cmd(m: *mut VfmModem, line: &str) {
    let c = CString::new(line).unwrap();
    unsafe { vfm_command(m, c.as_ptr()) };
}

fn tick(side: &Shared, m: *mut VfmModem) -> Vec<f32> {
    let mut out = vec![0.0f32; CHUNK];
    {
        let mut s = side.lock().unwrap();
        if s.ptt {
            s.keyed_for += 1;
        }
        for o in out.iter_mut() {
            let Some(x) = s.queue.pop_front() else { break };
            s.played += 1;
            *o = x;
        }
        unsafe { vfm_tx_played(m, s.played) };
        if !(s.ptt && s.keyed_for > KEYUP) {
            out.fill(0.0);
        }
    }
    out
}

#[test]
fn session_through_c_abi_with_advance_while_keyed() {
    let sides = [Shared::default(), Shared::default()];
    let modems = [open(&sides[0], 1), open(&sides[1], 2)];
    cmd(modems[0], "MYCALL LA1AAA");
    cmd(modems[0], "CONNECT LA1AAA LA2BBB");
    cmd(modems[1], "MYCALL LA2BBB");
    cmd(modems[1], "LISTEN ON");

    let to_a: Vec<u8> = (0..1500u32).map(|i| (i * 7) as u8).collect();
    let to_b: Vec<u8> = (0..1000u32).map(|i| (i * 13 + 5) as u8).collect();
    let mut air = [vec![0.0f32; CHUNK], vec![0.0f32; CHUNK]];
    let (mut sent_b, mut sent_a, mut disc) = (false, false, false);

    for _ in 0..60_000 {
        for i in 0..2 {
            let keyed = sides[i].lock().unwrap().ptt;
            if i == 0 && keyed {
                unsafe { vfm_advance(modems[i], CHUNK as u64) };
            } else {
                let heard = air[1 - i].clone();
                unsafe { vfm_rx_audio(modems[i], heard.as_ptr(), heard.len()) };
            }
        }
        for i in 0..2 {
            air[i] = tick(&sides[i], modems[i]);
        }

        if !sent_b && sides[1].lock().unwrap().lines.iter().any(|l| l.starts_with("CONNECTED")) {
            unsafe { vfm_send(modems[1], to_a.as_ptr(), to_a.len()) };
            sent_b = true;
        }
        if !sent_a && sides[0].lock().unwrap().data.len() >= to_a.len() {
            unsafe { vfm_send(modems[0], to_b.as_ptr(), to_b.len()) };
            sent_a = true;
        }
        if !disc
            && sides[1].lock().unwrap().data.len() >= to_b.len()
            && sides[0].lock().unwrap().lines.last().is_some_and(|l| l == "BUFFER 0")
        {
            cmd(modems[0], "DISCONNECT");
            disc = true;
        }
        let a = sides[0].lock().unwrap();
        let b = sides[1].lock().unwrap();
        if a.lines.iter().any(|l| l == "DISCONNECTED") && b.lines.iter().any(|l| l == "DISCONNECTED") {
            assert_eq!(a.data, to_a);
            assert_eq!(b.data, to_b);
            assert!(a.ptt_before_audio, "PTT must be keyed before tx_audio");
            assert!(a.tx_reports > 0, "expected TX reports");
            assert!(b.rx_reports > 0, "expected RX reports");
            drop(a);
            drop(b);
            unsafe {
                vfm_free(modems[0]);
                vfm_free(modems[1]);
            }
            return;
        }
    }
    panic!("session did not finish");
}

/// `vfm_command` from inside `host_line` is queued and drained by the outer call.
#[test]
fn command_from_inside_host_line_is_queued() {
    struct Nest {
        side: Shared,
        modem: *mut VfmModem,
        /// Nest VERSION only once, or the VERSION reply's OK would loop forever.
        nested: Mutex<bool>,
    }

    extern "C" fn nested_line(ctx: *mut c_void, line: *const c_char) {
        let nest = unsafe { &*(ctx as *const Nest) };
        let text = unsafe { CStr::from_ptr(line) }.to_string_lossy().into_owned();
        nest.side.lock().unwrap().lines.push(text.clone());
        if text == "OK" {
            let mut once = nest.nested.lock().unwrap();
            if !*once {
                *once = true;
                drop(once);
                let v = CString::new("VERSION").unwrap();
                unsafe { vfm_command(nest.modem, v.as_ptr()) };
            }
        }
    }

    let side = Shared::default();
    let nest = Box::new(Nest { side: side.clone(), modem: std::ptr::null_mut(), nested: Mutex::new(false) });
    let nest_ptr = Box::into_raw(nest);
    let mut c = cbs();
    c.host_line = Some(nested_line);
    let cfg = default_cfg(9);
    let m = unsafe { vfm_new(&cfg, &c, nest_ptr as *mut c_void) };
    assert!(!m.is_null());
    unsafe { (*nest_ptr).modem = m };

    cmd(m, "MYCALL LA1AAA");
    let lines = side.lock().unwrap().lines.clone();
    assert!(lines.iter().any(|l| l == "OK"), "{lines:?}");
    assert!(lines.iter().any(|l| l.starts_with("VERSION ")), "{lines:?}");

    unsafe {
        vfm_free(m);
        drop(Box::from_raw(nest_ptr));
    }
}
