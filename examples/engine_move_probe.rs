//! Supplied-boundary engine caller JSONL protocol; no scene results are invented.
use movement_lab::contact_finalizer::FinalizerState;
use movement_lab::contact_reports::ContactReports;
use movement_lab::engine_move::{
    EngineCallbacks, EngineEvent, EngineMoveGlobals, EngineMoveState, run,
};
use serde_json::{Value, json};
use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, BufWriter, Write};

fn hex<const N: usize>(values: [f32; N]) -> String {
    values
        .into_iter()
        .flat_map(f32::to_le_bytes)
        .map(|b| format!("{b:02x}"))
        .collect()
}
fn record(v: Option<&Value>) -> Result<ContactReports, String> {
    let text = v.map_or(Ok("55".repeat(85)), |v| {
        v.as_str()
            .map(str::to_owned)
            .ok_or_else(|| "record must be hex".to_owned())
    })?;
    if text.len() != 170 || !text.is_ascii() {
        return Err("record must contain 85 bytes".into());
    }
    let bytes: Vec<u8> = (0..85)
        .map(|i| {
            u8::from_str_radix(&text[i * 2..i * 2 + 2], 16)
                .map_err(|_| "invalid record hex".to_owned())
        })
        .collect::<Result<_, _>>()?;
    Ok(ContactReports::from_record(
        bytes.as_slice().try_into().unwrap(),
    ))
}
fn number(v: &Value, key: &str, default: f32) -> Result<f32, String> {
    let x = v.get(key).map_or(Ok(default), |v| {
        v.as_f64()
            .map(|x| x as f32)
            .ok_or(format!("{key} must be numeric"))
    })?;
    if !x.is_finite() {
        return Err(format!("{key} must be finite f32"));
    }
    Ok(x)
}
fn byte(v: &Value, key: &str, default: u8) -> Result<u8, String> {
    v.get(key).map_or(Ok(default), |v| {
        if let Some(b) = v.as_bool() {
            Ok(u8::from(b))
        } else {
            v.as_u64()
                .and_then(|n| u8::try_from(n).ok())
                .ok_or(format!("{key} must be bool or byte"))
        }
    })
}
fn vector(v: &Value, key: &str, default: [f32; 4]) -> Result<[f32; 4], String> {
    let Some(v) = v.get(key) else {
        return Ok(default);
    };
    let a = v
        .as_array()
        .filter(|a| a.len() == 4)
        .ok_or(format!("{key} must have four lanes"))?;
    let mut out = [0.; 4];
    for i in 0..4 {
        out[i] = a[i].as_f64().ok_or(format!("{key} must be numeric"))? as f32;
        if !out[i].is_finite() {
            return Err(format!("{key} must be finite f32"));
        }
    }
    Ok(out)
}
fn snapshot(kind: &str, s: &EngineMoveState, request: [f32; 4]) -> Value {
    json!({"kind":kind,"request":hex(request),"flags":[u8::from(s.contact.late_flag),s.split_vertical],"timers":hex([s.contact.age,s.contact.timer]),"scalar":hex([s.contact.scalar]),"record":s.contact.reports.to_record().into_iter().map(|b|format!("{b:02x}")).collect::<String>()})
}
struct Boundary {
    input: Value,
    trace: Vec<Value>,
}
impl Boundary {
    fn effect(&mut self, kind: &str, s: &mut EngineMoveState, request: &mut [f32; 4]) {
        self.trace.push(snapshot(kind, s, *request));
        let Some(e) = self.input.get(format!("{kind}_effects")) else {
            return;
        };
        if e.get("request").is_some() {
            *request = vector(e, "request", [0.; 4]).unwrap();
        }
        if e.get("record").is_some() {
            s.contact.reports = record(e.get("record")).unwrap();
        }
        if e.get("flag890").is_some() {
            s.contact.late_flag = byte(e, "flag890", 0).unwrap() != 0;
        }
        if e.get("flag891").is_some() {
            s.split_vertical = byte(e, "flag891", 0).unwrap();
        }
        if e.get("timer82c").is_some() {
            s.contact.age = number(e, "timer82c", 0.).unwrap();
        }
        if e.get("timer834").is_some() {
            s.contact.timer = number(e, "timer834", 0.).unwrap();
        }
        if e.get("scalar6f8").is_some() {
            s.contact.scalar = number(e, "scalar6f8", 0.).unwrap();
        }
    }
}
impl EngineCallbacks for Boundary {
    fn event(&mut self, e: EngineEvent) {
        self.trace.push(match e {
            EngineEvent::Admission(b) => json!({"kind":"admission","result":b}),
            EngineEvent::PriorStatePredicate(b) => json!({"kind":"ground_predicate","result":b}),
            EngineEvent::NormalPredicate(b) => json!({"kind":"normal_predicate","result":b}),
            EngineEvent::InitializeMask(m) => json!({"kind":"initialize_mask","value":hex(m)}),
            EngineEvent::ResetReports => json!({"kind":"reset_reports"}),
        });
    }
    fn optional_adjustment(
        &mut self,
        s: &mut EngineMoveState,
        _: &mut EngineMoveGlobals,
        r: &mut [f32; 4],
        _: f32,
    ) {
        self.effect("optional_adjustment", s, r);
    }
    fn move_request(
        &mut self,
        s: &mut EngineMoveState,
        _: &mut EngineMoveGlobals,
        r: &mut [f32; 4],
        _: f32,
    ) -> [f32; 4] {
        self.effect("move", s, r);
        vector(&self.input, "actual", [0.; 4]).unwrap()
    }
    fn debug_observer(
        &mut self,
        s: &mut EngineMoveState,
        _: &mut EngineMoveGlobals,
        r: &mut [f32; 4],
        _: [f32; 4],
        _: f32,
    ) {
        self.effect("debug", s, r);
    }
    fn finalize(
        &mut self,
        s: &mut EngineMoveState,
        _: &mut EngineMoveGlobals,
        r: &mut [f32; 4],
        _: f32,
    ) {
        self.trace.push(snapshot("finalizer", s, *r));
    }
}
fn evaluate(c: Value) -> Result<Value, String> {
    // Validate supplied boundaries before invoking them, so malformed protocol
    // cannot panic inside the callback implementation.
    for kind in ["optional_adjustment", "move", "debug"] {
        if let Some(e) = c.get(format!("{kind}_effects")) {
            if !e.is_object() {
                return Err("effects must be an object".into());
            }
            vector(e, "request", [0.; 4])?;
            record(e.get("record"))?;
            for k in ["timer82c", "timer834", "scalar6f8"] {
                number(e, k, 0.)?;
            }
            for k in ["flag890", "flag891"] {
                byte(e, k, 0)?;
            }
        }
    }
    let flags = c
        .get("flags700")
        .map_or(Ok(0), |v| v.as_u64().ok_or("flags700 must be u64"))?;
    let mut state = EngineMoveState {
        contact: FinalizerState {
            reports: record(c.get("record"))?,
            working_normal: vector(&c, "normal", [0., 1., 0., 0.])?,
            other_normal: [0.; 4],
            normal_y: 0.,
            cached_y: 0.,
            defer: false,
            mask: c.get("mask740").map_or(Ok(0), |v| {
                v.as_u64()
                    .and_then(|x| u32::try_from(x).ok())
                    .ok_or("mask740 must be u32")
            })?,
            suppress: byte(&c, "flag749", 0)? != 0,
            scalar: number(&c, "scalar6f8", 0.)?,
            age: number(&c, "timer82c", 0.5)?,
            timer: number(&c, "timer834", 0.5)?,
            position_y: 0.,
            step: 0.,
            late_flag: byte(&c, "flag890", 0)? != 0,
            native_controller_present: false,
        },
        flags,
        configuration_disabled: byte(&c, "cfg8e", 0)? != 0,
        movement_disabled: byte(&c, "flag6f3", 0)? != 0,
        force_move: byte(&c, "force6f4", 0)? != 0,
        skip_request_shaping: byte(&c, "flag6f5", 0)? != 0,
        optional_adjustment_enabled: byte(&c, "cfgcf", 0)? != 0,
        extra_down_guard: byte(&c, "flag4d0", 0)? != 0,
        extra_down_value: number(&c, "value4b4", 0.)?,
        contact_offset: number(&c, "skin", 0.02)?,
        split_vertical: byte(&c, "flag891", 0)?,
        debug_enabled: byte(&c, "debug", 0)? != 0,
    };
    let mut globals = EngineMoveGlobals {
        normal_lower: number(&c, "normal_lower", 0.25881907)?,
        normal_upper: number(&c, "normal_upper", 0.99904823)?,
        projection_mask: vector(&c, "mask", [1., 0., 1., 0.])?,
        initialize_mask: byte(&c, "initialize_mask", 0)? != 0,
        first_axis: vector(&c, "axis_a", [1., 0., 0., 0.])?,
        second_axis: vector(&c, "axis_b", [0., 0., 1., 0.])?,
    };
    if globals.normal_lower <= 0. && globals.normal_upper >= 0. {
        return Err("projection interval must exclude zero".into());
    }
    let mut request = vector(&c, "request", [0.; 4])?;
    let dt = number(&c, "dt", 0.)?;
    vector(&c, "actual", [0.; 4])?;
    let mut boundary = Boundary {
        input: c,
        trace: Vec::new(),
    };
    run(&mut state, &mut globals, &mut request, dt, &mut boundary);
    let mut out = snapshot("finalizer", &state, request);
    out.as_object_mut().unwrap().remove("kind");
    out["trace"] = json!(boundary.trace);
    Ok(out)
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args_os().collect();
    if args.len() != 3 {
        return Err("usage: engine_move_probe INPUT OUTPUT".into());
    }
    let input = BufReader::new(File::open(&args[1])?);
    let mut output = BufWriter::new(
        OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&args[2])?,
    );
    for (i, line) in input.lines().enumerate() {
        let out =
            evaluate(serde_json::from_str(&line?)?).map_err(|e| format!("line {}: {e}", i + 1))?;
        serde_json::to_writer(&mut output, &out)?;
        output.write_all(b"\n")?;
    }
    output.flush()?;
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn malformed_effect_is_rejected_without_callback_panic() {
        assert!(evaluate(json!({"move_effects":{"request":[0,1]}})).is_err());
    }
    #[test]
    fn no_request_still_finalizes_without_reset() {
        let out = evaluate(json!({"request":[0,0,0,1]})).unwrap();
        assert_eq!(out["trace"].as_array().unwrap().len(), 1);
        assert_eq!(out["record"], "55".repeat(85));
    }
    #[test]
    fn callback_zero_request_skips_reset_but_still_moves() {
        let out=evaluate(json!({"request":[1,0,0,0],"cfgcf":true,"optional_adjustment_effects":{"request":[0,0,0,0]}})).unwrap();
        let trace = out["trace"].as_array().unwrap();
        assert!(trace.iter().any(|x| x["kind"] == "move"));
        assert!(!trace.iter().any(|x| x["kind"] == "reset_reports"));
        assert_eq!(out["record"], "55".repeat(85));
    }
}
