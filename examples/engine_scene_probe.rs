//! Supplied-boundary engine caller JSONL protocol; no scene results are invented.
use movement_lab::contact_finalizer::FinalizerState;
use movement_lab::contact_reports::ContactReports;
use movement_lab::engine_move::{EngineMoveGlobals, EngineMoveState, run};
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
fn evaluate(c: Value) -> Result<Value, String> {
    let flags = c
        .get("flags700")
        .map_or(Ok(0), |v| v.as_u64().ok_or("flags700 must be u64"))?;
    let mut state = EngineMoveState {
        contact: FinalizerState {
            reports: record(c.get("record"))?,
            working_normal: vector(&c, "normal", [0., 1., 0., 0.])?,
            other_normal: [0.; 4],
            normal_y: 0.,
            cached_y: number(&c, "cached_y", 0.)?,
            defer: byte(&c, "defer", 0)? != 0,
            mask: c.get("mask740").map_or(Ok(0), |v| {
                v.as_u64()
                    .and_then(|x| u32::try_from(x).ok())
                    .ok_or("mask740 must be u32")
            })?,
            suppress: byte(&c, "flag749", 0)? != 0,
            scalar: number(&c, "scalar6f8", 0.)?,
            age: number(&c, "timer82c", 0.5)?,
            timer: number(&c, "timer834", 0.5)?,
            position_y: number(&c, "position_y", 0.)?,
            step: number(&c, "step", number(&c["scene"], "step_offset", 0.)?)?,
            late_flag: byte(&c, "flag890", 0)? != 0,
            native_controller_present: true,
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

    let sc = &c["scene"];
    let vec3 = |v: &Value| -> Result<[f32; 3], String> {
        let a = v.as_array().filter(|a| a.len() == 3).ok_or("expected3")?;
        Ok([
            a[0].as_f64().ok_or("float")? as f32,
            a[1].as_f64().ok_or("float")? as f32,
            a[2].as_f64().ok_or("float")? as f32,
        ])
    };
    let center_bits: [u64; 3] =
        serde_json::from_value(sc["center_bits"].clone()).map_err(|e| e.to_string())?;
    let center = center_bits.map(f64::from_bits);
    let boxes: Vec<_> = sc["boxes"]
        .as_array()
        .ok_or("boxes")?
        .iter()
        .map(|b| {
            Ok(movement_lab::cct_scene::Collider {
                center: vec3(&b["center"])?,
                extents: vec3(&b["extents"])?,
            })
        })
        .collect::<Result<_, String>>()?;
    let settings = movement_lab::cct_policy::Settings {
        up: [0., 1., 0.],
        contact_offset: number(sc, "contact_offset", 0.02)?,
        step_offset: number(&c, "step", number(sc, "step_offset", 0.45)?)?,
        min_distance: 1e-6,
        overlap_recovery: byte(&c, "overlap", 1)? != 0,
        prevent_ceiling_slide: false,
        normalize_components: false,
        custom_response: byte(&c, "custom", 1)? != 0,
        walk_experiment: false,
        standing_on_moving: false,
        nonwalkable_mode: 0,
        slope_limit: 0.7,
    };
    let mut adapter = movement_lab::engine_scene::EngineScene {
        center,
        capsule: movement_lab::cct_scene::Capsule {
            radius: number(sc, "radius", 0.3)?,
            height: number(sc, "cylinder_height", 1.2)?,
            rotation: vector(sc, "capsule_q", [0., 0., 0., 1.])?,
        },
        boxes: &boxes,
        settings,
        behavior: byte(sc, "behavior", 4)?,
        metadata: byte(sc, "metadata", 0)? != 0,
        finalizer_globals: movement_lab::contact_finalizer::FinalizerGlobals {
            zero: vector(&c, "zero", [0.; 4])?,
            up: vector(&c, "up", [0., 1., 0., 0.])?,
            fallback: globals.first_axis,
        },
        slope_cosine: f32::from_bits(c["cosine_bits"].as_u64().ok_or("cosine_bits")? as u32),
        hits: Vec::new(),
        error: None,
        native_moves: 0,
        split_vertical: false,
        bridge_projection: std::array::from_fn(|i| globals.first_axis[i] + globals.second_axis[i]),
        adjustment: movement_lab::engine_scene::AdjustmentContext {
            value_3c8: number(&c, "value_3c8", 0.)?,
            cached_lower: number(&c, "adjustment_lower", -0.8660254)?,
            upper: number(&c, "adjustment_upper", -0.00872656144)?,
            projection_mask: vector(&c, "adjustment_mask", [1., 0., 1., 0.])?,
            default_normal: vector(&c, "zero", [0.; 4])?,
        },
    };
    run(&mut state, &mut globals, &mut request, dt, &mut adapter);
    if let Some(error) = adapter.error {
        return Err(format!("{error:?}"));
    }
    let mut out = snapshot("final", &state, request);
    out.as_object_mut().unwrap().remove("kind");
    out["position"] = json!(adapter.center.map(f64::to_bits));
    out["mask"] = json!(state.contact.mask);
    out["normal"] = json!(hex(state.contact.working_normal));
    out["other_normal"] = json!(hex(state.contact.other_normal));
    out["normal_y"] = json!(hex([state.contact.normal_y]));
    out["defer"] = json!(state.contact.defer);
    out["cached_y"] = json!(hex([state.contact.cached_y]));
    out["native_moves"] = json!(adapter.native_moves);
    Ok(out)
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args_os().collect();
    if args.len() != 3 {
        return Err("usage: engine_scene_probe INPUT OUTPUT".into());
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
