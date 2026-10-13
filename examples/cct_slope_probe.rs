//! JSONL supplied-publication scheduler protocol; geometry is not fabricated.
use movement_lab::cct_slope::{self, Backend, Iteration, Outcome, Publication, Settings, State};
use serde_json::{Value, json};
use std::{
    fs::{File, OpenOptions},
    io::{BufRead, BufReader, BufWriter, Write},
};
fn f(v: &Value, key: &str, default: f32) -> Result<f32, String> {
    let x = v.get(key).map_or(Ok(default), |x| {
        x.as_f64().map(|x| x as f32).ok_or(format!("numeric {key}"))
    })?;
    if x.is_finite() {
        Ok(x)
    } else {
        Err(format!("nonfinite {key}"))
    }
}
fn vec3(v: &Value, key: &str, default: [f32; 3]) -> Result<[f32; 3], String> {
    let Some(a) = v.get(key) else {
        return Ok(default);
    };
    let a = a
        .as_array()
        .filter(|a| a.len() == 3)
        .ok_or(format!("three lanes {key}"))?;
    let mut r = [0.; 3];
    for i in 0..3 {
        r[i] = a[i].as_f64().ok_or("numeric lane")? as f32;
    }
    if r.into_iter().all(f32::is_finite) {
        Ok(r)
    } else {
        Err("nonfinite vector".into())
    }
}
fn b(v: &Value, key: &str, default: bool) -> Result<bool, String> {
    v.get(key)
        .map_or(Ok(default), |x| x.as_bool().ok_or(format!("boolean {key}")))
}
fn u(v: &Value, key: &str, default: u32) -> Result<u32, String> {
    v.get(key).map_or(Ok(default), |x| {
        x.as_u64()
            .and_then(|x| u32::try_from(x).ok())
            .ok_or(format!("u32 {key}"))
    })
}
fn position(v: &Value, key: &str, default: [f64; 3]) -> Result<[f64; 3], String> {
    let bitkey = format!("{key}_bits");
    let r = if let Some(bits) = v.get(&bitkey) {
        let a: [u64; 3] = serde_json::from_value(bits.clone()).map_err(|e| e.to_string())?;
        a.map(f64::from_bits)
    } else if let Some(a) = v.get(key) {
        let a = a
            .as_array()
            .filter(|x| x.len() == 3)
            .ok_or("three position lanes")?;
        [
            a[0].as_f64().ok_or("double")?,
            a[1].as_f64().ok_or("double")?,
            a[2].as_f64().ok_or("double")?,
        ]
    } else {
        default
    };
    if r.into_iter().all(f64::is_finite) {
        Ok(r)
    } else {
        Err("nonfinite position".into())
    }
}
fn hex(bytes: impl IntoIterator<Item = u8>) -> String {
    bytes.into_iter().map(|x| format!("{x:02x}")).collect()
}
fn floathex<const N: usize>(v: [f32; N]) -> String {
    hex(v.into_iter().flat_map(f32::to_le_bytes))
}
fn doublehex<const N: usize>(v: [f64; N]) -> String {
    hex(v.into_iter().flat_map(f64::to_le_bytes))
}
struct Script {
    outcomes: Vec<Value>,
    index: usize,
    trace: Vec<Value>,
    error: Option<String>,
}
impl Backend for Script {
    fn prepare(&mut self, s: &mut State, d: [f32; 3], side: [f32; 3]) {
        self.trace.push(json!({"kind":"prepare","position":doublehex(s.position),"delta":floathex(d),"side":floathex(side),"flags":s.flags}));
    }
    fn iterate(&mut self, s: &mut State, r: Iteration) -> Outcome {
        self.trace.push(json!({"kind":"iteration","position":doublehex(s.position),"delta":floathex(r.displacement),"side":floathex(r.side),"budget":r.budget,"minimum":floathex([r.minimum]),"pass_id":r.pass,"tag":s.tag,"flags":s.flags}));
        let o = self.outcomes.get(self.index).cloned().unwrap_or(json!({}));
        self.index += 1;
        let result = (|| -> Result<Outcome, String> {
            if o.get("position").is_some() || o.get("position_bits").is_some() {
                s.position = position(&o, "position", s.position)?;
            } else {
                let fraction = f(&o, "fraction", 1.)?;
                s.position = std::array::from_fn(|i| {
                    s.position[i] + f64::from(r.displacement[i] * fraction)
                });
            }
            s.flags = (s.flags & !u(&o, "clear_flags", 0)?) | u(&o, "set_flags", 0)?;
            s.down_normal = vec3(&o, "down_normal", s.down_normal)?;
            s.side_normal = vec3(&o, "side_normal", s.side_normal)?;
            s.triangle_height = f(&o, "triangle_height", s.triangle_height)?;
            s.bound_height = f(&o, "bound_height", s.bound_height)?;
            s.contact_height = f(&o, "contact_height", s.contact_height)?;
            Ok(Outcome {
                attempted: b(&o, "attempted", r.displacement.into_iter().any(|x| x != 0.))?,
                collisions: u(&o, "count", 0)?,
            })
        })();
        match result {
            Ok(o) => o,
            Err(e) => {
                self.error = Some(e);
                Outcome {
                    attempted: false,
                    collisions: 0,
                }
            }
        }
    }
}
fn evaluate(c: Value) -> Result<Value, String> {
    let mut s = State {
        position: position(&c, "center", [0.; 3])?,
        flags: u(&c, "flags", 0)?,
        tag: 0,
        down_normal: vec3(&c, "down_normal", [0., 1., 0.])?,
        side_normal: vec3(&c, "side_normal", [1., 0., 0.])?,
        triangle_height: f(&c, "triangle_height", 0.)?,
        bound_height: f(&c, "bound_height", 0.)?,
        contact_height: f(&c, "contact_height", 0.)?,
        collision_flags: 0,
    };
    let settings = Settings {
        up: vec3(&c, "up", [0., 1., 0.])?,
        step_offset: f(&c, "step_offset", 0.3)?,
        min_distance: f(&c, "min_distance", 1e-6)?,
        nonwalkable_mode: u(&c, "nonwalkable_mode", 0)?,
        standing: b(&c, "standing", false)?,
        prevent_ceiling_slide: b(&c, "prevent_ceiling_slide", false)?,
        slope_enabled: b(&c, "slope_enabled", true)?,
        slope_limit: f(&c, "slope_limit", 0.7)?,
        constrain: b(&c, "constrain", false)?,
        extent: f(&c, "extent", 0.9)?,
    };
    let displacement = vec3(&c, "displacement", [1., -1., 0.])?;
    let outcomes = c.get("outcomes").map_or(Ok(Vec::new()), |v| {
        v.as_array().cloned().ok_or("outcomes array")
    })?;
    let mut script = Script {
        outcomes,
        index: 0,
        trace: Vec::new(),
        error: None,
    };
    let common = c.get("mode").and_then(Value::as_str) == Some("common");
    let mut publication = Publication {
        elapsed: c
            .get("elapsed")
            .map_or(Ok(0.), |v| v.as_f64().ok_or("elapsed double"))?,
        pending: vec3(&c, "pending", [0.; 3])?,
        position: s.position,
        collision_flags: 0,
    };
    if common {
        cct_slope::common(
            &mut s,
            &settings,
            displacement,
            &mut publication,
            f(&c, "dt", 0.016)?,
            &mut script,
        )
    } else {
        cct_slope::split(&mut s, &settings, displacement, &mut script)
    }
    .map_err(|e| format!("{e:?}"))?;
    if let Some(e) = script.error {
        return Err(e);
    }
    let mut out = json!({"position":doublehex(s.position),"flags":s.flags,"tag":s.tag,"collision_flags":s.collision_flags,"trace":script.trace});
    if common {
        out["common"] = json!({"elapsed":doublehex([publication.elapsed]),"pending":floathex(publication.pending),"published":doublehex(publication.position),"published_flags":publication.collision_flags});
    }
    Ok(out)
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args_os().collect();
    if args.len() != 3 {
        return Err("usage: cct_slope_probe INPUT OUTPUT".into());
    }
    let input = BufReader::new(File::open(&args[1])?);
    let mut output = BufWriter::new(
        OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&args[2])?,
    );
    for (i, line) in input.lines().enumerate() {
        let value =
            evaluate(serde_json::from_str(&line?)?).map_err(|e| format!("line{}:{e}", i + 1))?;
        serde_json::to_writer(&mut output, &value)?;
        output.write_all(b"\n")?;
    }
    output.flush()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn bit_only_position_outcome_preserves_exact_double() {
        let value = 1.00000000000003f64;
        let c = json!({"center_bits":[0,0,0],"outcomes":[{"position_bits":[value.to_bits(),0,0]},{"position_bits":[value.to_bits(),0,0]},{"position_bits":[value.to_bits(),0,0]}]});
        assert_eq!(evaluate(c).unwrap()["position"], doublehex([value, 0., 0.]));
    }
    #[test]
    fn nonfinite_bits_and_malformed_outcome_fail_protocol() {
        assert!(evaluate(json!({"center_bits":[f64::INFINITY.to_bits(),0,0]})).is_err());
        assert!(evaluate(json!({"outcomes":[{"position_bits":[0,0]}]})).is_err());
        assert!(evaluate(json!({"outcomes":3})).is_err());
    }
}
