//! Strict JSONL probe for the supplied mesh-contact publication fragment.
//! Exact output float bytes retain signed zero; cache_bounds_bits transports f64.
//! geometry_word/shape_bounds are legacy fixture keys for the supplied object word
//! and object.v48 bounds; the protocol assigns no shape/class identity to them.
use movement_lab::mesh_publication as m;
use movement_lab::precise_capsule::Triangle;
use serde::de::DeserializeOwned;
use serde_json::{Value, json};
use std::io::{self, BufRead};

fn field<T: DeserializeOwned>(c: &Value, key: &str) -> Result<T, ()> {
    serde_json::from_value(c.get(key).ok_or(())?.clone()).map_err(|_| ())
}
fn optional<T: DeserializeOwned>(c: &Value, key: &str, default: T) -> Result<T, ()> {
    match c.get(key) {
        None => Ok(default),
        Some(x) => serde_json::from_value(x.clone()).map_err(|_| ()),
    }
}
fn vector<const N: usize>(c: &Value, key: &str, default: Option<[f32; N]>) -> Result<[f32; N], ()> {
    // Decimal alternatives are still checked when exact bits take precedence.
    let numeric = if let Some(v) = c.get(key) {
        let numbers: Vec<f32> = serde_json::from_value(v.clone()).map_err(|_| ())?;
        let a = <[f32; N]>::try_from(numbers).map_err(|_| ())?;
        if !a.into_iter().all(f32::is_finite) {
            return Err(());
        }
        Some(a)
    } else {
        None
    };
    let bitkey = format!("{key}_bits");
    if let Some(v) = c.get(&bitkey) {
        let bits: Vec<u32> = serde_json::from_value(v.clone()).map_err(|_| ())?;
        return Ok(<[u32; N]>::try_from(bits)
            .map_err(|_| ())?
            .map(f32::from_bits));
    }
    numeric.or(default).ok_or(())
}
fn scalar(c: &Value, key: &str, default: f32) -> Result<f32, ()> {
    let numeric: f32 = optional(c, key, default)?;
    if !numeric.is_finite() {
        return Err(());
    }
    let bitkey = format!("{key}_bits");
    if c.get(&bitkey).is_some() {
        Ok(f32::from_bits(field(c, &bitkey)?))
    } else {
        Ok(numeric)
    }
}
fn bounds(c: &Value) -> Result<[f64; 6], ()> {
    let numeric = if c.get("cache_bounds").is_some() {
        let a: [f64; 6] = field(c, "cache_bounds")?;
        if !a.into_iter().all(f64::is_finite) {
            return Err(());
        }
        Some(a)
    } else {
        None
    };
    if c.get("cache_bounds_bits").is_some() {
        let bits: [u64; 6] = field(c, "cache_bounds_bits")?;
        Ok(bits.map(f64::from_bits))
    } else {
        numeric.ok_or(())
    }
}
fn hex(x: f32) -> String {
    x.to_le_bytes()
        .into_iter()
        .map(|b| format!("{b:02x}"))
        .collect()
}
fn evaluate(v: Value) -> Result<Value, ()> {
    let c = v.get("input").unwrap_or(&v);
    if !c.is_object() {
        return Err(());
    }
    let triangles: Vec<Triangle> = field(c, "triangles")?;
    let mut state = m::State {
        flags: field(c, "flags")?,
        down_normal: vector(c, "down_normal", Some([0.2, 0.3, 0.4]))?,
        side_normal: vector(c, "side_normal", Some([0.5, 0.6, 0.7]))?,
        triangle_min: scalar(c, "triangle_min", -9.)?,
        triangle_height: scalar(c, "triangle_height", 8.)?,
        bound_height: scalar(c, "bound_height", 7.)?,
    };
    let input = m::Input {
        pass_id: field(c, "pass_id")?,
        object_word: field(c, "geometry_word")?,
        nonwalkable_mode: field(c, "nonwalkable_mode")?,
        triangle_index: field(c, "triangle_index")?,
        triangles: &triangles,
        up: vector(c, "up", None)?,
        cache_bounds: bounds(c)?,
        object_bounds: vector(c, "shape_bounds", None)?,
        prevent_ceiling_slide: field(c, "prevent_ceiling_slide")?,
    };
    let outcome = match m::publish(&mut state, &input) {
        Ok(r) => r,
        Err(e) => return Ok(json!({"error":format!("{e:?}")})),
    };
    let events = outcome
        .bounds_request
        .map(|margin| json!({"kind":"shape_bounds","margin":hex(margin)}))
        .into_iter()
        .collect::<Vec<_>>();
    Ok(json!({
        "flags":state.flags,"backface":u8::from(outcome.backface),"events":events,
        "down_normal":state.down_normal.map(hex),"side_normal":state.side_normal.map(hex),
        "triangle_min":hex(state.triangle_min),"triangle_height":hex(state.triangle_height),"bound_height":hex(state.bound_height)
    }))
}
fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.expect("stdin");
        if !line.trim().is_empty() {
            let value = serde_json::from_str(&line)
                .map_err(|_| ())
                .and_then(evaluate)
                .unwrap_or_else(|_| json!({"error":"InvalidInput"}));
            println!("{value}");
        }
    }
}
