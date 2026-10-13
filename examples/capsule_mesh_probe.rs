//! JSONL probe for supplied private arbitrary-triangle query fixtures.
use movement_lab::capsule_mesh as m;
use serde::de::DeserializeOwned;
use serde_json::{Value, json};
use std::io::{self, BufRead};

fn invalid() -> Value {
    json!({"error":"InvalidInput"})
}
fn field<T: DeserializeOwned>(v: &Value, key: &str) -> Result<T, ()> {
    serde_json::from_value(v.get(key).ok_or(())?.clone()).map_err(|_| ())
}
fn flag(v: &Value, key: &str, default: bool) -> Result<bool, ()> {
    match v.get(key) {
        None => Ok(default),
        Some(x) => x.as_bool().ok_or(()),
    }
}
fn number(v: &Value, key: &str, default: u32) -> Result<u32, ()> {
    match v.get(key) {
        None => Ok(default),
        Some(x) => {
            let n = u32::try_from(x.as_u64().ok_or(())?).map_err(|_| ())?;
            if key == "pass_id" && n > 2 {
                return Err(());
            }
            Ok(n)
        }
    }
}
fn float_field(v: &Value, key: &str) -> Result<f32, ()> {
    let x: f32 = field(v, key)?;
    if !x.is_finite()
        || (x != 0. && !x.is_normal())
        || (["radius", "height"].contains(&key) && (x < 0. || x > 10_000.))
    {
        return Err(());
    }
    Ok(x)
}
fn float_vector<const N: usize>(v: &Value, key: &str) -> Result<[f32; N], ()> {
    let values: Vec<f32> = field(v, key)?;
    let x: [f32; N] = values.try_into().map_err(|_| ())?;
    if !x
        .into_iter()
        .all(|v| v.is_finite() && (v == 0. || v.is_normal()))
    {
        return Err(());
    }
    if ["a", "b", "direction", "quaternion"].contains(&key)
        && x.into_iter().any(|v| v.abs() > 10_000.)
    {
        return Err(());
    }
    if key == "direction"
        && N == 3
        && (((x[0] * x[0] + x[1] * x[1]) + x[2] * x[2]) - 1.).abs() > 0.001
    {
        return Err(());
    }
    if key == "quaternion"
        && N == 4
        && ((((x[0] * x[0] + x[1] * x[1]) + x[2] * x[2]) + x[3] * x[3]) - 1.).abs() > 0.001
    {
        return Err(());
    }
    Ok(x)
}
fn double_vector(v: &Value, key: &str) -> Result<[f64; 3], ()> {
    let plain: Option<[f64; 3]> = if v.get(key).is_some() {
        Some(field(v, key)?)
    } else {
        None
    };
    if plain.is_some_and(|x| !x.into_iter().all(f64::is_finite)) {
        return Err(());
    }
    let bits_key = format!("{key}_bits");
    if v.get(&bits_key).is_some() {
        let bits: [u64; 3] = field(v, &bits_key)?;
        let x = bits.map(f64::from_bits);
        if !x.into_iter().all(f64::is_finite) {
            return Err(());
        }
        Ok(x)
    } else {
        plain.ok_or(())
    }
}
fn run_inner(v: Value) -> Result<Value, ()> {
    let c = v.get("input").filter(|x| x.is_object()).ok_or(())?;
    let kind = v.get("kind").and_then(Value::as_str).ok_or(())?;
    if kind == "witness" {
        let a: [f32; 4] = float_vector(c, "segment_a")?;
        let b: [f32; 4] = float_vector(c, "segment_b")?;
        let ta: [f32; 4] = float_vector(c, "tri_a")?;
        let tb: [f32; 4] = float_vector(c, "tri_b")?;
        let tc: [f32; 4] = float_vector(c, "tri_c")?;
        if [a, b, ta, tb, tc].into_iter().any(|v| v[3] != 0.) {
            return Err(());
        }
        let r = movement_lab::segment_triangle::witness_packed(a, b, [ta, tb, tc]).ok_or(())?;
        let distance_bits = [r.distance.to_bits(); 4];
        return Ok(
            json!({"distance_bits":distance_bits,"segment_bits":r.segment.map(f32::to_bits),"triangle_bits":r.triangle.map(f32::to_bits)}),
        );
    }
    if kind == "bounds" {
        let a = float_vector(c, "a")?;
        let b = float_vector(c, "b")?;
        let d = float_vector(c, "direction")?;
        let r = float_field(c, "radius")?;
        let distance = float_field(c, "distance")?;
        let obb = movement_lab::capsule_bounds::capsule_obb(a, b, r);
        let swept = movement_lab::capsule_bounds::swept_obb(obb, d, distance);
        let bits = |b: movement_lab::capsule_bounds::Bounds| {
            b.axes
                .into_iter()
                .flatten()
                .chain(b.center)
                .chain(b.extents)
                .map(f32::to_bits)
                .collect::<Vec<_>>()
        };
        return Ok(json!({"capsule_obb_bits":bits(obb),"swept_obb_bits":bits(swept)}));
    }
    let tris: Vec<movement_lab::precise_capsule::Triangle> = field(c, "triangles")?;
    if tris
        .iter()
        .flatten()
        .flatten()
        .any(|x| !x.is_finite() || (*x != 0. && !x.is_normal()))
    {
        return Err(());
    }
    Ok(match v.get("kind").and_then(Value::as_str).ok_or(())? {
        "kernel" => match m::triangle_sweep(
            &m::AxialTriangleInput {
                triangles: &tris,
                capsule_a: float_vector(c, "a")?,
                capsule_b: float_vector(c, "b")?,
                radius: float_field(c, "radius")?,
                direction: float_vector(c, "direction")?,
                max_distance: float_field(c, "max_distance")?,
                cached_index: number(c, "cached_index", 0)?,
                flags: u16::try_from(number(c, "flags", 0x503)?).map_err(|_| ())?,
                double_sided: flag(c, "double_sided", false)?,
            },
            match c.get("bounds") {
                None => None,
                Some(b) => {
                    let axes: [[f32; 3]; 3] = field(b, "axes")?;
                    if axes.into_iter().flatten().any(|x| !x.is_finite()) {
                        return Err(());
                    }
                    Some(movement_lab::capsule_bounds::Bounds {
                        axes,
                        center: float_vector(b, "center")?,
                        extents: float_vector(b, "extents")?,
                    })
                }
            },
        ) {
            Err(e) => json!({"error":format!("{e:?}")}),
            Ok(result) => {
                if let Some(r) = result.hit {
                    let mut value = json!({"hit":true,"index":r.index,"flags":r.flags,"normal":r.normal,"distance":r.distance,"aux":r.aux});
                    if let Some(p) = r.point {
                        value["point"] = json!(p);
                    }
                    value
                } else {
                    json!({"hit":false,"index_write":result.index_write.unwrap_or(0x55555555)})
                }
            }
        },
        "adapter" => {
            let ids: Vec<u32> = field(c, "ids")?;
            if ids.len() != tris.len() {
                return Err(());
            }
            let start = number(c, "triangle_offset", 0)?;
            let count = if c.get("count").is_none() {
                u32::try_from(tris.len())
                    .map_err(|_| ())?
                    .checked_sub(start)
                    .ok_or(())?
            } else {
                number(c, "count", 0)?
            };
            if (start as usize)
                .checked_add(count as usize)
                .is_none_or(|end| end > tris.len())
            {
                return Err(());
            }
            match m::cached_mesh_query(&m::CachedMeshInput {
                triangles: &tris,
                original_ids: &ids,
                quaternion: float_vector(c, "quaternion")?,
                radius: float_field(c, "radius")?,
                height: float_field(c, "height")?,
                center: double_vector(c, "center")?,
                offset: double_vector(c, "offset")?,
                direction: float_vector(c, "direction")?,
                max_distance: float_field(c, "max_distance")?,
                triangle_offset: start,
                triangle_count: count,
                cached_index: number(c, "cached_index", 0)?,
                pass_id: number(c, "pass_id", 2)?,
                overlap: flag(c, "overlap", true)?,
                precise: flag(c, "precise", true)?,
            }) {
                Err(e) => json!({"error":format!("{e:?}")}),
                Ok(r) => match r.hit {
                    None => json!({"hit":false,"cached_index":r.cached_index}),
                    Some(h) => {
                        json!({"hit":true,"point":h.world_point,"point_bits":h.world_point.map(f64::to_bits),"normal":h.normal,"distance":h.distance,
                        "global_index":h.global_index,"original_index":h.original_index,"cached_index":r.cached_index})
                    }
                },
            }
        }
        _ => return Err(()),
    })
}
fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.expect("stdin");
        if !line.trim().is_empty() {
            let result = serde_json::from_str(&line)
                .map_err(|_| ())
                .and_then(run_inner)
                .unwrap_or_else(|_| invalid());
            println!("{result}");
        }
    }
}
