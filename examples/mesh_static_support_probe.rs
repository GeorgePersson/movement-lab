//! Strict exact-bit JSONL probe for reviewed static support plus prepared Common.
//! Object services/identities are supplied fixtures, not a live scene recovery.
use movement_lab::{
    cct_scene::Capsule, cct_slope, mesh_iteration as m, mesh_static_support as support,
    precise_capsule::Triangle,
};
use serde::de::DeserializeOwned;
use serde_json::{Value, json};
use std::io::{self, BufRead};
fn field<T: DeserializeOwned>(c: &Value, k: &str) -> Result<T, ()> {
    serde_json::from_value(c.get(k).ok_or(())?.clone()).map_err(|_| ())
}
fn optional<T: DeserializeOwned>(c: &Value, k: &str, d: T) -> Result<T, ()> {
    match c.get(k) {
        None => Ok(d),
        Some(v) => serde_json::from_value(v.clone()).map_err(|_| ()),
    }
}
fn vector<const N: usize>(c: &Value, k: &str, d: Option<[f32; N]>) -> Result<[f32; N], ()> {
    let decimal = if let Some(v) = c.get(k) {
        let a =
            <[f32; N]>::try_from(serde_json::from_value::<Vec<f32>>(v.clone()).map_err(|_| ())?)
                .map_err(|_| ())?;
        if !a.into_iter().all(f32::is_finite) {
            return Err(());
        }
        Some(a)
    } else {
        None
    };
    let key = format!("{k}_bits");
    let result = if c.get(&key).is_some() {
        <[u32; N]>::try_from(field::<Vec<u32>>(c, &key)?)
            .map_err(|_| ())?
            .map(f32::from_bits)
    } else {
        decimal.or(d).ok_or(())?
    };
    if result.into_iter().all(f32::is_finite) {
        Ok(result)
    } else {
        Err(())
    }
}
fn doubles<const N: usize>(c: &Value, k: &str, d: Option<[f64; N]>) -> Result<[f64; N], ()> {
    let decimal = if let Some(v) = c.get(k) {
        let a =
            <[f64; N]>::try_from(serde_json::from_value::<Vec<f64>>(v.clone()).map_err(|_| ())?)
                .map_err(|_| ())?;
        if !a.into_iter().all(f64::is_finite) {
            return Err(());
        }
        Some(a)
    } else {
        None
    };
    let key = format!("{k}_bits");
    let result = if c.get(&key).is_some() {
        <[u64; N]>::try_from(field::<Vec<u64>>(c, &key)?)
            .map_err(|_| ())?
            .map(f64::from_bits)
    } else {
        decimal.or(d).ok_or(())?
    };
    if result.into_iter().all(f64::is_finite) {
        Ok(result)
    } else {
        Err(())
    }
}
fn scalar(c: &Value, k: &str, d: f32) -> Result<f32, ()> {
    let decimal: f32 = optional(c, k, d)?;
    if !decimal.is_finite() {
        return Err(());
    }
    let key = format!("{k}_bits");
    let v = if c.get(&key).is_some() {
        f32::from_bits(field(c, &key)?)
    } else {
        decimal
    };
    if v.is_finite() { Ok(v) } else { Err(()) }
}
fn double(c: &Value, k: &str, d: f64) -> Result<f64, ()> {
    let decimal: f64 = optional(c, k, d)?;
    if !decimal.is_finite() {
        return Err(());
    }
    let key = format!("{k}_bits");
    let v = if c.get(&key).is_some() {
        f64::from_bits(field(c, &key)?)
    } else {
        decimal
    };
    if v.is_finite() { Ok(v) } else { Err(()) }
}
fn handles(c: &Value) -> Result<m::Handles, ()> {
    let values: Vec<Option<String>> = optional(c, "handles", vec![None, None])?;
    if values.len() != 2
        || values[0].as_deref().is_some_and(|x| x != "actor")
        || values[1].as_deref().is_some_and(|x| x != "shape")
    {
        return Err(());
    }
    Ok(m::Handles {
        actor: values[0].as_ref().map(|_| m::ActorHandle),
        shape: values[1].as_ref().map(|_| m::ShapeHandle),
    })
}
fn tri(c: &Value) -> Result<Vec<Triangle>, ()> {
    let decimal: Option<Vec<Triangle>> = match c.get("triangles") {
        None => None,
        Some(v) => Some(serde_json::from_value(v.clone()).map_err(|_| ())?),
    };
    if decimal
        .as_ref()
        .is_some_and(|t| t.iter().flatten().flatten().any(|x| !x.is_finite()))
    {
        return Err(());
    }
    let values = if c.get("triangles_bits").is_some() {
        let bits: Vec<[[u32; 3]; 3]> = field(c, "triangles_bits")?;
        if decimal.as_ref().is_some_and(|t| t.len() != bits.len()) {
            return Err(());
        }
        bits.into_iter()
            .map(|t| t.map(|p| p.map(f32::from_bits)))
            .collect()
    } else {
        decimal.ok_or(())?
    };
    if values.iter().flatten().flatten().any(|x| !x.is_finite()) {
        return Err(());
    }
    Ok(values)
}
fn hf(v: impl IntoIterator<Item = f32>) -> String {
    v.into_iter()
        .flat_map(f32::to_le_bytes)
        .map(|b| format!("{b:02x}"))
        .collect()
}
fn hd(v: impl IntoIterator<Item = f64>) -> String {
    v.into_iter()
        .flat_map(f64::to_le_bytes)
        .map(|b| format!("{b:02x}"))
        .collect()
}
fn snapshot(s: &m::Snapshot) -> Value {
    json!({"position":hd(s.position),"flags":s.flags,"tag":s.tag,"cache":s.cache,
      "down":hf(s.down),"side":hf(s.side),"heights":hf(s.heights),"contact_height":hf([s.contact_height])})
}
fn handle_values(h: m::Handles) -> Value {
    json!([h.actor.map(|_| "actor"), h.shape.map(|_| "shape")])
}
fn output(t: m::Trace) -> Value {
    let iterations: Vec<_> = t
        .iterations
        .iter()
        .map(|r| {
            json!({"pass_id":r.pass_id,"budget":r.budget,
      "delta":hf(r.delta),"side":hf(r.side),"minimum":hf([r.minimum]),"state":snapshot(&r.state)})
        })
        .collect();
    let returns: Vec<_> = t
        .returns
        .iter()
        .map(|r| {
            json!({"pass_id":r.pass_id,"attempted":r.attempted,
      "count":r.count,"state":snapshot(&r.state)})
        })
        .collect();
    let queries: Vec<_> = t
        .queries
        .iter()
        .map(|q| {
            let mut r = json!({"hit":q.result.hit.is_some(),"cache":q.result.cache});
            if let Some(h) = q.result.hit {
                let obj = r.as_object_mut().unwrap();
                obj.insert("point".into(), json!(hd(h.point)));
                obj.insert("normal".into(), json!(hf(h.normal)));
                obj.insert("distance".into(), json!(hf([h.distance])));
                obj.insert("global_index".into(), json!(h.global_index));
                obj.insert("original_index".into(), json!(h.original_index));
                obj.insert("record".into(), json!(h.record));
            }
            json!({"center":hd(q.center),"direction":hf(q.direction),"maximum":hf([q.maximum]),
          "pass_tag":q.pass_tag,"cache":q.cache,"result":r})
        })
        .collect();
    let publications: Vec<_> = t
        .publications
        .iter()
        .map(|p| {
            json!({"pass_id":p.pass_id,"index":p.index,
      "before":snapshot(&p.before),"after":snapshot(&p.after),"backface":p.backface})
        })
        .collect();
    let reports:Vec<_>=t.reports.iter().map(|r|json!({"point":hd(r.point),"normal":hf(r.normal),
      "direction":hf(r.direction),"distance":hf([r.distance]),"source_id":r.source_id,"record":r.record,
      "owner_identity":r.owner_identity,"actor":"actor","shape":"shape"})).collect();
    let semantics:Vec<_>=t.iteration_semantics.iter().map(|s|json!({"counter":s.counter,"obstacle":s.obstacle,
      "local_contact":hf(s.local_contact),"world_contact":hf(s.world_contact),"handles":handle_values(s.handles)})).collect();
    let mut result = json!({"state":snapshot(&t.state),"collision_flags":t.collision_flags,
      "iterations":iterations,"returns":returns,"queries":queries,"publications":publications,
      "reports":reports,"transforms":t.transforms.into_iter().map(hf).collect::<Vec<_>>(),"iteration_semantics":semantics});
    if let Some(c) = t.common {
        result.as_object_mut().unwrap().insert("common".into(),json!({"elapsed":hd([c.numeric.elapsed]),
          "pending":hf(c.numeric.pending),"published":hd(c.numeric.position),"published_flags":c.numeric.collision_flags,
          "actor":c.handles.actor.map(|_|"actor"),"shape":c.handles.shape.map(|_|"shape")}));
    }
    result
}
fn evaluate(v: Value) -> Result<Value, ()> {
    let c = v.get("input").unwrap_or(&v);
    if !c.is_object() {
        return Err(());
    }
    if ["outcomes", "native", "iteration_results"]
        .into_iter()
        .any(|k| c.get(k).is_some())
    {
        return Err(());
    }
    let keys = [
        "overlap",
        "precise",
        "object_word",
        "standing",
        "slope_enabled",
        "down_normal",
        "side_normal",
        "triangle_min",
        "triangle_height",
        "bound_height",
        "contact_height",
        "buffered_poses",
        "initial",
        "support",
        "previous_handles",
        "previous_velocity",
        "previous_moving",
        "mode",
        "triangles",
        "ids",
        "offset",
        "records",
        "up",
        "minimum",
        "displacement",
        "side",
        "budget",
        "pass_id",
        "radius",
        "height",
        "quaternion",
        "center",
        "cache",
        "flags",
        "elapsed",
        "pending",
        "bounds",
        "slot12_pose",
        "slot4_pose",
        "behavior",
        "step_offset",
        "nonwalkable_mode",
        "prevent_ceiling_slide",
        "slope_limit",
        "constrain",
        "extent",
        "contact_offset",
        "custom_response",
        "dt",
    ];
    let floatkeys = [
        "previous_velocity",
        "down_normal",
        "side_normal",
        "triangle_min",
        "triangle_height",
        "bound_height",
        "contact_height",
        "triangles",
        "offset",
        "up",
        "minimum",
        "displacement",
        "side",
        "radius",
        "height",
        "quaternion",
        "center",
        "elapsed",
        "pending",
        "bounds",
        "slot12_pose",
        "slot4_pose",
        "step_offset",
        "slope_limit",
        "extent",
        "contact_offset",
        "dt",
    ];
    if c.as_object().unwrap().keys().any(|k| {
        !keys.contains(&k.as_str())
            && !k
                .strip_suffix("_bits")
                .is_some_and(|base| floatkeys.contains(&base))
    }) {
        return Err(());
    }
    for key in ["radius", "height"] {
        if c.get(key).is_none() && c.get(format!("{key}_bits")).is_none() {
            return Err(());
        }
    }
    let domain = optional::<bool>(c, "overlap", false)?
        || !optional::<bool>(c, "precise", true)?
        || optional::<u16>(c, "object_word", 6)? != 6;
    let standing = optional(c, "standing", false)?;
    let slope_enabled = optional(c, "slope_enabled", true)?;
    let down = vector(c, "down_normal", Some([0., 1., 0.]))?;
    let side = vector(c, "side_normal", Some([1., 0., 0.]))?;
    let height = scalar(c, "triangle_height", 0.)?;
    let bound = scalar(c, "bound_height", 0.)?;
    let contact = scalar(c, "contact_height", 0.)?;
    let triangle_min = scalar(c, "triangle_min", 0.)?;
    if domain
        || standing
        || !slope_enabled
        || down != [0., 1., 0.]
        || side != [1., 0., 0.]
        || height != 0.
        || bound != 0.
        || contact != 0.
        || triangle_min != 0.
    {
        return Ok(json!({"error":"UnsupportedDomain"}));
    }
    let _buffered_poses: bool = optional(c, "buffered_poses", false)?;
    let empty = json!({});
    let init = c.get("initial").unwrap_or(&empty);
    if !init.is_object() {
        return Err(());
    }
    let known = [
        "down_normal",
        "side_normal",
        "triangle_min",
        "triangle_height",
        "bound_height",
        "contact_height",
        "cache",
        "tag",
        "counter",
        "obstacle",
        "local_contact",
        "world_contact",
        "handles",
    ];
    if init.as_object().unwrap().keys().any(|k| {
        !known.contains(&k.as_str())
            && !k.strip_suffix("_bits").is_some_and(|base| {
                [
                    "down_normal",
                    "side_normal",
                    "triangle_min",
                    "triangle_height",
                    "bound_height",
                    "contact_height",
                    "local_contact",
                    "world_contact",
                ]
                .contains(&base)
            })
    }) {
        return Err(());
    }
    let mode: String = optional(c, "mode", "common".to_string())?;
    let triangles = tri(c)?;
    let defaults: Vec<u32> = (0..triangles.len())
        .map(|i| u32::try_from(i).map_err(|_| ()))
        .collect::<Result<_, _>>()?;
    let ids: Vec<u32> = optional(c, "ids", defaults)?;
    let origin = doubles(c, "offset", Some([0.; 3]))?;
    let raw_records: Vec<Value> = optional(
        c,
        "records",
        vec![json!({"start":0,"count":triangles.len()})],
    )?;
    let records =
        raw_records
            .into_iter()
            .map(|r| {
                if !r.is_object() || optional::<u32>(&r, "type", 2)? != 2 {
                    return Err(());
                }
                if r.as_object().unwrap().keys().any(|k| {
                    !["start", "count", "offset", "offset_bits", "type"].contains(&k.as_str())
                }) {
                    return Err(());
                }
                Ok(m::Record {
                    start: field(&r, "start")?,
                    count: field(&r, "count")?,
                    offset: doubles(&r, "offset", Some(origin))?,
                })
            })
            .collect::<Result<Vec<_>, ()>>()?;
    let up = vector(c, "up", Some([0., 1., 0.]))?;
    let minimum = scalar(c, "minimum", 1e-6)?;
    let displacement = vector(c, "displacement", None)?;
    // Present iteration-only fields are still parsed in other entry modes.
    let side = vector(c, "side", Some(displacement))?;
    let budget: u32 = optional(c, "budget", 10)?;
    let pass_id: u32 = optional(c, "pass_id", 1)?;
    if budget > 10 || pass_id > 3 {
        return Err(());
    }
    let capsule = Capsule {
        radius: scalar(c, "radius", 0.3)?,
        height: scalar(c, "height", 1.2)?,
        rotation: vector(c, "quaternion", None)?,
    };
    let center = doubles(c, "center", None)?;
    let cache: [u32; 3] = optional(c, "cache", [0; 3])?;
    let mut state = m::State {
        scheduler: cct_slope::State {
            position: center,
            flags: optional(c, "flags", 0)?,
            tag: optional(init, "tag", 0)?,
            down_normal: vector(init, "down_normal", Some([0., 1., 0.]))?,
            side_normal: vector(init, "side_normal", Some([1., 0., 0.]))?,
            triangle_height: scalar(init, "triangle_height", 0.)?,
            bound_height: scalar(init, "bound_height", 0.)?,
            contact_height: scalar(init, "contact_height", 0.)?,
            collision_flags: 0,
        },
        mesh: m::MeshState {
            cache: optional(init, "cache", cache)?,
            triangle_min: scalar(init, "triangle_min", 0.)?,
            counter: optional(init, "counter", 0)?,
            obstacle: optional(
                init,
                "obstacle",
                if mode == "common" { u32::MAX } else { 0 },
            )?,
            local_contact: vector(init, "local_contact", Some([0.; 3]))?,
            world_contact: vector(init, "world_contact", Some([0.; 3]))?,
            handles: handles(init)?,
        },
    };
    let previous: Vec<Option<String>> = optional(
        c,
        "previous_handles",
        vec![Some("actor".to_string()), Some("shape".to_string())],
    )?;
    let previous_handles = handles(&json!({"handles": previous}))?;
    let raw_support = c.get("support").unwrap_or(&empty);
    if !raw_support.is_object()
        || raw_support.as_object().unwrap().keys().any(|k| {
            ![
                "shapes",
                "scene_state",
                "scene_null",
                "scene_other",
                "buffer_bits",
                "flags",
                "buffer_flags",
            ]
            .contains(&k.as_str())
        })
    {
        return Err(());
    }
    let memberships: Vec<bool> = optional(raw_support, "shapes", vec![true])?;
    let memberships: Vec<_> = memberships
        .into_iter()
        .map(|matched| {
            if matched {
                support::ShapeMembership::PreviousShape
            } else {
                support::ShapeMembership::OtherFixtureShape
            }
        })
        .collect();
    let static_support = support::StaticSupport {
        shapes: &memberships,
        scene_state: support::SceneState::from_raw(optional(raw_support, "scene_state", 1)?)
            .map_err(|_| ())?,
        scene_null: optional(raw_support, "scene_null", false)?,
        scene_other: optional(raw_support, "scene_other", false)?,
        buffer_bits: optional(raw_support, "buffer_bits", 0)?,
        flags: optional(raw_support, "flags", 2)?,
        buffer_flags: optional(raw_support, "buffer_flags", 2)?,
    };
    let mut motion = support::Motion {
        velocity: vector(c, "previous_velocity", Some([7., 8., 9.]))?,
        moving: optional(c, "previous_moving", 1)?,
    };
    let mut common = m::CommonPublication {
        numeric: cct_slope::Publication {
            elapsed: double(c, "elapsed", 0.)?,
            pending: vector(c, "pending", Some([0.; 3]))?,
            position: center,
            collision_flags: 0,
        },
        handles: previous_handles,
    };
    let scene = m::PreparedScene {
        triangles: &triangles,
        source_ids: &ids,
        records: &records,
        cache_origin: origin,
        object_bounds: vector(c, "bounds", None)?,
        actor_pose: vector(c, "slot12_pose", Some([0., 0., 0., 1., 0., 0., 0.]))?,
        shape_pose: vector(c, "slot4_pose", Some([0., 0., 0., 1., 0., 0., 0.]))?,
        behavior: optional(c, "behavior", 0)?,
    };
    let settings = m::Settings {
        scheduler: cct_slope::Settings {
            up,
            step_offset: scalar(c, "step_offset", 0.3)?,
            min_distance: minimum,
            nonwalkable_mode: optional(c, "nonwalkable_mode", 0)?,
            standing,
            prevent_ceiling_slide: optional(c, "prevent_ceiling_slide", false)?,
            slope_enabled,
            slope_limit: scalar(c, "slope_limit", 0.7)?,
            constrain: optional(c, "constrain", false)?,
            extent: scalar(c, "extent", 0.9)?,
        },
        contact_offset: scalar(c, "contact_offset", 0.02)?,
        custom_response: optional(c, "custom_response", true)?,
    };
    let dt = scalar(c, "dt", 0.016)?;
    if mode != "common" {
        return Ok(json!({"error":"UnsupportedDomain"}));
    }
    let _parsed_unused = (side, budget, pass_id);
    Ok(
        match m::run_static_common(
            &mut state,
            &mut common,
            &mut motion,
            &scene,
            capsule,
            settings,
            displacement,
            dt,
            static_support,
        ) {
            Ok(trace) => {
                let prefix = trace.support_prefix;
                let events: Vec<Value> = prefix
                    .events
                    .into_iter()
                    .map(|e| match e {
                        support::Event::Count => json!(["count", null]),
                        support::Event::Enumerate(i) => json!(["enumerate", i]),
                        support::Event::Scene => json!(["scene", null]),
                        support::Event::Flags => json!(["flags", null]),
                        support::Event::StaticMotion => json!(["static_motion", null]),
                        support::Event::DisabledProbe => json!(["disabled_probe", null]),
                    })
                    .collect();
                let mut result = output(trace.numerical);
                result.as_object_mut().unwrap().insert("support_prefix".into(),json!({"events":events,"handles":handle_values(prefix.handles),"velocity":hf(prefix.velocity),"moving":prefix.moving,"request":hf(prefix.request)}));
                result
            }
            Err(error) => json!({"error":format!("{error:?}")}),
        },
    )
}

fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.expect("stdin");
        if !line.trim().is_empty() {
            let out = serde_json::from_str(&line)
                .map_err(|_| ())
                .and_then(evaluate)
                .unwrap_or_else(|_| json!({"error":"InvalidInput"}));
            println!("{out}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn decimal_f64_transport_rounds_to_nearest_and_bits_remain_authoritative() {
        let literal: Value = serde_json::from_str(
            r#"{"center":[0.0,1.7855523726117835,0.0],"elapsed":1.7855523726117835}"#,
        )
        .unwrap();
        assert_eq!(
            doubles::<3>(&literal, "center", None).unwrap()[1].to_bits(),
            0x3ffc919f5d5a9ae5
        );
        assert_eq!(
            double(&literal, "elapsed", 0.).unwrap().to_bits(),
            0x3ffc919f5d5a9ae5
        );
        let mut alternate = literal;
        alternate["center_bits"] = json!([0_u64, 0x3ffc919f5d5a9ae4_u64, 0_u64]);
        assert_eq!(
            doubles::<3>(&alternate, "center", None).unwrap()[1].to_bits(),
            0x3ffc919f5d5a9ae4
        );
        alternate["center"] = json!([0., "malformed", 0.]);
        assert!(doubles::<3>(&alternate, "center", None).is_err());
    }
}
