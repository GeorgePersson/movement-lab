//! JSONL arithmetic probe. Private fixtures are supplied over stdin.
use movement_lab::precise_capsule as p;
use serde::de::DeserializeOwned;
use serde_json::{Value, json};
use std::io::{self, BufRead};
fn field<T: DeserializeOwned>(v: &Value, key: &str) -> T {
    serde_json::from_value(v[key].clone()).unwrap_or_else(|e| panic!("{key}: {e}"))
}
fn flag(v: &Value, key: &str) -> bool {
    v[key].as_bool().unwrap_or(false)
}
fn run(v: Value) -> Value {
    let c = &v["c"];
    match v["kind"].as_str().unwrap() {
        "builder" => {
            let q: Option<[f32; 4]> = field(c, "box_q");
            let pos: [f32; 3] = field(c, "box_p");
            let (t, parents) = p::build_triangles(
                field(c, "extents"),
                q.map(|q| (q, pos)),
                field(c, "half"),
                field(c, "direction"),
            );
            json!({"triangles":t,"parents":parents})
        }
        "closest" => {
            let (point, b, cw) = p::closest(field(c, "p"), field(c, "tri"));
            json!([point, b, cw])
        }
        "contact" => {
            let (point, normal) = p::contact(
                field(c, "center"),
                field(c, "direction"),
                field(c, "time"),
                field(c, "tri"),
            );
            json!([point, normal])
        }
        "triangle" => match p::triangle_hit(
            field(c, "tri"),
            field(c, "normal"),
            field(c, "center"),
            field(c, "radius"),
            field(c, "direction"),
            flag(c, "check_initial"),
        ) {
            Some(r) => json!({"hit":true,"time":r.time,"feature":r.feature}),
            None => json!({"hit":false,"feature":0}),
        },
        "collection" => {
            let tris: Vec<p::Triangle> = field(c, "triangles");
            let options = p::CollectionOptions {
                cached_index: c["cached_index"].as_u64().unwrap_or(0) as usize,
                flag_a: flag(c, "flag_a"),
                flag_b: flag(c, "flag_b"),
                any_hit: flag(c, "any_hit"),
                check_initial: flag(c, "check_initial"),
            };
            match p::collection_sweep(
                &tris,
                field(c, "center"),
                field(c, "radius"),
                field(c, "direction"),
                field(c, "max_distance"),
                options,
            ) {
                None => json!({"hit":false}),
                Some(r) => {
                    let mut out = json!({"hit":true,"index":r.index,"flags":r.flags,"normal":r.normal,"distance":r.distance,"aux":r.aux});
                    if let Some(point) = r.point {
                        out["point"] = json!(point);
                    }
                    out
                }
            }
        }
        "capsule" => {
            let r = p::capsule_box(
                field(c, "a"),
                field(c, "b"),
                field(c, "radius"),
                field(c, "box_q"),
                field(c, "box_p"),
                field(c, "extents"),
                field(c, "direction"),
                field(c, "distance"),
                v["flags"].as_u64().unwrap_or(0) as u16,
            );
            json!({"hit":r.hit,"distance":r.distance,"normal":r.normal,"point":r.point,"initial":r.initial})
        }
        other => panic!("unknown probe kind {other}"),
    }
}
fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.expect("stdin");
        if line.trim().is_empty() {
            continue;
        }
        println!("{}", run(serde_json::from_str(&line).expect("JSON")));
    }
}
