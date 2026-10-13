//! JSONL adapter for independently supplied finite geometry cases.
use movement_lab::precise_sphere::*;
use serde_json::{Value, json};
use std::io::{self, BufRead};
fn vec3(c: &Value, name: &str) -> Vec3 {
    serde_json::from_value(c[name].clone()).expect("three floats")
}
fn matrix(c: &Value) -> [Vec3; 3] {
    serde_json::from_value(c["rotation"].clone()).expect("three columns")
}
fn num(c: &Value, name: &str) -> f32 {
    c[name].as_f64().expect("float") as f32
}
fn run(row: &Value) -> Value {
    let c = &row["case"];
    match row["kind"].as_str().unwrap_or("sphere_box") {
        "capsule" => json!(ray_capsule(
            vec3(c, "origin"),
            vec3(c, "direction"),
            vec3(c, "a"),
            vec3(c, "b"),
            num(c, "radius")
        )),
        "sphere" | "sphere-basic" => {
            let f = if row["kind"] == "sphere" {
                ray_sphere
            } else {
                ray_sphere_basic
            };
            json!(f(
                vec3(c, "origin"),
                vec3(c, "direction"),
                num(c, "max_distance"),
                vec3(c, "center"),
                num(c, "radius"),
                c["want_point"].as_bool().unwrap_or(false)
            ))
        }
        "aabb" => json!(ray_aabb(
            vec3(c, "minimum"),
            vec3(c, "maximum"),
            vec3(c, "origin"),
            vec3(c, "direction")
        )),
        "points" => {
            json!({"points":obb_points(vec3(c,"center"),vec3(c,"extents"),matrix(c)).into_iter().flatten().collect::<Vec<_>>()})
        }
        "overlap" => {
            json!({"hit":sphere_overlap(vec3(c,"origin"),num(c,"radius"),vec3(c,"center"),vec3(c,"extents"),matrix(c))})
        }
        "sphere_box" => json!(sphere_box(
            vec3(c, "center"),
            vec3(c, "extents"),
            matrix(c),
            vec3(c, "origin"),
            num(c, "radius"),
            vec3(c, "direction"),
            num(c, "max_distance"),
            c["flags"]
                .as_u64()
                .unwrap_or(0)
                .try_into()
                .expect("u16 flags")
        )),
        kind => panic!("unknown kind {kind}"),
    }
}
fn main() {
    for line in io::stdin().lock().lines() {
        let line = line.expect("input line");
        if line.trim().is_empty() {
            continue;
        }
        let row: Value = serde_json::from_str(&line).expect("JSON case");
        println!("{}", run(&row));
    }
}
