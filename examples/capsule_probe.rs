//! JSONL adapter for supplied low-level capsule sweep inputs.
//! Outputs are kernel comparisons, not measured gameplay or world-fit evidence.
//!
//! Usage: capsule_probe INPUT.jsonl OUTPUT.jsonl
//! The output is created exclusively; existing files are never replaced.

use std::{
    env,
    fs::{File, OpenOptions},
    io::{self, BufRead, BufReader, BufWriter, Write},
    path::Path,
};

use movement_lab::capsule_geometry as geometry;
use movement_lab::capsule_sweep::{SweepInput, sweep};
use serde::Deserialize;
use serde_json::{Value, json};

const MAX_BYTES: u64 = 64 * 1024 * 1024;
const MAX_LINE: usize = 64 * 1024;
const MAX_CASES: usize = 100_000;

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum HelperInput {
    CapsuleSupport {
        a: [f32; 4],
        b: [f32; 4],
        direction: [f32; 4],
    },
    CapsuleVertex {
        a: [f32; 4],
        b: [f32; 4],
        vertex_index: usize,
    },
    BoxSupport {
        extents: [f32; 4],
        direction: [f32; 4],
    },
    BoxVertex {
        extents: [f32; 4],
        vertex_index: usize,
    },
    SegmentWeight {
        point: [f32; 4],
        a: [f32; 4],
        b: [f32; 4],
    },
    TriangleWeights {
        point: [f32; 4],
        a: [f32; 4],
        b: [f32; 4],
        c: [f32; 4],
    },
    Tetrahedron {
        q: [[f32; 4]; 4],
    },
}

fn helper(index: usize, input: HelperInput) -> Value {
    let vector = |value: &[f32]| value.iter().all(|x| x.is_finite());
    let mut keep = None;
    let value = match input {
        HelperInput::CapsuleSupport { a, b, direction }
            if vector(&a) && vector(&b) && vector(&direction) =>
        {
            Some(geometry::capsule_support(a, b, direction).0.to_vec())
        }
        HelperInput::CapsuleVertex { a, b, vertex_index } if vector(&a) && vector(&b) => {
            geometry::capsule_vertex(a, b, vertex_index).map(|x| x.to_vec())
        }
        HelperInput::BoxSupport { extents, direction }
            if vector(&extents) && vector(&direction) =>
        {
            Some(geometry::box_support(extents, direction).0.to_vec())
        }
        HelperInput::BoxVertex {
            extents,
            vertex_index,
        } if vector(&extents) => geometry::box_vertex(extents, vertex_index).map(|x| x.to_vec()),
        HelperInput::SegmentWeight { point, a, b }
            if vector(&point) && vector(&a) && vector(&b) =>
        {
            Some(geometry::segment_weight_lanes(point, a, b).to_vec())
        }
        HelperInput::TriangleWeights { point, a, b, c }
            if vector(&point) && vector(&a) && vector(&b) && vector(&c) =>
        {
            let (u, v) = geometry::triangle_weight_lanes(point, a, b, c);
            Some(u.into_iter().chain(v).collect())
        }
        HelperInput::Tetrahedron { q } if q.iter().all(|x| vector(x)) => {
            let reduction = geometry::reduce_tetrahedron(q);
            keep = Some(reduction.active_indices().to_vec());
            Some(reduction.closest.to_vec())
        }
        _ => None,
    };
    match value {
        Some(value) if vector(&value) => {
            let mut result = json!({"index": index, "value": value});
            if let Some(keep) = keep {
                result["keep"] = json!(keep);
            }
            result
        }
        Some(_) => json!({"index": index, "error": "NonFiniteArithmetic"}),
        None => json!({"index": index, "error": "InvalidInput"}),
    }
}

fn evaluate(index: usize, line: &str) -> Value {
    // An explicit helper kind is optional; ordinary records are direct SweepInput.
    if let Ok(value) = serde_json::from_str::<Value>(line) {
        if value.get("kind").is_some_and(|x| x != "sweep") {
            return match serde_json::from_value::<HelperInput>(value) {
                Ok(input) => helper(index, input),
                Err(_) => json!({"index": index, "error": "InvalidJson"}),
            };
        }
    }
    let input = match serde_json::from_str::<SweepInput>(line) {
        Ok(input) => input,
        Err(_) => return json!({"index": index, "error": "InvalidJson"}),
    };
    match sweep(&input) {
        Ok(Some(hit)) => json!({
            "index": index, "hit": true, "fraction": hit.fraction,
            "normal": hit.normal, "point": hit.point,
        }),
        Ok(None) => json!({"index": index, "hit": false}),
        Err(error) => json!({"index": index, "error": format!("{error:?}")}),
    }
}

fn run(reader: impl BufRead, mut writer: impl Write) -> io::Result<usize> {
    let mut count = 0;
    let mut total = 0_u64;
    for (index, line) in reader.lines().enumerate() {
        let line = line?;
        total += line.len() as u64 + 1;
        if line.len() > MAX_LINE || total > MAX_BYTES || count >= MAX_CASES {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "input work limit",
            ));
        }
        // Empty lines still receive a parse error, preserving source line indexes.
        serde_json::to_writer(&mut writer, &evaluate(index, &line))?;
        writer.write_all(b"\n")?;
        count += 1;
    }
    writer.flush()?;
    Ok(count)
}

fn files(input: &Path, output: &Path) -> io::Result<usize> {
    let source = File::open(input)?;
    if source.metadata()?.len() > MAX_BYTES {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            "input byte limit",
        ));
    }
    let destination = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(output)?;
    run(BufReader::new(source), BufWriter::new(destination))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    if args.len() != 2 {
        return Err("usage: capsule_probe INPUT.jsonl OUTPUT.jsonl".into());
    }
    let cases = files(Path::new(&args[0]), Path::new(&args[1]))?;
    eprintln!("processed {cases} supplied capsule inputs; no gameplay-fit claim");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU64, Ordering};

    fn input() -> Value {
        json!({
            "a": [5.,0.,0.,0.], "b": [5.,0.,0.,0.],
            "extents": [1.,1.,1.,0.], "initial": [5.,0.,0.,0.],
            "start": [0.,0.,0.,0.], "motion": [-10.,0.,0.,0.],
            "radius": 0., "shape_radius": 0., "radius_point": true,
            "box_margin": 0.05, "label": "private corpus metadata"
        })
    }

    #[test]
    fn direct_model_schema_accepts_metadata_without_using_it() {
        let result = evaluate(7, &input().to_string());
        assert_eq!(result["index"], 7);
        assert_eq!(result["hit"], false);
        assert!(result.get("fraction").is_none());
        assert!(result.get("label").is_none());
    }

    #[test]
    fn malformed_record_does_not_shift_later_source_indexes() {
        let text = format!("{}\nnot-json\n{}\n", input(), input());
        let mut bytes = Vec::new();
        assert_eq!(run(io::Cursor::new(text), &mut bytes).unwrap(), 3);
        let rows: Vec<Value> = String::from_utf8(bytes)
            .unwrap()
            .lines()
            .map(|line| serde_json::from_str(line).unwrap())
            .collect();
        assert_eq!(rows[1], json!({"index": 1, "error": "InvalidJson"}));
        assert_eq!(rows[2]["index"], 2);
        assert_eq!(rows[2]["hit"], false);
    }

    #[test]
    fn existing_destination_including_input_is_never_overwritten() {
        static COUNTER: AtomicU64 = AtomicU64::new(0);
        let path = env::temp_dir().join(format!(
            "capsule-probe-{}-{}",
            std::process::id(),
            COUNTER.fetch_add(1, Ordering::Relaxed)
        ));
        let original = format!("{}\n", input());
        OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(&path)
            .unwrap()
            .write_all(original.as_bytes())
            .unwrap();
        let error = files(&path, &path).unwrap_err();
        assert_eq!(error.kind(), io::ErrorKind::AlreadyExists);
        assert_eq!(std::fs::read_to_string(&path).unwrap(), original);
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn oversized_line_is_rejected_without_serialized_result() {
        let mut output = Vec::new();
        let error = run(io::Cursor::new("x".repeat(MAX_LINE + 1)), &mut output).unwrap_err();
        assert_eq!(error.kind(), io::ErrorKind::InvalidData);
        assert!(output.is_empty());
    }

    #[test]
    fn helper_records_keep_four_lane_results_and_reject_invalid_vertex_index() {
        let good = json!({"kind":"capsule_vertex", "a":[1.,2.,3.,0.],
            "b":[4.,5.,6.,0.], "vertex_index":0});
        assert_eq!(
            evaluate(0, &good.to_string())["value"],
            json!([4., 5., 6., 0.])
        );
        let mut bad = good;
        bad["vertex_index"] = json!(2);
        assert_eq!(evaluate(1, &bad.to_string())["error"], "InvalidInput");
    }
}
