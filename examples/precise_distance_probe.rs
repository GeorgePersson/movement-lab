//! JSONL runner for supplied distance inputs. Output contains exact float bits.
//! Usage: precise_distance_probe INPUT OUTPUT (OUTPUT must not exist).
use movement_lab::precise_distance::{
    line_box_distance_sq, point_box_distance_sq, segment_box_distance_sq,
};
use serde::{Deserialize, Serialize};
use std::fs::{File, OpenOptions};
use std::io::{self, BufRead, BufReader, BufWriter, Write};

#[derive(Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
enum Kind {
    Point,
    Line,
    Segment,
}
#[derive(Deserialize)]
struct Input {
    kind: Kind,
    a: [f32; 3],
    b: [f32; 3],
    center: [f32; 3],
    extents: [f32; 3],
    matrix: [f32; 9],
}
#[derive(Debug, Serialize, Deserialize, PartialEq)]
struct Output {
    distance_bits: u32,
    point_bits: [u32; 3],
    t_bits: Option<u32>,
}
fn evaluate(input: Input) -> Result<Output, String> {
    if !input
        .a
        .iter()
        .chain(input.b.iter())
        .chain(input.center.iter())
        .chain(input.extents.iter())
        .chain(input.matrix.iter())
        .all(|v| v.is_finite())
        || input.extents.iter().any(|v| *v < 0.0)
    {
        return Err("finite inputs and nonnegative extents required".into());
    }
    // Rotation validity and moderate arithmetic domain belong to the caller.
    match input.kind {
        Kind::Point => {
            let r = point_box_distance_sq(input.a, input.center, input.extents, input.matrix);
            Ok(Output {
                distance_bits: r.distance.to_bits(),
                point_bits: r.point.map(f32::to_bits),
                t_bits: None,
            })
        }
        kind => {
            let r = match kind {
                Kind::Line => line_box_distance_sq(
                    input.a,
                    input.b,
                    input.center,
                    input.extents,
                    input.matrix,
                ),
                _ => segment_box_distance_sq(
                    input.a,
                    input.b,
                    input.center,
                    input.extents,
                    input.matrix,
                ),
            };
            Ok(Output {
                distance_bits: r.distance.to_bits(),
                point_bits: r.point.map(f32::to_bits),
                t_bits: Some(r.t.to_bits()),
            })
        }
    }
}
fn replay(reader: impl BufRead, mut writer: impl Write) -> Result<usize, String> {
    let mut count = 0;
    for (index, line) in reader.lines().enumerate() {
        let line = line.map_err(|e| e.to_string())?;
        let input: Input =
            serde_json::from_str(&line).map_err(|e| format!("line {}: {e}", index + 1))?;
        let output = evaluate(input).map_err(|e| format!("line {}: {e}", index + 1))?;
        serde_json::to_writer(&mut writer, &output).map_err(|e| e.to_string())?;
        writer.write_all(b"\n").map_err(|e| e.to_string())?;
        count += 1;
    }
    writer.flush().map_err(|e| e.to_string())?;
    Ok(count)
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args_os().collect();
    if args.len() != 3 {
        return Err("usage: precise_distance_probe INPUT OUTPUT (new output only)".into());
    }
    let reader = BufReader::new(File::open(&args[1])?);
    let writer = BufWriter::new(
        OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&args[2])?,
    );
    let count = replay(reader, writer).map_err(io::Error::other)?;
    eprintln!("simulated distance queries: {count}; supplied geometry, no gameplay fit");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;
    fn input() -> serde_json::Value {
        serde_json::json!({"kind":"point","a":[3.0,0.0,0.0],"b":[0.0,0.0,0.0],"center":[0.0,0.0,0.0],"extents":[1.0,1.0,1.0],"matrix":[1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]})
    }
    #[test]
    fn rejects_unknown_kind_and_bad_dimensions() {
        let mut v = input();
        v["kind"] = "triangle".into();
        assert!(replay(Cursor::new(v.to_string()), Vec::new()).is_err());
        let mut v = input();
        v["matrix"] = serde_json::json!([1.0, 0.0]);
        assert!(replay(Cursor::new(v.to_string()), Vec::new()).is_err());
    }
    #[test]
    fn rejects_negative_extents_and_overflowing_input() {
        let mut v = input();
        v["extents"][0] = (-1.0).into();
        assert!(replay(Cursor::new(v.to_string()), Vec::new()).is_err());
        let v = input().to_string().replace("3.0", "1e100");
        assert!(replay(Cursor::new(v), Vec::new()).is_err());
    }
    #[test]
    fn preserves_row_count_and_emits_raw_bits() {
        let v = input().to_string();
        let mut output = Vec::new();
        assert_eq!(
            replay(Cursor::new(format!("{v}\n{v}\n")), &mut output).unwrap(),
            2
        );
        let rows: Vec<Output> = String::from_utf8(output)
            .unwrap()
            .lines()
            .map(|x| serde_json::from_str(x).unwrap())
            .collect();
        assert_eq!(
            rows[0],
            Output {
                distance_bits: 4.0_f32.to_bits(),
                point_bits: [1.0_f32.to_bits(), 0, 0],
                t_bits: None
            }
        );
        assert_eq!(rows[0], rows[1]);
    }
}
