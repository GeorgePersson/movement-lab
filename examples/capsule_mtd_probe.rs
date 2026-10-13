//! Supplied finite pose JSONL comparison protocol; output never overwrites.
use movement_lab::capsule_mtd::{MtdInput, capsule_box_mtd};
use serde::Serialize;
use std::fs::{File, OpenOptions};
use std::io::{self, BufRead, BufReader, BufWriter, Write};
#[derive(Serialize)]
struct Output {
    hit: bool,
    normal_bits: Option<[u32; 3]>,
    depth_bits: Option<u32>,
}
fn evaluate(c: &MtdInput) -> Result<Output, String> {
    if !c
        .capsule_rotation
        .iter()
        .chain(c.capsule_position.iter())
        .chain(c.box_rotation.iter())
        .chain(c.box_position.iter())
        .chain(c.extents.iter())
        .chain([c.radius, c.half_height].iter())
        .all(|x| x.is_finite())
        || c.radius < 0.0
        || c.half_height < 0.0
        || c.extents.iter().any(|x| *x < 0.0)
    {
        return Err("finite valid geometry required".into());
    }
    let r = capsule_box_mtd(c);
    if r.is_some_and(|v| !v.depth.is_finite() || !v.normal.iter().all(|x| x.is_finite())) {
        return Err("nonfinite arithmetic outside comparison domain".into());
    }
    Ok(Output {
        hit: r.is_some(),
        normal_bits: r.map(|v| v.normal.map(f32::to_bits)),
        depth_bits: r.map(|v| v.depth.to_bits()),
    })
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let a: Vec<_> = std::env::args_os().collect();
    if a.len() != 3 {
        return Err("usage: capsule_mtd_probe INPUT OUTPUT".into());
    }
    let input = BufReader::new(File::open(&a[1])?);
    let mut out = BufWriter::new(
        OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&a[2])?,
    );
    for (i, line) in input.lines().enumerate() {
        let c: MtdInput = serde_json::from_str(&line?)?;
        let r = evaluate(&c).map_err(|s| {
            io::Error::new(io::ErrorKind::InvalidInput, format!("line {}: {s}", i + 1))
        })?;
        serde_json::to_writer(&mut out, &r)?;
        out.write_all(b"\n")?;
    }
    out.flush()?;
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    fn input() -> MtdInput {
        MtdInput {
            radius: 0.3,
            half_height: 0.6,
            capsule_rotation: [0., 0., 0., 1.],
            capsule_position: [0.; 3],
            box_rotation: [0., 0., 0., 1.],
            box_position: [0.; 3],
            extents: [1.; 3],
        }
    }
    #[test]
    fn embedded_and_clear() {
        let mut c = input();
        let r = capsule_box_mtd(&c).unwrap();
        assert_eq!(r.normal, [0., 1., 0.]);
        assert_eq!(r.depth, 1.3);
        c.capsule_position = [4., 0., 0.];
        assert!(capsule_box_mtd(&c).is_none());
    }
    #[test]
    fn reject_invalid_protocol() {
        let mut c = input();
        c.radius = -1.;
        assert!(evaluate(&c).is_err());
        c.radius = f32::NAN;
        assert!(evaluate(&c).is_err());
    }
    #[test]
    fn asymmetric_surface_direction() {
        let mut c = input();
        c.half_height = 0.;
        c.capsule_position = [-1.2, 0., 0.];
        let r = capsule_box_mtd(&c).unwrap();
        assert_eq!(r.normal, [-1., 0., 0.]);
        assert!(r.depth > 0. && r.depth < 0.11);
    }
}
