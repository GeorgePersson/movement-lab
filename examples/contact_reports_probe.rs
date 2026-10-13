//! JSONL transport for finite supplied contact-report sequences; no overwrite.
use movement_lab::contact_reports::{Contact, ContactReports};
use serde::{Deserialize, Serialize};
use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, BufWriter, Write};

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "lowercase", deny_unknown_fields)]
enum Event {
    Reset,
    Direct {
        point: [f32; 4],
        normal: [f32; 4],
        controller: u8,
        metadata: u8,
    },
    Shape {
        point: [f64; 3],
        normal: [f32; 3],
        metadata: u8,
    },
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    initial: String,
    events: Vec<Event>,
}
#[derive(Serialize)]
struct Output {
    records: Vec<String>,
}

fn decode(value: &str) -> Result<[u8; 85], String> {
    if value.len() != 170 || !value.is_ascii() {
        return Err("initial must be 170 ASCII hex characters".into());
    }
    let mut out = [0; 85];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte =
            u8::from_str_radix(&value[i * 2..i * 2 + 2], 16).map_err(|_| "invalid hex record")?;
    }
    Ok(out)
}

fn evaluate(input: Input) -> Result<Output, String> {
    if input.events.len() > 10_000 {
        return Err("event limit exceeded".into());
    }
    let mut state = ContactReports::from_record(&decode(&input.initial)?);
    let mut records = Vec::new();
    for event in input.events {
        match event {
            Event::Reset => state.reset(),
            Event::Direct {
                point,
                normal,
                controller,
                metadata,
            } => {
                if !point.iter().chain(normal.iter()).all(|v| v.is_finite())
                    || controller > 1
                    || metadata > 1
                {
                    return Err("finite contact and boolean flags required".into());
                }
                state.add(Contact { point, normal }, controller != 0, metadata != 0);
            }
            Event::Shape {
                point,
                normal,
                metadata,
            } => {
                if !point
                    .iter()
                    .all(|v| v.is_finite() && (*v as f32).is_finite())
                    || !normal.iter().all(|v| v.is_finite())
                    || metadata > 1
                {
                    return Err("finite narrowed point/normal and boolean metadata required".into());
                }
                state.add_shape(point, normal, metadata != 0);
            }
        }
        records.push(
            state
                .to_record()
                .iter()
                .map(|b| format!("{b:02x}"))
                .collect(),
        );
    }
    Ok(Output { records })
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args_os().collect();
    if args.len() != 3 {
        return Err("usage: contact_reports_probe INPUT OUTPUT".into());
    }
    let input = BufReader::new(File::open(&args[1])?);
    let mut output = BufWriter::new(
        OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(&args[2])?,
    );
    for (i, line) in input.lines().enumerate() {
        let result =
            evaluate(serde_json::from_str(&line?)?).map_err(|e| format!("line {}: {e}", i + 1))?;
        serde_json::to_writer(&mut output, &result)?;
        output.write_all(b"\n")?;
    }
    output.flush()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rejects_bad_record_and_non_boolean_flags() {
        assert!(decode("zz").is_err());
        let input = Input {
            initial: "00".repeat(85),
            events: vec![Event::Direct {
                point: [0.; 4],
                normal: [0.; 4],
                controller: 2,
                metadata: 0,
            }],
        };
        assert!(evaluate(input).is_err());
    }
    #[test]
    fn rejects_nonfinite_or_overflowing_narrowed_point() {
        for x in [f64::NAN, f64::INFINITY, f64::MAX] {
            let input = Input {
                initial: "00".repeat(85),
                events: vec![Event::Shape {
                    point: [x, 0., 0.],
                    normal: [0., 1., 0.],
                    metadata: 0,
                }],
            };
            assert!(evaluate(input).is_err());
        }
    }
}
