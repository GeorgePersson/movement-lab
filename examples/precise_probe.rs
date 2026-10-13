//! JSONL comparison adapter for the complete precise capsule/box wrapper.
//! Synthetic inputs/results are not gameplay measurements. Outputs are exclusive.

use movement_lab::precise_sweep::{SweepInput, sweep};
use serde_json::{Value, json};
use std::{
    env,
    fs::{File, OpenOptions},
    io::{self, BufRead, BufReader, BufWriter, Write},
};

fn evaluate(line: &str) -> Value {
    let input: SweepInput = match serde_json::from_str(line) {
        Ok(value) => value,
        Err(_) => return json!({"error": "InvalidJson"}),
    };
    let result = match sweep(&input) {
        Ok(value) => value,
        Err(error) => return json!({"error": error}),
    };
    let mut record = [0x55_u8; 64];
    record[52..56].copy_from_slice(&result.distance.to_le_bytes());
    if let Some(flags) = result.flags {
        record[24..26].copy_from_slice(&flags.to_le_bytes());
    }
    for (offset, vector) in [(28, result.point), (40, result.normal)] {
        if let Some(vector) = vector {
            for (i, value) in vector.into_iter().enumerate() {
                record[offset + 4 * i..offset + 4 * i + 4].copy_from_slice(&value.to_le_bytes());
            }
        }
    }
    let record_hex: String = record.iter().map(|x| format!("{x:02x}")).collect();
    json!({"hit": result.hit, "record_hex": record_hex})
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = env::args_os().collect();
    if args.len() != 3 {
        return Err("usage: precise_probe INPUT.jsonl NEW_OUTPUT.jsonl".into());
    }
    let input = File::open(&args[1])?;
    if !input.metadata()?.is_file() || input.metadata()?.len() > 64 * 1024 * 1024 {
        return Err("input must be a regular file at most 64 MiB".into());
    }
    let mut output = BufWriter::new(
        OpenOptions::new()
            .create_new(true)
            .write(true)
            .open(&args[2])?,
    );
    for (index, line) in BufReader::new(input).lines().enumerate() {
        let line = line?;
        if index >= 100_000 || line.len() > 64 * 1024 {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "input limit exceeded").into());
        }
        let mut result = evaluate(&line);
        result["index"] = json!(index);
        serde_json::to_writer(&mut output, &result)?;
        writeln!(output)?;
    }
    output.flush()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn invalid_direction_is_an_error_instead_of_a_miss() {
        let input = json!({"a":[5.,0.,0.],"b":[5.,1.,0.],"radius":0.3,
            "box_q":[0.,0.,0.,1.],"box_p":[0.,0.,0.],"extents":[1.,1.,1.],
            "direction":[0.,0.,0.],"distance":5.,"flags":1283});
        let result = evaluate(&input.to_string());
        assert_eq!(result["error"], "InvalidInput");
        assert!(result.get("hit").is_none());
    }

    #[test]
    fn malformed_record_is_reported_without_a_result() {
        let result = evaluate("{\"a\":[0,0]}");
        assert_eq!(result["error"], "InvalidJson");
        assert!(result.get("record_hex").is_none());
    }
}
