//! Supplied finalizer JSONL protocol; secondary movement is an explicit boundary.
use movement_lab::contact_finalizer::{FinalizerGlobals, FinalizerState, SecondaryMove, finalize};
use movement_lab::contact_reports::{Contact, ContactReports};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, BufWriter, Write};

#[derive(Clone, Copy, Deserialize)]
#[serde(untagged)]
enum Flag {
    Boolean(bool),
    Byte(u8),
}
impl Flag {
    fn byte(self) -> u8 {
        match self {
            Self::Boolean(b) => u8::from(b),
            Self::Byte(b) => b,
        }
    }
    fn boolean(self) -> bool {
        self.byte() != 0
    }
}
#[derive(Deserialize)]
struct After {
    actual: [f32; 4],
    normal: [f32; 4],
    has_report: Flag,
    late_flag: Flag,
    native_controller: Option<Flag>,
    step: Option<f32>,
    zero: Option<[f32; 4]>,
    defer: Option<Flag>,
}
#[derive(Deserialize)]
struct Input {
    normal: [f32; 4],
    secondary_normal: [f32; 4],
    zero: [f32; 4],
    up: [f32; 4],
    fallback: [f32; 4],
    has_report: Flag,
    has_secondary: Flag,
    latest_cct: Flag,
    metadata: Flag,
    suppress: Flag,
    mask: u32,
    scalar: f32,
    step: f32,
    age: f32,
    timer: f32,
    cached_y: f32,
    position_y: f32,
    late_flag: Flag,
    native_controller: Flag,
    dt: f32,
    after: After,
    /// Already evaluated cosine-family result, transported exactly by bits.
    cosine_bits: u32,
}
#[derive(Serialize)]
struct Output {
    normal: String,
    other_normal: String,
    normal_y: String,
    cached_y: String,
    defer: bool,
    late_flag: bool,
    trace: Vec<Value>,
}

fn hex<const N: usize>(values: [f32; N]) -> String {
    values
        .into_iter()
        .flat_map(f32::to_le_bytes)
        .map(|b| format!("{b:02x}"))
        .collect()
}
struct Boundary {
    after: After,
    trace: Vec<Value>,
}
impl SecondaryMove for Boundary {
    fn set_step(&mut self, value: f32, _: &mut FinalizerState, _: &mut FinalizerGlobals) {
        self.trace.push(json!({"step": value}));
    }
    fn move_request(
        &mut self,
        request: [f32; 4],
        dt: f32,
        s: &mut FinalizerState,
        g: &mut FinalizerGlobals,
    ) -> [f32; 4] {
        self.trace
            .push(json!({"move": hex(request), "dt": hex([dt])}));
        s.reports.primary.normal = self.after.normal;
        s.reports.primary_valid = self.after.has_report.byte();
        s.late_flag = self.after.late_flag.boolean();
        if let Some(v) = self.after.native_controller {
            s.native_controller_present = v.boolean();
        }
        if let Some(v) = self.after.step {
            s.step = v;
        }
        if let Some(v) = self.after.zero {
            g.zero = v;
        }
        if let Some(v) = self.after.defer {
            s.defer = v.boolean();
        }
        self.after.actual
    }
}

fn evaluate(c: Input) -> Result<Output, String> {
    let cosine = f32::from_bits(c.cosine_bits);
    if !c
        .normal
        .iter()
        .chain(c.secondary_normal.iter())
        .chain(c.zero.iter())
        .chain(c.up.iter())
        .chain(c.fallback.iter())
        .chain(c.after.actual.iter())
        .chain(c.after.normal.iter())
        .chain(c.after.zero.iter().flatten())
        .chain(
            [
                c.scalar,
                c.step,
                c.age,
                c.timer,
                c.cached_y,
                c.position_y,
                c.dt,
                cosine,
            ]
            .iter(),
        )
        .chain(c.after.step.iter())
        .all(|v| v.is_finite())
    {
        return Err("finite supplied state required".into());
    }
    let reports = ContactReports {
        primary: Contact {
            point: [0.; 4],
            normal: c.normal,
        },
        secondary: Contact {
            point: [0.; 4],
            normal: c.secondary_normal,
        },
        primary_valid: c.has_report.byte(),
        secondary_valid: c.has_secondary.byte(),
        latest_was_controller: c.latest_cct.byte(),
        reset_byte_813: 0,
        latest_metadata: c.metadata.byte(),
        gap_7e0: [0; 16],
    };
    let initial_flag_or = (reports.latest_was_controller != 0
        && ((c.normal[0] * c.up[0] + c.normal[1] * c.up[1]) + c.normal[2] * c.up[2]) > 0.)
        || reports.latest_metadata != 0;
    let mut boundary = Boundary {
        after: c.after,
        trace: Vec::new(),
    };
    if !initial_flag_or {
        boundary.trace.push(json!("slope"));
    }
    boundary.trace.push(json!("move_gate"));
    let mut globals = FinalizerGlobals {
        zero: c.zero,
        up: c.up,
        fallback: c.fallback,
    };
    let mut state = FinalizerState {
        reports,
        working_normal: [0.; 4],
        other_normal: [0.; 4],
        normal_y: 0.,
        cached_y: c.cached_y,
        defer: false,
        mask: c.mask,
        suppress: c.suppress.boolean(),
        scalar: c.scalar,
        age: c.age,
        timer: c.timer,
        position_y: c.position_y,
        step: c.step,
        late_flag: c.late_flag.boolean(),
        native_controller_present: c.native_controller.boolean(),
    };
    finalize(&mut state, &mut globals, cosine, c.dt, &mut boundary);
    if !state
        .working_normal
        .iter()
        .chain(state.other_normal.iter())
        .chain([state.normal_y, state.cached_y].iter())
        .all(|v| v.is_finite())
    {
        return Err("nonfinite output outside comparison domain".into());
    }
    Ok(Output {
        normal: hex(state.working_normal),
        other_normal: hex(state.other_normal),
        normal_y: hex([state.normal_y]),
        cached_y: hex([state.cached_y]),
        defer: state.defer,
        late_flag: state.late_flag,
        trace: boundary.trace,
    })
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args_os().collect();
    if args.len() != 3 {
        return Err("usage: contact_finalizer_probe INPUT OUTPUT".into());
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
    fn flag_transport_preserves_nonzero_byte() {
        let flag: Flag = serde_json::from_str("2").unwrap();
        assert_eq!(flag.byte(), 2);
        assert!(flag.boolean());
    }
    #[test]
    fn hexadecimal_output_preserves_signed_zero() {
        assert_eq!(hex([0., -0.]), "0000000000000080");
    }
}
