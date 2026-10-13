//! Original IV_JUMP binding kernel for supplied graph operations.
//!
//! The valid enum subset is 0..=4. Metadata conversion/clamping, other graph
//! variables, FSM selection, event queues and scheduling belong to the caller.
//! Existing `JumpProperties` stays byte-valued for legacy callers; operations
//! that read a byte as a typed value reject unsupported values without mutation.

use crate::jump_lifecycle::JumpProperties;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum JumpValue {
    #[default]
    Idle = 0,
    Start = 1,
    Jumping = 2,
    Falling = 3,
    Landing = 4,
}

impl JumpValue {
    pub const fn value(self) -> u8 {
        self as u8
    }
}

impl TryFrom<u8> for JumpValue {
    type Error = BindingError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Idle),
            1 => Ok(Self::Start),
            2 => Ok(Self::Jumping),
            3 => Ok(Self::Falling),
            4 => Ok(Self::Landing),
            other => Err(BindingError::UnsupportedValue(other)),
        }
    }
}

/// Supplied graph setter modes recovered from the concrete setter.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum WriteMode {
    StageOnly = 0,
    RespectBinding = 1,
    ReplaceBinding = 2,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BindingError {
    UnsupportedValue(u8),
    CountOverflow,
}

/// One logical binding record for IV_JUMP. A zero count represents no record.
/// Repeated acquisitions share the first captured evaluated value.
///
/// Missing serialized fields default to no binding; an owning legacy structure
/// should also mark its new binding field `#[serde(default)]`. Checked overflow
/// and treating count zero as absent are LAB safeguards: native raw counters
/// can wrap if operated outside their valid lifetime. This models no such wrap.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct JumpBinding {
    count: u32,
    captured: JumpValue,
}

impl JumpBinding {
    pub const fn lock_count(&self) -> u32 {
        self.count
    }

    pub fn captured_value(&self) -> Option<JumpValue> {
        (self.count != 0).then_some(self.captured)
    }

    /// Mode 0 writes staged only. Mode 1 also writes evaluated when unlocked.
    /// Mode 2 writes both channels and replaces an existing binding's capture.
    /// Typed input needs no descriptor conversion; unrelated existing raw bytes
    /// are preserved when the selected mode does not write that channel.
    pub fn write(&mut self, properties: &mut JumpProperties, value: JumpValue, mode: WriteMode) {
        properties.staged = value.value();
        match mode {
            WriteMode::StageOnly => {}
            WriteMode::RespectBinding => {
                if self.count == 0 {
                    properties.evaluated = value.value();
                }
            }
            WriteMode::ReplaceBinding => {
                properties.evaluated = value.value();
                if self.count != 0 {
                    self.captured = value;
                }
            }
        }
    }

    /// Capture evaluated, not staged, on the first acquisition. Later owners
    /// increment the same count without resampling either property channel.
    pub fn acquire(&mut self, properties: &JumpProperties) -> Result<(), BindingError> {
        let next = self
            .count
            .checked_add(1)
            .ok_or(BindingError::CountOverflow)?;
        let captured = if self.count == 0 {
            JumpValue::try_from(properties.evaluated)?
        } else {
            self.captured
        };
        self.count = next;
        self.captured = captured;
        Ok(())
    }

    /// Copy staged into evaluated and an existing binding's captured value,
    /// preserving its count. Call explicitly at the recovered synchronization
    /// phase; releasing a lock does not perform this operation.
    pub fn synchronize(&mut self, properties: &mut JumpProperties) -> Result<(), BindingError> {
        let value = JumpValue::try_from(properties.staged)?;
        properties.evaluated = value.value();
        if self.count != 0 {
            self.captured = value;
        }
        Ok(())
    }

    /// Release one owner; remove the logical record at zero. Returns false for
    /// an absent record. Neither property channel is changed or published.
    pub fn release(&mut self) -> bool {
        if self.count == 0 {
            return false;
        }
        self.count -= 1;
        if self.count == 0 {
            self.captured = JumpValue::Idle;
        }
        true
    }

    /// Explicit graph promotion: copy staged, then apply an active binding's
    /// captured override. No automatic frame or event cadence is implied.
    pub fn promote(&self, properties: &mut JumpProperties) -> Result<(), BindingError> {
        let staged = JumpValue::try_from(properties.staged)?;
        properties.evaluated = self.captured_value().unwrap_or(staged).value();
        Ok(())
    }
}
