//! Original finite overlap-recovery scan with a supplied penetration callback.
//!
//! This subset accepts static box records with identity actor/local rotations.
//! It does not fabricate penetration normals, depth, scene candidates or actor
//! transforms. The caller supplies those boundaries explicitly. The scan guard
//! is a LAB safety policy; the recovered loop does not have that termination cap.
use serde::{Deserialize, Serialize};

pub type Vector = [f32; 3];

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct BoxRecord {
    /// Native record admission discriminator; values 0 and 1 are skipped.
    pub record_type: u32,
    pub actor_type: u16,
    pub actor_flags: u8,
    pub extents: Vector,
    pub actor_position: Vector,
    pub local_position: Vector,
}

#[derive(Clone, Debug, Serialize)]
pub struct Query {
    pub record: usize,
    pub capsule_q: [f32; 4],
    pub radius: f32,
    pub half_height: f32,
    pub center: Vector,
    pub box_extents: Vector,
    pub box_center: Vector,
}

#[derive(Clone, Copy, Debug)]
pub struct Penetration {
    pub normal: Vector,
    pub depth: f32,
}

#[derive(Clone, Debug, Serialize)]
pub struct Recovery {
    pub position: Vector,
    pub successes: usize,
    pub scans: usize,
    pub queries: Vec<Query>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Error {
    InvalidInput,
    NonFiniteArithmetic,
    QueryFailed,
    /// LAB guard only: no claim that native terminates an all-skipped scan.
    ScanLimit,
}

/// Recover a supplied center against ordered box records. `None` is a completed
/// nonpenetrating query; it does not stop the current scan. Skipped records do
/// not overwrite the last query result. A scan may perform more than four
/// corrections because the success threshold is checked only between scans.
#[allow(clippy::too_many_arguments)]
pub fn recover<F>(
    center: Vector,
    capsule_q: [f32; 4],
    radius: f32,
    height: f32,
    contact_offset: f32,
    records: &[BoxRecord],
    max_scans: usize,
    mut penetration: F,
) -> Result<Recovery, Error>
where
    F: FnMut(&Query) -> Result<Option<Penetration>, Error>,
{
    if !center
        .into_iter()
        .chain(capsule_q)
        .chain([radius, height, contact_offset])
        .all(f32::is_finite)
        || radius < 0.0
        || height < 0.0
        || contact_offset < 0.0
        || records.iter().any(|r| {
            !r.extents
                .into_iter()
                .chain(r.actor_position)
                .chain(r.local_position)
                .all(f32::is_finite)
                || r.extents.into_iter().any(|v| v < 0.0)
        })
    {
        return Err(Error::InvalidInput);
    }
    let inflated_radius = radius + contact_offset;
    let half_height = height * 0.5;
    if !inflated_radius.is_finite() || !half_height.is_finite() {
        return Err(Error::NonFiniteArithmetic);
    }
    let mut result = Recovery {
        position: center,
        successes: 0,
        scans: 0,
        queries: Vec::new(),
    };
    let mut last_hit = true;
    while last_hit {
        if result.successes >= 4 {
            return Ok(result);
        }
        if result.scans >= max_scans {
            return Err(Error::ScanLimit);
        }
        result.scans += 1;
        for (index, record) in records.iter().enumerate() {
            if record.record_type <= 1 {
                continue;
            }
            if !(record.actor_type == 6 || (record.actor_type == 5 && record.actor_flags & 1 != 0))
            {
                continue;
            }
            let box_center =
                std::array::from_fn(|i| record.actor_position[i] + record.local_position[i]);
            if !box_center.into_iter().all(f32::is_finite) {
                return Err(Error::NonFiniteArithmetic);
            }
            let query = Query {
                record: index,
                capsule_q,
                radius: inflated_radius,
                half_height,
                center: result.position,
                box_extents: record.extents,
                box_center,
            };
            let hit = penetration(&query)?;
            result.queries.push(query);
            last_hit = hit.is_some();
            if let Some(hit) = hit {
                if !hit
                    .normal
                    .into_iter()
                    .chain([hit.depth])
                    .all(f32::is_finite)
                {
                    return Err(Error::InvalidInput);
                }
                result.successes += 1;
                for (position, normal) in result.position.iter_mut().zip(hit.normal) {
                    let correction = normal * hit.depth;
                    *position += correction;
                }
                if !result.position.into_iter().all(f32::is_finite) {
                    return Err(Error::NonFiniteArithmetic);
                }
            }
        }
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn record() -> BoxRecord {
        BoxRecord {
            record_type: 3,
            actor_type: 6,
            actor_flags: 0,
            extents: [1.0; 3],
            actor_position: [0.0; 3],
            local_position: [0.0; 3],
        }
    }
    fn hit() -> Option<Penetration> {
        Some(Penetration {
            normal: [1.0, 0.0, 0.0],
            depth: 0.25,
        })
    }
    #[test]
    fn threshold_is_between_scans() {
        let out = recover(
            [0.0; 3],
            [0.0, 0.0, 0.0, 1.0],
            0.3,
            1.2,
            0.1,
            &vec![record(); 6],
            16,
            |_| Ok(hit()),
        )
        .unwrap();
        assert_eq!((out.successes, out.scans, out.queries.len()), (6, 1, 6));
        assert_eq!(out.position, [1.5, 0.0, 0.0]);
    }
    #[test]
    fn final_query_result_controls_repeat_and_skips_preserve_it() {
        let mut skipped = record();
        skipped.record_type = 0;
        let mut call = 0;
        let out = recover(
            [0.0; 3],
            [0.0, 0.0, 0.0, 1.0],
            0.3,
            1.2,
            0.1,
            &[record(), record(), skipped],
            16,
            |_| {
                call += 1;
                Ok(if call % 2 == 1 { hit() } else { None })
            },
        )
        .unwrap();
        assert_eq!((out.successes, out.scans, out.queries.len()), (1, 1, 2));
    }
    #[test]
    fn empty_or_ineligible_stream_is_explicit_guard_error() {
        assert!(matches!(
            recover(
                [0.0; 3],
                [0.0, 0.0, 0.0, 1.0],
                0.3,
                1.2,
                0.1,
                &[],
                2,
                |_| unreachable!()
            ),
            Err(Error::ScanLimit)
        ));
        let mut skipped = record();
        skipped.actor_type = 5;
        assert!(matches!(
            recover(
                [0.0; 3],
                [0.0, 0.0, 0.0, 1.0],
                0.3,
                1.2,
                0.1,
                &[skipped],
                2,
                |_| unreachable!()
            ),
            Err(Error::ScanLimit)
        ));
    }
}
