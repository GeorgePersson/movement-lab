//! Original normal capsule/box sweep arithmetic recovered from an offline binary.
//!
//! This is a finite, zero-W primary kernel, not a complete character controller or
//! the optional penetration-refinement algorithm. The input limits and iteration
//! budget below are explicit laboratory safeguards, not recovered native policy.

use crate::capsule_geometry::{
    Reduction, Vec4, add, cross, dot, neg, reduce_segment, reduce_tetrahedron,
    reduce_triangle_face, scale, segment_weight, sub, triangle_weights,
};
use serde::{Deserialize, Serialize};

/// Laboratory bound on supplied coordinates, displacements and scalar dimensions.
/// It keeps high-order simplex arithmetic away from overflow in the supported domain.
pub const MAX_COMPONENT_MAGNITUDE: f32 = 10_000.0;
/// Laboratory termination safeguard. The native primary has no literal loop cap.
pub const MAX_ITERATIONS: usize = 256;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct SweepInput {
    pub a: Vec4,
    pub b: Vec4,
    pub extents: Vec4,
    pub initial: Vec4,
    pub start: Vec4,
    pub motion: Vec4,
    pub radius: f32,
    pub shape_radius: f32,
    pub radius_point: bool,
    pub box_margin: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SweepHit {
    pub fraction: f32,
    pub normal: Vec4,
    pub point: Vec4,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SweepError {
    /// Outside the explicitly supported finite, zero-W, zero-start input domain.
    InvalidInput,
    /// A required arithmetic result became nonfinite.
    NonFiniteArithmetic,
    /// Laboratory iteration budget exhausted; this is never reported as a miss.
    IterationLimit,
}

fn finite(v: Vec4) -> bool {
    v.into_iter().all(f32::is_finite)
}

fn scalar(x: f32) -> Result<f32, SweepError> {
    if x.is_finite() {
        Ok(x)
    } else {
        Err(SweepError::NonFiniteArithmetic)
    }
}

fn vector(v: Vec4) -> Result<Vec4, SweepError> {
    if finite(v) {
        Ok(v)
    } else {
        Err(SweepError::NonFiniteArithmetic)
    }
}

fn validate(input: &SweepInput) -> Result<(), SweepError> {
    for v in [
        input.a,
        input.b,
        input.extents,
        input.initial,
        input.start,
        input.motion,
    ] {
        if v[3] != 0.0
            || !v
                .into_iter()
                .all(|x| x.is_finite() && x.abs() <= MAX_COMPONENT_MAGNITUDE)
        {
            return Err(SweepError::InvalidInput);
        }
    }
    if input.start != [0.0; 4]
        || input.extents[..3].iter().any(|&x| x <= 0.0)
        || [input.radius, input.shape_radius, input.box_margin]
            .into_iter()
            .any(|x| !x.is_finite() || !(0.0..=MAX_COMPONENT_MAGNITUDE).contains(&x))
    {
        return Err(SweepError::InvalidInput);
    }
    Ok(())
}

fn unit(v: Vec4) -> Result<Vec4, SweepError> {
    let length = scalar(dot(v, v))?.sqrt();
    if length == 0.0 {
        return Err(SweepError::NonFiniteArithmetic);
    }
    // Native DIVPS divides each lane. A reciprocal multiply rounds differently.
    vector(v.map(|x| x / length))
}

fn capsule_support(input: &SweepInput, direction: Vec4) -> Vec4 {
    if dot(input.b, direction) < dot(input.a, direction) {
        input.a
    } else {
        input.b
    }
}

fn box_support(input: &SweepInput, direction: Vec4) -> Vec4 {
    std::array::from_fn(|i| {
        if direction[i] > 0.0 {
            input.extents[i]
        } else {
            0.0 - input.extents[i]
        }
    })
}

fn primary_triangle(q: [Vec4; 3]) -> Result<Reduction, SweepError> {
    let n = cross(sub(q[1], q[0]), sub(q[2], q[0]));
    if scalar(dot(n, n))? <= f32::EPSILON {
        // The primary's epsilon fallback uses AB, not the longest triangle edge.
        Ok(reduce_segment(q[0], q[1]))
    } else {
        // The separate face reducer's exact-zero guard is preserved in that helper.
        reduce_triangle_face(q).ok_or(SweepError::NonFiniteArithmetic)
    }
}

/// Cast a capsule endpoint support set against a translating box support set.
///
/// Inputs are box-local vectors with W=0. `motion` moves the box relative to the
/// capsule, so it is the negative of a capsule's displacement. Only zero `start`
/// is supported: the recovered initialization omits start from its first support.
/// Returns `Ok(None)` only for the recovered not-approaching or past-end misses.
/// The output point is the capsule witness in the initial local frame; a caller
/// moving the capsule must add its displacement times `fraction` afterward.
pub fn sweep(input: &SweepInput) -> Result<Option<SweepHit>, SweepError> {
    sweep_with_limit(input, MAX_ITERATIONS)
}

fn sweep_with_limit(
    input: &SweepInput,
    iteration_limit: usize,
) -> Result<Option<SweepHit>, SweepError> {
    validate(input)?;
    let initial = if scalar(dot(input.initial, input.initial))? > f32::EPSILON {
        input.initial
    } else {
        [1.0, 0.0, 0.0, 0.0]
    };
    let n = unit(initial)?;
    let mut aa = [[0.0; 4]; 4];
    let mut bb = [[0.0; 4]; 4];
    let mut q = [[0.0; 4]; 4];
    aa[0] = capsule_support(input, neg(n));
    bb[0] = box_support(input, n);
    q[0] = sub(aa[0], bb[0]);
    let mut count = 1;
    let mut closest = q[0];
    let mut search = neg(closest);
    let mut saved_search = search;
    let mut old_closest = closest;
    let mut translation = input.start;
    let mut fraction = 0.0_f32;
    // Supported nonnegative margins produce zero here; retain the native formula.
    let minimum_margin = if input.box_margin == 0.0 {
        input.box_margin
    } else {
        0.0
    };
    let tolerance = minimum_margin * 0.1;
    let tolerance_squared = tolerance * tolerance;
    let radius = scalar(input.radius + tolerance)?;
    let radius_squared = scalar(radius * radius)?;
    let mut distance_squared = scalar(dot(search, search))?;
    let mut improved = true;
    let mut iterations = 0;

    loop {
        let continue_cast = if iterations == 0 {
            distance_squared > tolerance_squared
        } else {
            distance_squared > radius_squared && improved
        };
        if !continue_cast {
            break;
        }
        if iterations >= iteration_limit {
            return Err(SweepError::IterationLimit);
        }
        let mut previous_distance = distance_squared;
        old_closest = closest;
        let n = unit(search)?;
        let a = capsule_support(input, n);
        let mut b = vector(add(box_support(input, neg(n)), translation))?;
        let mut w = vector(sub(a, b))?;
        let gap = scalar(dot(neg(w), n) - radius)?;
        if gap > 0.0 {
            let denominator = scalar(dot(input.motion, n))?;
            if denominator >= 0.0 {
                return Ok(None);
            }
            let old_fraction = fraction;
            fraction = scalar(fraction - gap / denominator)?;
            if fraction > old_fraction {
                if fraction > 1.0 {
                    return Ok(None);
                }
                let new_translation = vector(add(scale(input.motion, fraction), input.start))?;
                let delta = sub(new_translation, translation);
                translation = new_translation;
                for i in 0..count {
                    bb[i] = vector(add(bb[i], delta))?;
                    q[i] = vector(sub(aa[i], bb[i]))?;
                }
                saved_search = search;
                previous_distance = f32::MAX;
                b = vector(add(box_support(input, neg(n)), translation))?;
                w = vector(sub(a, b))?;
            }
        }
        if count >= 4 {
            return Err(SweepError::NonFiniteArithmetic);
        }
        aa[count] = a;
        bb[count] = b;
        q[count] = w;
        count += 1;
        let reduction = match count {
            2 => reduce_segment(q[0], q[1]),
            3 => primary_triangle([q[0], q[1], q[2]])?,
            4 => reduce_tetrahedron(q),
            _ => unreachable!("a valid simplex grows from at least one point"),
        };
        closest = vector(reduction.closest)?;
        let old_q = q;
        let old_a = aa;
        let old_b = bb;
        count = reduction.count;
        if !(1..=4).contains(&count) {
            return Err(SweepError::NonFiniteArithmetic);
        }
        for (i, &index) in reduction.active_indices().iter().enumerate() {
            q[i] = old_q[index];
            aa[i] = old_a[index];
            bb[i] = old_b[index];
        }
        search = neg(closest);
        distance_squared = scalar(dot(closest, closest))?;
        improved = distance_squared < previous_distance;
        iterations += 1;
    }

    let normal_source = if distance_squared > tolerance_squared && improved {
        search
    } else {
        saved_search
    };
    let length = scalar(dot(normal_source, normal_source))?.sqrt();
    let normal = if length > f32::EPSILON {
        neg(unit(normal_source)?)
    } else {
        [0.0; 4]
    };
    let interpolation_point = if improved { closest } else { old_closest };
    let witness = match count {
        1 => aa[0],
        2 => {
            let t = scalar(segment_weight(interpolation_point, q[0], q[1]))?;
            add(scale(sub(aa[1], aa[0]), t), aa[0])
        }
        3 => {
            let (u, v) = triangle_weights(interpolation_point, q[0], q[1], q[2]);
            scalar(u)?;
            scalar(v)?;
            add(
                add(scale(sub(aa[1], aa[0]), u), scale(sub(aa[2], aa[0]), v)),
                aa[0],
            )
        }
        _ => [0.0; 4],
    };
    let point = vector(if input.radius_point {
        sub(witness, scale(normal, input.shape_radius))
    } else {
        witness
    })?;
    Ok(Some(SweepHit {
        fraction,
        normal,
        point,
    }))
}

/// Supplied-geometry adapter result. Initial overlap has no recovered hit point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CapsuleBoxHit {
    pub fraction: f32,
    pub normal: [f32; 3],
    pub point: Option<[f32; 3]>,
}

/// Laboratory endpoint/displacement adapter for an axis-aligned box.
///
/// Endpoints and dimensions are supplied geometry. This does not infer live native
/// character dimensions. Zero displacement is outside this adapter: callers should
/// skip a stationary cast and use their explicit overlap policy instead.
pub fn sweep_capsule_box(
    a: [f32; 3],
    b: [f32; 3],
    radius: f32,
    box_center: [f32; 3],
    box_extents: [f32; 3],
    displacement: [f32; 3],
) -> Result<Option<CapsuleBoxHit>, SweepError> {
    let v4 = |v: [f32; 3]| [v[0], v[1], v[2], 0.0];
    let displacement = v4(displacement);
    let length_squared = dot(displacement, displacement);
    if !length_squared.is_finite() || length_squared <= 0.0 {
        return Err(SweepError::InvalidInput);
    }
    let center = v4(box_center);
    let a = sub(v4(a), center);
    let b = sub(v4(b), center);
    let input = SweepInput {
        a,
        b,
        extents: v4(box_extents),
        initial: scale(add(a, b), 0.5),
        start: [0.0; 4],
        motion: neg(displacement),
        radius,
        shape_radius: radius,
        radius_point: true,
        box_margin: box_extents[0].min(box_extents[1]).min(box_extents[2]) * 0.05,
    };
    let Some(hit) = sweep(&input)? else {
        return Ok(None);
    };
    if hit.fraction <= 0.0 {
        let normal = neg(unit(displacement)?);
        return Ok(Some(CapsuleBoxHit {
            fraction: 0.0,
            normal: [normal[0], normal[1], normal[2]],
            point: None,
        }));
    }
    let point = vector(add(
        add(hit.point, center),
        scale(displacement, hit.fraction),
    ))?;
    Ok(Some(CapsuleBoxHit {
        fraction: hit.fraction,
        normal: [hit.normal[0], hit.normal[1], hit.normal[2]],
        point: Some([point[0], point[1], point[2]]),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approaching() -> SweepInput {
        SweepInput {
            a: [5.0, -0.5, 0.0, 0.0],
            b: [5.0, 0.5, 0.0, 0.0],
            extents: [1.0, 1.0, 1.0, 0.0],
            initial: [5.0, 0.0, 0.0, 0.0],
            start: [0.0; 4],
            motion: [10.0, 0.0, 0.0, 0.0],
            radius: 0.5,
            shape_radius: 0.5,
            radius_point: true,
            box_margin: 0.05,
        }
    }

    #[test]
    fn cast_and_miss_remain_distinct_from_errors() {
        let input = approaching();
        let hit = sweep(&input).unwrap().unwrap();
        assert!((hit.fraction - 0.35).abs() < 1.0e-6);
        assert_eq!(hit.normal[..3], [1.0, 0.0, 0.0]);
        let mut away = input;
        away.motion = [-10.0, 0.0, 0.0, 0.0];
        assert_eq!(sweep(&away), Ok(None));
        assert_eq!(sweep_with_limit(&input, 0), Err(SweepError::IterationLimit));
        let mut invalid = input;
        invalid.a[0] = f32::INFINITY;
        assert_eq!(sweep(&invalid), Err(SweepError::InvalidInput));
        invalid = input;
        invalid.a[3] = 1.0;
        assert_eq!(sweep(&invalid), Err(SweepError::InvalidInput));
    }

    #[test]
    fn adapter_translates_positive_witness_and_omits_initial_overlap_point() {
        let hit = sweep_capsule_box(
            [15.0, -0.5, 0.0],
            [15.0, 0.5, 0.0],
            0.5,
            [10.0, 0.0, 0.0],
            [1.0; 3],
            [-10.0, 0.0, 0.0],
        )
        .unwrap()
        .unwrap();
        assert!((hit.fraction - 0.35).abs() < 1.0e-6);
        assert!((hit.point.unwrap()[0] - 11.0).abs() < 1.0e-6);
        let overlap = sweep_capsule_box(
            [10.0, -0.5, 0.0],
            [10.0, 0.5, 0.0],
            0.5,
            [10.0, 0.0, 0.0],
            [1.0; 3],
            [1.0, 0.0, 0.0],
        )
        .unwrap()
        .unwrap();
        assert_eq!(overlap.fraction, 0.0);
        assert_eq!(overlap.normal, [-1.0, 0.0, 0.0]);
        assert_eq!(overlap.point, None);
    }
}
