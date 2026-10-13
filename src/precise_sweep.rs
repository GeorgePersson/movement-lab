//! Original finite precise capsule/box sweep, assembled from recovered equations.
//!
//! This module preserves the two solver branches and their output-write policy.
//! It does not provide scene filtering, contact response, or gameplay calibration.
//! Domain limits and reported errors are laboratory boundary checks.

use crate::{precise_capsule, precise_distance, precise_sphere};
use serde::{Deserialize, Serialize};

pub type Vector = [f32; 3];
pub const MAX_COMPONENT: f32 = 10_000.;

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct SweepInput {
    pub a: Vector,
    pub b: Vector,
    pub radius: f32,
    pub box_q: [f32; 4],
    pub box_p: Vector,
    pub extents: Vector,
    pub direction: Vector,
    pub distance: f32,
    pub flags: u16,
}

/// Optional fields describe writes, including writes on a miss. `flags` describes
/// which hit fields the recovered wrapper marks valid, separately from storage.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct SweepResult {
    pub hit: bool,
    pub distance: f32,
    pub normal: Option<Vector>,
    pub point: Option<Vector>,
    pub flags: Option<u16>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum SweepError {
    InvalidInput,
    NonFiniteArithmetic,
}

fn bounded(values: &[f32]) -> bool {
    values
        .iter()
        .all(|x| x.is_finite() && x.abs() <= MAX_COMPONENT)
}

fn validate(c: &SweepInput) -> Result<(), SweepError> {
    if ![c.a, c.b, c.box_p, c.extents, c.direction]
        .iter()
        .all(|v| bounded(v))
        || !bounded(&c.box_q)
        || !bounded(&[c.radius, c.distance])
        || c.extents.iter().any(|&x| x <= 0.)
        || c.radius < 0.
        || c.distance < 0.
    {
        return Err(SweepError::InvalidInput);
    }
    let q = c.box_q;
    let q_length = ((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3];
    let d = c.direction;
    let d_length = (d[0] * d[0] + d[1] * d[1]) + d[2] * d[2];
    // Supplied rounded unit rotations/directions, rather than implicit repair.
    if (q_length - 1.).abs() > 0.001 || (d_length - 1.).abs() > 0.001 {
        return Err(SweepError::InvalidInput);
    }
    Ok(())
}

/// Three matrix columns. Grouping differs from common quaternion conveniences.
pub fn matrix(q: [f32; 4]) -> [[f32; 3]; 3] {
    let [x, y, z, w] = q;
    let [xx, yy, zz, ww] = q.map(|v| v + v);
    let wx = w * xx + 0.;
    let wy = w * yy + 0.;
    let wz = w * zz + 0.;
    let diagonal = w * ww + -1.;
    [
        [xx * x + diagonal, xx * y + wz, xx * z + (0. - wy)],
        [yy * x + (0. - wz), yy * y + diagonal, yy * z + wx],
        [zz * x + wy, zz * y + (0. - wx), zz * z + diagonal],
    ]
}

fn flatten(m: [[f32; 3]; 3]) -> [f32; 9] {
    std::array::from_fn(|i| m[i / 3][i % 3])
}

fn translate(p: Vector, direction: Vector, distance: f32) -> Vector {
    std::array::from_fn(|i| distance * direction[i] + p[i])
}

fn sphere_point(local: Vector, m: [[f32; 3]; 3], p: Vector) -> Vector {
    std::array::from_fn(|i| ((m[0][i] * local[0] + m[1][i] * local[1]) + m[2][i] * local[2]) + p[i])
}

fn capsule_point(local: Vector, q: [f32; 4], p: Vector) -> Vector {
    let [x, y, z] = local.map(|v| v * 2.);
    let [qx, qy, qz, w] = q;
    let dot = (qy * y + qx * x) + qz * z;
    let k = w * w - 0.5;
    [
        ((qy * z - qz * y) * w + x * k) + (qx * dot + p[0]),
        ((qz * x - qx * z) * w + p[1]) + (qy * dot + y * k),
        ((qx * y - qy * x) * w + p[2]) + (qz * dot + z * k),
    ]
}

pub fn sweep(c: &SweepInput) -> Result<SweepResult, SweepError> {
    validate(c)?;
    let collapsed = c.a == c.b;
    let m = matrix(c.box_q);
    let mut result = if collapsed {
        let r = precise_sphere::sphere_box(
            c.box_p,
            c.extents,
            m,
            c.a,
            c.radius,
            c.direction,
            c.distance,
            c.flags,
        );
        SweepResult {
            hit: r.hit,
            distance: r.distance,
            normal: r.normal,
            point: None,
            flags: None,
        }
    } else {
        let r = precise_capsule::capsule_box(
            c.a,
            c.b,
            c.radius,
            c.box_q,
            c.box_p,
            c.extents,
            c.direction,
            c.distance,
            c.flags,
        );
        SweepResult {
            hit: r.hit,
            distance: r.distance,
            normal: r.normal,
            point: r.point,
            flags: None,
        }
    };
    if result.hit {
        if collapsed {
            result.normal = result
                .normal
                .map(|v| v.map(|x| f32::from_bits(x.to_bits() ^ 0x8000_0000)));
        }
        result.flags = Some(2);
        if c.flags & 1 != 0 && result.distance != 0. {
            let a = translate(c.a, c.direction, result.distance);
            result.point = Some(if collapsed {
                let local =
                    precise_distance::point_box_distance_sq(a, c.box_p, c.extents, flatten(m))
                        .point;
                sphere_point(local, m, c.box_p)
            } else {
                let b = translate(c.b, c.direction, result.distance);
                let local =
                    precise_distance::segment_box_distance_sq(a, b, c.box_p, c.extents, flatten(m))
                        .point;
                capsule_point(local, c.box_q, c.box_p)
            });
            result.flags = Some(3);
        }
    }
    if !result.distance.is_finite()
        || result
            .normal
            .iter()
            .chain(result.point.iter())
            .flatten()
            .any(|x| !x.is_finite())
        || (result.hit && result.normal.is_none())
    {
        return Err(SweepError::NonFiniteArithmetic);
    }
    Ok(result)
}

#[derive(Clone, Copy, Debug)]
pub struct CastHit {
    pub fraction: f32,
    pub normal: Vector,
}

/// LAB adapter from a finite displacement to the recovered normalized-ray API.
pub fn sweep_capsule_box(
    a: Vector,
    b: Vector,
    radius: f32,
    center: Vector,
    extents: Vector,
    displacement: Vector,
) -> Result<Option<CastHit>, SweepError> {
    if !bounded(&displacement) {
        return Err(SweepError::InvalidInput);
    }
    let [x, y, z] = displacement;
    let length = ((x * x + y * y) + z * z).sqrt();
    if !length.is_finite() || length <= 0. {
        return Err(SweepError::InvalidInput);
    }
    let result = sweep(&SweepInput {
        a,
        b,
        radius,
        box_q: [0., 0., 0., 1.],
        box_p: center,
        extents,
        direction: displacement.map(|x| x / length),
        distance: length,
        flags: 0x503,
    })?;
    if !result.hit {
        return Ok(None);
    }
    let fraction = result.distance / length;
    if !fraction.is_finite() || !(0. ..=1.).contains(&fraction) {
        return Err(SweepError::NonFiniteArithmetic);
    }
    Ok(Some(CastHit {
        fraction,
        normal: result.normal.ok_or(SweepError::NonFiniteArithmetic)?,
    }))
}
