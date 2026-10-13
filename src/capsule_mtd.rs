//! Original capsule–box minimum-translation arithmetic for supplied finite poses.
//! This is a penetration primitive, separate from sweep and scene scan policy.
//! The low-level caller supplies unit quaternions, nonnegative geometry and a
//! finite moderate arithmetic domain. No scene/configuration defaults are implied.
use crate::precise_distance::segment_box_distance_sq;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct MtdInput {
    pub radius: f32,
    pub half_height: f32,
    pub capsule_rotation: [f32; 4],
    pub capsule_position: [f32; 3],
    pub box_rotation: [f32; 4],
    pub box_position: [f32; 3],
    pub extents: [f32; 3],
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Mtd {
    pub normal: [f32; 3],
    pub depth: f32,
}
fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    (a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]
}
fn neg(v: f32) -> f32 {
    f32::from_bits(v.to_bits() ^ 0x8000_0000)
}
/// MTD-specific endpoint order; sweep quaternion preprocessing groups X differently.
pub fn capsule_endpoints(q: [f32; 4], p: [f32; 3], h: f32) -> ([f32; 3], [f32; 3]) {
    let [x, y, z, w] = q;
    let xx = x + x;
    let ww = w + w;
    let delta = [
        ((w * ww + x * xx) - 1.0) * h,
        (z * ww + y * xx) * h,
        (z * xx - y * ww) * h,
    ];
    (
        std::array::from_fn(|i| p[i] + delta[i]),
        std::array::from_fn(|i| p[i] - delta[i]),
    )
}
fn matrix(q: [f32; 4]) -> [f32; 9] {
    let [x, y, z, w] = q;
    let (xx, yy, zz) = (x + x, y + y, z + z);
    let (wx, wy, wz) = (w * xx + 0.0, w * yy + 0.0, w * zz + 0.0);
    let diag = w * (w + w) + (-1.0);
    [
        xx * x + diag,
        xx * y + wz,
        xx * z + (0.0 - wy),
        yy * x + (0.0 - wz),
        yy * y + diag,
        yy * z + wx,
        zz * x + wy,
        zz * y + (0.0 - wx),
        zz * z + diag,
    ]
}
fn axis_depth(
    axis: [f32; 3],
    a: [f32; 3],
    b: [f32; 3],
    r: f32,
    m: [f32; 9],
    p: [f32; 3],
    e: [f32; 3],
) -> Option<f32> {
    let (da, db) = (dot(axis, a), dot(axis, b));
    let hi = if db <= da { da } else { db };
    let lo = if da <= db { da } else { db };
    let center = dot(axis, p);
    let radius = (dot(axis, [m[3], m[4], m[5]]).abs() * e[1]
        + dot(axis, [m[0], m[1], m[2]]).abs() * e[0])
        + dot(axis, [m[6], m[7], m[8]]).abs() * e[2];
    let (low, high) = (center - radius, radius + center);
    let (cap_high, cap_low) = (hi + r, lo - r);
    if cap_high < low || high < cap_low {
        return None;
    }
    let (left, right) = (cap_high - low, high - cap_low);
    Some(if right <= left { right } else { left })
}
fn fallback(
    a: [f32; 3],
    b: [f32; 3],
    r: f32,
    m: [f32; 9],
    p: [f32; 3],
    e: [f32; 3],
) -> Option<Mtd> {
    let mut best = f32::MAX;
    let mut normal = [0.0; 3];
    for i in 0..3 {
        let axis = [m[3 * i], m[3 * i + 1], m[3 * i + 2]];
        let depth = axis_depth(axis, a, b, r, m, p, e)?;
        if depth < best {
            best = depth;
            normal = axis;
        }
    }
    let mut v = std::array::from_fn(|i| b[i] - a[i]);
    let square = dot(v, v);
    if square <= 0.0 {
        v = [0.0; 3];
    } else {
        let inv = 1.0 / square.sqrt();
        v = v.map(|x| inv * x);
    }
    for i in 0..3 {
        let ax = [m[3 * i], m[3 * i + 1], m[3 * i + 2]];
        let mut cross = [
            ax[2] * v[1] - ax[1] * v[2],
            ax[0] * v[2] - ax[2] * v[0],
            ax[1] * v[0] - ax[0] * v[1],
        ];
        if cross.iter().any(|x| x.abs() > 1e-6) {
            // This path sums X and Y squares before Z, in the recovered order.
            let square = (cross[0] * cross[0] + cross[1] * cross[1]) + cross[2] * cross[2];
            if square <= 0.0 {
                cross = [0.0; 3];
            } else {
                let inv = 1.0 / square.sqrt();
                cross = cross.map(|x| inv * x);
            }
            let depth = axis_depth(cross, a, b, r, m, p, e)?;
            if depth < best {
                best = depth;
                normal = cross;
            }
        }
    }
    let midpoint = std::array::from_fn(|i| (a[i] + b[i]) * 0.5 - p[i]);
    if dot(midpoint, normal) < 0.0 {
        normal = normal.map(neg);
    }
    Some(Mtd {
        normal,
        depth: if best <= 0.0 { 0.0 } else { best },
    })
}
/// Returns a supplied-pose penetration result. Exact tangency can return depth zero.
pub fn capsule_box_mtd(c: &MtdInput) -> Option<Mtd> {
    let (a, b) = capsule_endpoints(c.capsule_rotation, c.capsule_position, c.half_height);
    let m = matrix(c.box_rotation);
    let result = segment_box_distance_sq(a, b, c.box_position, c.extents, m);
    if c.radius * c.radius < result.distance {
        return None;
    }
    if result.distance != 0.0 {
        let q = result.point;
        let world: [f32; 3] = std::array::from_fn(|j| {
            ((m[j] * q[0] + m[3 + j] * q[1]) + m[6 + j] * q[2]) + c.box_position[j]
        });
        let delta = std::array::from_fn(|i| ((b[i] - a[i]) * result.t + a[i]) - world[i]);
        let length = dot(delta, delta).sqrt();
        if length != 0.0 {
            let inv = 1.0 / length;
            let depth = c.radius - result.distance.sqrt();
            return Some(Mtd {
                normal: delta.map(|x| x * inv),
                depth: if depth <= 0.0 { 0.0 } else { depth },
            });
        }
    }
    fallback(a, b, c.radius, m, c.box_position, c.extents)
}
