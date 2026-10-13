//! Original finite-f32 sphere/box and ray equations recovered and compared offline.
//!
//! Inputs must satisfy the caller's finite, nonoverflowing geometry domain. These
//! low-level functions perform no domain validation. Rotations are supplied as
//! three columns. Ordinary scalar operations intentionally retain their order;
//! do not replace them with fused multiply-adds or vector reductions.
use serde::Serialize;

pub type Vec3 = [f32; 3];
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RaySphereHit {
    pub hit: bool,
    /// A written parameter can exist even when the final distance check fails.
    pub t: Option<f32>,
    pub point: Option<Vec3>,
}
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct RayCapsuleHit {
    pub count: usize,
    /// Written slots in their original order; consume only `count` roots.
    /// A degenerate miss can still write the first slot.
    pub roots: Vec<f32>,
}
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct SphereHit {
    pub hit: bool,
    pub distance: f32,
    /// None denotes an untouched output, including ordinary misses.
    pub normal: Option<Vec3>,
}
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct RayAabbHit {
    pub face: i32,
    pub near: f32,
    pub far: f32,
}

pub fn dot(a: Vec3, b: Vec3) -> f32 {
    (a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]
}
pub fn vsub(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] - b[i])
}
pub fn vadd(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] + b[i])
}
pub fn scale(v: Vec3, t: f32) -> Vec3 {
    v.map(|x| x * t)
}
pub fn unit(v: Vec3) -> Vec3 {
    let n = dot(v, v).sqrt();
    if n > 0.0 { scale(v, 1.0 / n) } else { v }
}

pub fn ray_sphere_basic(
    origin: Vec3,
    direction: Vec3,
    max_distance: f32,
    center: Vec3,
    radius: f32,
    want_point: bool,
) -> RaySphereHit {
    let delta = vsub(center, origin);
    let projection = dot(delta, direction);
    let distance2 = dot(delta, delta);
    let radius2 = radius * radius;
    if distance2 <= radius2 {
        return RaySphereHit {
            hit: true,
            t: Some(0.0),
            point: want_point.then_some(origin),
        };
    }
    if projection <= 0.0 || projection - max_distance > radius {
        return RaySphereHit {
            hit: false,
            t: None,
            point: None,
        };
    }
    let discr = radius2 - (distance2 - projection * projection);
    if discr < 0.0 {
        return RaySphereHit {
            hit: false,
            t: None,
            point: None,
        };
    }
    let t = projection - discr.sqrt();
    if t > max_distance {
        return RaySphereHit {
            hit: false,
            t: Some(t),
            point: None,
        };
    }
    RaySphereHit {
        hit: true,
        t: Some(t),
        point: want_point.then(|| vadd(scale(direction, t), origin)),
    }
}

pub fn ray_sphere(
    origin: Vec3,
    direction: Vec3,
    max_distance: f32,
    center: Vec3,
    radius: f32,
    want_point: bool,
) -> RaySphereHit {
    let delta = vsub(origin, center);
    let mut shift = (dot(delta, delta).sqrt() - radius) - 10.0;
    if shift <= 0.0 {
        shift = 0.0;
    }
    let mut result = ray_sphere_basic(
        vadd(scale(direction, shift), origin),
        direction,
        max_distance - shift,
        center,
        radius,
        want_point,
    );
    if result.hit {
        result.t = result.t.map(|t| shift + t);
    }
    result
}

fn roots_result(roots: Vec<f32>) -> RayCapsuleHit {
    RayCapsuleHit {
        count: roots.len(),
        roots,
    }
}

pub fn ray_capsule(origin: Vec3, direction: Vec3, a: Vec3, b: Vec3, radius: f32) -> RayCapsuleHit {
    let mut axis = vsub(b, a);
    let length = dot(axis, axis).sqrt();
    if length != 0.0 {
        axis = scale(axis, 1.0 / length);
    }
    if length <= 1e-6_f32 {
        let da = vsub(origin, a);
        let db = vsub(origin, b);
        let m = dot(db, db).max(dot(da, da));
        let limit = (m + m) + (radius + radius);
        let r = ray_sphere(origin, direction, limit, a, radius, false);
        return RayCapsuleHit {
            count: usize::from(r.hit),
            roots: r.t.into_iter().collect(),
        };
    }
    let [x, y, z] = axis;
    let u = if x.abs() < y.abs() {
        let inv = 1.0 / (y * y + z * z).sqrt();
        [0.0, inv * z, -(inv * y)]
    } else {
        let inv = 1.0 / (x * x + z * z).sqrt();
        [-(inv * z), 0.0, inv * x]
    };
    let v = unit([
        u[2] * y - u[1] * z,
        u[0] * z - u[2] * x,
        u[1] * x - u[0] * y,
    ]);
    let dx = dot(u, direction);
    let dy = dot(v, direction);
    let dz = dot(axis, direction);
    let dlen = ((dy * dy + dx * dx) + dz * dz).sqrt();
    let invlen = if dlen == 0.0 { 0.0 } else { 1.0 / dlen };
    let nx = dx * invlen;
    let ny = dy * invlen;
    let nz = dz * invlen;
    let delta = vsub(origin, a);
    let partial = y * delta[1] + x * delta[0];
    let pz = z * delta[2] + partial;
    let px = dot(u, delta);
    let py = dot(v, delta);
    if nz.abs() >= f32::from_bits(0x3f7ffffe) || dlen < f32::EPSILON {
        let disc = (radius * radius - px * px) - py * py;
        if dz < 0.0 && disc >= 0.0 {
            let root = disc.sqrt();
            return roots_result(vec![
                (pz + root) * invlen,
                -(((length - pz) + root) * invlen),
            ]);
        }
        if dz > 0.0 && disc >= 0.0 {
            let root = disc.sqrt();
            return roots_result(vec![
                -((pz + root) * invlen),
                ((length - pz) + root) * invlen,
            ]);
        }
        return roots_result(vec![]);
    }
    let qa = nx * nx + ny * ny;
    let mut qb = px * nx + py * ny;
    let mut qc = (px * px + py * py) - radius * radius;
    let discr = qb * qb - qc * qa;
    if discr < 0.0 {
        return roots_result(vec![]);
    }
    let mut roots = Vec::with_capacity(2);
    let inva = 1.0 / qa;
    if discr <= 0.0 {
        let t = -(inva * qb);
        let axial = t * nz + pz;
        if 0.0 <= axial && axial <= length {
            return roots_result(vec![t * invlen]);
        }
    } else {
        let root = discr.sqrt();
        let lo = (-qb - root) * inva;
        let hi = (root - qb) * inva;
        for t in [lo, hi] {
            let axial = t * nz + pz;
            if -0.001_f32 <= axial && axial <= length + 0.001_f32 {
                roots.push(t * invlen);
            }
        }
        if roots.len() == 2 {
            return roots_result(roots);
        }
    }
    qc += pz * pz;
    qb = pz * nz + qb;
    let discr = qb * qb - qc;
    if discr > 0.0 {
        let root = discr.sqrt();
        for t in [-qb - root, root - qb] {
            if nz * t + pz <= 0.0 {
                roots.push(t * invlen);
                if roots.len() == 2 {
                    return roots_result(roots);
                }
            }
        }
    } else if discr == 0.0 {
        let t = -qb;
        if nz * t + pz <= 0.0 {
            roots.push(t * invlen);
            if roots.len() == 2 {
                return roots_result(roots);
            }
        }
    }
    qb -= nz * length;
    let axial2 = ((z + z) * delta[2] + partial) + partial;
    let discr = qb * qb - ((length - axial2) * length + qc);
    if discr > 0.0 {
        let root = discr.sqrt();
        for t in [-qb - root, root - qb] {
            if length <= nz * t + pz {
                roots.push(t * invlen);
                if roots.len() == 2 {
                    return roots_result(roots);
                }
            }
        }
    } else if discr == 0.0 {
        let t = -qb;
        if nz * t + pz >= length {
            roots.push(t * invlen);
        }
    }
    roots_result(roots)
}

pub fn obb_points(center: Vec3, extents: Vec3, rotation: [Vec3; 3]) -> [Vec3; 8] {
    let x = scale(rotation[0], extents[0]);
    let y = scale(rotation[1], extents[1]);
    let z = scale(rotation[2], extents[2]);
    let minus = vsub(center, x);
    let plus = vadd(center, x);
    let yz = vadd(z, y);
    let ymz = vsub(y, z);
    [
        vsub(minus, yz),
        vsub(plus, yz),
        vadd(ymz, plus),
        vadd(minus, ymz),
        vsub(minus, ymz),
        vsub(plus, ymz),
        vadd(yz, plus),
        vadd(minus, yz),
    ]
}
pub fn rotate(rotation: [Vec3; 3], v: Vec3) -> Vec3 {
    std::array::from_fn(|i| (rotation[1][i] * v[1] + rotation[0][i] * v[0]) + rotation[2][i] * v[2])
}
pub fn inverse_rotate(rotation: [Vec3; 3], v: Vec3) -> Vec3 {
    rotation.map(|col| dot(col, v))
}
pub fn sphere_overlap(
    origin: Vec3,
    radius: f32,
    center: Vec3,
    extents: Vec3,
    rotation: [Vec3; 3],
) -> bool {
    let delta = vsub(origin, center);
    let local = inverse_rotate(rotation, delta);
    let mut clamped = local;
    let mut outside = false;
    for i in 0..3 {
        if local[i] < -extents[i] {
            clamped[i] = -extents[i];
            outside = true;
        } else if local[i] > extents[i] {
            clamped[i] = extents[i];
            outside = true;
        }
    }
    if !outside {
        return true;
    }
    let residual = vsub(delta, rotate(rotation, clamped));
    dot(residual, residual) <= radius * radius
}
pub fn ray_aabb(minimum: Vec3, maximum: Vec3, origin: Vec3, direction: Vec3) -> RayAabbHit {
    let mut near = -f32::MAX;
    let mut far = f32::MAX;
    let mut face = -1;
    let eps = f32::EPSILON;
    for i in 0..3 {
        let d = direction[i];
        if -eps < d && d < eps {
            if origin[i] < minimum[i] || origin[i] > maximum[i] {
                return RayAabbHit {
                    face: -1,
                    near,
                    far,
                };
            }
        } else {
            let inv = 1.0 / d;
            let mut lo = (minimum[i] - origin[i]) * inv;
            let mut hi = (maximum[i] - origin[i]) * inv;
            let mut which = i as i32;
            if lo > hi {
                std::mem::swap(&mut lo, &mut hi);
                which = i as i32 + 3;
            }
            if lo > near {
                near = lo;
                face = which;
            }
            if hi < far {
                far = hi;
            }
            if far < near || far < eps {
                return RayAabbHit {
                    face: -1,
                    near,
                    far,
                };
            }
        }
    }
    RayAabbHit {
        face: if far >= near && far >= eps { face } else { -1 },
        near,
        far,
    }
}

/// Always returns the last distance output; a miss can leave the normal untouched.
#[allow(clippy::too_many_arguments)]
pub fn sphere_box(
    center: Vec3,
    extents: Vec3,
    rotation: [Vec3; 3],
    origin: Vec3,
    radius: f32,
    direction: Vec3,
    max_distance: f32,
    flags: u16,
) -> SphereHit {
    if flags & 0x10 == 0 && sphere_overlap(origin, radius, center, extents, rotation) {
        return SphereHit {
            hit: true,
            distance: 0.0,
            normal: Some(direction.map(|x| -x)),
        };
    }
    let points = obb_points(center, extents, rotation);
    let mut best = max_distance;
    let mut normal = None;
    let mut hit = false;
    // Ordered geometric edges of the corner convention above, not an unordered set.
    for (i, j) in [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (7, 6),
        (6, 5),
        (5, 4),
        (4, 7),
        (1, 5),
        (6, 2),
        (3, 7),
        (4, 0),
    ] {
        let a = points[i];
        let b = points[j];
        let delta = vsub(origin, a);
        let edge = vsub(b, a);
        let projection = dot(delta, edge);
        let residual = if projection > 0.0 {
            let edge2 = dot(edge, edge);
            vsub(
                delta,
                if projection < edge2 {
                    scale(edge, projection / edge2)
                } else {
                    edge
                },
            )
        } else {
            delta
        };
        let gap = dot(residual, residual).sqrt() - radius;
        let t = if gap > 0.0 {
            let shift = if gap <= 10.0 { 0.0 } else { gap + -10.0 };
            let result = ray_capsule(
                vadd(scale(direction, shift), origin),
                direction,
                a,
                b,
                radius,
            );
            if result.count == 0 {
                continue;
            }
            let t = if result.count == 1 || result.roots[0] <= result.roots[1] {
                result.roots[0]
            } else {
                result.roots[1]
            };
            let t = t + shift;
            if t < 0.0 {
                continue;
            }
            t
        } else {
            0.0
        };
        if t <= best {
            let p = vadd(scale(direction, t), origin);
            let edge = vsub(b, a);
            let projection = dot(vsub(p, a), edge);
            let weight = if projection > 0.0 {
                let den = dot(edge, edge);
                if projection < den {
                    projection / den
                } else {
                    1.0
                }
            } else {
                0.0
            };
            normal = Some(unit(vsub(vadd(scale(edge, weight), a), p)));
            hit = true;
            best = t;
        }
    }
    let local = inverse_rotate(rotation, vsub(origin, center));
    let ray = inverse_rotate(rotation, direction);
    for axis in 0..3 {
        let mut expanded = extents;
        expanded[axis] += radius;
        let result = ray_aabb(expanded.map(|x| -x), expanded, local, ray);
        if result.face != -1 && -1e-5_f32 <= result.near && result.near <= best {
            best = if result.near <= 0.0 { 0.0 } else { result.near };
            hit = true;
            let mut face_normal = [0.0; 3];
            face_normal[result.face as usize % 3] = if result.face < 3 { 1.0 } else { -1.0 };
            normal = Some(rotate(rotation, face_normal));
        }
    }
    SphereHit {
        hit,
        distance: best,
        normal,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn ray_final_reject_retains_written_parameter() {
        let r = ray_sphere_basic([-2.0, 0.5, 0.0], [1.0, 0.0, 0.0], 1.0, [0.0; 3], 1.0, true);
        assert!(!r.hit);
        assert_eq!(r.t.unwrap().to_bits(), (2.0 - 0.75_f32.sqrt()).to_bits());
        assert!(r.point.is_none());
        let away = ray_sphere_basic(
            [-2.0, 0.5, 0.0],
            [-1.0, 0.0, 0.0],
            10.0,
            [0.0; 3],
            1.0,
            true,
        );
        assert!(!away.hit && away.t.is_none() && away.point.is_none());
    }
    #[test]
    fn slab_equal_entry_retains_first_face() {
        let r = ray_aabb([-1.0; 3], [1.0; 3], [-2.0, -2.0, 0.0], [1.0, 1.0, 0.0]);
        assert_eq!(
            r,
            RayAabbHit {
                face: 0,
                near: 1.0,
                far: 3.0
            }
        );
    }
    #[test]
    fn sphere_overlap_keeps_signed_normal_and_miss_output() {
        let rotation = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let h = sphere_box(
            [0.0; 3],
            [1.0; 3],
            rotation,
            [0.0; 3],
            0.5,
            [1.0, 0.0, -0.0],
            10.0,
            0,
        );
        assert!(h.hit);
        assert_eq!(
            h.normal.unwrap().map(f32::to_bits),
            [-1.0_f32, -0.0, 0.0].map(f32::to_bits)
        );
        let m = sphere_box(
            [0.0; 3],
            [1.0; 3],
            rotation,
            [5.0, 5.0, 5.0],
            0.5,
            [1.0, 0.0, 0.0],
            10.0,
            0,
        );
        assert!(!m.hit && m.normal.is_none());
        assert_eq!(m.distance, 10.0);
    }
}
