//! Original packed segment/triangle witness (961F0/94700), finite XYZ and W=0.
//! Packed reductions differ from scalar dot products; tie order is intentional.
use crate::precise_capsule::{Triangle, Vec3};
type V4 = [f32; 4];
#[derive(Clone, Copy, Debug)]
pub struct Witness {
    pub distance: f32,
    pub segment: V4,
    pub triangle: V4,
}
fn add(a: V4, b: V4) -> V4 {
    std::array::from_fn(|i| a[i] + b[i])
}
fn sub(a: V4, b: V4) -> V4 {
    std::array::from_fn(|i| a[i] - b[i])
}
fn scale(a: V4, s: f32) -> V4 {
    a.map(|x| x * s)
}
fn dot4(a: V4, b: V4) -> f32 {
    (a[0] * b[0] + a[2] * b[2]) + (a[1] * b[1] + a[3] * b[3])
}
fn dot3(a: V4, b: V4) -> f32 {
    (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]
}
fn yxz(a: V4, b: V4) -> f32 {
    (a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]
}
fn cross(a: V4, b: V4) -> V4 {
    [
        a[1] * b[2] - b[1] * a[2],
        a[2] * b[0] - b[2] * a[0],
        a[0] * b[1] - b[0] * a[1],
        0.,
    ]
}
fn clamp(x: f32) -> f32 {
    if x.is_nan() { 1. } else { x.min(1.).max(0.) }
}
fn v4(v: Vec3) -> V4 {
    [v[0], v[1], v[2], 0.]
}
fn edge(p: V4, d: V4, q: V4, v: V4) -> (f32, V4, V4) {
    let a = dot4(d, d);
    let r = sub(p, q);
    let c = dot3(v, v);
    let b = yxz(d, v);
    let dd = yxz(d, r);
    let e = dot3(r, v);
    let det = a * c - b * b;
    let s = if det <= f32::EPSILON {
        0.5
    } else {
        clamp((e * b - dd * c) / det)
    };
    let t = clamp((s * b + e) * (1. / c));
    let s = if a <= f32::EPSILON {
        0.
    } else {
        clamp((t * b - dd) * (1. / a))
    };
    let sp = add(p, scale(d, s));
    let tp = add(q, scale(v, t));
    let delta = sub(sp, tp);
    (
        (delta[1] * delta[1] + delta[2] * delta[2]) + delta[0] * delta[0],
        sp,
        tp,
    )
}
/// None means the finite nondegenerate witness domain was exceeded.
pub fn witness(p: Vec3, q: Vec3, tri: Triangle) -> Option<Witness> {
    witness_packed(v4(p), v4(q), tri.map(v4))
}
/// Packed leaf companion for W=0, including supplied signed-zero W lanes.
/// Nonzero W, nonfinite values and arbitrary FTZ/DAZ behavior are outside scope.
pub fn witness_packed(p: V4, q: V4, tri: [V4; 3]) -> Option<Witness> {
    if [p, q, tri[0], tri[1], tri[2]]
        .into_iter()
        .any(|v| v[3] != 0.)
    {
        return None;
    }
    let [a, b, c] = tri;
    let d = sub(q, p);
    let e1 = sub(b, a);
    let e2 = sub(c, a);
    let e3 = sub(c, b);
    let aa = dot4(e1, e1);
    let ab = dot4(e2, e1);
    let bb = dot4(e2, e2);
    let det = bb * aa - ab * ab;
    let inv = if det > 0. { 1. / det } else { 0. };
    let mut n = cross(e1, e2);
    let length = dot4(n, n).sqrt();
    if !(length > 0. && length.is_finite()) {
        return None;
    }
    n = n.map(|x| x / length);
    let hp = dot4(sub(p, a), n);
    let hq = dot4(sub(q, a), n);
    let dp = hp * hp;
    let dq = hq * hq;
    let inside = |point: V4| {
        let r = sub(point, a);
        let ra = dot4(r, e1);
        let rb = dot4(r, e2);
        let u = (rb * aa - ra * ab) * inv;
        let v = (ra * bb - rb * ab) * inv;
        -f32::EPSILON <= u
            && u <= 1. + f32::EPSILON
            && -f32::EPSILON <= v
            && v <= 1. + f32::EPSILON
            && u + v < 1. + f32::EPSILON
    };
    if hp * hq < 0. {
        let hit = add(p, scale(d, (1. / dot4(d, n)) * (0. - hp)));
        if inside(hit) {
            return Some(Witness {
                distance: 0.,
                segment: hit,
                triangle: hit,
            });
        }
    }
    let edges = [edge(p, d, a, e1), edge(p, d, b, e3), edge(p, d, a, e2)];
    let ds = edges.map(|e| e.0);
    let i = if ds[0] < ds[1] && ds[0] < ds[2] {
        0
    } else if ds[1] <= ds[0] && ds[1] < ds[2] {
        1
    } else {
        2
    };
    let (_, mut sp, mut tp) = edges[i];
    let projp = sub(p, scale(n, hp));
    let projq = sub(q, scale(n, hq));
    let ip = inside(projp);
    let iq = inside(projq);
    if ip && iq {
        if dp < dq {
            sp = p;
            tp = projp;
        } else {
            sp = q;
            tp = projq;
        }
    } else if ip && dp < ds[i] {
        sp = p;
        tp = projp;
    } else if iq && dq < ds[i] {
        sp = q;
        tp = projq;
    }
    let delta = sub(tp, sp);
    let distance = dot4(delta, delta);
    if !distance.is_finite() || !sp.into_iter().chain(tp).all(f32::is_finite) {
        return None;
    }
    Some(Witness {
        distance,
        segment: sp,
        triangle: tp,
    })
}

#[cfg(test)]
mod tests {
    #[test]
    fn packed_leaf_preserves_negative_zero_w_and_rejects_nonzero_w() {
        let p = [0.25, 0.25, -0.0, -0.0];
        let mut q = p;
        let tri = [[0., 0., 0., -0.0], [1., 0., 0., -0.0], [0., 1., 0., -0.0]];
        let r = super::witness_packed(p, q, tri).unwrap();
        assert_eq!(r.segment[3].to_bits(), 0x80000000);
        q[3] = 1.;
        assert!(super::witness_packed(p, q, tri).is_none());
    }
}
