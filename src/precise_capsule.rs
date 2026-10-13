//! Original finite capsule/box sweep arithmetic reconstructed from private evidence.
//! Separate scalar operations preserve the tested f32 order; no fused operations.
//! Callers validate geometry and finite intermediate bounds. This module does not
//! implement scene traversal, allocation failure, or controller contact policy.

use crate::precise_distance::segment_box_distance_sq;
use crate::precise_sphere::{ray_capsule, ray_sphere};

pub type Vec3 = [f32; 3];
pub type Triangle = [Vec3; 3];

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CapsuleHit {
    pub hit: bool,
    pub distance: f32,
    pub normal: Option<Vec3>,
    pub point: Option<Vec3>,
    pub initial: bool,
}

#[inline]
fn add(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] + b[i])
}
#[inline]
fn sub(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] - b[i])
}
#[inline]
fn scale(a: Vec3, b: f32) -> Vec3 {
    std::array::from_fn(|i| a[i] * b)
}
#[inline]
fn neg(a: Vec3) -> Vec3 {
    a.map(|x| f32::from_bits(x.to_bits() ^ 0x8000_0000))
}
#[inline]
fn dot(a: Vec3, b: Vec3) -> f32 {
    (a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]
}
#[inline]
fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - b[1] * a[2],
        b[0] * a[2] - a[0] * b[2],
        b[1] * a[0] - a[1] * b[0],
    ]
}
#[inline]
fn length(a: Vec3) -> f32 {
    dot(a, a).sqrt()
}

/// Column-major rotation, retaining the recovered arithmetic grouping.
pub fn matrix(q: [f32; 4]) -> [f32; 9] {
    let [x, y, z, w] = q;
    let [xx, yy, zz, ww] = q.map(|a| a + a);
    let wx = w * xx + 0.0;
    let wy = w * yy + 0.0;
    let wz = w * zz + 0.0;
    let diag = w * ww + -1.0;
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
fn transform(v: Vec3, q: [f32; 4], p: Vec3, third: bool) -> Vec3 {
    let d = add(v, v);
    let xyz = [q[0], q[1], q[2]];
    let s = dot(d, xyz);
    let k = q[3] * q[3] - 0.5;
    let cr = cross(xyz, d);
    std::array::from_fn(|i| {
        if third {
            (cr[i] * q[3] + d[i] * k) + (s * q[i] + p[i])
        } else {
            (cr[i] * q[3] + p[i]) + (d[i] * k + s * q[i])
        }
    })
}

/// Construct a box's faces geometrically, then extrude each admitted triangle.
/// The optional pose supports isolated arithmetic checks; actual capsule calls
/// always provide a pose. Returns triangles and their source face-triangle IDs.
pub fn build_triangles(
    extents: Vec3,
    pose: Option<([f32; 4], Vec3)>,
    half: Vec3,
    direction: Vec3,
) -> (Vec<Triangle>, Vec<usize>) {
    let [x, y, z] = extents;
    let nx = -x;
    let ny = -y;
    let nz = -z;
    let ring = [[nx, nz], [x, nz], [x, z], [nx, z]];
    let mut quads = Vec::with_capacity(6);
    for i in 0..4 {
        let a = ring[i];
        let b = ring[(i + 1) % 4];
        quads.push([
            [a[0], ny, a[1]],
            [a[0], y, a[1]],
            [b[0], y, b[1]],
            [b[0], ny, b[1]],
        ]);
    }
    quads.push([[nx, y, nz], [nx, y, z], [x, y, z], [x, y, nz]]);
    quads.push([[x, ny, z], [nx, ny, z], [nx, ny, nz], [x, ny, nz]]);
    let mut triangles = Vec::with_capacity(84);
    let mut parents = Vec::with_capacity(84);
    for (face, quad) in quads.into_iter().enumerate() {
        for (split, mut tri) in [[quad[0], quad[2], quad[3]], [quad[0], quad[1], quad[2]]]
            .into_iter()
            .enumerate()
        {
            if let Some((q, p)) = pose {
                for (j, v) in tri.iter_mut().enumerate() {
                    *v = transform(*v, q, p, j == 2);
                }
            }
            let [a, b, c] = tri;
            let n = cross(sub(b, a), sub(c, a));
            if dot(n, direction) > 0.0 {
                continue;
            }
            let ap = add(half, a);
            let bp = add(half, b);
            let cp = add(half, c);
            let am = sub(a, half);
            let bm = sub(b, half);
            let cm = sub(c, half);
            triangles.push(if dot(n, half) >= 0.0 {
                [ap, bp, cp]
            } else {
                [am, bm, cm]
            });
            parents.push(face * 2 + split);
            for mut side in [
                [bm, bp, cp],
                [bm, cp, cm],
                [am, cm, cp],
                [am, cp, ap],
                [ap, bp, bm],
                [ap, bm, am],
            ] {
                if dot(
                    cross(sub(side[1], side[0]), sub(side[2], side[0])),
                    direction,
                ) > 0.0
                {
                    side.swap(1, 2);
                }
                triangles.push(side);
                parents.push(face * 2 + split);
            }
        }
    }
    (triangles, parents)
}

/// Closest point plus weights for triangle vertices B and C.
pub fn closest(p: Vec3, tri: Triangle) -> (Vec3, f32, f32) {
    let [a, b, c] = tri;
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ap = sub(p, a);
    let d1 = dot(ap, ab);
    let d2 = dot(ap, ac);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, 0.0, 0.0);
    }
    let bp = sub(p, b);
    let d3 = dot(bp, ab);
    let d4 = dot(bp, ac);
    if d3 >= 0.0 && d4 <= d3 {
        return (b, 1.0, 0.0);
    }
    let vc = d4 * d1 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (add(scale(ab, v), a), v, 0.0);
    }
    let cp = sub(p, c);
    let d5 = dot(cp, ab);
    let d6 = dot(cp, ac);
    if d6 >= 0.0 && d5 <= d6 {
        return (c, 0.0, 1.0);
    }
    let vb = d5 * d2 - d6 * d1;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (add(scale(ac, w), a), 0.0, w);
    }
    let va = d6 * d3 - d5 * d4;
    let n = d4 - d3;
    if va <= 0.0 && n >= 0.0 && d5 - d6 >= 0.0 {
        let w = n / ((d5 - d6) + n);
        return (add(scale(sub(c, b), w), b), 1.0 - w, w);
    }
    let inv = 1.0 / ((va + vb) + vc);
    let w = inv * vc;
    let v = inv * vb;
    (add(add(scale(ab, v), a), scale(ac, w)), v, w)
}

pub fn contact(center: Vec3, direction: Vec3, time: f32, tri: Triangle) -> (Vec3, Vec3) {
    let p = add(scale(direction, time), center);
    let (point, _, _) = closest(p, tri);
    let mut n = sub(p, point);
    let l = length(n);
    if l > 0.0 {
        n = scale(n, 1.0 / l);
    }
    if l < 0.001 {
        n = cross(sub(tri[1], tri[0]), sub(tri[2], tri[0]));
        let l = length(n);
        if l > 0.0 {
            n = scale(n, 1.0 / l);
        }
    }
    (point, n)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TriangleHit {
    pub time: f32,
    pub feature: u8,
}
#[derive(Clone, Copy)]
enum Feature {
    Vertex(usize),
    Edge(usize, usize),
}
fn past(p: Vec3, origin: Vec3, end: Vec3) -> bool {
    let e = sub(end, origin);
    dot(e, e) <= dot(sub(p, origin), e)
}

/// One classified feature is queried; this deliberately does not replace the
/// classification with a minimum over every edge and vertex.
pub fn triangle_hit(
    tri: Triangle,
    normal: Vec3,
    center: Vec3,
    radius: f32,
    direction: Vec3,
    check_initial: bool,
) -> Option<TriangleHit> {
    let [a, b, c] = tri;
    let ab = sub(b, a);
    let ac = sub(c, a);
    if check_initial {
        let (p, _, _) = closest(center, tri);
        let d = sub(p, center);
        if dot(d, d) <= radius * radius {
            return Some(TriangleHit {
                time: 0.0,
                feature: 0,
            });
        }
    }
    let mut offset = scale(normal, radius);
    if dot(direction, offset) >= 0.0 {
        offset = neg(offset);
    }
    let pv = cross(direction, ac);
    let det = dot(pv, ab);
    if -1e-5 < det && det < 1e-5 {
        return None;
    }
    let tv = sub(sub(center, offset), a);
    let inv = 1.0 / det;
    let u = dot(tv, pv) * inv;
    let qv = cross(tv, ab);
    let v = dot(direction, qv) * inv;
    if u >= 0.0 && u <= 1.0 && v >= 0.0 && u + v <= 1.0 {
        let time = dot(qv, ac) * inv;
        return (time >= 0.0).then_some(TriangleHit { time, feature: 1 });
    }
    let projected = || {
        let w = (1.0 - u) - v;
        add(add(scale(c, v), scale(b, u)), scale(a, w))
    };
    let feature = if u < 0.0 {
        if v >= 0.0 {
            if u + v <= 1.0 {
                Feature::Edge(0, 2)
            } else {
                let p = projected();
                if past(p, a, c) {
                    if past(p, b, c) {
                        Feature::Vertex(2)
                    } else {
                        Feature::Edge(2, 1)
                    }
                } else {
                    Feature::Edge(2, 0)
                }
            }
        } else {
            let p = projected();
            if !past(p, b, a) {
                Feature::Edge(0, 1)
            } else if !past(p, c, a) {
                Feature::Edge(0, 2)
            } else {
                Feature::Vertex(0)
            }
        }
    } else if v >= 0.0 {
        Feature::Edge(1, 2)
    } else if u + v <= 1.0 {
        Feature::Edge(0, 1)
    } else {
        let p = projected();
        if past(p, a, b) {
            if past(p, c, b) {
                Feature::Vertex(1)
            } else {
                Feature::Edge(1, 2)
            }
        } else {
            Feature::Edge(1, 0)
        }
    };
    let (i, j) = match feature {
        Feature::Vertex(i) => {
            let r = ray_sphere(center, direction, f32::MAX, tri[i], radius, false);
            return if r.hit {
                r.t.map(|time| TriangleHit { time, feature: 0 })
            } else {
                None
            };
        }
        Feature::Edge(i, j) => (i, j),
    };
    let ea = tri[i];
    let eb = tri[j];
    let mut e = sub(eb, ea);
    let mut delta = sub(center, ea);
    let projection = dot(delta, e);
    if projection > 0.0 {
        let den = (e[0] * e[0] + e[1] * e[1]) + e[2] * e[2];
        if projection < den {
            e = scale(e, projection / den);
        }
        delta = sub(delta, e);
    }
    let mut shift = length(delta) - radius;
    if shift <= 0.0 {
        return Some(TriangleHit {
            time: 0.0,
            feature: 0,
        });
    }
    shift = if shift <= 10.0 { 0.0 } else { shift + -10.0 };
    let roots = ray_capsule(
        add(scale(direction, shift), center),
        direction,
        ea,
        eb,
        radius,
    );
    if roots.count == 0 {
        return None;
    }
    let root = if roots.count == 1 || roots.roots[0] <= roots.roots[1] {
        roots.roots[0]
    } else {
        roots.roots[1]
    };
    let time = root + shift;
    (time >= 0.0).then_some(TriangleHit { time, feature: 0 })
}

#[derive(Clone, Copy, Debug, Default)]
pub struct CollectionOptions {
    pub cached_index: usize,
    pub flag_a: bool,
    pub flag_b: bool,
    pub any_hit: bool,
    pub check_initial: bool,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CollectionHit {
    pub index: usize,
    pub flags: u16,
    pub point: Option<Vec3>,
    pub normal: Vec3,
    pub distance: f32,
    pub aux: Vec3,
}

pub fn collection_sweep(
    triangles: &[Triangle],
    center: Vec3,
    radius: f32,
    direction: Vec3,
    max_distance: f32,
    options: CollectionOptions,
) -> Option<CollectionHit> {
    let mut best = max_distance;
    let mut alignment = 2.0;
    let mut winner = None;
    let mut saved = [0.0; 3];
    let origin_projection = dot(center, direction);
    for ordinal in 0..triangles.len() {
        let idx = if ordinal == 0 {
            options.cached_index
        } else if ordinal == options.cached_index {
            0
        } else {
            ordinal
        };
        let tri = triangles[idx];
        let [a, b, c] = tri;
        let centroid = scale(add(add(b, a), c), 1.0 / 3.0);
        let d = sub(centroid, center);
        let projection = dot(d, direction);
        let t = if projection > 0.0 { projection } else { 0.0 };
        let t = if t < best { t } else { best };
        let r = (length(sub(d, scale(direction, t))) - radius) - 0.0001;
        if r >= 0.0
            && !tri.iter().any(|p| {
                let v = sub(centroid, *p);
                r * r <= (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2]
            })
        {
            continue;
        }
        let proj = tri.map(|p| dot(p, direction));
        let pmin = proj[0].min(proj[1]).min(proj[2]);
        if pmin > ((origin_projection + best) + radius) + 0.012
            || !proj.iter().any(|x| *x >= origin_projection)
        {
            continue;
        }
        let mut n = cross(sub(b, a), sub(c, a));
        if !options.flag_a && !options.flag_b && dot(n, direction) > 0.0 {
            continue;
        }
        let l = length(n);
        if l == 0.0 {
            continue;
        }
        n = scale(n, 1.0 / l);
        let Some(candidate) =
            triangle_hit(tri, n, center, radius, direction, options.check_initial)
        else {
            continue;
        };
        let t = candidate.time;
        let score = -((n[0] * direction[0] + n[1] * direction[1]) + n[2] * direction[2]).abs();
        if t > max_distance {
            continue;
        }
        let epsilon = t.max(best).max(1.0) * 0.001;
        if !(t < best - epsilon
            || (t < epsilon + best && score < alignment)
            || (score == alignment && t < best)
            || t == 0.0)
        {
            continue;
        }
        if t == 0.0 {
            return Some(CollectionHit {
                index: idx,
                flags: 0x402,
                point: None,
                normal: neg(direction),
                distance: 0.0,
                aux: neg(direction),
            });
        }
        best = t;
        saved = n;
        alignment = score;
        winner = Some(idx);
        if options.any_hit {
            break;
        }
    }
    let winner = winner?;
    let (point, mut normal) = contact(center, direction, best, triangles[winner]);
    if options.flag_b && !options.flag_a && dot(saved, direction) > 0.0 {
        normal = neg(normal);
    }
    Some(CollectionHit {
        index: winner,
        flags: 3,
        point: Some(point),
        normal,
        distance: best,
        aux: saved,
    })
}

/// Lower nonzero-capsule branch. `None` fields mean that the corresponding
/// caller output remains untouched. A miss still publishes `max_distance`.
#[allow(clippy::too_many_arguments)]
pub fn capsule_box(
    a: Vec3,
    b: Vec3,
    radius: f32,
    box_q: [f32; 4],
    box_p: Vec3,
    extents: Vec3,
    direction: Vec3,
    max_distance: f32,
    flags: u16,
) -> CapsuleHit {
    if flags & 0x10 == 0 {
        let ds = segment_box_distance_sq(a, b, box_p, extents, matrix(box_q)).distance;
        if ds < radius * radius {
            return CapsuleHit {
                hit: true,
                distance: 0.0,
                normal: Some(neg(direction)),
                point: None,
                initial: true,
            };
        }
    }
    let half = scale(sub(b, a), 0.5);
    let (tris, _) = build_triangles(extents, Some((box_q, box_p)), half, direction);
    let center = scale(add(b, a), 0.5);
    match collection_sweep(
        &tris,
        center,
        radius,
        direction,
        max_distance,
        CollectionOptions::default(),
    ) {
        None => CapsuleHit {
            hit: false,
            distance: max_distance,
            normal: None,
            point: None,
            initial: false,
        },
        Some(r) => CapsuleHit {
            hit: true,
            distance: r.distance,
            normal: Some(r.normal),
            point: Some(r.point.unwrap_or([0.0; 3])),
            initial: false,
        },
    }
}
