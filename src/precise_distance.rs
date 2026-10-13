//! Original finite-f32 point, line and segment distance equations for a box.
//!
//! `matrix` is a flat column-major rotation. Witness points are in BOX LOCAL
//! coordinates. The low-level API assumes finite moderate inputs, nonnegative
//! extents and valid supplied rotations; wrapper validation is separate.
//! Products and sums deliberately preserve the verified operation grouping.
//! A line parameter is always requested internally, including for segment calls.

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PointDistance {
    pub distance: f32,
    pub point: [f32; 3],
}

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SegmentDistance {
    pub distance: f32,
    pub t: f32,
    pub point: [f32; 3],
}

pub type LineDistance = SegmentDistance;

fn add(a: f32, b: f32) -> f32 {
    a + b
}
fn sub(a: f32, b: f32) -> f32 {
    a - b
}
fn mul(a: f32, b: f32) -> f32 {
    a * b
}
fn div(a: f32, b: f32) -> f32 {
    a / b
}
fn neg(a: f32) -> f32 {
    f32::from_bits(a.to_bits() ^ 0x8000_0000)
}

fn transform(v: [f32; 3], m: [f32; 9]) -> [f32; 3] {
    std::array::from_fn(|i| {
        let k = 3 * i;
        add(
            add(mul(v[1], m[k + 1]), mul(v[0], m[k])),
            mul(v[2], m[k + 2]),
        )
    })
}

fn clamp_point(mut p: [f32; 3], e: [f32; 3]) -> PointDistance {
    let mut distance = 0.0;
    for i in 0..3 {
        if p[i] < neg(e[i]) {
            let residual = add(p[i], e[i]);
            p[i] = neg(e[i]);
            distance = add(distance, mul(residual, residual));
        } else if p[i] > e[i] {
            let residual = sub(p[i], e[i]);
            p[i] = e[i];
            distance = add(distance, mul(residual, residual));
        }
    }
    PointDistance { distance, point: p }
}

pub fn point_box_distance_sq(
    point: [f32; 3],
    center: [f32; 3],
    extents: [f32; 3],
    matrix: [f32; 9],
) -> PointDistance {
    clamp_point(
        transform(std::array::from_fn(|i| sub(point[i], center[i])), matrix),
        extents,
    )
}

pub fn line_box_distance_sq(
    point: [f32; 3],
    direction: [f32; 3],
    center: [f32; 3],
    extents: [f32; 3],
    matrix: [f32; 9],
) -> LineDistance {
    let mut p = transform(std::array::from_fn(|i| sub(point[i], center[i])), matrix);
    let mut d = transform(direction, matrix);
    let e = extents;
    let reflections = d.map(|x| x < 0.0);
    for i in 0..3 {
        if reflections[i] {
            p[i] = neg(p[i]);
            d[i] = neg(d[i]);
        }
    }
    let mut axes = [0; 3];
    let mut count = 0;
    for (i, value) in d.iter().enumerate() {
        if *value > 0.0 {
            axes[count] = i;
            count += 1;
        }
    }
    let (distance, t) = match count {
        0 => {
            let result = clamp_point(p, e);
            p = result.point;
            (result.distance, 0.0)
        }
        1 => {
            let axis = axes[0];
            let t = div(sub(e[axis], p[axis]), d[axis]);
            p[axis] = e[axis];
            let result = clamp_point(p, e);
            p = result.point;
            (result.distance, t)
        }
        2 => {
            let k = (0..3).find(|i| !axes[..2].contains(i)).unwrap();
            two_axes(axes[0], axes[1], k, &mut p, d, e)
        }
        _ => {
            let pm = std::array::from_fn(|i| sub(p[i], e[i]));
            let (i, j, k) = if mul(pm[0], d[1]) < mul(pm[1], d[0]) {
                if mul(pm[1], d[2]) < mul(pm[2], d[1]) {
                    (2, 0, 1)
                } else {
                    (1, 2, 0)
                }
            } else if mul(pm[2], d[0]) <= mul(pm[0], d[2]) {
                (0, 1, 2)
            } else {
                (2, 0, 1)
            };
            face(i, j, k, &mut p, d, e, pm)
        }
    };
    for i in 0..3 {
        if reflections[i] {
            p[i] = neg(p[i]);
        }
    }
    SegmentDistance {
        distance,
        t,
        point: p,
    }
}

pub fn segment_box_distance_sq(
    a: [f32; 3],
    b: [f32; 3],
    center: [f32; 3],
    extents: [f32; 3],
    matrix: [f32; 9],
) -> SegmentDistance {
    let result = line_box_distance_sq(
        a,
        std::array::from_fn(|i| sub(b[i], a[i])),
        center,
        extents,
        matrix,
    );
    if result.t < 0.0 {
        let point = point_box_distance_sq(a, center, extents, matrix);
        SegmentDistance {
            distance: point.distance,
            t: 0.0,
            point: point.point,
        }
    } else if result.t > 1.0 {
        let point = point_box_distance_sq(b, center, extents, matrix);
        SegmentDistance {
            distance: point.distance,
            t: 1.0,
            point: point.point,
        }
    } else {
        result
    }
}

fn two_axes(
    i: usize,
    j: usize,
    k: usize,
    p: &mut [f32; 3],
    d: [f32; 3],
    e: [f32; 3],
) -> (f32, f32) {
    let mut distance = 0.0;
    let pi_minus = sub(p[i], e[i]);
    let pj_minus = sub(p[j], e[j]);
    let cross_j = mul(pj_minus, d[i]);
    let cross_i = mul(pi_minus, d[j]);
    let t;
    if cross_i < cross_j {
        p[j] = e[j];
        let old = p[i];
        let plus = add(old, e[i]);
        let gap = sub(cross_j, mul(plus, d[j]));
        if gap < 0.0 {
            let inv = div(1.0, d[j]);
            p[i] = sub(old, mul(inv, cross_j));
            t = neg(mul(inv, pj_minus));
        } else {
            let inv = div(1.0, add(mul(d[i], d[i]), mul(d[j], d[j])));
            distance = add(mul(mul(gap, gap), inv), distance);
            p[i] = neg(e[i]);
            t = neg(mul(add(mul(pj_minus, d[j]), mul(plus, d[i])), inv));
        }
    } else {
        p[i] = e[i];
        let old = p[j];
        let plus = add(old, e[j]);
        let gap = sub(cross_i, mul(plus, d[i]));
        if gap < 0.0 {
            let inv = div(1.0, d[i]);
            p[j] = sub(old, mul(inv, cross_i));
            t = neg(mul(inv, pi_minus));
        } else {
            let inv = div(1.0, add(mul(d[i], d[i]), mul(d[j], d[j])));
            distance = add(mul(mul(gap, gap), inv), distance);
            p[j] = neg(e[j]);
            t = neg(mul(add(mul(plus, d[j]), mul(pi_minus, d[i])), inv));
        }
    }
    if p[k] < neg(e[k]) {
        let residual = add(e[k], p[k]);
        distance = add(mul(residual, residual), distance);
        p[k] = neg(e[k]);
    } else if p[k] > e[k] {
        let residual = sub(p[k], e[k]);
        distance = add(mul(residual, residual), distance);
        p[k] = e[k];
    }
    (distance, t)
}

// Three-direction face/edge/corner cases. u/v/w are positive direction,
// x is p_i-e_i, y/z are p_j+e_j and p_k+e_k. Grouping differs by branch.
fn face(
    i: usize,
    j: usize,
    k: usize,
    p: &mut [f32; 3],
    d: [f32; 3],
    e: [f32; 3],
    pm: [f32; 3],
) -> (f32, f32) {
    let (u, v, w) = (d[i], d[j], d[k]);
    let x = pm[i];
    let y = add(e[j], p[j]);
    let z = add(e[k], p[k]);
    let mut distance = 0.0;
    let t;
    if mul(x, v) <= mul(y, u) {
        if mul(u, z) < mul(x, w) {
            let wz = mul(w, z);
            let plane = add(mul(u, u), mul(w, w));
            let ux = mul(x, u);
            let total = add(mul(v, v), plane);
            let gap = sub(mul(plane, y), mul(add(wz, ux), v));
            if mul(add(plane, plane), e[j]) < gap {
                let yy = pm[j];
                let projection = add(add(mul(yy, v), ux), wz);
                t = neg(div(projection, total));
                distance = add(
                    add(add(mul(t, projection), distance), mul(z, z)),
                    add(mul(yy, yy), mul(x, x)),
                );
                p[i] = e[i];
                p[j] = e[j];
                p[k] = neg(e[k]);
            } else {
                let offset = div(gap, plane);
                let yy = sub(y, offset);
                let projection = add(add(mul(yy, v), ux), wz);
                t = neg(div(projection, total));
                distance = add(
                    add(add(mul(t, projection), distance), mul(z, z)),
                    add(mul(yy, yy), mul(x, x)),
                );
                p[i] = e[i];
                p[j] = sub(offset, e[j]);
                p[k] = neg(e[k]);
            }
        } else {
            p[i] = e[i];
            let inv = div(1.0, u);
            p[j] = sub(p[j], mul(mul(x, v), inv));
            p[k] = sub(p[k], mul(mul(x, w), inv));
            t = neg(mul(inv, x));
        }
    } else {
        let uu = mul(u, u);
        let ux = mul(u, x);
        let vv = mul(v, v);
        let ww = mul(w, w);
        let xx = mul(x, x);
        if mul(u, z) < mul(x, w) {
            let wz = mul(w, z);
            let plane = add(uu, ww);
            let gap = sub(mul(y, plane), mul(add(ux, wz), v));
            if gap < 0.0 {
                let plane2 = add(uu, vv);
                let projection = add(mul(v, y), ux);
                let total = add(ww, plane2);
                let gap2 = sub(mul(z, plane2), mul(w, projection));
                if gap2 < 0.0 {
                    let combined = add(projection, wz);
                    t = neg(div(combined, total));
                    distance = add(
                        add(add(mul(combined, t), distance), mul(z, z)),
                        add(xx, mul(y, y)),
                    );
                    p[i] = e[i];
                    p[j] = neg(e[j]);
                    p[k] = neg(e[k]);
                } else {
                    let fixed = add(mul(y, y), xx);
                    if mul(add(plane2, plane2), e[k]) < gap2 {
                        let zz = pm[k];
                        let projection = add(mul(w, zz), projection);
                        t = neg(div(projection, total));
                        distance = add(add(mul(zz, zz), fixed), add(mul(t, projection), distance));
                        p[i] = e[i];
                        p[j] = neg(e[j]);
                        p[k] = e[k];
                    } else {
                        let zz = sub(z, div(gap2, plane2));
                        let projection = add(mul(w, zz), projection);
                        t = neg(div(projection, total));
                        distance = add(add(mul(zz, zz), fixed), add(mul(t, projection), distance));
                        p[i] = e[i];
                        p[j] = neg(e[j]);
                        p[k] = sub(div(gap2, plane2), e[k]);
                    }
                }
            } else {
                if mul(add(plane, plane), e[j]) < gap {
                    let yy = pm[j];
                    let projection = add(add(mul(yy, v), ux), wz);
                    t = neg(div(projection, add(plane, vv)));
                    distance = add(
                        add(add(mul(t, projection), distance), mul(z, z)),
                        add(mul(yy, yy), xx),
                    );
                    p[i] = e[i];
                    p[j] = e[j];
                } else {
                    let yy = sub(y, div(gap, plane));
                    let projection = add(add(mul(yy, v), ux), wz);
                    t = neg(div(projection, add(plane, vv)));
                    distance = add(
                        add(add(mul(t, projection), distance), mul(z, z)),
                        add(mul(yy, yy), xx),
                    );
                    p[i] = e[i];
                    p[j] = sub(div(gap, plane), e[j]);
                }
                p[k] = neg(e[k]);
            }
        } else {
            let plane = add(uu, vv);
            let projection = add(mul(v, y), ux);
            let fixed = add(mul(y, y), xx);
            let gap = sub(mul(plane, z), mul(w, projection));
            let total = add(ww, plane);
            if mul(add(plane, plane), e[k]) < gap {
                let zz = pm[k];
                let projection = add(mul(w, zz), projection);
                t = neg(div(projection, total));
                distance = add(add(mul(t, projection), distance), add(mul(zz, zz), fixed));
                p[i] = e[i];
                p[j] = neg(e[j]);
                p[k] = e[k];
            } else {
                let offset = div(gap, plane);
                let zz = sub(z, offset);
                let projection = add(mul(w, zz), projection);
                t = neg(div(projection, total));
                distance = add(add(mul(t, projection), distance), add(mul(zz, zz), fixed));
                p[i] = e[i];
                p[j] = neg(e[j]);
                p[k] = sub(offset, e[k]);
            }
        }
    }
    (distance, t)
}
