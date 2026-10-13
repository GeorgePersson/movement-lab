//! Original capsule OBB, swept OBB and triangle/OBB SAT finite arithmetic.
//! Column-specific reductions and signed residual sweep terms are intentional.
use crate::precise_capsule::{Triangle, Vec3};
#[derive(Clone, Copy, Debug)]
pub struct Bounds {
    pub axes: [Vec3; 3],
    pub center: Vec3,
    pub extents: Vec3,
}
fn add(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] + b[i])
}
fn sub(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] - b[i])
}
fn scale(a: Vec3, s: f32) -> Vec3 {
    a.map(|x| x * s)
}
fn xyz(a: Vec3, b: Vec3) -> f32 {
    (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]
}
fn yxz(a: Vec3, b: Vec3) -> f32 {
    (a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]
}
fn norm(v: Vec3) -> Vec3 {
    let l = yxz(v, v).sqrt();
    if l > 0. { scale(v, 1. / l) } else { v }
}
fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - b[1] * a[2],
        a[2] * b[0] - b[2] * a[0],
        a[0] * b[1] - b[0] * a[1],
    ]
}
pub fn capsule_obb(a: Vec3, b: Vec3, r: f32) -> Bounds {
    let center = scale(add(a, b), 0.5);
    let initial = sub(a, b);
    let l = yxz(initial, initial).sqrt();
    let extents = [l * 0.5 + r, r, r];
    if l == 0. {
        return Bounds {
            axes: [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            center,
            extents,
        };
    }
    let mut u = sub(b, a);
    let l2 = xyz(u, u).sqrt();
    if l2 > 0. {
        u = scale(u, 1. / l2);
    }
    let [x, y, z] = u;
    let (v, w) = if y.abs() > 0.9999 {
        let den = ((-y) * (-y) + z * z).sqrt();
        let w = if den > 0. {
            [0., z * (1. / den), (-y) * (1. / den)]
        } else {
            [0., z, -y]
        };
        ([1., 0., 0.], w)
    } else {
        let den = ((-x) * (-x) + z * z).sqrt();
        let (vx, vz) = if den > 0. {
            (z * (1. / den), (-x) * (1. / den))
        } else {
            (z, -x)
        };
        ([vx, 0., vz], [vz * y, vx * z - vz * x, -(vx * y)])
    };
    Bounds {
        axes: [u, v, w],
        center,
        extents,
    }
}
pub fn swept_obb(obb: Bounds, d: Vec3, distance: f32) -> Bounds {
    let cols = obb.axes;
    let ext = obb.extents;
    let p = [
        xyz(d, cols[0]).abs(),
        yxz(d, cols[1]).abs(),
        yxz(d, cols[2]).abs(),
    ];
    let max = if p[1] > p[0] { p[1] } else { p[0] };
    let z_not_largest = p[2] <= max;
    let candidate = if z_not_largest { 2 } else { 1 };
    let mut i = if z_not_largest {
        if p[1] <= p[0] { 1 } else { 0 }
    } else {
        0
    };
    if p[candidate] < p[i] {
        i = candidate;
    }
    let selected = cols[i];
    let alignment = yxz(selected, d);
    let v = norm(sub(selected, scale(d, alignment)));
    let w = cross(d, v);
    let ad = [
        xyz(d, cols[0]).abs(),
        yxz(d, cols[1]).abs(),
        yxz(d, cols[2]).abs(),
    ];
    let av = [
        yxz(v, cols[0]).abs(),
        yxz(v, cols[1]).abs(),
        xyz(v, cols[2]).abs(),
    ];
    let aw = [
        xyz(w, cols[0]).abs(),
        yxz(w, cols[1]).abs(),
        yxz(w, cols[2]).abs(),
    ];
    let e0 = (ad[1] * ext[1] + ad[2] * ext[2]) + (ad[0] * ext[0] + distance * 0.5);
    let correction1 = (yxz(v, d) * distance) * 0.5;
    let correction2 = (xyz(d, w) * distance) * 0.5;
    let e1 = (av[2] * ext[2] + av[1] * ext[1]) + (av[0] * ext[0] + correction1);
    let e2 = (aw[2] * ext[2] + aw[1] * ext[1]) + (aw[0] * ext[0] + correction2);
    let center = std::array::from_fn(|j| ((distance * d[j]) * 0.5) + obb.center[j]);
    Bounds {
        axes: [d, v, w],
        center,
        extents: [e0, e1, e2],
    }
}
pub fn triangle_bounds(tri: Triangle, b: Bounds) -> bool {
    let p = tri.map(|point| b.axes.map(|axis| yxz(sub(point, b.center), axis)));
    let e = b.extents;
    if (0..3).all(|j| p[0][j].abs() <= e[j]) {
        return true;
    }
    if (0..3).any(|j| e[j] < p[0][j].min(p[1][j]).min(p[2][j])) {
        return false;
    }
    if (0..3).any(|j| p[0][j].max(p[1][j]).max(p[2][j]) < -e[j]) {
        return false;
    }
    let edges = [sub(p[1], p[0]), sub(p[2], p[1]), sub(p[0], p[2])];
    let n = cross(edges[0], edges[1]);
    let h = yxz(n, p[0]);
    let radius = yxz(n.map(f32::abs), e);
    if radius < h || h < -radius {
        return false;
    }
    for edge in edges {
        for j in 0..3 {
            let k = (j + 1) % 3;
            let proj = p.map(|point| edge[k] * point[j] - edge[j] * point[k]);
            let radius = edge[k].abs() * e[j] + edge[j].abs() * e[k];
            if radius < proj[0].min(proj[1]).min(proj[2])
                || proj[0].max(proj[1]).max(proj[2]) < -radius
            {
                return false;
            }
        }
    }
    true
}
