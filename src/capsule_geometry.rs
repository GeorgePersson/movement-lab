//! Original finite-input equations for recovered capsule/box geometry helpers.
//!
//! Operations retain their observed f32 order, including four-lane dot reductions.
//! The validated domain uses finite, nonoverflowing shape vectors with W = 0.
//! These helpers do not supply scene filtering, scheduling, or live game policy.

pub type Vec4 = [f32; 4];

const ZERO: Vec4 = [0.0; 4];
const EPSILON: f32 = f32::EPSILON;
const PLANE_DISTANCE: f32 = 1e-4;
const FACE_PRODUCT: f32 = -1e-6;

/// Active records refer to the original simplex before compaction.
/// The caller must apply this same order to both support-point companion arrays.
/// Entries after `count` do not describe native inactive-storage writes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reduction {
    pub closest: Vec4,
    pub indices: [usize; 4],
    pub count: usize,
}

impl Reduction {
    pub fn active_indices(&self) -> &[usize] {
        &self.indices[..self.count]
    }

    fn new(closest: Vec4, active: &[usize]) -> Self {
        let mut indices = [0; 4];
        indices[..active.len()].copy_from_slice(active);
        Self {
            closest,
            indices,
            count: active.len(),
        }
    }
}

pub fn add(a: Vec4, b: Vec4) -> Vec4 {
    std::array::from_fn(|i| a[i] + b[i])
}

pub fn sub(a: Vec4, b: Vec4) -> Vec4 {
    std::array::from_fn(|i| a[i] - b[i])
}

pub fn mul(a: Vec4, b: Vec4) -> Vec4 {
    std::array::from_fn(|i| a[i] * b[i])
}

pub fn scale(a: Vec4, factor: f32) -> Vec4 {
    mul(a, [factor; 4])
}

/// Subtraction from positive zero is intentional, including signed-zero behavior.
pub fn neg(a: Vec4) -> Vec4 {
    sub(ZERO, a)
}

fn div(a: Vec4, b: Vec4) -> Vec4 {
    std::array::from_fn(|i| a[i] / b[i])
}

fn horizontal(a: Vec4) -> Vec4 {
    let pairs = add([a[2], a[3], a[0], a[1]], a);
    add([pairs[1], pairs[0], pairs[3], pairs[2]], pairs)
}

/// Multiply, then add swapped pairs, then add their swapped totals.
/// Do not replace this with a left-associated sum or a fused multiply-add.
pub fn dot_lanes(a: Vec4, b: Vec4) -> Vec4 {
    horizontal(mul(a, b))
}

pub fn dot(a: Vec4, b: Vec4) -> f32 {
    dot_lanes(a, b)[0]
}

pub fn cross(a: Vec4, b: Vec4) -> Vec4 {
    sub(
        mul([a[1], a[2], a[0], a[3]], [b[2], b[0], b[1], b[3]]),
        mul([a[2], a[0], a[1], a[3]], [b[1], b[2], b[0], b[3]]),
    )
}

fn all_le(a: Vec4, b: Vec4) -> bool {
    a.into_iter().zip(b).all(|(a, b)| a <= b)
}

fn all_lt(a: Vec4, b: Vec4) -> bool {
    a.into_iter().zip(b).all(|(a, b)| a < b)
}

fn guarded_reciprocal(denominator: Vec4) -> Vec4 {
    denominator.map(|d| if d.abs() > EPSILON { 1.0 / d } else { 0.0 })
}

/// Endpoint-only support: equal dot products choose `endpoint_b` (index zero).
/// The support operation itself does not add capsule radius.
pub fn capsule_support(endpoint_a: Vec4, endpoint_b: Vec4, direction: Vec4) -> (Vec4, usize) {
    let da = dot_lanes(endpoint_a, direction);
    let db = dot_lanes(endpoint_b, direction);
    let mask = std::array::from_fn::<_, 4, _>(|i| db[i] < da[i]);
    (
        std::array::from_fn(|i| {
            if mask[i] {
                endpoint_a[i]
            } else {
                endpoint_b[i]
            }
        }),
        usize::from(mask[0]),
    )
}

/// Per-lane positive support only when direction is strictly positive.
/// Zero direction takes the negative extent; the index records XYZ signs only.
pub fn box_support(extents: Vec4, direction: Vec4) -> (Vec4, usize) {
    let negative = neg(extents);
    let mask = direction.map(|x| x > 0.0);
    let index = usize::from(mask[0]) | (usize::from(mask[1]) << 1) | (usize::from(mask[2]) << 2);
    (
        std::array::from_fn(|i| if mask[i] { extents[i] } else { negative[i] }),
        index,
    )
}

/// Valid-index view of the recovered endpoint getter. Bounds checking is lab API policy.
pub fn capsule_vertex(endpoint_a: Vec4, endpoint_b: Vec4, index: usize) -> Option<Vec4> {
    match index {
        0 => Some(endpoint_b),
        1 => Some(endpoint_a),
        _ => None,
    }
}

/// Valid-index view of the initialized corner masks. W always takes negative extent.
/// Bounds checking is lab API policy; the native indexed leaf has no such check.
pub fn box_vertex(extents: Vec4, index: usize) -> Option<Vec4> {
    if index >= 8 {
        return None;
    }
    let negative = neg(extents);
    Some(std::array::from_fn(|i| {
        if i < 3 && (index & (1 << i)) != 0 {
            extents[i]
        } else {
            negative[i]
        }
    }))
}

/// Unclamped witness weight. Preserve the two subtractions involving `point`.
pub fn segment_weight_lanes(point: Vec4, a: Vec4, b: Vec4) -> Vec4 {
    let ap = sub(a, point);
    let edge = sub(sub(b, point), ap);
    let denominator = dot_lanes(edge, edge);
    let numerator = dot_lanes(neg(ap), edge);
    let reciprocal = denominator.map(|d| if d > 0.0 { 1.0 / d } else { 0.0 });
    mul(reciprocal, numerator)
}

pub fn segment_weight(point: Vec4, a: Vec4, b: Vec4) -> f32 {
    segment_weight_lanes(point, a, b)[0]
}

/// Unclamped B/C witness weights, with denominator `area_a + (area_c + area_b)`.
pub fn triangle_weight_lanes(point: Vec4, a: Vec4, b: Vec4, c: Vec4) -> (Vec4, Vec4) {
    let normal = cross(sub(b, a), sub(c, a));
    let bp = sub(b, point);
    let cp = sub(c, point);
    let ap = sub(a, point);
    let area_a = dot_lanes(cross(bp, cp), normal);
    let area_b = dot_lanes(cross(cp, ap), normal);
    let area_c = dot_lanes(cross(ap, bp), normal);
    let denominator = add(area_a, add(area_c, area_b));
    let reciprocal = denominator.map(|d| if d != 0.0 { 1.0 / d } else { 0.0 });
    (mul(reciprocal, area_b), mul(reciprocal, area_c))
}

pub fn triangle_weights(point: Vec4, a: Vec4, b: Vec4, c: Vec4) -> (f32, f32) {
    let (u, v) = triangle_weight_lanes(point, a, b, c);
    (u[0], v[0])
}

/// A nondegenerate segment retains both records even when projection reaches an endpoint.
pub fn reduce_segment(a: Vec4, b: Vec4) -> Reduction {
    let edge = sub(b, a);
    let denominator = dot_lanes(edge, edge);
    if all_le(denominator, [EPSILON; 4]) {
        return Reduction::new(a, &[0]);
    }
    let projected = div(dot_lanes(neg(a), edge), denominator);
    let t = projected.map(|v| {
        // MIN with second operand 1, then MAX with second operand +0.
        let upper = if v < 1.0 { v } else { 1.0 };
        if upper > 0.0 { upper } else { 0.0 }
    });
    Reduction::new(add(mul(t, edge), a), &[0, 1])
}

/// Face reducer with an exact-zero normal guard, distinct from the primary sweep's
/// epsilon fallback to AB. `None` denotes that exact-zero face.
pub fn reduce_triangle_face(q: [Vec4; 3]) -> Option<Reduction> {
    let [a, b, c] = q;
    let ab = sub(b, a);
    let ac = sub(c, a);
    let normal = cross(ab, ac);
    let normal_squared = dot_lanes(normal, normal);
    if normal_squared[0] == 0.0 {
        return None;
    }
    let sa = dot_lanes(cross(b, c), normal);
    let sb = dot_lanes(cross(c, a), normal);
    let sc = dot_lanes(cross(a, b), normal);
    if all_le(ZERO, sa) && all_le(ZERO, sb) && all_le(ZERO, sc) {
        return Some(Reduction::new(
            mul(div(dot_lanes(normal, a), normal_squared), normal),
            &[0, 1, 2],
        ));
    }
    let d1 = dot_lanes(neg(a), ab);
    let d2 = dot_lanes(neg(a), ac);
    let d3 = dot_lanes(neg(b), ab);
    let d4 = dot_lanes(neg(b), ac);
    let d5 = dot_lanes(neg(c), ab);
    let d6 = dot_lanes(neg(c), ac);
    if all_le(ZERO, d1) && all_le(d3, ZERO) && all_le(sc, ZERO) {
        let t = mul(guarded_reciprocal(sub(d1, d3)), d1);
        return Some(Reduction::new(add(mul(t, ab), a), &[0, 1]));
    }
    if all_le(d3, d4) && all_le(d6, d5) && all_le(sa, ZERO) {
        let numerator = sub(d4, d3);
        let denominator = add(sub(d5, d6), numerator);
        let t = mul(guarded_reciprocal(denominator), numerator);
        return Some(Reduction::new(add(mul(t, sub(c, b)), b), &[1, 2]));
    }
    if all_le(ZERO, d2) && all_le(d6, ZERO) && all_le(sb, ZERO) {
        let t = mul(guarded_reciprocal(sub(d2, d6)), d2);
        return Some(Reduction::new(add(mul(t, ac), a), &[0, 2]));
    }
    if all_le(d2, ZERO) && all_le(d1, ZERO) {
        return Some(Reduction::new(a, &[0]));
    }
    if all_le(ZERO, d3) && all_le(d4, d3) {
        return Some(Reduction::new(b, &[1]));
    }
    Some(Reduction::new(c, &[2]))
}

/// Reduce four points and return the active original indices for all three arrays.
/// The near-plane gate is inclusive on the far side; unordered distance also goes far.
pub fn reduce_tetrahedron(q: [Vec4; 4]) -> Reduction {
    let [a, b, c, d] = q;
    let ab = sub(b, a);
    let ac = sub(c, a);
    let ad = sub(d, a);
    let normal = cross(ab, ac);
    let normal_squared = dot_lanes(normal, normal);
    let normalized = div(normal, normal_squared.map(f32::sqrt));
    let distance = dot_lanes(normalized, ad)[0].abs();
    if distance < PLANE_DISTANCE {
        if normal_squared[0] <= EPSILON {
            return reduce_segment(a, b);
        }
        // The same normal_squared was just proved strictly above epsilon.
        return reduce_triangle_face([a, b, c]).expect("nonzero near-plane face");
    }
    let normals = [
        normal,
        cross(ac, ad),
        cross(ad, ab),
        cross(sub(d, b), sub(c, b)),
    ];
    let products = [
        mul(dot_lanes(normals[0], d), dot_lanes(normals[0], a))[0],
        mul(dot_lanes(normals[1], b), dot_lanes(normals[1], a))[0],
        mul(dot_lanes(normals[2], a), dot_lanes(normals[2], c))[0],
        mul(dot_lanes(normals[3], a), dot_lanes(normals[3], b))[0],
    ];
    let enabled = products.map(|v| FACE_PRODUCT <= v);
    if !enabled.into_iter().any(|on| on) {
        return Reduction::new(ZERO, &[0, 1, 2, 3]);
    }
    let mut best = Reduction::new(ZERO, &[0, 1, 2, 3]);
    let mut best_distance = [f32::MAX; 4];
    let faces = [[0, 1, 2], [0, 2, 3], [0, 3, 1], [1, 3, 2]];
    for (face_number, (face, on)) in faces.into_iter().zip(enabled).enumerate() {
        if !on {
            continue;
        }
        let Some(candidate) = reduce_triangle_face(face.map(|i| q[i])) else {
            if face_number == 0 {
                best.count = 3;
            }
            continue;
        };
        let distance = dot_lanes(candidate.closest, candidate.closest);
        if face_number == 0 || all_lt(distance, best_distance) {
            let mut indices = [0; 4];
            for (slot, &index) in candidate.active_indices().iter().enumerate() {
                indices[slot] = face[index];
            }
            best = Reduction {
                closest: candidate.closest,
                indices,
                count: candidate.count,
            };
            best_distance = distance;
        }
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn support_ties_and_zero_direction_preserve_selected_side() {
        let a = [1.0, 0.0, 0.0, 0.0];
        let b = [-1.0, 0.0, 0.0, 0.0];
        assert_eq!(capsule_support(a, b, ZERO), (b, 0));
        assert_eq!(box_support([1.0, 2.0, 3.0, 0.0], ZERO).1, 0);
        assert_eq!(
            box_support([1.0, 2.0, 3.0, 0.0], ZERO).0,
            [-1.0, -2.0, -3.0, 0.0]
        );
    }

    #[test]
    fn witness_weights_remain_unclamped() {
        let a = [0.0; 4];
        let b = [1.0, 0.0, 0.0, 0.0];
        let c = [0.0, 1.0, 0.0, 0.0];
        assert_eq!(segment_weight([2.0, 0.0, 0.0, 0.0], a, b), 2.0);
        assert_eq!(triangle_weights([2.0, 3.0, 0.0, 0.0], a, b, c), (2.0, 3.0));
    }

    #[test]
    fn clamped_segment_keeps_two_records_at_endpoint() {
        let reduced = reduce_segment([1.0, 0.0, 0.0, 0.0], [2.0, 0.0, 0.0, 0.0]);
        assert_eq!(reduced.closest, [1.0, 0.0, 0.0, 0.0]);
        assert_eq!(reduced.active_indices(), &[0, 1]);
    }

    #[test]
    fn interior_tetrahedron_preserves_four_active_records() {
        let reduced = reduce_tetrahedron([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [-1.0, -1.0, -1.0, 0.0],
        ]);
        assert_eq!(reduced.closest, ZERO);
        assert_eq!(reduced.active_indices(), &[0, 1, 2, 3]);
    }
}
