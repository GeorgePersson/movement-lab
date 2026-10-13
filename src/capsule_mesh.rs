//! Original bounded capsule query against supplied static triangles.
//!
//! The native mesh kernel's zero-length and axial branch queries the original
//! triangle collection at the capsule's leading end. The general branch uses
//! original triangle extrusion, capsule/triangle witness and swept OBB filtering.
//! Scene gathering, shape association and slope/world publication stay supplied.
//! Bit equivalence applies to finite normal/zero arithmetic; subnormal FTZ/DAZ,
//! overflow and nonfinite cases are outside the recovered domain (the native
//! exported adapter temporarily forces MXCSR 0x9fc0).

use crate::precise_capsule::{self, CollectionHit, CollectionOptions, Triangle, Vec3};

/// Native inclusive axial cosine threshold (constant bits 0x3f7fff58).
pub const AXIAL_COSINE: f32 = f32::from_bits(0x3f7f_ff58);
/// Laboratory input guard; this is not a recovered native validation threshold.
const MAX_COMPONENT: f32 = 10_000.;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeshError {
    InvalidInput,
    UnsupportedNonaxial,
    NonFiniteArithmetic,
}

pub struct AxialTriangleInput<'a> {
    pub triangles: &'a [Triangle],
    pub capsule_a: Vec3,
    pub capsule_b: Vec3,
    pub radius: f32,
    pub direction: Vec3,
    pub max_distance: f32,
    pub cached_index: u32,
    pub flags: u16,
    pub double_sided: bool,
}

pub struct CachedMeshInput<'a> {
    pub triangles: &'a [Triangle],
    pub original_ids: &'a [u32],
    pub quaternion: [f32; 4],
    pub radius: f32,
    /// Cylinder length excluding the two hemispheres.
    pub height: f32,
    pub center: [f64; 3],
    pub offset: [f64; 3],
    pub direction: Vec3,
    pub max_distance: f32,
    pub triangle_offset: u32,
    pub triangle_count: u32,
    /// Native local triangle index; it is not a source or global triangle ID.
    pub cached_index: u32,
    /// Supplied native cache-slot selector; known movement passes are 0, 1, 2.
    pub pass_id: u32,
    pub overlap: bool,
    pub precise: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeshHit {
    pub world_point: [f64; 3],
    pub normal: Vec3,
    pub distance: f32,
    pub global_index: u32,
    pub original_index: u32,
    /// False for initial-overlap flags402. General flags3 zero-time candidates
    /// still write a witness; an untouched local point publishes the stream offset.
    pub source_point_written: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CachedMeshResult {
    pub hit: Option<MeshHit>,
    pub cached_index: u32,
}

fn bounded(x: f32) -> bool {
    x.is_finite() && (x == 0. || x.is_normal()) && x.abs() <= MAX_COMPONENT
}
fn unit_direction(d: Vec3) -> bool {
    d.into_iter().all(bounded) && (((d[0] * d[0] + d[1] * d[1]) + d[2] * d[2]) - 1.).abs() <= 0.001
}

/// Actual Common 18013BD30 axial branch. `None` preserves the native hit output.
/// `CollectionHit::point == None` preserves the native point bytes on zero hits.
/// Direct maximum values above 1e8 are outside this checked LAB boundary; the
/// cached-mesh adapter clamps to the recovered native 1e8 maximum first.
/// These boundary checks are LAB guards; the native primitive assumes readable,
/// finite supplied records and does not introduce these Rust validation errors.
pub fn axial_triangle_sweep(
    c: &AxialTriangleInput<'_>,
) -> Result<Option<CollectionHit>, MeshError> {
    if c.triangles.is_empty() {
        return Ok(None);
    }
    if c.cached_index as usize >= c.triangles.len()
        || !c
            .capsule_a
            .into_iter()
            .chain(c.capsule_b)
            .chain([c.radius])
            .all(bounded)
        || c.radius < 0.
        || !c.max_distance.is_finite()
        || c.max_distance < 0.
        || c.max_distance > 1e8
        || !unit_direction(c.direction)
        || c.triangles.iter().flatten().flatten().any(|x| !bounded(*x))
    {
        return Err(MeshError::InvalidInput);
    }
    axial_unchecked(c)
}

fn axial_unchecked(c: &AxialTriangleInput<'_>) -> Result<Option<CollectionHit>, MeshError> {
    let delta: Vec3 = std::array::from_fn(|i| c.capsule_a[i] - c.capsule_b[i]);
    let midpoint: Vec3 = std::array::from_fn(|i| (c.capsule_b[i] + c.capsule_a[i]) * 0.5);
    let half = delta.map(|x| x * 0.5);
    let half_length =
        (((delta[1] * delta[1] + delta[0] * delta[0]) + delta[2] * delta[2]) * 0.25).sqrt();
    if half_length != 0. {
        let inv = 1. / half_length;
        let axis = half.map(|x| x * inv);
        let cosine = ((axis[0] * c.direction[0] + axis[1] * c.direction[1])
            + axis[2] * c.direction[2])
            .abs();
        if !cosine.is_finite() {
            return Err(MeshError::NonFiniteArithmetic);
        }
        if cosine < AXIAL_COSINE {
            return Err(MeshError::UnsupportedNonaxial);
        }
    }
    let sphere_center = std::array::from_fn(|i| half_length * c.direction[i] + midpoint[i]);
    let result = precise_capsule::collection_sweep(
        c.triangles,
        sphere_center,
        c.radius,
        c.direction,
        c.max_distance,
        CollectionOptions {
            cached_index: c.cached_index as usize,
            flag_a: c.double_sided,
            flag_b: c.flags & 0x80 != 0,
            any_hit: c.flags & 0x40 != 0,
            check_initial: c.flags & 0x10 == 0,
        },
    );
    if let Some(hit) = result {
        if !hit.distance.is_finite()
            || !hit.normal.into_iter().chain(hit.aux).all(f32::is_finite)
            || hit
                .point
                .is_some_and(|p| !p.into_iter().all(f32::is_finite))
        {
            return Err(MeshError::NonFiniteArithmetic);
        }
    }
    Ok(result)
}

/// Mesh-query capsule endpoints from Common 180038F40. The X addition grouping
/// differs from the capsule-box query wrapper, so keep this helper separate.
pub fn mesh_endpoints(q: [f32; 4], position: Vec3, half_height: f32) -> (Vec3, Vec3) {
    let [x, y, z, w] = q;
    let ww = w + w;
    let xx = x + x;
    let delta = [
        ((w * ww + x * xx) - 1.) * half_height,
        (ww * z + xx * y) * half_height,
        (xx * z - ww * y) * half_height,
    ];
    (
        std::array::from_fn(|i| position[i] + delta[i]),
        std::array::from_fn(|i| position[i] - delta[i]),
    )
}

/// Native output writes represented separately from the hit boolean. None leaves
/// the primitive index bytes untouched; Some(MAX) is the general miss write.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrimitiveMeshResult {
    pub hit: Option<CollectionHit>,
    pub index_write: Option<u32>,
}
fn va(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] + b[i])
}
fn vs(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|i| a[i] - b[i])
}
fn vm(a: Vec3, s: f32) -> Vec3 {
    a.map(|x| x * s)
}
fn dot(a: Vec3, b: Vec3) -> f32 {
    (a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]
}
fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - b[1] * a[2],
        b[0] * a[2] - a[0] * b[2],
        b[1] * a[0] - a[1] * b[0],
    ]
}
fn len(a: Vec3) -> f32 {
    dot(a, a).sqrt()
}
fn neg(a: Vec3) -> Vec3 {
    a.map(|x| f32::from_bits(x.to_bits() ^ 0x80000000))
}
fn clamp_initial(x: f32) -> f32 {
    if x.is_nan() { 0. } else { x.max(0.).min(1.) }
}
/// Original capsule/triangle initial SAT (B0290), including vertex shortcut.
fn capsule_initial(tri: Triangle, a: Vec3, b: Vec3, radius: f32) -> bool {
    let edge = vs(b, a);
    let sq = dot(edge, edge);
    let inv = if sq != 0. { 1. / sq } else { 0. };
    let mut delta = vs(tri[0], a);
    let t = dot(edge, delta);
    if t > 0. {
        delta = vs(
            delta,
            if t >= dot(edge, edge) {
                edge
            } else {
                vm(edge, t / dot(edge, edge))
            },
        );
    }
    if dot(delta, delta) <= radius * radius {
        return true;
    }
    let separated = |axis: Vec3| {
        let rr = len(axis) * radius;
        let s0 = dot(a, axis);
        let s1 = dot(b, axis);
        let [t0, t1, t2] = tri.map(|p| dot(p, axis));
        s1.max(s0) + rr < t1.min(t0).min(t2) || t2.max(t1.max(t0)) < s1.min(s0) - rr
    };
    if separated(cross(vs(tri[1], tri[0]), vs(tri[2], tri[0]))) {
        return false;
    }
    for i in 0..3 {
        let p = tri[i];
        let q = tri[(i + 1) % 3];
        let e = vs(q, p);
        let d = vs(a, p);
        let e2 = dot(e, e);
        let ed = dot(edge, e);
        let de = dot(d, e);
        let dd = dot(edge, d);
        let denom = e2 * sq - ed * ed;
        let mut u = clamp_initial(if denom != 0. {
            (de * sq - dd * ed) / denom
        } else {
            0.
        });
        let mut t = (ed * u - dd) * inv;
        if t < 0. {
            t = 0.;
            u = clamp_initial(de / e2);
        } else if t > 1. {
            t = 1.;
            u = clamp_initial((de + ed) / e2);
        }
        if separated(vs(va(vm(edge, t), d), vm(e, u))) {
            return false;
        }
    }
    true
}
fn extrude(tri: Triangle, half: Vec3, direction: Vec3) -> [(Triangle, Vec3); 7] {
    let [a, b, c] = tri;
    let normal = cross(vs(b, a), vs(c, a));
    let [am, bm, cm] = tri.map(|p| vs(p, half));
    let [ap, bp, cp] = tri.map(|p| va(half, p));
    let cap = if dot(normal, half) >= 0. {
        [ap, bp, cp]
    } else {
        [am, bm, cm]
    };
    let faces = [
        cap,
        [bm, bp, cp],
        [bm, cp, cm],
        [am, cm, cp],
        [am, cp, ap],
        [ap, bp, bm],
        [ap, bm, am],
    ];
    std::array::from_fn(|i| {
        let mut face = faces[i];
        let mut n = cross(vs(face[1], face[0]), vs(face[2], face[0]));
        if i != 0 && dot(n, direction) > 0. {
            face.swap(1, 2);
            n = neg(n);
        }
        (face, n)
    })
}
/// Full original static-triangle numerical kernel, supplied optional OBB.
/// Guard limits are LAB policy, not native validation thresholds. No scene query.
pub fn triangle_sweep(
    c: &AxialTriangleInput<'_>,
    bounds: Option<crate::capsule_bounds::Bounds>,
) -> Result<PrimitiveMeshResult, MeshError> {
    if c.triangles.is_empty() {
        return Ok(PrimitiveMeshResult {
            hit: None,
            index_write: None,
        });
    }
    if c.cached_index as usize >= c.triangles.len()
        || !c
            .capsule_a
            .into_iter()
            .chain(c.capsule_b)
            .chain([c.radius])
            .all(bounded)
        || c.radius < 0.
        || !c.max_distance.is_finite()
        || c.max_distance.abs() > 1e8
        || !unit_direction(c.direction)
        || c.triangles.iter().flatten().flatten().any(|x| !bounded(*x))
        || bounds.is_some_and(|b| {
            !b.axes
                .into_iter()
                .flatten()
                .chain(b.center)
                .chain(b.extents)
                .all(f32::is_finite)
        })
    {
        return Err(MeshError::InvalidInput);
    }
    let center = vm(va(c.capsule_b, c.capsule_a), 0.5);
    let delta = vs(c.capsule_a, c.capsule_b);
    let half = vm(delta, 0.5);
    let half_length = (dot(delta, delta) * 0.25).sqrt();
    if half_length == 0. || dot(vm(half, 1. / half_length), c.direction).abs() >= AXIAL_COSINE {
        let hit = axial_unchecked(c)?;
        return Ok(PrimitiveMeshResult {
            index_write: hit.map(|h| h.index as u32),
            hit,
        });
    }
    let mut best = c.max_distance;
    let mut alignment = 2.;
    let mut winner = None;
    let mut saved = [0.; 3];
    let mut winner_face = [[0.; 3]; 3];
    let origin_projection = dot(center, c.direction);
    let cull = !c.double_sided && c.flags & 0x80 == 0;
    for ordinal in 0..c.triangles.len() {
        let idx = if ordinal == 0 {
            c.cached_index as usize
        } else if ordinal == c.cached_index as usize {
            0
        } else {
            ordinal
        };
        let tri = c.triangles[idx];
        let raw_n = cross(vs(tri[1], tri[0]), vs(tri[2], tri[0]));
        if cull && dot(raw_n, c.direction) > 0. {
            continue;
        }
        if bounds.is_some_and(|b| !crate::capsule_bounds::triangle_bounds(tri, b)) {
            continue;
        }
        if c.flags & 0x10 == 0 && capsule_initial(tri, c.capsule_a, c.capsule_b, c.radius) {
            let hit = CollectionHit {
                index: idx,
                flags: 0x402,
                point: None,
                normal: neg(c.direction),
                distance: 0.,
                aux: neg(c.direction),
            };
            return Ok(PrimitiveMeshResult {
                hit: Some(hit),
                index_write: Some(idx as u32),
            });
        }
        let source_n = if len(raw_n) > 0. {
            vm(raw_n, 1. / len(raw_n))
        } else {
            raw_n
        };
        let score = -dot(source_n, c.direction).abs();
        for (face, raw_normal) in extrude(tri, half, c.direction) {
            if cull && dot(raw_normal, c.direction) > 0. {
                continue;
            }
            let centroid = vm(va(va(face[1], face[0]), face[2]), 1. / 3.);
            let d = vs(centroid, center);
            let t = dot(d, c.direction).max(0.).min(best);
            let r = (len(vs(d, vm(c.direction, t))) - c.radius) - 1e-4;
            if r >= 0.
                && !face
                    .into_iter()
                    .any(|p| r * r <= dot(vs(centroid, p), vs(centroid, p)))
            {
                continue;
            }
            let proj = face.map(|p| dot(p, c.direction));
            if proj[0].min(proj[1]).min(proj[2]) > (origin_projection + best) + (c.radius + 0.012)
                || !proj.into_iter().any(|p| p >= origin_projection)
            {
                continue;
            }
            let l = len(raw_normal);
            if l == 0. {
                continue;
            }
            let normal = vm(raw_normal, 1. / l);
            let Some(candidate) =
                precise_capsule::triangle_hit(face, normal, center, c.radius, c.direction, false)
            else {
                continue;
            };
            let t = candidate.time;
            if t > c.max_distance {
                continue;
            }
            let epsilon = t.max(best).max(1.) * 0.001;
            if !(t < best - epsilon
                || (t < epsilon + best && score < alignment)
                || (score == alignment && t < best)
                || t == 0.)
            {
                continue;
            }
            best = t;
            saved = source_n;
            alignment = score;
            winner = Some(idx);
            winner_face = face;
            if c.flags & 0x40 != 0 {
                break;
            }
        }
        if winner.is_some() && c.flags & 0x40 != 0 {
            break;
        }
    }
    let Some(idx) = winner else {
        return Ok(PrimitiveMeshResult {
            hit: None,
            index_write: Some(u32::MAX),
        });
    };
    let (_, mut normal) = precise_capsule::contact(center, c.direction, best, winner_face);
    if c.flags & 0x80 != 0 && !c.double_sided && dot(saved, c.direction) > 0. {
        normal = neg(normal);
    }
    let moved_a = va(vm(c.direction, best), c.capsule_a);
    let moved_b = va(vm(c.direction, best), c.capsule_b);
    let witness = crate::segment_triangle::witness(moved_a, moved_b, c.triangles[idx])
        .ok_or(MeshError::NonFiniteArithmetic)?;
    let point = [
        witness.triangle[0],
        witness.triangle[1],
        witness.triangle[2],
    ];
    if !best.is_finite()
        || !normal
            .into_iter()
            .chain(saved)
            .chain(point)
            .all(f32::is_finite)
    {
        return Err(MeshError::NonFiniteArithmetic);
    }
    Ok(PrimitiveMeshResult {
        hit: Some(CollectionHit {
            index: idx,
            flags: 3,
            point: Some(point),
            normal,
            distance: best,
            aux: saved,
        }),
        index_write: Some(idx as u32),
    })
}

/// Actual Main 141EE4640 / 141EE7010 cached-mesh adapter, within the recovered
/// finite capsule domain. Misses retain the supplied cache. Successful local
/// indices update it, while returned global and source IDs remain distinct.
/// The caller commits this returned state only after the entire solve succeeds.
pub fn cached_mesh_query(c: &CachedMeshInput<'_>) -> Result<CachedMeshResult, MeshError> {
    let miss = CachedMeshResult {
        hit: None,
        cached_index: c.cached_index,
    };
    if c.triangle_count == 0 {
        return Ok(miss);
    }
    let start = c.triangle_offset as usize;
    let end = start
        .checked_add(c.triangle_count as usize)
        .ok_or(MeshError::InvalidInput)?;
    let triangles = c.triangles.get(start..end).ok_or(MeshError::InvalidInput)?;
    if c.original_ids.get(start..end).is_none()
        || c.pass_id > 2
        || !c
            .quaternion
            .into_iter()
            .chain([c.radius, c.height])
            .all(bounded)
        || c.height < 0.
        || c.radius < 0.
        || !c.center.into_iter().chain(c.offset).all(f64::is_finite)
        || !c.max_distance.is_finite()
    {
        return Err(MeshError::InvalidInput);
    }
    let q = c.quaternion;
    let q_squared = ((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3];
    if (q_squared - 1.).abs() > 0.001 {
        return Err(MeshError::InvalidInput);
    }
    let position = std::array::from_fn(|i| (c.center[i] - c.offset[i]) as f32);
    let (a, b) = mesh_endpoints(c.quaternion, position, c.height * 0.5);
    let cache = if c.cached_index < c.triangle_count {
        c.cached_index
    } else {
        0
    };
    let flags = (if c.precise { 0x503 } else { 0x403 }) | if c.overlap { 0 } else { 0x10 };
    let bounds = crate::capsule_bounds::swept_obb(
        crate::capsule_bounds::capsule_obb(a, b, c.radius),
        c.direction,
        c.max_distance.min(1e8),
    );
    let result = triangle_sweep(
        &AxialTriangleInput {
            triangles,
            capsule_a: a,
            capsule_b: b,
            radius: c.radius,
            direction: c.direction,
            max_distance: c.max_distance.min(1e8),
            cached_index: cache,
            flags,
            double_sided: false,
        },
        Some(bounds),
    )?;
    let Some(hit) = result.hit else {
        return Ok(miss);
    };
    if hit.distance >= c.max_distance {
        return Ok(miss);
    }
    let local = u32::try_from(hit.index).map_err(|_| MeshError::InvalidInput)?;
    let global = c
        .triangle_offset
        .checked_add(local)
        .ok_or(MeshError::InvalidInput)?;
    let point = hit.point.unwrap_or([0.; 3]);
    let world_point = std::array::from_fn(|i| f64::from(point[i]) + c.offset[i]);
    if !world_point.into_iter().all(f64::is_finite) {
        return Err(MeshError::NonFiniteArithmetic);
    }
    Ok(CachedMeshResult {
        cached_index: local,
        hit: Some(MeshHit {
            world_point,
            normal: hit.normal,
            distance: hit.distance,
            global_index: global,
            original_index: c.original_ids[global as usize],
            source_point_written: hit.point.is_some(),
        }),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    const TRI: Triangle = [[-5., 0., -5.], [0., 0., 5.], [5., 0., -5.]];

    #[test]
    fn general_initial_and_candidate_zero_have_distinct_point_writes() {
        let tris = [TRI];
        let norm = (0.13_f32 * 0.13 + 1. + 0.17 * 0.17).sqrt();
        let mut c = AxialTriangleInput {
            triangles: &tris,
            capsule_a: [-0.75, 0.5, 0.],
            capsule_b: [0.75, 0.5, 0.],
            radius: 0.5,
            direction: [0.13 / norm, -1. / norm, 0.17 / norm],
            max_distance: 5.,
            cached_index: 0,
            flags: 0x503,
            double_sided: false,
        };
        let initial = triangle_sweep(&c, None).unwrap().hit.unwrap();
        assert_eq!(initial.flags, 0x402);
        assert_eq!(initial.distance, 0.);
        assert_eq!(initial.point, None);
        c.flags = 0x513;
        let candidate = triangle_sweep(&c, None).unwrap().hit.unwrap();
        assert_eq!(candidate.flags, 3);
        assert_eq!(candidate.distance, 0.);
        assert!(candidate.point.is_some());
    }

    #[test]
    fn supplied_bounds_reject_general_triangle_and_publish_only_miss_index() {
        let tris = [TRI];
        let c = AxialTriangleInput {
            triangles: &tris,
            capsule_a: [-0.75, 0.25, 0.],
            capsule_b: [0.75, 0.25, 0.],
            radius: 0.5,
            direction: [0., -1., 0.],
            max_distance: 5.,
            cached_index: 0,
            flags: 0x503,
            double_sided: false,
        };
        let bounds = crate::capsule_bounds::Bounds {
            axes: [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            center: [100., 100., 100.],
            extents: [1., 1., 1.],
        };
        assert_eq!(
            triangle_sweep(&c, Some(bounds)).unwrap(),
            PrimitiveMeshResult {
                hit: None,
                index_write: Some(u32::MAX)
            }
        );
        assert!(triangle_sweep(&c, None).unwrap().hit.is_some());
        let empty = AxialTriangleInput {
            triangles: &[],
            ..c
        };
        assert_eq!(
            triangle_sweep(&empty, None).unwrap(),
            PrimitiveMeshResult {
                hit: None,
                index_write: None
            }
        );
    }

    #[test]
    fn actual_axial_domain_rejects_oblique_moves_and_admits_boundary() {
        let triangles = [TRI];
        let mut c = AxialTriangleInput {
            triangles: &triangles,
            capsule_a: [0., 1., 0.],
            capsule_b: [0., -1., 0.],
            radius: 0.3,
            direction: [1., 0., 0.],
            max_distance: 5.,
            cached_index: 0,
            flags: 0x513,
            double_sided: false,
        };
        assert_eq!(
            axial_triangle_sweep(&c),
            Err(MeshError::UnsupportedNonaxial)
        );
        for offset in [-1_i32, 0, 1] {
            let cosine = f32::from_bits((AXIAL_COSINE.to_bits() as i32 + offset) as u32);
            c.direction = [(1. - cosine * cosine).sqrt(), -cosine, 0.];
            assert_eq!(axial_triangle_sweep(&c).is_err(), offset < 0);
        }
    }

    #[test]
    fn cache_is_local_source_ids_are_distinct_and_equal_limit_preserves_cache() {
        let triangles = [TRI, TRI];
        let ids = [20, 901];
        let root_half = std::f32::consts::FRAC_1_SQRT_2;
        let mut c = CachedMeshInput {
            triangles: &triangles,
            original_ids: &ids,
            quaternion: [0., 0., root_half, root_half],
            radius: 0.3,
            height: 1.4,
            center: [100., 203., 300.],
            offset: [100., 200., 300.],
            direction: [0., -1., 0.],
            max_distance: 5.,
            triangle_offset: 1,
            triangle_count: 1,
            cached_index: 7,
            pass_id: 2,
            overlap: true,
            precise: true,
        };
        let r = cached_mesh_query(&c).unwrap();
        let hit = r.hit.unwrap();
        assert_eq!(
            (r.cached_index, hit.global_index, hit.original_index),
            (0, 1, 901)
        );
        assert_eq!(hit.world_point, c.offset);
        c.max_distance = hit.distance;
        let r = cached_mesh_query(&c).unwrap();
        assert_eq!(
            r,
            CachedMeshResult {
                hit: None,
                cached_index: 7
            }
        );
    }

    #[test]
    fn zero_hit_publishes_initialized_offset_and_invalid_ranges_return_error() {
        let triangles = [TRI];
        let ids = [9];
        let root_half = std::f32::consts::FRAC_1_SQRT_2;
        let mut c = CachedMeshInput {
            triangles: &triangles,
            original_ids: &ids,
            quaternion: [0., 0., root_half, root_half],
            radius: 0.3,
            height: 1.4,
            center: [100., 200.99, 300.],
            offset: [100., 200., 300.],
            direction: [0., -1., 0.],
            max_distance: 5.,
            triangle_offset: 0,
            triangle_count: 1,
            cached_index: 0,
            pass_id: 2,
            overlap: true,
            precise: true,
        };
        let hit = cached_mesh_query(&c).unwrap().hit.unwrap();
        assert_eq!(hit.distance, 0.);
        assert!(!hit.source_point_written);
        assert_eq!(hit.world_point, c.offset);
        c.triangle_offset = 1;
        assert_eq!(cached_mesh_query(&c), Err(MeshError::InvalidInput));
    }
}
