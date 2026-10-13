//! Original, bounded static-box CCT pass scheduling and target response.
//!
//! Geometry queries and reports are supplied boundaries. This is not the native
//! scene/cache/actor implementation. Callbacks may not mutate controller settings
//! or positions through aliases. Mesh, moving-object and whole slope-retry paths
//! require additional state and are explicitly outside this API.

use crate::cct_response;
use serde::{Deserialize, Serialize};

pub type Vector = [f32; 3];
pub type Position = [f64; 3];

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct Settings {
    pub up: Vector,
    pub contact_offset: f32,
    pub step_offset: f32,
    pub min_distance: f32,
    pub overlap_recovery: bool,
    pub prevent_ceiling_slide: bool,
    pub normalize_components: bool,
    pub custom_response: bool,
    pub walk_experiment: bool,
    pub standing_on_moving: bool,
    pub nonwalkable_mode: u32,
    pub slope_limit: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[repr(u8)]
pub enum Pass {
    Up = 0,
    Side = 1,
    Down = 2,
}

/// A winning static box contact. Other object and mesh records must not be
/// coerced into this type; their publication/response rules differ.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct BoxHit {
    pub distance: f32,
    pub normal: Vector,
    pub point: Option<Position>,
    pub shape_id: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub enum Error {
    InvalidInput,
    NonFiniteArithmetic,
    UnsupportedPolicy,
    OverlapRecoveryRequired,
    InvalidQueryResult,
    QueryFailed,
    RecoveryScanLimit,
}

/// Supplied native boundaries. Query admission is strict: reject distance >=
/// the current winner limit, preserve earlier ties, and reject zero distances
/// when overlap recovery is disabled. Report returns supplied behavior flags;
/// bit 2 extends only the first DOWN contact's remaining iteration budget.
pub trait Scene {
    fn prepare(&mut self, center: Position, remaining: Vector, side: Vector) -> Result<(), Error>;
    fn query(
        &mut self,
        center: Position,
        direction: Vector,
        limit: f32,
        pass: Pass,
    ) -> Result<Option<BoxHit>, Error>;
    fn report(
        &mut self,
        hit: &BoxHit,
        direction: Vector,
        remaining_distance: f32,
        pass: Pass,
    ) -> Result<u8, Error>;
    /// Native zero-hit branch needs a separate penetration scan. The default
    /// exposes the missing dependency instead of fabricating a normal push.
    fn recover_overlap(&mut self, _center: Position) -> Result<Position, Error> {
        Err(Error::OverlapRecoveryRequired)
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct IterationResult {
    pub position: Position,
    pub attempted: bool,
    pub collisions: u32,
    pub iterations: u32,
    pub hits: Vec<BoxHit>,
}

#[derive(Clone, Debug, Serialize)]
pub struct MoveResult {
    pub position: Position,
    pub collision_flags: u8,
    pub passes: Vec<IterationResult>,
}

fn dot_yx(a: Vector, b: Vector) -> f32 {
    (a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]
}
fn dot_xy(a: Vector, b: Vector) -> f32 {
    (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]
}
fn height(up: Vector, p: Position) -> f64 {
    (f64::from(up[0]) * p[0] + f64::from(up[1]) * p[1]) + f64::from(up[2]) * p[2]
}
fn side_is_zero(side: Vector) -> bool {
    side.into_iter().all(|x| x.abs() <= 1.0e-6_f32)
}
fn validate(center: Position, displacement: Vector, settings: &Settings) -> Result<(), Error> {
    if !center.into_iter().all(f64::is_finite)
        || !displacement
            .into_iter()
            .chain(settings.up)
            .chain([
                settings.contact_offset,
                settings.step_offset,
                settings.min_distance,
                settings.slope_limit,
            ])
            .all(f32::is_finite)
        || settings.contact_offset < 0.
        || settings.step_offset < 0.
        || settings.min_distance < 0.
        || (dot_yx(settings.up, settings.up) - 1.).abs() > 0.001
    {
        return Err(Error::InvalidInput);
    }
    if settings.standing_on_moving {
        return Err(Error::UnsupportedPolicy);
    }
    Ok(())
}

/// Isolated native iteration policy with supplied geometry/report boundaries.
/// No velocity channel is accepted or modified. Initial-overlap handling remains
/// explicit until the penetration-recovery dependency is implemented.
#[allow(clippy::too_many_arguments)]
pub fn iterate(
    center: Position,
    displacement: Vector,
    side: Vector,
    pass: Pass,
    mut budget: u32,
    settings: &Settings,
    scene: &mut impl Scene,
) -> Result<IterationResult, Error> {
    validate(center, displacement, settings)?;
    // This caller bound is a LAB API guard, not an inferred native rejection.
    if !side.into_iter().all(f32::is_finite) || budget > 10 {
        return Err(Error::InvalidInput);
    }
    let mut out = IterationResult {
        position: center,
        attempted: false,
        collisions: 0,
        iterations: 0,
        hits: Vec::new(),
    };
    if displacement.into_iter().all(|x| x == 0.) {
        return Ok(out);
    }
    let mut target: Position = std::array::from_fn(|i| center[i] + f64::from(displacement[i]));
    let mut previous_bias = [0.; 3];
    let mut repeated_bias = 0_u32;
    while budget != 0 {
        out.iterations += 1;
        let mut next_budget = budget - 1;
        let delta: Vector = std::array::from_fn(|i| (target[i] - out.position[i]) as f32);
        if !delta.into_iter().all(f32::is_finite) {
            return Err(Error::NonFiniteArithmetic);
        }
        scene.prepare(out.position, delta, side)?;
        let distance = dot_yx(delta, delta).sqrt();
        if !distance.is_finite() {
            return Err(Error::NonFiniteArithmetic);
        }
        if distance <= settings.min_distance {
            break;
        }
        let inverse = 1. / distance;
        let direction = delta.map(|x| x * inverse);
        if dot_xy(direction, displacement) <= 0. {
            break;
        }
        out.attempted = true;
        let limit = distance + settings.contact_offset;
        if !limit.is_finite() {
            return Err(Error::NonFiniteArithmetic);
        }
        let Some(hit) = scene.query(out.position, direction, limit, pass)? else {
            out.position = target;
            break;
        };
        if !hit.distance.is_finite()
            || !hit.normal.into_iter().all(f32::is_finite)
            || hit
                .point
                .is_some_and(|p| !p.into_iter().all(f64::is_finite))
            || (hit.distance != 0. && hit.point.is_none())
            || hit.distance < 0.
            || hit.distance >= limit
        {
            return Err(Error::InvalidQueryResult);
        }
        if hit.distance == 0. {
            if !settings.overlap_recovery {
                return Err(Error::InvalidQueryResult);
            }
            out.position = scene.recover_overlap(out.position)?;
            if !out.position.into_iter().all(f64::is_finite) {
                return Err(Error::NonFiniteArithmetic);
            }
            out.collisions += 1;
            out.hits.push(hit);
            return Ok(out);
        }
        let behavior = scene.report(&hit, direction, distance, pass)?;
        if pass == Pass::Down && behavior & 2 != 0 && out.collisions == 0 {
            next_budget = budget + 8;
        }
        out.collisions += 1;
        if hit.distance > settings.contact_offset {
            let advance = hit.distance - settings.contact_offset;
            out.position =
                std::array::from_fn(|i| out.position[i] + f64::from(direction[i] * advance));
        }
        let mut normal = hit.normal;
        if settings.walk_experiment && settings.nonwalkable_mode != 1 {
            let q = dot_xy(normal, settings.up);
            normal = normalize_positive(std::array::from_fn(|i| normal[i] - settings.up[i] * q));
        }
        let normal_up = dot_xy(hit.normal, settings.up);
        let mut lifted = false;
        if settings.custom_response
            && settings.walk_experiment
            && pass == Pass::Side
            && normal_up > settings.slope_limit
        {
            let remaining: Vector = std::array::from_fn(|i| (target[i] - out.position[i]) as f32);
            let projection = dot_yx(remaining, settings.up);
            let vertical = settings.up.map(|u| projection * u);
            let room = settings.step_offset - dot_xy(vertical, vertical).sqrt();
            if room > 0. {
                let tangent: Vector = std::array::from_fn(|i| remaining[i] - vertical[i]);
                let candidate = (dot_yx(tangent, tangent) * (1. - normal_up * normal_up)).sqrt();
                // Native MINSS chooses room on unordered/equal inputs.
                let lift = if candidate < room { candidate } else { room };
                target = std::array::from_fn(|i| target[i] + f64::from(lift * settings.up[i]));
                lifted = true;
            }
        }
        if !lifted {
            target = cct_response::response(
                target,
                out.position,
                direction,
                normal,
                0.,
                1.,
                settings.normalize_components,
            );
            if settings.custom_response {
                let point = hit.point.ok_or(Error::InvalidQueryResult)?;
                let delta: Vector = std::array::from_fn(|i| (out.position[i] - point[i]) as f32);
                let square = dot_yx(delta, delta);
                let bias = if square > 0. {
                    let inverse = 1. / square.sqrt();
                    delta.map(|x| inverse * x)
                } else {
                    [0.; 3]
                };
                let q = dot_yx(settings.up, bias);
                let bias =
                    normalize_positive(std::array::from_fn(|i| bias[i] - settings.up[i] * q));
                let scale = settings.contact_offset * 0.2_f32;
                target = std::array::from_fn(|i| {
                    target[i] + f64::from((scale * bias[i]) * repeated_bias as f32)
                });
                if dot_yx(bias, previous_bias) > 0.99_f32 {
                    repeated_bias = repeated_bias.wrapping_add(1);
                }
                previous_bias = bias;
            }
        }
        if !target.into_iter().chain(out.position).all(f64::is_finite) {
            return Err(Error::NonFiniteArithmetic);
        }
        out.hits.push(hit);
        budget = next_budget;
    }
    Ok(out)
}

/// Split a request into UP, SIDE and DOWN passes for the bounded static scene.
/// Returned collision bits are not equivalent to engine actor landing/grounding.
pub fn move_with_scene(
    center: Position,
    displacement: Vector,
    settings: &Settings,
    scene: &mut impl Scene,
) -> Result<MoveResult, Error> {
    validate(center, displacement, settings)?;
    if settings.walk_experiment {
        return Err(Error::UnsupportedPolicy);
    }
    let up = settings.up;
    let up_dot = dot_yx(up, displacement);
    let vertical_dot = dot_xy(up, displacement);
    let vertical = up.map(|x| x * vertical_dot);
    let side: Vector = std::array::from_fn(|i| displacement[i] - vertical[i]);
    let mut up_motion = if up_dot > 0. { vertical } else { [0.; 3] };
    let mut down_motion = if up_dot > 0. { [0.; 3] } else { vertical };
    let mut step = if up_dot > 0. {
        0.
    } else {
        settings.step_offset
    };
    let side_zero = side_is_zero(side);
    if !side_zero {
        up_motion = std::array::from_fn(|i| up[i] * step + up_motion[i]);
    }
    scene.prepare(center, displacement, side)?;
    let original_height = height(up, center);
    let mut flags = 0;
    let up_budget = if settings.prevent_ceiling_slide || !side_zero {
        1
    } else {
        10
    };
    let up_result = iterate(
        center,
        up_motion,
        side,
        Pass::Up,
        up_budget,
        settings,
        scene,
    )?;
    if up_result.attempted && up_result.collisions != 0 {
        flags |= 2;
        step = step.min((height(up, up_result.position) - original_height) as f32);
    }
    let side_result = iterate(
        up_result.position,
        side,
        side,
        Pass::Side,
        10,
        settings,
        scene,
    )?;
    if side_result.attempted && side_result.collisions != 0 {
        flags |= 1;
    }
    if !side_zero {
        down_motion = std::array::from_fn(|i| down_motion[i] - step * up[i]);
    }
    let down_result = iterate(
        side_result.position,
        down_motion,
        side,
        Pass::Down,
        1,
        settings,
        scene,
    )?;
    if down_result.attempted && down_result.collisions != 0 && up_dot <= 0. {
        flags |= 4;
    }
    Ok(MoveResult {
        position: down_result.position,
        collision_flags: flags,
        passes: vec![up_result, side_result, down_result],
    })
}

fn normalize_positive(v: Vector) -> Vector {
    let length = dot_yx(v, v).sqrt();
    if length > 0. {
        let inverse = 1. / length;
        v.map(|x| inverse * x)
    } else {
        v
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    struct MissingPoint {
        distance: f32,
        recovered: bool,
        reported: bool,
    }
    impl Scene for MissingPoint {
        fn prepare(&mut self, _: Position, _: Vector, _: Vector) -> Result<(), Error> {
            Ok(())
        }
        fn query(
            &mut self,
            _: Position,
            _: Vector,
            _: f32,
            _: Pass,
        ) -> Result<Option<BoxHit>, Error> {
            Ok(Some(BoxHit {
                distance: self.distance,
                normal: [-1., 0., 0.],
                point: None,
                shape_id: 0,
            }))
        }
        fn report(&mut self, _: &BoxHit, _: Vector, _: f32, _: Pass) -> Result<u8, Error> {
            self.reported = true;
            Ok(4)
        }
        fn recover_overlap(&mut self, _: Position) -> Result<Position, Error> {
            self.recovered = true;
            Ok([0.25, 0., 0.])
        }
    }
    fn settings() -> Settings {
        Settings {
            up: [0., 1., 0.],
            contact_offset: 0.02,
            step_offset: 0.,
            min_distance: 1e-6,
            overlap_recovery: true,
            prevent_ceiling_slide: false,
            normalize_components: false,
            custom_response: true,
            walk_experiment: false,
            standing_on_moving: false,
            nonwalkable_mode: 0,
            slope_limit: 0.7,
        }
    }
    #[test]
    fn positive_contact_requires_valid_point_before_report() {
        let mut scene = MissingPoint {
            distance: 0.5,
            recovered: false,
            reported: false,
        };
        assert!(matches!(
            iterate(
                [0.; 3],
                [1., 0., 0.],
                [1., 0., 0.],
                Pass::Side,
                10,
                &settings(),
                &mut scene
            ),
            Err(Error::InvalidQueryResult)
        ));
        assert!(!scene.recovered && !scene.reported);
    }
    #[test]
    fn zero_contact_without_point_delegates_recovery_and_skips_report() {
        let mut scene = MissingPoint {
            distance: 0.,
            recovered: false,
            reported: false,
        };
        let out = iterate(
            [0.; 3],
            [1., 0., 0.],
            [1., 0., 0.],
            Pass::Side,
            10,
            &settings(),
            &mut scene,
        )
        .unwrap();
        assert!(scene.recovered && !scene.reported && out.attempted);
        assert_eq!(out.position, [0.25, 0., 0.]);
        assert_eq!((out.collisions, out.iterations), (1, 1));
    }
}
