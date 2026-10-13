//! Recovered finite slope/pass scheduler with explicit iteration publication.
//!
//! The backend owns contact geometry. In particular, box normals cannot stand
//! in for native mesh flags or triangle/bound heights. This module supplies no
//! moving-actor policy, live configuration, scene filtering or frame cadence.

pub type Vector = [f32; 3];
pub type Position = [f64; 3];

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct State {
    pub position: Position,
    pub flags: u32,
    pub tag: u32,
    pub down_normal: Vector,
    pub side_normal: Vector,
    pub triangle_height: f32,
    pub bound_height: f32,
    pub contact_height: f32,
    pub collision_flags: u8,
}

#[derive(Clone, Copy, Debug)]
pub struct Settings {
    pub up: Vector,
    pub step_offset: f32,
    pub min_distance: f32,
    pub nonwalkable_mode: u32,
    pub standing: bool,
    pub prevent_ceiling_slide: bool,
    pub slope_enabled: bool,
    pub slope_limit: f32,
    pub constrain: bool,
    pub extent: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Iteration {
    pub displacement: Vector,
    pub side: Vector,
    pub budget: u32,
    pub minimum: f32,
    /// The correction calls pass zero while state.tag remains two.
    pub pass: u32,
}

#[derive(Clone, Copy, Debug)]
pub struct Outcome {
    pub attempted: bool,
    pub collisions: u32,
}

/// The validated preparation boundary records its arguments without mutating State.
/// Validated iteration publications change position, flags, normals and heights;
/// the scheduler owns tag and collision_flags. Other callback mutations are outside
/// the native comparison domain. All publications must preserve finite arithmetic.
/// Settings are immutable; arbitrary native callback alias effects are external.
pub trait Backend {
    fn prepare(&mut self, state: &mut State, displacement: Vector, side: Vector);
    fn iterate(&mut self, state: &mut State, request: Iteration) -> Outcome;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    NonFinite,
}

#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Publication {
    pub elapsed: f64,
    pub pending: Vector,
    pub position: Position,
    pub collision_flags: u8,
}

fn xy(a: Vector, b: Vector) -> f32 {
    (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]
}
fn yx(a: Vector, b: Vector) -> f32 {
    (a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]
}
fn double_xy(a: Vector, b: Position) -> f64 {
    (f64::from(a[0]) * b[0] + f64::from(a[1]) * b[1]) + f64::from(a[2]) * b[2]
}
fn double_yx(a: Vector, b: Position) -> f64 {
    (f64::from(a[1]) * b[1] + f64::from(a[0]) * b[0]) + f64::from(a[2]) * b[2]
}
fn finite_state(s: &State) -> bool {
    s.position.into_iter().all(f64::is_finite)
        && s.down_normal
            .into_iter()
            .chain(s.side_normal)
            .chain([s.triangle_height, s.bound_height, s.contact_height])
            .all(f32::is_finite)
}
fn validate(s: &State, c: &Settings, d: Vector) -> Result<(), Error> {
    if finite_state(s)
        && d.into_iter()
            .chain(c.up)
            .chain([c.step_offset, c.min_distance, c.slope_limit, c.extent])
            .all(f32::is_finite)
    {
        Ok(())
    } else {
        Err(Error::NonFinite)
    }
}

/// Original splitter arithmetic for finite normal intermediates and zeros.
/// Subnormal FTZ/DAZ equivalence is outside the validated domain. Geometry/type guards
/// that publish mesh state belong to the backend, not this scheduler.
pub fn split(
    s: &mut State,
    c: &Settings,
    d: Vector,
    backend: &mut impl Backend,
) -> Result<(), Error> {
    validate(s, c, d)?;
    let up = c.up;
    let mut step = c.step_offset;
    let mut minimum = c.min_distance;
    s.collision_flags = 0;
    s.flags &= !1;
    let down_budget = if s.flags & 2 != 0 && c.nonwalkable_mode == 1 {
        10
    } else {
        1
    };
    let height = double_xy(up, s.position);
    let bottom = height - f64::from(c.extent);
    let q = yx(up, d);
    if q > 0. {
        s.flags |= 0x100;
        if !c.standing {
            step = 0.;
        }
    } else {
        s.flags &= !0x100;
    }
    let v = xy(up, d);
    let vertical = up.map(|u| u * v);
    let side = std::array::from_fn(|i| d[i] - vertical[i]);
    let mut up_delta = if q > 0. { vertical } else { [0.; 3] };
    let mut down = if q > 0. { [0.; 3] } else { vertical };
    let side_zero = !c.standing && side.into_iter().all(|x| x.abs() <= 1e-6);
    if !side_zero {
        up_delta = std::array::from_fn(|i| up[i] * step + up_delta[i]);
    }
    if !height.is_finite()
        || !bottom.is_finite()
        || !q.is_finite()
        || !up_delta
            .into_iter()
            .chain(down)
            .chain(side)
            .all(f32::is_finite)
    {
        return Err(Error::NonFinite);
    }
    backend.prepare(s, d, side);
    finish(s)?;
    s.tag = 0;
    let budget = if c.prevent_ceiling_slide || side.into_iter().any(|x| x.abs() > 1e-6) {
        1
    } else {
        10
    };
    if s.flags & 2 == 0 {
        let o = iteration(
            backend,
            s,
            Iteration {
                displacement: up_delta,
                side,
                budget,
                minimum,
                pass: 0,
            },
        )?;
        if o.attempted && o.collisions != 0 {
            s.collision_flags |= 2;
            let actual = (double_xy(up, s.position) - height) as f32;
            if actual < step {
                step = actual;
            }
        }
    }
    s.flags &= !8;
    s.tag = 1;
    let o = iteration(
        backend,
        s,
        Iteration {
            displacement: side,
            side,
            budget: 10,
            minimum,
            pass: 1,
        },
    )?;
    if o.attempted && o.collisions != 0 {
        s.collision_flags |= 1;
    }
    s.tag = 2;
    if !side_zero {
        down = std::array::from_fn(|i| down[i] - step * up[i]);
    }
    s.flags &= !4;
    let o = iteration(
        backend,
        s,
        Iteration {
            displacement: down,
            side,
            budget: down_budget,
            minimum,
            pass: 2,
        },
    )?;
    if o.attempted {
        if o.collisions == 0 {
            if s.flags & 2 != 0 && !side_zero {
                s.position = std::array::from_fn(|i| f64::from(step * up[i]) + s.position[i]);
            }
        } else {
            if q <= 0. {
                s.collision_flags |= 4;
            }
            if c.slope_enabled
                && s.flags & 0x38 == 8
                && yx(s.side_normal, up) < c.slope_limit
                && c.constrain
                && f64::from(step) + bottom < f64::from(s.contact_height)
            {
                s.flags |= 1;
                if s.flags & 2 == 0 {
                    return finish(s);
                }
            }
        }
    }
    if c.slope_enabled && s.flags & 0x34 == 4 && q <= 0. {
        let triangle = (f64::from(s.triangle_height) - bottom) as f32;
        let bound = (f64::from(s.bound_height) - bottom) as f32;
        let normal = yx(s.down_normal, up);
        if c.nonwalkable_mode == 3 {
            if triangle <= c.step_offset && (normal < 0. || normal >= c.slope_limit) {
                return finish(s);
            }
            if bound <= c.step_offset {
                return finish(s);
            }
        } else if triangle <= c.step_offset || normal < 0. || normal >= c.slope_limit {
            return finish(s);
        }
        s.flags |= 1;
        if s.flags & 2 != 0 {
            s.flags |= 0x40;
            let current = double_yx(up, s.position);
            let rise = if current > height {
                (current - height) as f32
            } else {
                0.
            };
            let magnitude = yx(up, d).abs() + rise;
            if magnitude < minimum {
                minimum = magnitude * 0.1;
            }
            let correction = up.map(|u| f32::from_bits((magnitude * u).to_bits() ^ 0x8000_0000));
            iteration(
                backend,
                s,
                Iteration {
                    displacement: correction,
                    side,
                    budget: 10,
                    minimum,
                    pass: 0,
                },
            )?;
            s.flags &= !0x40;
        }
    }
    finish(s)
}
fn iteration(
    backend: &mut impl Backend,
    s: &mut State,
    request: Iteration,
) -> Result<Outcome, Error> {
    if !request
        .displacement
        .into_iter()
        .chain(request.side)
        .chain([request.minimum])
        .all(f32::is_finite)
    {
        return Err(Error::NonFinite);
    }
    let result = backend.iterate(s, request);
    finish(s)?;
    Ok(result)
}
fn finish(s: &State) -> Result<(), Error> {
    if finite_state(s) {
        Ok(())
    } else {
        Err(Error::NonFinite)
    }
}

/// Original common-call accumulation/retry/publication, with moving-object and
/// other wrapper boundaries outside this API. A retry restores position only;
/// the result byte is replaced by split rather than merged with the first call.
pub fn common(
    s: &mut State,
    c: &Settings,
    displacement: Vector,
    publication: &mut Publication,
    dt: f32,
    backend: &mut impl Backend,
) -> Result<(), Error> {
    if !dt.is_finite()
        || !publication.elapsed.is_finite()
        || !publication.pending.into_iter().all(f32::is_finite)
    {
        return Err(Error::NonFinite);
    }
    let mut d = std::array::from_fn(|i| displacement[i] + publication.pending[i]);
    validate(s, c, d)?;
    publication.elapsed += f64::from(dt);
    publication.pending = [0.; 3];
    if !publication.elapsed.is_finite() {
        return Err(Error::NonFinite);
    }
    s.flags = (s.flags | 0x80) & !2;
    let original = s.position;
    split(s, c, d, backend)?;
    if s.flags & 1 != 0 {
        s.flags |= 2;
        s.position = original;
        if c.nonwalkable_mode == 1 {
            let projection = yx(c.up, d);
            d = c.up.map(|u| u * projection);
        }
        split(s, c, d, backend)?;
        s.flags &= !2;
    }
    publication.position = s.position;
    publication.collision_flags = s.collision_flags;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    fn state(flags: u32) -> State {
        State {
            position: [0.; 3],
            flags,
            tag: 0,
            down_normal: [0., 1., 0.],
            side_normal: [1., 0., 0.],
            triangle_height: 0.,
            bound_height: 0.,
            contact_height: 0.,
            collision_flags: 0,
        }
    }
    fn settings(mode: u32) -> Settings {
        Settings {
            up: [0., 1., 0.],
            step_offset: 0.3,
            min_distance: 1e-6,
            nonwalkable_mode: mode,
            standing: false,
            prevent_ceiling_slide: false,
            slope_enabled: true,
            slope_limit: 0.7,
            constrain: true,
            extent: 0.9,
        }
    }
    struct Witness {
        correction: bool,
        calls: Vec<(u32, u32, u32, Position, f32)>,
        prepares: usize,
    }
    impl Backend for Witness {
        fn prepare(&mut self, _: &mut State, _: Vector, _: Vector) {
            self.prepares += 1;
        }
        fn iterate(&mut self, s: &mut State, r: Iteration) -> Outcome {
            let n = self.calls.len();
            self.calls
                .push((r.pass, s.tag, s.flags, s.position, r.minimum));
            if self.correction {
                if n == 1 {
                    s.flags |= 4;
                    s.down_normal = [0., 0.2, 0.];
                    s.triangle_height = 10.;
                    s.bound_height = 10.;
                    return Outcome {
                        attempted: true,
                        collisions: 1,
                    };
                }
                // Correction result is ignored for the caller collision byte.
                return Outcome {
                    attempted: false,
                    collisions: 9,
                };
            }
            if n < 3 {
                s.position[0] += 10.;
                if n == 2 {
                    s.flags |= 4;
                    s.down_normal = [0., 0.2, 0.];
                    s.triangle_height = 10.;
                }
                return Outcome {
                    attempted: true,
                    collisions: 1,
                };
            }
            s.flags &= !4;
            Outcome {
                attempted: false,
                collisions: 0,
            }
        }
    }
    #[test]
    fn retry_restores_only_position_and_replaces_result_byte() {
        let mut s = state(0);
        let c = settings(0);
        let mut backend = Witness {
            correction: false,
            calls: Vec::new(),
            prepares: 0,
        };
        let mut p = Publication {
            elapsed: 1.,
            pending: [0.1, 0., 0.],
            position: [99.; 3],
            collision_flags: 255,
        };
        common(&mut s, &c, [1., -1., 0.], &mut p, 0.016, &mut backend).unwrap();
        assert_eq!(backend.prepares, 2);
        assert_eq!(backend.calls[3].0, 1, "retry skips UP");
        assert_eq!(
            backend.calls[3].3, [0.; 3],
            "position restored before retry"
        );
        assert_ne!(backend.calls[3].2 & 2, 0);
        assert_eq!(s.down_normal, [0., 0.2, 0.]);
        assert_eq!(
            s.triangle_height, 10.,
            "published triangle state is retained"
        );
        assert_eq!(s.collision_flags, 0, "second result replaces first7");
        assert_eq!(p.collision_flags, 0);
        assert_eq!(p.pending, [0.; 3]);
    }
    #[test]
    fn corrective_pass_zero_keeps_down_tag_and_ignores_its_result() {
        let mut s = state(2);
        let mut c = settings(3);
        c.min_distance = 0.1;
        let mut backend = Witness {
            correction: true,
            calls: Vec::new(),
            prepares: 0,
        };
        split(&mut s, &c, [0., -1e-6, 0.], &mut backend).unwrap();
        let last = backend.calls.last().unwrap();
        assert_eq!((last.0, last.1), (0, 2));
        assert_ne!(last.2 & 0x40, 0);
        assert_eq!(last.4.to_bits(), (1e-6f32 * 0.1).to_bits());
        assert_eq!(s.flags & 0x40, 0);
        assert_eq!(s.collision_flags, 4);
    }
    #[test]
    fn finite_input_overflow_rejects_before_backend_receives_request() {
        let mut s = state(0);
        let mut c = settings(0);
        c.up = [f32::MAX; 3];
        let mut backend = Witness {
            correction: false,
            calls: Vec::new(),
            prepares: 0,
        };
        assert_eq!(
            split(&mut s, &c, [2., 2., 2.], &mut backend),
            Err(Error::NonFinite)
        );
        assert_eq!(backend.prepares, 0);
        assert!(backend.calls.is_empty());
    }
}
