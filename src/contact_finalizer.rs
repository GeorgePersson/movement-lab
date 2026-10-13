//! Supplied finite-state contact finalization, separate from scene and actor air.
//! Cosine, globals and secondary scene movement are explicit inputs. No host
//! cosine, ground classification, scene selection or publication cadence is added.
use crate::contact_reports::ContactReports;

const EPS: f32 = 1.0e-6;
const AGE_LIMIT: f32 = 0.25;
const PROJECT_MIN: f32 = 1.0e-12;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FinalizerGlobals {
    pub zero: [f32; 4],
    pub up: [f32; 4],
    pub fallback: [f32; 4],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FinalizerState {
    pub reports: ContactReports,
    pub working_normal: [f32; 4],
    pub other_normal: [f32; 4],
    pub normal_y: f32,
    pub cached_y: f32,
    pub defer: bool,
    pub mask: u32,
    pub suppress: bool,
    pub scalar: f32,
    pub age: f32,
    pub timer: f32,
    pub position_y: f32,
    pub step: f32,
    pub late_flag: bool,
    pub native_controller_present: bool,
}

/// Both boundaries may mutate supplied aliased state/globals. This interface
/// makes the native rereads representable; it does not schedule engine callbacks.
pub trait SecondaryMove {
    fn set_step(&mut self, value: f32, state: &mut FinalizerState, globals: &mut FinalizerGlobals);
    fn move_request(
        &mut self,
        request: [f32; 4],
        dt: f32,
        state: &mut FinalizerState,
        globals: &mut FinalizerGlobals,
    ) -> [f32; 4];
}

fn mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    std::array::from_fn(|i| a[i] * b[i])
}
fn sub(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    std::array::from_fn(|i| a[i] - b[i])
}
fn shuffle(v: [f32; 4], indices: [usize; 4]) -> [f32; 4] {
    indices.map(|i| v[i])
}
fn dot3(v: [f32; 4]) -> f32 {
    (v[0] + v[1]) + v[2]
}
fn length(v: [f32; 4]) -> f32 {
    dot3(mul(v, v)).sqrt()
}

/// Finite moderate input domain; supplied cosine is the already-evaluated native
/// family result. Its angle calculation and implementation are external.
pub fn finalize(
    state: &mut FinalizerState,
    globals: &mut FinalizerGlobals,
    supplied_cosine: f32,
    dt: f32,
    secondary: &mut impl SecondaryMove,
) {
    let age_below_limit = AGE_LIMIT > state.age;
    let original_primary_address = state.reports.has_primary();
    let selected = if original_primary_address {
        state.reports.primary.normal
    } else {
        globals.zero
    };
    state.working_normal = selected;
    state.other_normal = if original_primary_address {
        if state.reports.has_secondary() {
            state.reports.secondary.normal
        } else {
            state.reports.primary.normal
        }
    } else {
        globals.zero
    };
    state.normal_y = selected[1];
    if state.timer > 1.0 {
        state.cached_y = state.position_y;
    }
    state.defer = (state.reports.latest_was_controller != 0
        && dot3(mul(state.reports.primary.normal, globals.up)) > 0.0)
        || state.reports.latest_metadata != 0
        || (!state.suppress && supplied_cosine > state.normal_y);

    let scalar = state.scalar;
    let magnitude = scalar.abs();
    let significant = magnitude > magnitude.max(0.0).max(1.0) * EPS;
    let admitted = (state.suppress || state.mask & 4 != 0)
        && state.defer
        && scalar < 0.0
        && significant
        && !age_below_limit;
    let mut failed = false;
    if admitted {
        if !state.late_flag {
            // Preserve the two packed cross products and per-lane grouping.
            let n = selected;
            let up = globals.up;
            let mut v = sub(
                mul(shuffle(up, [2, 0, 1, 0]), shuffle(n, [1, 2, 0, 0])),
                mul(shuffle(up, [1, 2, 0, 0]), shuffle(n, [2, 0, 1, 0])),
            );
            v = sub(
                mul(shuffle(v, [2, 0, 1, 0]), shuffle(n, [1, 2, 0, 0])),
                mul(shuffle(v, [1, 2, 0, 0]), shuffle(n, [2, 0, 1, 0])),
            );
            let mut magnitude = length(v);
            if magnitude <= EPS {
                let component = v[0].abs().max(v[1].abs()).max(v[2].abs());
                if component > EPS {
                    v = mul(v, [1.0 / component; 4]);
                    magnitude = length(v);
                }
            }
            let scaled = dt * scalar;
            let request = if magnitude > EPS {
                v = mul(v, [1.0 / magnitude; 4]);
                let projection = dot3(mul(v, [0.0, scaled, 0.0, 0.0]));
                mul(v, [projection; 4])
            } else {
                mul(globals.fallback, [scaled; 4])
            };
            if state.native_controller_present {
                secondary.set_step(0.0, state, globals);
            }
            let actual = secondary.move_request(request, dt, state, globals);
            // Controller identity and configured step are reread after movement.
            if state.native_controller_present {
                secondary.set_step(state.step, state, globals);
            }
            state.working_normal = if state.reports.has_primary() {
                state.reports.primary.normal
            } else {
                globals.zero
            };
            // Native RBP retained the original address, rather than reselecting it.
            state.normal_y = if original_primary_address {
                state.reports.primary.normal[1]
            } else {
                globals.zero[1]
            };
            failed = dot3(mul(request, request)) >= PROJECT_MIN
                && actual[..3].iter().all(|v| v.abs() <= EPS);
        }
    } else {
        state.late_flag = false;
    }
    if state.late_flag || failed {
        state.defer = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contact_reports::Contact;
    struct Boundary {
        moves: usize,
        steps: Vec<f32>,
        after_valid: bool,
    }
    impl SecondaryMove for Boundary {
        fn set_step(&mut self, value: f32, _: &mut FinalizerState, _: &mut FinalizerGlobals) {
            self.steps.push(value);
        }
        fn move_request(
            &mut self,
            _: [f32; 4],
            _: f32,
            s: &mut FinalizerState,
            _: &mut FinalizerGlobals,
        ) -> [f32; 4] {
            self.moves += 1;
            s.reports.primary.normal = [0., 0.4, 0., 0.];
            s.reports.primary_valid = u8::from(self.after_valid);
            [0.1, 0., 0., 0.]
        }
    }
    fn setup(valid: bool) -> (FinalizerState, FinalizerGlobals, Boundary) {
        let mut reports = ContactReports::default();
        reports.add(
            Contact {
                point: [0.; 4],
                normal: [0.6, 0.8, 0., 0.],
            },
            false,
            true,
        );
        reports.primary_valid = u8::from(valid);
        (
            FinalizerState {
                reports,
                working_normal: [0.; 4],
                other_normal: [0.; 4],
                normal_y: 0.,
                cached_y: 9.,
                defer: false,
                mask: 4,
                suppress: false,
                scalar: -2.,
                age: 1.,
                timer: 0.,
                position_y: 2.,
                step: 0.45,
                late_flag: false,
                native_controller_present: true,
            },
            FinalizerGlobals {
                zero: [0.; 4],
                up: [0., 1., 0., 0.],
                fallback: [0., 1., 0., 0.],
            },
            Boundary {
                moves: 0,
                steps: Vec::new(),
                after_valid: !valid,
            },
        )
    }
    #[test]
    fn normal_y_keeps_original_address_when_validity_flips() {
        for valid in [false, true] {
            let (mut s, mut g, mut b) = setup(valid);
            finalize(&mut s, &mut g, 0.7, 1. / 60., &mut b);
            assert_eq!(b.moves, 1);
            assert_eq!(b.steps, vec![0., 0.45]);
            assert_eq!(s.working_normal[1], if valid { 0. } else { 0.4 });
            assert_eq!(s.normal_y, if valid { 0.4 } else { 0. });
        }
    }
    #[test]
    fn ineligible_branch_clears_late_flag_without_secondary_move() {
        let (mut s, mut g, mut b) = setup(true);
        s.age = 0.249;
        s.late_flag = true;
        finalize(&mut s, &mut g, 0.7, 1. / 60., &mut b);
        assert_eq!(b.moves, 0);
        assert!(b.steps.is_empty());
        assert!(!s.late_flag);
        assert!(s.defer);
    }
}
