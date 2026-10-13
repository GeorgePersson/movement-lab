//! Finite supplied-boundary engine movement request ordering.
//! Scene movement, optional adjustment, debugging and finalization are explicit
//! callbacks. This module does not classify actor grounding or schedule frames.
use crate::contact_finalizer::FinalizerState;

const EPS: f32 = 1.0e-6;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EngineMoveState {
    pub contact: FinalizerState,
    pub flags: u64,
    pub configuration_disabled: bool,
    pub movement_disabled: bool,
    pub force_move: bool,
    pub skip_request_shaping: bool,
    pub optional_adjustment_enabled: bool,
    pub extra_down_guard: bool,
    pub extra_down_value: f32,
    pub contact_offset: f32,
    pub split_vertical: u8,
    pub debug_enabled: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EngineMoveGlobals {
    pub normal_lower: f32,
    pub normal_upper: f32,
    pub projection_mask: [f32; 4],
    pub initialize_mask: bool,
    pub first_axis: [f32; 4],
    pub second_axis: [f32; 4],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EngineEvent {
    Admission(bool),
    PriorStatePredicate(bool),
    NormalPredicate(bool),
    InitializeMask([f32; 4]),
    ResetReports,
}

/// Mutation across these boundaries is intentional. In particular, late rise
/// checks reread request and scalar after movement and debugging. Original dt is
/// passed unchanged. The immutable observer is diagnostic only.
pub trait EngineCallbacks {
    fn event(&mut self, _event: EngineEvent) {}
    fn optional_adjustment(
        &mut self,
        state: &mut EngineMoveState,
        globals: &mut EngineMoveGlobals,
        request: &mut [f32; 4],
        dt: f32,
    );
    fn move_request(
        &mut self,
        state: &mut EngineMoveState,
        globals: &mut EngineMoveGlobals,
        request: &mut [f32; 4],
        dt: f32,
    ) -> [f32; 4];
    fn debug_observer(
        &mut self,
        state: &mut EngineMoveState,
        globals: &mut EngineMoveGlobals,
        request: &mut [f32; 4],
        actual: [f32; 4],
        dt: f32,
    );
    fn finalize(
        &mut self,
        state: &mut EngineMoveState,
        globals: &mut EngineMoveGlobals,
        request: &mut [f32; 4],
        dt: f32,
    );
}

fn nonzero(v: [f32; 4]) -> bool {
    v[..3].iter().any(|x| x.abs() > EPS)
}
fn significant(x: f32) -> bool {
    x.abs() > x.abs().max(0.0).max(1.0) * EPS
}
fn neg(x: f32) -> f32 {
    f32::from_bits(x.to_bits() ^ 0x8000_0000)
}

/// Finite moderate domain. Supplied bounds must keep any admitted projection
/// denominator nonzero; NaN, overflow and live global initialization are outside
/// the validated domain. No contact normal projection of velocity is introduced.
pub fn run(
    state: &mut EngineMoveState,
    globals: &mut EngineMoveGlobals,
    request: &mut [f32; 4],
    dt: f32,
    callbacks: &mut impl EngineCallbacks,
) {
    if state.force_move || nonzero(*request) {
        let admitted = !(state.flags & 0x20 != 0
            || state.configuration_disabled
            || state.movement_disabled
            || state.flags & 6 == 2);
        callbacks.event(EngineEvent::Admission(admitted));
        if admitted {
            if dt >= request[1] && !state.skip_request_shaping {
                let prior = state.contact.suppress || state.contact.mask & 4 != 0;
                callbacks.event(EngineEvent::PriorStatePredicate(prior));
                if prior {
                    let normal = state.contact.working_normal;
                    let allowed =
                        globals.normal_lower <= normal[1] && normal[1] <= globals.normal_upper;
                    callbacks.event(EngineEvent::NormalPredicate(allowed));
                    if allowed {
                        if globals.initialize_mask {
                            globals.projection_mask = std::array::from_fn(|i| {
                                globals.first_axis[i] + globals.second_axis[i]
                            });
                            globals.initialize_mask = false;
                            callbacks.event(EngineEvent::InitializeMask(globals.projection_mask));
                        }
                        let projected: [f32; 4] =
                            std::array::from_fn(|i| globals.projection_mask[i] * request[i]);
                        if nonzero(projected) {
                            let products: [f32; 4] =
                                std::array::from_fn(|i| projected[i] * normal[i]);
                            let slope_y =
                                neg(((products[0] + products[1]) + products[2]) / normal[1]);
                            if slope_y <= 0.0 {
                                request[1] = slope_y - (state.contact_offset + 0.005);
                            }
                        }
                    }
                    if !state.contact.suppress
                        && (!state.extra_down_guard
                            || neg(state.contact_offset) < state.extra_down_value)
                    {
                        // Native packed addition also operates on X/Z/W, retaining
                        // its signed-zero behavior rather than only updating Y.
                        *request = std::array::from_fn(|i| {
                            request[i] + if i == 1 { neg(dt) } else { 0.0 }
                        });
                    }
                }
            }
            if state.optional_adjustment_enabled {
                callbacks.optional_adjustment(state, globals, request, dt);
            }
        }
        if nonzero(*request) {
            callbacks.event(EngineEvent::ResetReports);
            state.contact.reports.reset();
            state.contact.timer = 0.0;
        }
        let actual = callbacks.move_request(state, globals, request, dt);
        if state.debug_enabled {
            callbacks.debug_observer(state, globals, request, actual, dt);
        }
        let y = actual[1];
        let scalar = state.contact.scalar;
        if y > 0.0 && significant(y) && !(scalar < 0.0 && significant(scalar)) {
            let threshold = request[1].max(0.0) + 0.025;
            if y > threshold && (threshold - y).abs() > threshold.abs().max(y.abs()).max(1.0) * EPS
            {
                state.contact.late_flag = true;
                state.contact.age = 0.0;
            }
        }
    }
    callbacks.finalize(state, globals, request, dt);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contact_reports::ContactReports;
    struct Boundaries {
        actual: [f32; 4],
        debug_scalar: Option<f32>,
        finals: usize,
        moves: usize,
    }
    impl EngineCallbacks for Boundaries {
        fn optional_adjustment(
            &mut self,
            _: &mut EngineMoveState,
            _: &mut EngineMoveGlobals,
            _: &mut [f32; 4],
            _: f32,
        ) {
        }
        fn move_request(
            &mut self,
            _: &mut EngineMoveState,
            _: &mut EngineMoveGlobals,
            _: &mut [f32; 4],
            _: f32,
        ) -> [f32; 4] {
            self.moves += 1;
            self.actual
        }
        fn debug_observer(
            &mut self,
            s: &mut EngineMoveState,
            _: &mut EngineMoveGlobals,
            r: &mut [f32; 4],
            _: [f32; 4],
            _: f32,
        ) {
            if let Some(v) = self.debug_scalar {
                s.contact.scalar = v;
            }
            r[1] = 0.0;
        }
        fn finalize(
            &mut self,
            _: &mut EngineMoveState,
            _: &mut EngineMoveGlobals,
            _: &mut [f32; 4],
            _: f32,
        ) {
            self.finals += 1;
        }
    }
    fn setup() -> (EngineMoveState, EngineMoveGlobals, Boundaries) {
        (
            EngineMoveState {
                contact: FinalizerState {
                    reports: ContactReports::default(),
                    working_normal: [0., 1., 0., 0.],
                    other_normal: [0.; 4],
                    normal_y: 0.,
                    cached_y: 0.,
                    defer: false,
                    mask: 0,
                    suppress: false,
                    scalar: 0.,
                    age: 2.,
                    timer: 3.,
                    position_y: 0.,
                    step: 0.,
                    late_flag: false,
                    native_controller_present: false,
                },
                flags: 0,
                configuration_disabled: false,
                movement_disabled: false,
                force_move: false,
                skip_request_shaping: false,
                optional_adjustment_enabled: false,
                extra_down_guard: false,
                extra_down_value: 0.,
                contact_offset: 0.02,
                split_vertical: 0,
                debug_enabled: false,
            },
            EngineMoveGlobals {
                normal_lower: 0.25,
                normal_upper: 0.999,
                projection_mask: [1., 0., 1., 0.],
                initialize_mask: false,
                first_axis: [1., 0., 0., 0.],
                second_axis: [0., 0., 1., 0.],
            },
            Boundaries {
                actual: [0.; 4],
                debug_scalar: None,
                finals: 0,
                moves: 0,
            },
        )
    }
    #[test]
    fn failed_adjustment_admission_still_resets_and_moves() {
        let (mut s, mut g, mut b) = setup();
        s.flags = 0x20;
        s.contact.reports.primary_valid = 1;
        s.contact.reports.primary.point = [1., 2., 3., 4.];
        run(&mut s, &mut g, &mut [1., 0., 0., 0.], 0.016, &mut b);
        assert_eq!(b.moves, 1);
        assert_eq!(b.finals, 1);
        assert_eq!(s.contact.timer, 0.);
        assert_eq!(s.contact.reports.primary_valid, 0);
        assert_eq!(s.contact.reports.primary.point, [1., 2., 3., 4.]);
    }
    #[test]
    fn late_check_uses_debug_mutated_scalar_and_request() {
        for scalar in [-1., 0.] {
            let (mut s, mut g, mut b) = setup();
            s.debug_enabled = true;
            b.actual = [0., 0.2, 0., 0.];
            b.debug_scalar = Some(scalar);
            run(&mut s, &mut g, &mut [1., 0.3, 0., 0.], 0.016, &mut b);
            assert_eq!(s.contact.late_flag, scalar == 0.);
            assert_eq!(s.contact.age, if scalar == 0. { 0. } else { 2. });
        }
    }
}
