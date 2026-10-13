//! Engine request/finalizer composition for supplied static boxes.
//!
//! The caller supplies configuration, staged state, cosine and globals. Native
//! filters, frame publication, role selection and moving actors remain outside
//! this adapter. Geometry/report arithmetic uses the recovered modules.
use crate::{
    cct_policy::{self, BoxHit, Error, Pass, Position, Settings},
    cct_scene::{Capsule, Collider, StaticBoxes},
    contact_finalizer::{self, FinalizerGlobals, FinalizerState, SecondaryMove},
    engine_move::{EngineCallbacks, EngineMoveGlobals, EngineMoveState},
    request_adjustment::{self, AdjustmentInput},
};

/// Cached native configuration and committed scalar are supplied independently
/// of the prepared movement scalar. No live configuration binding is inferred.
#[derive(Clone, Copy, Debug)]
pub struct AdjustmentContext {
    pub value_3c8: f32,
    pub cached_lower: f32,
    pub upper: f32,
    pub projection_mask: [f32; 4],
    pub default_normal: [f32; 4],
}

pub struct EngineScene<'a> {
    pub center: Position,
    pub capsule: Capsule,
    pub boxes: &'a [Collider],
    pub settings: Settings,
    pub behavior: u8,
    pub metadata: bool,
    pub finalizer_globals: FinalizerGlobals,
    pub slope_cosine: f32,
    pub hits: Vec<(Pass, BoxHit)>,
    pub error: Option<Error>,
    pub native_moves: u32,
    /// Current engine split request flag; sampled again on finalizer entry.
    pub split_vertical: bool,
    pub bridge_projection: [f32; 4],
    pub adjustment: AdjustmentContext,
}

impl EngineScene<'_> {
    fn native_move(&mut self, state: &mut FinalizerState, request: [f32; 3]) -> u8 {
        if self.error.is_some() {
            return 0;
        }
        let mut scene = StaticBoxes {
            capsule: self.capsule,
            boxes: self.boxes,
            contact_offset: self.settings.contact_offset,
            overlap_recovery: self.settings.overlap_recovery,
            behavior: self.behavior,
            recovery_scan_limit: 16,
            reports: Vec::new(),
            contact_reports: state.reports,
            report_metadata: self.metadata,
        };
        self.native_moves += 1;
        match cct_policy::move_with_scene(self.center, request, &self.settings, &mut scene) {
            Ok(motion) => {
                self.center = motion.position;
                state.reports = scene.contact_reports;
                for (pass, result) in [Pass::Up, Pass::Side, Pass::Down]
                    .into_iter()
                    .zip(motion.passes)
                {
                    self.hits
                        .extend(result.hits.into_iter().map(|hit| (pass, hit)));
                }
                motion.collision_flags
            }
            Err(error) => {
                self.error = Some(error);
                0
            }
        }
    }

    /// Original bridge skips CCT dispatch for insignificant XYZ, retaining mask.
    /// Otherwise it replaces the mask, optionally OR-ing a later horizontal call.
    /// Actual displacement narrows the difference of double positions to float.
    fn bridge(&mut self, state: &mut FinalizerState, request: [f32; 4]) -> [f32; 4] {
        let before = self.center;
        if request[..3].iter().any(|x| x.abs() > 1.0e-6) {
            if self.split_vertical {
                state.mask = u32::from(self.native_move(state, [0.0, request[1], 0.0]));
                let horizontal =
                    std::array::from_fn::<_, 3, _>(|i| request[i] * self.bridge_projection[i]);
                if horizontal.into_iter().any(|x| x.abs() > 1.0e-6) {
                    state.mask |= u32::from(self.native_move(state, horizontal));
                }
            } else {
                state.mask =
                    u32::from(self.native_move(state, [request[0], request[1], request[2]]));
            }
        }
        [
            (self.center[0] - before[0]) as f32,
            (self.center[1] - before[1]) as f32,
            (self.center[2] - before[2]) as f32,
            0.0,
        ]
    }
}

impl EngineCallbacks for EngineScene<'_> {
    fn optional_adjustment(
        &mut self,
        state: &mut EngineMoveState,
        _: &mut EngineMoveGlobals,
        request: &mut [f32; 4],
        dt: f32,
    ) {
        match request_adjustment::adjust(AdjustmentInput {
            request: *request,
            dt,
            value_3c8: self.adjustment.value_3c8,
            normal: request_adjustment::select_normal(
                &state.contact.reports,
                self.adjustment.default_normal,
            ),
            cached_lower: self.adjustment.cached_lower,
            upper: self.adjustment.upper,
            projection_mask: self.adjustment.projection_mask,
            split_vertical: state.split_vertical,
        }) {
            Ok(adjusted) => {
                *request = adjusted.request;
                state.split_vertical = adjusted.split_vertical;
            }
            Err(error) => {
                self.error = Some(match error {
                    request_adjustment::AdjustmentError::NonFinite => Error::NonFiniteArithmetic,
                    request_adjustment::AdjustmentError::UnsupportedTarget => {
                        Error::UnsupportedPolicy
                    }
                });
            }
        }
    }
    fn move_request(
        &mut self,
        state: &mut EngineMoveState,
        _: &mut EngineMoveGlobals,
        request: &mut [f32; 4],
        _: f32,
    ) -> [f32; 4] {
        self.split_vertical = state.split_vertical != 0;
        self.bridge(&mut state.contact, *request)
    }
    fn debug_observer(
        &mut self,
        _: &mut EngineMoveState,
        _: &mut EngineMoveGlobals,
        _: &mut [f32; 4],
        _: [f32; 4],
        _: f32,
    ) {
        // LAB has no engine debug observer or associated state mutation.
    }
    fn finalize(
        &mut self,
        state: &mut EngineMoveState,
        _: &mut EngineMoveGlobals,
        _: &mut [f32; 4],
        dt: f32,
    ) {
        self.split_vertical = state.split_vertical != 0;
        let mut globals = self.finalizer_globals;
        contact_finalizer::finalize(
            &mut state.contact,
            &mut globals,
            self.slope_cosine,
            dt,
            self,
        );
        self.finalizer_globals = globals;
    }
}
impl SecondaryMove for EngineScene<'_> {
    fn set_step(&mut self, value: f32, _: &mut FinalizerState, _: &mut FinalizerGlobals) {
        self.settings.step_offset = value;
    }
    fn move_request(
        &mut self,
        request: [f32; 4],
        _: f32,
        state: &mut FinalizerState,
        _: &mut FinalizerGlobals,
    ) -> [f32; 4] {
        self.bridge(state, request)
    }
}
