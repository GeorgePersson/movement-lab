use crate::{
    cct_policy, cct_scene,
    engine_mesh_scene::MeshRuntime,
    static_terrain::{StaticTerrain, TerrainError},
};
use glam::Vec3;
use serde::{Deserialize, Serialize};

const CONTACT_EPS: f32 = 0.00001;
const WALL_INSIDE_EPS: f32 = 0.001;

/// Supplied LAB settings. Numeric defaults use the recovered Tenno own-record
/// candidates; active inheritance/scale and mutable manager state are unmeasured.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct CollisionTuning {
    pub contact_offset: f32,
    pub step_offset: f32,
    pub overlap_recovery: bool,
    pub custom_response: bool,
    pub prevent_ceiling_slide: bool,
    pub normalize_components: bool,
    pub behavior_flags: u8,
    pub engine: EngineContactPolicy,
}
impl Default for CollisionTuning {
    fn default() -> Self {
        Self {
            contact_offset: 0.02,
            step_offset: 0.45,
            overlap_recovery: true,
            custom_response: true,
            prevent_ceiling_slide: false,
            normalize_components: false,
            behavior_flags: 4,
            engine: EngineContactPolicy::default(),
        }
    }
}

/// Explicit LAB bindings for the recovered ordinary local Human-role branch.
/// Dimensions, active configuration and frame cadence are not inferred here.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct EngineContactPolicy {
    pub callback_valid: bool,
    pub actor_air_mask: u32,
    pub flyer: bool,
    pub suppress: bool,
    pub optional_adjustment: bool,
    pub slope_cosine: f32,
    pub adjustment_lower: f32,
    pub adjustment_upper: f32,
}
impl Default for EngineContactPolicy {
    fn default() -> Self {
        Self {
            callback_valid: true,
            actor_air_mask: 0,
            flyer: false,
            suppress: false,
            // Constructor candidates, supplied independently of live selection.
            optional_adjustment: true,
            slope_cosine: f32::from_bits(1060439283), // supplied 45 degrees
            adjustment_lower: -0.8660253882408142,
            adjustment_upper: -0.008726561442017555,
        }
    }
}
impl EngineContactPolicy {
    pub fn is_valid(&self) -> bool {
        [
            self.slope_cosine,
            self.adjustment_lower,
            self.adjustment_upper,
        ]
        .into_iter()
        .all(|x| x.is_finite() && (-1.0..=1.0).contains(&x))
            && self.adjustment_lower <= self.adjustment_upper
    }
}

/// Persistent native-center and published-contact transport. The byte record
/// preserves stale vectors, validity bytes and the untouched report gap exactly.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(default)]
pub struct EngineContactRuntime {
    pub initialized: bool,
    pub center: [f64; 3],
    pub published_feet: Vec3,
    pub height: f32,
    pub reports: Vec<u8>,
    pub working_normal: [f32; 4],
    pub other_normal: [f32; 4],
    pub normal_y: f32,
    pub cached_y: f32,
    pub defer: bool,
    pub mask: u32,
    pub suppress: bool,
    pub age: f32,
    pub timer: f32,
    pub late_flag: bool,
    /// Complete per-player prepared mesh state; no frame-by-frame reinitialization.
    pub mesh: Option<MeshRuntime>,
    pub last_request_base: Vec3,
    pub last_request_scalar: f32,
    pub last_request_dt: f32,
}
impl Default for EngineContactRuntime {
    fn default() -> Self {
        Self {
            initialized: false,
            center: [0.; 3],
            published_feet: Vec3::ZERO,
            height: 0.,
            reports: vec![0; 85],
            working_normal: [0.; 4],
            other_normal: [0.; 4],
            normal_y: 0.,
            cached_y: 0.,
            defer: false,
            mask: 0,
            suppress: false,
            age: 0.,
            timer: 0.,
            late_flag: false,
            mesh: None,
            last_request_base: Vec3::ZERO,
            last_request_scalar: 0.,
            last_request_dt: 0.,
        }
    }
}
impl EngineContactRuntime {
    /// World override absent, ordinary actor state, attached=false, Human role8.
    /// Invalid callback follows the recovered actor-mask fallback instead.
    pub fn air(&self, cached_air: bool, policy: &EngineContactPolicy) -> bool {
        if !policy.callback_valid {
            return policy.actor_air_mask & 0x8000 != 0;
        }
        let physical_air = !self.suppress && !policy.flyer && self.mask & 4 == 0;
        physical_air || (cached_air && self.defer)
    }
}

/// Static axis-aligned obstacle in world coordinates.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Solid {
    pub min: Vec3,
    pub max: Vec3,
}

impl Solid {
    pub fn new(min: Vec3, max: Vec3) -> Self {
        assert!(min.is_finite() && max.is_finite() && min.cmple(max).all());
        Self { min, max }
    }
    pub fn center(&self) -> Vec3 {
        (self.min + self.max) * 0.5
    }
    pub fn size(&self) -> Vec3 {
        self.max - self.min
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum WorldTerrain {
    #[default]
    Boxes,
    Mesh {
        asset: StaticTerrain,
    },
}
impl WorldTerrain {
    fn is_boxes(&self) -> bool {
        matches!(self, Self::Boxes)
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct World {
    pub solids: Vec<Solid>,
    #[serde(default, skip_serializing_if = "WorldTerrain::is_boxes")]
    pub terrain: WorldTerrain,
}

#[derive(Clone, Debug)]
pub(crate) struct MotionResult {
    pub position: Vec3,
    pub velocity: Vec3,
    pub grounded: bool,
    pub wall_normal: Option<Vec3>,
    pub contact_normals: Vec<Vec3>,
    pub failed: bool,
}

impl World {
    pub fn boxes(solids: Vec<Solid>) -> Self {
        Self {
            solids,
            terrain: WorldTerrain::Boxes,
        }
    }
    pub fn mesh(asset: StaticTerrain) -> Result<Self, TerrainError> {
        asset.validate()?;
        Ok(Self {
            solids: Vec::new(),
            terrain: WorldTerrain::Mesh { asset },
        })
    }
    pub fn validate(&self) -> Result<(), TerrainError> {
        match &self.terrain {
            WorldTerrain::Boxes => {
                if self.solids.iter().any(|s| !valid_solid(s)) {
                    return Err(TerrainError::InvalidAsset);
                }
                Ok(())
            }
            WorldTerrain::Mesh { asset } => {
                if !self.solids.is_empty() {
                    return Err(TerrainError::MixedSolids);
                }
                asset.validate()
            }
        }
    }
    pub fn mesh_terrain(&self) -> Option<&StaticTerrain> {
        match &self.terrain {
            WorldTerrain::Boxes => None,
            WorldTerrain::Mesh { asset } => Some(asset),
        }
    }
    pub fn spawn(&self) -> Vec3 {
        self.mesh_terrain()
            .map_or(Vec3::new(0., 0.05, 8.), StaticTerrain::safe_spawn)
    }
    /// Shared camera service. Boxes retain the old expanded-box approximation;
    /// triangles use the selected asset's explicitly supplied clearance policy.
    pub fn camera_fraction(&self, target: Vec3, desired: Vec3, clearance: f32) -> f32 {
        if !target.is_finite()
            || !desired.is_finite()
            || !clearance.is_finite()
            || clearance < 0.
            || self.validate().is_err()
        {
            return 0.;
        }
        if let Some(asset) = self.mesh_terrain() {
            return asset
                .camera_fraction(target, desired, clearance)
                .unwrap_or(0.);
        }
        let delta = desired - target;
        let mut fraction: f32 = 1.;
        for solid in &self.solids {
            let min = solid.min - Vec3::splat(clearance);
            let max = solid.max + Vec3::splat(clearance);
            let mut enter: f32 = 0.;
            let mut leave: f32 = 1.;
            let mut hit = true;
            for axis in 0..3 {
                if delta[axis].abs() < 1e-6 {
                    if target[axis] < min[axis] || target[axis] > max[axis] {
                        hit = false;
                        break;
                    }
                } else {
                    let a = (min[axis] - target[axis]) / delta[axis];
                    let b = (max[axis] - target[axis]) / delta[axis];
                    enter = enter.max(a.min(b));
                    leave = leave.min(a.max(b));
                    if enter > leave {
                        hit = false;
                        break;
                    }
                }
            }
            if hit && enter > 0. && enter <= 1. {
                fraction = fraction.min((enter - 0.025).max(0.08));
            }
        }
        fraction
    }
    /// Admission for a serialized continuation; never clears or rebinds state.
    pub fn validate_player_runtime(
        &self,
        runtime: &EngineContactRuntime,
        feet: Vec3,
        radius: f32,
        height: f32,
    ) -> Result<(), String> {
        self.validate()
            .map_err(|e| format!("invalid selected world: {e:?}"))?;
        if !self.can_occupy(feet, radius, height) {
            return Err("resumed capsule does not fit selected geometry".into());
        }
        if runtime.initialized
            && (runtime.published_feet.to_array().map(f32::to_bits)
                != feet.to_array().map(f32::to_bits)
                || runtime.height.to_bits() != height.to_bits())
        {
            return Err("resume feet/height do not match published runtime".into());
        }
        if runtime.initialized {
            let derived = Vec3::new(
                runtime.center[0] as f32,
                runtime.center[1] as f32 - height * 0.5,
                runtime.center[2] as f32,
            );
            if derived.to_array().map(f32::to_bits) != feet.to_array().map(f32::to_bits) {
                return Err("resume center does not publish the supplied feet".into());
            }
        }
        match (self.mesh_terrain(), &runtime.mesh) {
            (Some(asset), Some(mesh)) if runtime.initialized => {
                mesh.validate(
                    asset,
                    cct_scene::Capsule {
                        radius,
                        height: height - 2. * radius,
                        rotation: [
                            0.,
                            0.,
                            std::f32::consts::FRAC_1_SQRT_2,
                            std::f32::consts::FRAC_1_SQRT_2,
                        ],
                    },
                    runtime.center,
                )
                .map_err(|e| format!("invalid resumed mesh runtime: {e:?}"))?;
            }
            (Some(_), None) if !runtime.initialized => {}
            (None, None) => {}
            _ => return Err("resume terrain/runtime identity mismatch".into()),
        }
        // serde rejects nonfinite values narrowed to null in any required
        // runtime scalar/vector lane; all retained roles are preserved.
        let encoded = serde_json::to_value(runtime).map_err(|e| e.to_string())?;
        let _: EngineContactRuntime = serde_json::from_value(encoded)
            .map_err(|e| format!("invalid resumed transport: {e}"))?;
        let record: &[u8; 85] = runtime
            .reports
            .as_slice()
            .try_into()
            .map_err(|_| "resume report record requires85 bytes")?;
        let reports = crate::contact_reports::ContactReports::from_record(record);
        if reports
            .primary
            .point
            .into_iter()
            .chain(reports.primary.normal)
            .chain(reports.secondary.point)
            .chain(reports.secondary.normal)
            .any(|x| !x.is_finite())
        {
            return Err("resumed report bytes contain nonfinite contact vectors".into());
        }
        if [
            runtime.age,
            runtime.timer,
            runtime.normal_y,
            runtime.cached_y,
        ]
        .into_iter()
        .chain(runtime.working_normal)
        .chain(runtime.other_normal)
        .any(|x| !x.is_finite() || (x != 0. && !x.is_normal()))
        {
            return Err("resume engine inputs require normal or zero numeric lanes".into());
        }
        Ok(())
    }

    /// Lab ray against static boxes. The native target uses a hit backoff of
    /// approximately 0.1 and an initially 5000-unit query extent. Camera origin,
    /// query filters and box geometry here remain original approximations.
    /// An origin inside a solid hits its exit face; outside the query cube is
    /// rejected. Those edge policies are lab choices, not recovered behavior.
    pub fn aim_target(&self, origin: Vec3, direction: Vec3, extent: f32) -> Option<Vec3> {
        if self.validate().is_err() {
            return None;
        }
        if let Some(asset) = self.mesh_terrain() {
            return asset.aim_target(origin, direction, extent);
        }
        if !origin.is_finite()
            || !direction.is_finite()
            || !extent.is_finite()
            || extent <= 0.
            || origin.abs().max_element() > extent
        {
            return None;
        }
        let direction = direction.try_normalize()?;
        let mut distance = extent;
        // Clip the ray segment to the world cube, preserving its direction.
        for axis in 0..3 {
            if direction[axis] != 0. {
                let boundary = extent.copysign(direction[axis]);
                distance = distance.min((boundary - origin[axis]) / direction[axis]);
            }
        }
        let mut nearest = None::<f32>;
        for solid in &self.solids {
            let mut entry = f32::NEG_INFINITY;
            let mut exit = f32::INFINITY;
            for axis in 0..3 {
                if direction[axis] == 0. {
                    if origin[axis] < solid.min[axis] || origin[axis] > solid.max[axis] {
                        exit = f32::NEG_INFINITY;
                        break;
                    }
                } else {
                    let a = (solid.min[axis] - origin[axis]) / direction[axis];
                    let b = (solid.max[axis] - origin[axis]) / direction[axis];
                    entry = entry.max(a.min(b));
                    exit = exit.min(a.max(b));
                }
            }
            let hit = if entry >= 0. { entry } else { exit };
            if entry <= exit && hit >= 0. && hit <= distance && nearest.is_none_or(|old| hit < old)
            {
                nearest = Some(hit);
            }
        }
        Some(origin + direction * nearest.map_or(distance, |hit| hit - 0.1))
    }

    /// Renderer and simulation share this geometry. Spawn feet at (0,0.05,8).
    pub fn graybox() -> Self {
        let mut world = Self::default();
        let mut box_at = |min: [f32; 3], max: [f32; 3]| {
            world
                .solids
                .push(Solid::new(Vec3::from(min), Vec3::from(max)));
        };
        box_at([-30., -2., -30.], [30., 0., 30.]);
        box_at([-5., 0., -14.], [-4., 8., 1.]);
        box_at([4., 0., -14.], [5., 8., 1.]);
        box_at([-4., 0., -15.], [4., 8., -14.8]);
        box_at([-2., 0., -6.], [2., 1.5, -3.]);
        box_at([8., 0., -8.], [12., 3., -4.]);
        box_at([13., 0., -14.], [18., 5., -10.]);
        box_at([-14., 0., -8.], [-9., 2.5, -3.]);
        // Crouch tunnel, with side rails and a low ceiling.
        box_at([-2., 1.15, 13.], [2., 2.0, 20.]);
        box_at([-2.5, 0., 13.], [-2., 2., 20.]);
        box_at([2., 0., 13.], [2.5, 2., 20.]);
        world
    }

    /// Upright capsule with feet origin. Dimensions and resting-contact epsilon
    /// are LAB policy; the segment-to-box distance here is analytic geometry.
    pub fn can_occupy(&self, feet: Vec3, radius: f32, height: f32) -> bool {
        if self.validate().is_err() {
            return false;
        }
        if let Some(asset) = self.mesh_terrain() {
            return asset.can_occupy(feet, radius, height);
        }
        valid_capsule(feet, radius, height)
            && self
                .solids
                .iter()
                .all(|s| valid_solid(s) && capsule_gap(feet, radius, height, s).0 >= -CONTACT_EPS)
    }

    /// LAB torso-height wall proximity, with rounded horizontal corner normals.
    /// This does not reproduce the native multi-query wall-selection policy.
    pub fn wall_contact(
        &self,
        feet: Vec3,
        radius: f32,
        height: f32,
        distance: f32,
    ) -> Option<Vec3> {
        if !valid_capsule(feet, radius, height) || !distance.is_finite() || distance < 0. {
            return None;
        }
        if self.validate().is_err() {
            return None;
        }
        if let Some(asset) = self.mesh_terrain() {
            return asset.wall_contact(feet, radius, height, distance);
        }
        let torso_min = feet.y + height * 0.2;
        let torso_max = feet.y + height * 0.85;
        let mut closest = distance;
        let mut normal = None;
        for s in &self.solids {
            if !valid_solid(s)
                || torso_max <= s.min.y + CONTACT_EPS
                || torso_min >= s.max.y - CONTACT_EPS
            {
                continue;
            }
            let delta = Vec3::new(
                feet.x - feet.x.clamp(s.min.x, s.max.x),
                0.,
                feet.z - feet.z.clamp(s.min.z, s.max.z),
            );
            let length = delta.length();
            let gap = length - radius;
            if length > 0. && gap >= -WALL_INSIDE_EPS && gap <= closest {
                closest = gap;
                normal = Some(delta / length);
            }
        }
        normal
    }

    #[cfg(test)]
    pub(crate) fn move_capsule(
        &self,
        start: Vec3,
        velocity: Vec3,
        dt: f32,
        radius: f32,
        height: f32,
    ) -> MotionResult {
        self.move_capsule_with_settings(
            start,
            velocity,
            dt,
            radius,
            height,
            &CollisionTuning::default(),
        )
    }

    /// LAB schedules the recovered prepare→engine/finalizer→stage→commit
    /// sequence once per substep. Mode0/700=1, valid controller, identity actor
    /// rotation and no root motion are supplied policies. State commits atomically.
    pub(crate) fn move_engine_capsule(
        &self,
        runtime: &mut EngineContactRuntime,
        start: Vec3,
        velocity: Vec3,
        scalar: f32,
        ignore_vertical: bool,
        cached_air: bool,
        dt: f32,
        radius: f32,
        height: f32,
        tuning: &CollisionTuning,
    ) -> MotionResult {
        use crate::{
            contact_finalizer::{FinalizerGlobals, FinalizerState},
            contact_reports::ContactReports,
            engine_move::{self, EngineMoveGlobals, EngineMoveState},
            engine_scene::{AdjustmentContext, EngineScene},
        };
        let mut result = MotionResult {
            position: start,
            velocity,
            grounded: !runtime.air(cached_air, &tuning.engine),
            wall_normal: None,
            contact_normals: Vec::new(),
            failed: true,
        };
        if !valid_capsule(start, radius, height)
            || !velocity.is_finite()
            || !scalar.is_finite()
            || !dt.is_finite()
            || dt <= 0.
            || !tuning.engine.is_valid()
            || self.validate().is_err()
        {
            return result;
        }
        let mut retained = runtime.clone();
        if let Some(mesh) = &retained.mesh {
            let Some(asset) = self.mesh_terrain() else {
                return result;
            };
            // Check the old binding first, even when an explicit teleport/stance
            // will replace its staged capsule state. Geometry never rebinds.
            let old_capsule = cct_scene::Capsule {
                radius: mesh.capsule_radius,
                height: mesh.cylinder_height,
                rotation: mesh.capsule_rotation,
            };
            if !retained.initialized || mesh.validate(asset, old_capsule, retained.center).is_err()
            {
                return result;
            }
        }
        if self.mesh_terrain().is_some() && retained.initialized && retained.mesh.is_none() {
            return result;
        }
        let mesh_dimension_changed = retained.mesh.as_ref().is_some_and(|m| {
            m.capsule_radius.to_bits() != radius.to_bits()
                || m.cylinder_height.to_bits() != (height - 2. * radius).to_bits()
        });
        let fresh_mesh = !retained.initialized
            || start != retained.published_feet
            || height != retained.height
            || mesh_dimension_changed;
        if fresh_mesh && self.mesh_terrain().is_some() {
            // Explicit LAB teleport/stance invalidation resets all contact roles,
            // after checking old geometry identity. This is not a native writer.
            retained = EngineContactRuntime::default();
        }
        if !retained.initialized || start != retained.published_feet || height != retained.height {
            // Explicit teleport/stance adapter, not a recovered shape-resize schedule.
            retained.center = [
                start.x as f64,
                start.y as f64 + (height * 0.5) as f64,
                start.z as f64,
            ];
            retained.published_feet = start;
            retained.height = height;
            retained.initialized = true;
        }
        let Ok(record) = <&[u8; 85]>::try_from(retained.reports.as_slice()) else {
            return result;
        };
        let reports = ContactReports::from_record(record);
        if retained.center.iter().any(|x| !x.is_finite())
            || retained
                .working_normal
                .iter()
                .chain(retained.other_normal.iter())
                .any(|x| !x.is_finite())
            || [
                retained.age,
                retained.timer,
                retained.cached_y,
                retained.normal_y,
            ]
            .iter()
            .any(|x| !x.is_finite())
            || reports
                .primary
                .normal
                .iter()
                .chain(reports.secondary.normal.iter())
                .chain(reports.primary.point.iter())
                .chain(reports.secondary.point.iter())
                .any(|x| !x.is_finite())
        {
            return result;
        }
        let capsule = cct_scene::Capsule {
            radius,
            height: height - 2. * radius,
            rotation: [
                0.,
                0.,
                std::f32::consts::FRAC_1_SQRT_2,
                std::f32::consts::FRAC_1_SQRT_2,
            ],
        };
        let half = radius + capsule.height * 0.5;
        let settings = cct_policy::Settings {
            up: [0., 1., 0.],
            contact_offset: tuning.contact_offset,
            step_offset: ((half + half) * 0.75).max(0.).min(tuning.step_offset),
            min_distance: 1e-6,
            overlap_recovery: tuning.overlap_recovery,
            prevent_ceiling_slide: tuning.prevent_ceiling_slide,
            normalize_components: tuning.normalize_components,
            custom_response: tuning.custom_response,
            walk_experiment: false,
            standing_on_moving: false,
            nonwalkable_mode: 0,
            slope_limit: 0.7,
        };
        let boxes: Vec<_> = self
            .solids
            .iter()
            .map(|s| cct_scene::Collider {
                center: s.center().to_array(),
                extents: (s.size() * 0.5).to_array(),
            })
            .collect();
        let mut state = EngineMoveState {
            contact: FinalizerState {
                reports,
                working_normal: retained.working_normal,
                other_normal: retained.other_normal,
                normal_y: retained.normal_y,
                cached_y: retained.cached_y,
                defer: retained.defer,
                mask: retained.mask,
                suppress: tuning.engine.suppress,
                scalar,
                age: retained.age + dt,
                timer: retained.timer + dt,
                position_y: retained.published_feet.y,
                step: settings.step_offset,
                late_flag: false,
                native_controller_present: true,
            },
            flags: 1,
            configuration_disabled: false,
            movement_disabled: false,
            force_move: false,
            // LAB binds the current launch marker to this supplied engine gate;
            // no live writer/cadence equivalence is implied.
            skip_request_shaping: ignore_vertical,
            optional_adjustment_enabled: tuning.engine.optional_adjustment,
            extra_down_guard: false,
            extra_down_value: 0.,
            contact_offset: tuning.contact_offset,
            split_vertical: 0,
            debug_enabled: false,
        };
        let mut globals = EngineMoveGlobals {
            normal_lower: 0.2588190734386444,
            normal_upper: 0.9990482330322266,
            projection_mask: [1., 0., 1., 0.],
            initialize_mask: false,
            first_axis: [1., 0., 0., 0.],
            second_axis: [0., 0., 1., 0.],
        };
        let finalizer_globals = FinalizerGlobals {
            zero: [0.; 4],
            up: [0., 1., 0., 0.],
            fallback: [1., 0., 0., 0.],
        };
        let adjustment = AdjustmentContext {
            value_3c8: scalar,
            cached_lower: tuning.engine.adjustment_lower,
            upper: tuning.engine.adjustment_upper,
            projection_mask: [1., 0., 1., 0.],
            default_normal: [0.; 4],
        };
        let base = [velocity.x, velocity.y, velocity.z, 0.];
        let up = [0., 1., 0., 0.];
        let mut request = std::array::from_fn(|i| (scalar * up[i] + base[i]) * dt);
        let (center, normals, wall) = if let Some(asset) = self.mesh_terrain() {
            use crate::{cct_slope, engine_mesh_scene::EngineMeshScene, mesh_iteration};
            if tuning.overlap_recovery {
                return result;
            }
            let mut mesh = match retained.mesh.take() {
                Some(m) => m,
                None => match MeshRuntime::new(asset, retained.center, capsule) {
                    Ok(m) => m,
                    Err(_) => return result,
                },
            };
            // Explicit LAB configuration binding to recovered response flag40;
            // this is independent of the slope constraint predicate.
            mesh.state.scheduler.flags = (mesh.state.scheduler.flags & !0x40)
                | if tuning.normalize_components { 0x40 } else { 0 };
            let mesh_settings = mesh_iteration::Settings {
                scheduler: cct_slope::Settings {
                    up: settings.up,
                    step_offset: settings.step_offset,
                    min_distance: settings.min_distance,
                    nonwalkable_mode: 0,
                    standing: false,
                    prevent_ceiling_slide: settings.prevent_ceiling_slide,
                    slope_enabled: true,
                    slope_limit: settings.slope_limit,
                    constrain: false,
                    extent: half,
                },
                contact_offset: settings.contact_offset,
                custom_response: settings.custom_response,
            };
            let mut scene = match EngineMeshScene::new(
                asset,
                mesh,
                capsule,
                mesh_settings,
                finalizer_globals,
                tuning.engine.slope_cosine,
                adjustment,
            ) {
                Ok(s) => s,
                Err(_) => return result,
            };
            scene.behavior = tuning.behavior_flags;
            engine_move::run(&mut state, &mut globals, &mut request, dt, &mut scene);
            if scene.error.is_some() {
                return result;
            }
            let center = scene.center();
            let mut normals = Vec::new();
            let mut wall = None;
            for contact in &scene.contacts {
                let normal = Vec3::from(contact.report.normal);
                normals.push(normal);
                if contact.metadata.pass_id == 1 && normal.y.abs() < 0.5 {
                    wall = Some(normal);
                }
            }
            retained.mesh = Some(scene.runtime);
            (center, normals, wall)
        } else {
            let mut scene = EngineScene {
                center: retained.center,
                capsule,
                boxes: &boxes,
                settings,
                behavior: tuning.behavior_flags,
                metadata: false,
                finalizer_globals: FinalizerGlobals {
                    zero: [0.; 4],
                    up: [0., 1., 0., 0.],
                    fallback: [1., 0., 0., 0.],
                },
                slope_cosine: tuning.engine.slope_cosine,
                hits: Vec::new(),
                error: None,
                native_moves: 0,
                split_vertical: false,
                bridge_projection: [1., 0., 1., 0.],
                adjustment: AdjustmentContext {
                    value_3c8: scalar,
                    cached_lower: tuning.engine.adjustment_lower,
                    upper: tuning.engine.adjustment_upper,
                    projection_mask: [1., 0., 1., 0.],
                    default_normal: [0.; 4],
                },
            };
            engine_move::run(&mut state, &mut globals, &mut request, dt, &mut scene);
            if scene.error.is_some() {
                return result;
            }
            let mut normals = Vec::new();
            let mut wall = None;
            for (pass, hit) in scene.hits {
                let normal = Vec3::from(hit.normal);
                normals.push(normal);
                if pass == cct_policy::Pass::Side && normal.y.abs() < 0.5 {
                    wall = Some(normal);
                }
            }
            (scene.center, normals, wall)
        };
        if center.iter().any(|x| !x.is_finite()) {
            return result;
        }
        // Native stage narrows center BEFORE subtracting the rotated float offset.
        let feet = Vec3::new(
            center[0] as f32,
            center[1] as f32 - height * 0.5,
            center[2] as f32,
        );
        if !self.can_occupy(feet, radius, height) {
            return result;
        }
        retained.center = center;
        retained.published_feet = feet;
        let c = state.contact;
        if self.mesh_terrain().is_some()
            && (!c.age.is_finite()
                || !c.timer.is_finite()
                || !c.cached_y.is_finite()
                || !c.normal_y.is_finite()
                || c.working_normal
                    .into_iter()
                    .chain(c.other_normal)
                    .any(|x| !x.is_finite()))
        {
            return result;
        }
        retained.reports = c.reports.to_record().to_vec();
        retained.working_normal = c.working_normal;
        retained.other_normal = c.other_normal;
        retained.normal_y = c.normal_y;
        retained.cached_y = c.cached_y;
        retained.defer = c.defer;
        retained.mask = c.mask;
        retained.suppress = c.suppress;
        retained.age = c.age;
        retained.timer = c.timer;
        retained.late_flag = c.late_flag;
        result.position = feet;
        result.grounded = !retained.air(cached_air, &tuning.engine);
        result.failed = false;
        result.contact_normals = normals;
        result.wall_normal = wall;
        retained.last_request_base = velocity;
        retained.last_request_scalar = scalar;
        retained.last_request_dt = dt;
        *runtime = retained;
        result
    }

    /// Isolated plain-CCT test adapter; the Player uses the engine sequence above.
    #[cfg(test)]
    pub(crate) fn move_capsule_with_settings(
        &self,
        start: Vec3,
        velocity: Vec3,
        dt: f32,
        radius: f32,
        height: f32,
        tuning: &CollisionTuning,
    ) -> MotionResult {
        let mut result = MotionResult {
            position: start,
            velocity,
            grounded: false,
            wall_normal: None,
            contact_normals: Vec::new(),
            failed: false,
        };
        if !valid_capsule(start, radius, height)
            || !velocity.is_finite()
            || !dt.is_finite()
            || dt < 0.
            || self.solids.iter().any(|s| !valid_solid(s))
        {
            result.failed = true;
            result.velocity = Vec3::ZERO;
            return result;
        }
        let boxes: Vec<_> = self
            .solids
            .iter()
            .map(|s| cct_scene::Collider {
                center: s.center().to_array(),
                extents: (s.size() * 0.5).to_array(),
            })
            .collect();
        let capsule = cct_scene::Capsule {
            radius,
            height: height - 2. * radius,
            // Supplied upright orientation; current native pose selection is separate.
            rotation: [
                0.,
                0.,
                std::f32::consts::FRAC_1_SQRT_2,
                std::f32::consts::FRAC_1_SQRT_2,
            ],
        };
        if !tuning.step_offset.is_finite() || tuning.step_offset < 0. {
            result.failed = true;
            result.velocity = Vec3::ZERO;
            return result;
        }
        let half_height = radius + capsule.height * 0.5;
        let settings = cct_policy::Settings {
            up: [0., 1., 0.],
            contact_offset: tuning.contact_offset,
            // Engine caps configured step by three quarters of total capsule height.
            step_offset: ((half_height + half_height) * 0.75)
                .max(0.)
                .min(tuning.step_offset),
            min_distance: 1e-6,
            overlap_recovery: tuning.overlap_recovery,
            prevent_ceiling_slide: tuning.prevent_ceiling_slide,
            normalize_components: tuning.normalize_components,
            custom_response: tuning.custom_response,
            walk_experiment: false,
            standing_on_moving: false,
            nonwalkable_mode: 0,
            slope_limit: 0.7,
        };
        let mut scene = cct_scene::StaticBoxes {
            capsule,
            boxes: &boxes,
            contact_offset: tuning.contact_offset,
            overlap_recovery: tuning.overlap_recovery,
            behavior: tuning.behavior_flags,
            recovery_scan_limit: 16,
            reports: Vec::new(),
            contact_reports: crate::contact_reports::ContactReports::default(),
            // LAB static actors have no engine metadata wrapper.
            report_metadata: false,
        };
        let center = [
            f64::from(start.x),
            f64::from(start.y) + f64::from(height * 0.5),
            f64::from(start.z),
        ];
        let solved =
            cct_policy::move_with_scene(center, (velocity * dt).to_array(), &settings, &mut scene);
        match solved {
            Ok(motion) => {
                let feet = Vec3::new(
                    motion.position[0] as f32,
                    (motion.position[1] - f64::from(height * 0.5)) as f32,
                    motion.position[2] as f32,
                );
                // LAB final occupancy guard; native callback/actor finalization is separate.
                if !self.can_occupy(feet, radius, height) {
                    result.failed = true;
                    result.velocity = Vec3::ZERO;
                    return result;
                }
                result.position = feet;
                for (pass, state) in motion.passes.iter().enumerate() {
                    for hit in &state.hits {
                        let normal = Vec3::from(hit.normal);
                        result.contact_normals.push(normal);
                        if pass == 1 && normal.y.abs() < 0.5 {
                            result.wall_normal = Some(normal);
                        }
                    }
                }
                // LAB actor-grounding adapter. Native DOWN flags alone do not establish
                // the game's landing callback/cache gates; no extra support cast is added.
                result.grounded = motion.collision_flags & 4 != 0
                    && motion.passes[2].hits.iter().any(|h| {
                        // Zero-overlap sweep normals oppose the cast and need not
                        // describe a supporting face. A valid contact point marks
                        // an actual surface hit; a recovery-only hit cannot land.
                        h.point.is_some() && h.normal[1] > 0.5
                    });
            }
            Err(_) => {
                result.failed = true;
                result.velocity = Vec3::ZERO;
            }
        }
        result
    }
}

fn valid_capsule(feet: Vec3, radius: f32, height: f32) -> bool {
    feet.is_finite()
        && radius.is_finite()
        && radius > 0.
        && height.is_finite()
        && height >= 2. * radius
}

fn valid_solid(s: &Solid) -> bool {
    s.min.is_finite() && s.max.is_finite() && s.min.cmplt(s.max).all()
}

/// Distance from upright capsule's center segment to the box, minus its radius.
fn capsule_gap(feet: Vec3, radius: f32, height: f32, s: &Solid) -> (f32, Option<Vec3>) {
    let low = feet.y + radius;
    let high = feet.y + height - radius;
    let dy = if low > s.max.y {
        low - s.max.y
    } else if high < s.min.y {
        high - s.min.y
    } else {
        0.
    };
    let delta = Vec3::new(
        feet.x - feet.x.clamp(s.min.x, s.max.x),
        dy,
        feet.z - feet.z.clamp(s.min.z, s.max.z),
    );
    (delta.length() - radius, delta.try_normalize())
}

#[cfg(test)]
mod tests {
    use super::*;
    // Exact floor/wall/corridor/overlap/ceiling positions below come from original
    // native code with the same supplied static scene and settings. Private proof:
    // stage3/physics/cct-coupled-pass51/run-002/cases.jsonl, first five fixtures.
    // They validate supplied-scene arithmetic, not measured gameplay fidelity.
    #[test]
    fn aim_ray_chooses_nearest_surface_and_backs_off() {
        let w = World {
            terrain: Default::default(),
            solids: vec![
                Solid::new(Vec3::new(-1., -1., -10.), Vec3::new(1., 1., -9.)),
                Solid::new(Vec3::new(-1., -1., -4.), Vec3::new(1., 1., -3.)),
            ],
        };
        assert!(
            w.aim_target(Vec3::ZERO, Vec3::NEG_Z * 2., 5000.)
                .unwrap()
                .distance(Vec3::new(0., 0., -2.9))
                < 1e-5
        );
    }
    #[test]
    fn aim_miss_clips_segment_without_changing_direction() {
        let w = World::default();
        assert_eq!(
            w.aim_target(Vec3::ZERO, Vec3::NEG_Z, 5000.),
            Some(Vec3::NEG_Z * 5000.)
        );
        let origin = Vec3::new(4999., 0., 0.);
        assert!(
            w.aim_target(origin, Vec3::new(1., 1., 0.), 5000.)
                .unwrap()
                .distance(Vec3::new(5000., 1., 0.))
                < 1e-4
        );
    }
    #[test]
    fn aim_ray_inside_box_uses_exit_and_invalid_input_is_rejected() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::splat(-1.), Vec3::ONE)],
        };
        assert!(
            w.aim_target(Vec3::ZERO, Vec3::X, 5000.)
                .unwrap()
                .distance(Vec3::X * 0.9)
                < 1e-6
        );
        assert!(w.aim_target(Vec3::ZERO, Vec3::ZERO, 5000.).is_none());
        assert!(w.aim_target(Vec3::ZERO, Vec3::NAN, 5000.).is_none());
        assert!(w.aim_target(Vec3::ZERO, Vec3::X, -1.).is_none());
        assert!(w.aim_target(Vec3::X * 5001., Vec3::NEG_X, 5000.).is_none());
    }
    #[test]
    fn separated_floor_contact_preserves_tangent_and_support() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(
                Vec3::new(-50., -2., -50.),
                Vec3::new(50., 0., 50.),
            )],
        };
        let r = w.move_capsule(Vec3::Y * 0.001, Vec3::new(2., -2., 2.), 0.01, 0.3, 1.8);
        assert!(!r.failed && r.grounded, "{r:?}");
        assert!(
            (r.position.x - 0.02).abs() < 1e-5 && (r.position.z - 0.02).abs() < 1e-5,
            "{r:?}"
        );
        assert_eq!(
            r.position.to_array().map(f32::to_bits),
            [1017370378, 1017370391, 1017370378]
        );
        assert_eq!(r.velocity, Vec3::new(2., -2., 2.));
    }

    #[test]
    fn tight_corridor_contact_preserves_nonoverlap_without_extra_skin_push() {
        let w = World {
            terrain: Default::default(),
            solids: vec![
                Solid::new(Vec3::new(-1., -5., -5.), Vec3::new(0., 5., 5.)),
                Solid::new(Vec3::new(0.6005, -5., -5.), Vec3::new(1.6, 5., 5.)),
            ],
        };
        let start = Vec3::new(0.30025, 0., 0.);
        assert!(w.can_occupy(start, 0.3, 1.8));
        let r = w.move_capsule(start, Vec3::X, 0.01, 0.3, 1.8);
        assert!(
            !r.failed && r.position == start && w.can_occupy(r.position, 0.3, 1.8),
            "{r:?}"
        );
    }

    #[test]
    fn unsupported_geometry_and_overflow_stop_at_validated_position() {
        let empty = World::default();
        let r = empty.move_capsule(Vec3::ZERO, Vec3::X * f32::MAX, f32::MAX, 0.3, 1.8);
        assert!(r.failed && r.position == Vec3::ZERO && r.velocity == Vec3::ZERO);
        // Unsupported large local coordinates must not silently become a miss.
        let large = World {
            terrain: Default::default(),
            solids: vec![Solid::new(
                Vec3::new(20_000., -5., -5.),
                Vec3::new(20_001., 5., 5.),
            )],
        };
        let r = large.move_capsule(Vec3::ZERO, Vec3::X, 1., 0.3, 1.8);
        assert!(r.failed && r.position == Vec3::ZERO);
        assert!(!empty.can_occupy(Vec3::ZERO, 0.6, 0.9));
    }

    #[test]
    fn rounded_corner_occupancy_and_sweep() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::new(0., -5., 0.), Vec3::new(1., 5., 1.))],
        };
        assert!(w.can_occupy(Vec3::new(-0.25, 0., -0.25), 0.3, 1.8));
        assert!(!w.can_occupy(Vec3::new(-0.2, 0., -0.2), 0.3, 1.8));
        let r = w.move_capsule(Vec3::new(-1., 0., -0.25), Vec3::X * 2., 1., 0.3, 1.8);
        assert!(!r.failed && w.can_occupy(r.position, 0.3, 1.8), "{r:?}");
        assert!(r.position.z < -0.3 && r.velocity.x > 0.);
    }

    #[test]
    fn tangent_cast_is_free_and_initial_overlap_recovers() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::new(0., -5., 0.), Vec3::new(1., 5., 1.))],
        };
        for z in [-0.3, -0.301] {
            let r = w.move_capsule(Vec3::new(-1., 0., z), Vec3::X * 3., 1., 0.3, 1.8);
            assert!(!r.failed && r.position.x > 1.99, "{r:?}");
        }
        let start = Vec3::new(-0.2, 0., -0.2);
        let r = w.move_capsule(start, Vec3::X * 3., 1., 0.3, 1.8);
        assert!(!r.failed && w.can_occupy(r.position, 0.3, 1.8));
        assert_eq!(
            r.position.to_array().map(f32::to_bits),
            [1069416668, 3042014514, 3218770671]
        );
        assert_eq!(r.velocity, Vec3::X * 3.);
    }

    #[test]
    fn sweep_stops_at_thin_wall_and_slides() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(
                Vec3::new(2., -1., -20.),
                Vec3::new(2.01, 10., 20.),
            )],
        };
        let r = w.move_capsule(Vec3::ZERO, Vec3::new(1000., 0., 10.), 0.1, 0.3, 1.8);
        assert_eq!(
            r.position.to_array().map(f32::to_bits),
            [1071057625, 0, 1065368405]
        );
        assert!(!r.failed);
        assert_eq!(r.velocity, Vec3::new(1000., 0., 10.));
        assert!(w.can_occupy(r.position, 0.3, 1.8));
    }
    #[test]
    fn ground_is_not_a_wall_and_parallel_motion_is_free() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(
                Vec3::new(-20., -2., -20.),
                Vec3::new(20., 0., 20.),
            )],
        };
        let r = w.move_capsule(Vec3::ZERO, Vec3::new(5., -1., 0.), 0.1, 0.3, 1.8);
        assert!(r.grounded);
        assert!((r.position.x - 0.5).abs() < 0.001);
        assert!(w.wall_contact(r.position, 0.3, 1.8, 0.2).is_none());
    }
    #[test]
    fn ceiling_stops_vertical_motion_without_wall_contact() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::new(-5., 3., -5.), Vec3::new(5., 4., 5.))],
        };
        let r = w.move_capsule(Vec3::ZERO, Vec3::Y * 100., 0.1, 0.3, 1.8);
        assert_eq!(r.position.to_array().map(f32::to_bits), [0, 1066863165, 0]);
        assert!(!r.failed);
        assert_eq!(r.velocity, Vec3::Y * 100.);
        assert!(!r.grounded);
        assert!(w.wall_contact(r.position, 0.3, 1.8, 0.2).is_none());
    }
    #[test]
    fn downward_wall_overlap_recovery_does_not_claim_floor_support() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(
                Vec3::new(0., -50., -50.),
                Vec3::new(1., 50., 50.),
            )],
        };
        let r = w.move_capsule(Vec3::new(-0.2, 10., 0.), Vec3::NEG_Y, 0.01, 0.3, 1.8);
        assert!(!r.failed && w.can_occupy(r.position, 0.3, 1.8), "{r:?}");
        assert!(
            r.position.x < -0.3,
            "recovery must move away from wall: {r:?}"
        );
        assert!(
            !r.grounded,
            "cast-opposed initial-overlap normal is not a floor: {r:?}"
        );
    }

    #[test]
    fn crouched_hull_fits_but_standing_hull_does_not() {
        let w = World {
            terrain: Default::default(),
            solids: vec![Solid::new(Vec3::new(-2., 1.1, -2.), Vec3::new(2., 2., 2.))],
        };
        assert!(w.can_occupy(Vec3::ZERO, 0.3, 0.9));
        assert!(!w.can_occupy(Vec3::ZERO, 0.3, 1.8));
    }
}

#[cfg(test)]
mod engine_runtime_tests {
    use super::*;
    fn step(
        w: &World,
        r: &mut EngineContactRuntime,
        p: Vec3,
        v: Vec3,
        scalar: f32,
        cached: bool,
        t: &CollisionTuning,
    ) -> MotionResult {
        w.move_engine_capsule(r, p, v, scalar, false, cached, 0.01, 0.3, 1.8, t)
    }
    #[test]
    fn native_center_survives_publication_and_narrows_before_offset() {
        let w = World::default();
        let t = CollisionTuning::default();
        let mut r = EngineContactRuntime::default();
        let first = step(
            &w,
            &mut r,
            Vec3::new(0., 100., 0.),
            Vec3::ZERO,
            0.,
            true,
            &t,
        );
        assert!(!first.failed);
        r.center[1] = 100.90000003;
        let expected = r.center[1] as f32 - 1.8f32 * 0.5;
        let second = step(&w, &mut r, first.position, Vec3::ZERO, 0., true, &t);
        assert!(!second.failed);
        assert_eq!(second.position.y.to_bits(), expected.to_bits());
        assert_eq!(r.center[1], 100.90000003);
        let teleported = step(&w, &mut r, Vec3::new(3., 10., 4.), Vec3::ZERO, 0., true, &t);
        assert!(!teleported.failed);
        assert_eq!(r.center[1], 10f64 + (1.8f32 * 0.5) as f64);
    }
    #[test]
    fn persistent_report_record_and_contact_fields_roundtrip() {
        let mut r = EngineContactRuntime::default();
        r.initialized = true;
        r.center = [1.00000000003, 2., 3.];
        r.reports[32] = 129;
        r.reports[80] = 2;
        r.age = 0.3125;
        r.defer = true;
        r.mask = 4;
        r.working_normal = [0., 0.3, 0.4, -0.];
        r.cached_y = 9.;
        let json = serde_json::to_string(&r).unwrap();
        let restored: EngineContactRuntime = serde_json::from_str(&json).unwrap();
        assert_eq!(serde_json::to_string(&restored).unwrap(), json);
        assert_eq!(restored.center[0].to_bits(), r.center[0].to_bits());
        assert_eq!(restored.working_normal[3].to_bits(), (-0f32).to_bits());
    }
    #[test]
    fn cached_air_and_defer_control_landing_independently_of_down() {
        let mut r = EngineContactRuntime::default();
        let mut p = EngineContactPolicy::default();
        r.mask = 4;
        r.defer = true;
        assert!(r.air(true, &p));
        assert!(!r.air(false, &p));
        r.defer = false;
        assert!(!r.air(true, &p));
        r.mask = 0;
        assert!(r.air(false, &p));
        p.callback_valid = false;
        assert!(!r.air(true, &p));
        p.actor_air_mask = 0x8000;
        assert!(r.air(false, &p));
    }
    #[test]
    fn failure_does_not_commit_partial_engine_state() {
        let w = World::default();
        let t = CollisionTuning::default();
        let mut r = EngineContactRuntime::default();
        let before = serde_json::to_string(&r).unwrap();
        let m = step(&w, &mut r, Vec3::ZERO, Vec3::splat(f32::MAX), 0., true, &t);
        assert!(m.failed);
        assert_eq!(serde_json::to_string(&r).unwrap(), before);
    }
    #[test]
    fn finalizer_observes_previous_staged_y_before_new_commit() {
        let w = World::default();
        let t = CollisionTuning::default();
        let mut r = EngineContactRuntime::default();
        let m = step(&w, &mut r, Vec3::new(0., 4., 0.), Vec3::ZERO, 0., true, &t);
        assert!(!m.failed);
        r.timer = 1.1;
        r.cached_y = -99.;
        r.center[1] = 14.0 + (1.8f32 * 0.5) as f64;
        let m = step(&w, &mut r, m.position, Vec3::ZERO, 0., true, &t);
        assert!(!m.failed);
        assert_eq!(r.cached_y, 4.);
        assert_eq!(
            m.position.y.to_bits(),
            (r.center[1] as f32 - 1.8f32 * 0.5).to_bits()
        );
        assert_eq!(m.position.y, 14.);
        assert!(r.timer > 1.1);
    }
    fn triangle_world() -> (World, CollisionTuning) {
        let world = World::mesh(StaticTerrain::course()).unwrap();
        let mut tuning = CollisionTuning::default();
        tuning.overlap_recovery = false;
        (world, tuning)
    }
    #[test]
    fn world_selection_preserves_legacy_shape_and_rejects_mixed_geometry() {
        let legacy = serde_json::json!({"solids":[]});
        let boxes: World = serde_json::from_value(legacy.clone()).unwrap();
        assert_eq!(serde_json::to_value(&boxes).unwrap(), legacy);
        let (mut mesh, _) = triangle_world();
        assert!(mesh.solids.is_empty());
        mesh.solids.push(Solid::new(Vec3::ZERO, Vec3::ONE));
        assert_eq!(mesh.validate(), Err(TerrainError::MixedSolids));
        assert!(!mesh.can_occupy(mesh.spawn(), 0.3, 1.8));
        assert!(mesh.aim_target(mesh.spawn(), Vec3::NEG_Z, 5000.).is_none());
        assert!(
            serde_json::from_value::<World>(
                serde_json::json!({"solids":[],"terrain":{"kind":"mesh"}})
            )
            .is_err()
        );
    }
    #[test]
    fn world_mesh_services_use_course_triangles_for_floor_and_ceiling() {
        let (world, _) = triangle_world();
        assert!(world.can_occupy(world.spawn(), 0.3, 1.8));
        assert!(!world.can_occupy(Vec3::new(0., -1., 8.), 0.3, 1.8));
        let tunnel = Vec3::new(0., 0.04, 16.);
        assert!(!world.can_occupy(tunnel, 0.3, 1.8));
        assert!(world.can_occupy(tunnel, 0.3, 1.));
        let origin = Vec3::new(0., 0.5, 16.);
        let hit = world.aim_target(origin, Vec3::Y, 5000.).unwrap();
        assert!((hit.y - 1.1).abs() < 0.0001, "{hit:?}");
        assert!(world.camera_fraction(origin, origin + Vec3::Y * 2., 0.15) < 1.);
        assert!(world.wall_contact(world.spawn(), 0.3, 1.8, 0.2).is_none());
    }
    #[test]
    fn world_mesh_retains_actual_landing_handles_and_serialized_continuation() {
        let (world, tuning) = triangle_world();
        let mut runtime = EngineContactRuntime::default();
        let a = world.move_engine_capsule(
            &mut runtime,
            world.spawn(),
            Vec3::ZERO,
            -1.,
            false,
            true,
            0.1,
            0.3,
            1.8,
            &tuning,
        );
        assert!(!a.failed && a.grounded, "{a:?}");
        let mesh = runtime.mesh.as_ref().unwrap();
        assert!(mesh.common.handles.actor.is_some() && mesh.common.handles.shape.is_some());
        assert!(mesh.state.mesh.handles.actor.is_some() && mesh.state.mesh.handles.shape.is_some());
        let encoded = serde_json::to_vec(&runtime).unwrap();
        let mut resumed: EngineContactRuntime = serde_json::from_slice(&encoded).unwrap();
        world
            .validate_player_runtime(&resumed, a.position, 0.3, 1.8)
            .unwrap();
        let b = world.move_engine_capsule(
            &mut runtime,
            a.position,
            Vec3::X,
            -1.,
            false,
            false,
            0.01,
            0.3,
            1.8,
            &tuning,
        );
        let c = world.move_engine_capsule(
            &mut resumed,
            a.position,
            Vec3::X,
            -1.,
            false,
            false,
            0.01,
            0.3,
            1.8,
            &tuning,
        );
        assert!(!b.failed && !c.failed, "{b:?} {c:?}");
        assert_eq!(
            serde_json::to_vec(&runtime).unwrap(),
            serde_json::to_vec(&resumed).unwrap()
        );
        assert_eq!(
            b.position.to_array().map(f32::to_bits),
            c.position.to_array().map(f32::to_bits)
        );
        assert!(runtime.mesh.as_ref().unwrap().common.numeric.elapsed > 0.1);
    }
    #[test]
    fn world_mesh_rejects_revision_change_before_teleport_and_keeps_runtime_atomic() {
        let (world, tuning) = triangle_world();
        let mut runtime = EngineContactRuntime::default();
        let landing = world.move_engine_capsule(
            &mut runtime,
            world.spawn(),
            Vec3::ZERO,
            -1.,
            false,
            true,
            0.1,
            0.3,
            1.8,
            &tuning,
        );
        assert!(!landing.failed);
        let before = serde_json::to_vec(&runtime).unwrap();
        let mut serialized = serde_json::to_value(&world).unwrap();
        serialized["terrain"]["asset"]["revision"] = serde_json::json!(60);
        let changed: World = serde_json::from_value(serialized).unwrap();
        let failed = changed.move_engine_capsule(
            &mut runtime,
            landing.position + Vec3::X,
            Vec3::ZERO,
            -1.,
            false,
            false,
            0.01,
            0.3,
            1.8,
            &tuning,
        );
        assert!(failed.failed);
        assert_eq!(serde_json::to_vec(&runtime).unwrap(), before);
        let mut unsupported = tuning;
        unsupported.overlap_recovery = true;
        assert!(
            world
                .move_engine_capsule(
                    &mut runtime,
                    landing.position,
                    Vec3::ZERO,
                    -1.,
                    false,
                    false,
                    0.01,
                    0.3,
                    1.8,
                    &unsupported
                )
                .failed
        );
        assert_eq!(serde_json::to_vec(&runtime).unwrap(), before);
    }
    #[test]
    fn post_publication_occupancy_failure_keeps_entire_world_runtime_atomic() {
        let (wide, tuning) = triangle_world();
        let mut value = serde_json::to_value(&wide).unwrap();
        value["terrain"]["asset"]["allowed_max"][0] = serde_json::json!(0.4);
        let narrow: World = serde_json::from_value(value).unwrap();
        let mut a = EngineContactRuntime::default();
        let mut b = EngineContactRuntime::default();
        let start = wide.spawn();
        let landing_a = wide.move_engine_capsule(
            &mut a,
            start,
            Vec3::ZERO,
            -1.,
            false,
            true,
            0.1,
            0.3,
            1.8,
            &tuning,
        );
        let landing_b = narrow.move_engine_capsule(
            &mut b,
            start,
            Vec3::ZERO,
            -1.,
            false,
            true,
            0.1,
            0.3,
            1.8,
            &tuning,
        );
        assert!(!landing_a.failed && !landing_b.failed);
        assert_eq!(
            serde_json::to_value(a.mesh.as_ref().unwrap().state).unwrap(),
            serde_json::to_value(b.mesh.as_ref().unwrap().state).unwrap()
        );
        let before = serde_json::to_vec(&b).unwrap();
        let counter = a.mesh.as_ref().unwrap().state.mesh.counter;
        let control = wide.move_engine_capsule(
            &mut a,
            landing_a.position,
            Vec3::X * 10.,
            -1.,
            false,
            false,
            0.1,
            0.3,
            1.8,
            &tuning,
        );
        assert!(!control.failed && control.position.x > 0.1, "{control:?}");
        assert!(
            a.mesh.as_ref().unwrap().state.mesh.counter > counter,
            "actual control must publish a new contact"
        );
        let failed = narrow.move_engine_capsule(
            &mut b,
            landing_b.position,
            Vec3::X * 10.,
            -1.,
            false,
            false,
            0.1,
            0.3,
            1.8,
            &tuning,
        );
        assert!(failed.failed && failed.position == landing_b.position);
        assert_eq!(serde_json::to_vec(&b).unwrap(), before);
    }
}
