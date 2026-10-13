//! Reviewed-model candidate for supplied prepared static triangle iteration.
//! Cache/services/object identity are supplied; native scene membership, moving
//! actors, overlap recovery and previous Common object tracking remain outside.
//! All LAB errors are atomic; native Common retries restore position only.
use crate::{
    capsule_mesh, cct_response,
    cct_scene::Capsule,
    cct_slope, mesh_publication,
    precise_capsule::{Triangle, Vec3},
};
pub type Position = [f64; 3];
pub type Pose = [f32; 7];

// These identify the one supplied fixture object pair; no class enum or live
// pointer ownership is inferred. Distinct Rust types prevent role reversal.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ActorHandle;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ShapeHandle;
macro_rules! fixture_handle_serde {
    ($ty:ty, $label:literal) => {
        impl serde::Serialize for $ty {
            fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                serializer.serialize_str($label)
            }
        }
        impl<'de> serde::Deserialize<'de> for $ty {
            fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
                let label = <String as serde::Deserialize>::deserialize(deserializer)?;
                if label == $label { Ok(Self) } else { Err(serde::de::Error::custom("wrong fixture handle role")) }
            }
        }
    };
}
fixture_handle_serde!(ActorHandle, "fixture_actor");
fixture_handle_serde!(ShapeHandle, "fixture_shape");
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Handles {
    #[serde(deserialize_with = "required_handle_option")]
    pub actor: Option<ActorHandle>,
    #[serde(deserialize_with = "required_handle_option")]
    pub shape: Option<ShapeHandle>,
}
fn required_handle_option<'de, D, T>(deserializer: D) -> Result<Option<T>, D::Error>
where D: serde::Deserializer<'de>, T: serde::Deserialize<'de> {
    <Option<T> as serde::Deserialize>::deserialize(deserializer)
}
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshState {
    pub cache: [u32; 3],
    pub triangle_min: f32,
    pub counter: u16,
    pub obstacle: u32,
    pub local_contact: Vec3,
    pub world_contact: Vec3,
    pub handles: Handles,
}
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct State {
    pub scheduler: cct_slope::State,
    pub mesh: MeshState,
}
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CommonPublication {
    pub numeric: cct_slope::Publication,
    /// Persistent Common actor/shape slots, separate from mesh output handles.
    /// Nonnull previous values remain an unsupported-domain error in this port.
    pub handles: Handles,
}
#[derive(Clone, Copy, Debug)]
pub struct Record {
    pub start: u32,
    pub count: u32,
    pub offset: Position,
}
pub struct PreparedScene<'a> {
    pub triangles: &'a [Triangle],
    pub source_ids: &'a [u32],
    pub records: &'a [Record],
    pub cache_origin: Position,
    pub object_bounds: [f32; 6],
    pub actor_pose: Pose,
    pub shape_pose: Pose,
    pub behavior: u8,
}
#[derive(Clone, Copy, Debug)]
pub struct Settings {
    pub scheduler: cct_slope::Settings,
    pub contact_offset: f32,
    pub custom_response: bool,
}
#[derive(Clone, Copy, Debug)]
pub enum Request {
    Iterate {
        displacement: Vec3,
        side: Vec3,
        budget: u32,
        minimum: f32,
        pass_id: u32,
    },
    Split {
        displacement: Vec3,
    },
    Common {
        displacement: Vec3,
        dt: f32,
    },
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    InvalidInput,
    UnsupportedDomain,
    NonFiniteArithmetic,
    QueryFailed,
    PublicationFailed,
}

// Typed trace. Probe serializes each vector/scalar to packed little-endian hex,
// matching the full normalized model tree, with exact keys and cardinalities.
#[derive(Clone, Copy, Debug)]
pub struct Snapshot {
    pub position: Position,
    pub flags: u32,
    pub tag: u32,
    pub cache: [u32; 3],
    pub down: Vec3,
    pub side: Vec3,
    pub heights: [f32; 3], // triangle_min, triangle_height, bound_height
    pub contact_height: f32,
}
#[derive(Clone, Debug)]
pub struct IterationRequest {
    pub pass_id: u32,
    pub budget: u32,
    pub delta: Vec3,
    pub side: Vec3,
    pub minimum: f32,
    pub state: Snapshot,
}
#[derive(Clone, Debug)]
pub struct IterationReturn {
    pub pass_id: u32,
    pub attempted: bool,
    pub count: u32,
    pub state: Snapshot,
}
#[derive(Clone, Copy, Debug)]
pub struct QueryHit {
    pub point: Position,
    pub normal: Vec3,
    pub distance: f32,
    pub global_index: u32,
    pub original_index: u32,
    pub record: usize,
}
#[derive(Clone, Debug)]
pub struct Query {
    pub center: Position,
    pub direction: Vec3,
    pub maximum: f32,
    pub pass_tag: u32,
    pub cache: [u32; 3],
    pub result: QueryResult,
}
#[derive(Clone, Debug)]
pub struct QueryResult {
    pub cache: [u32; 3],
    pub hit: Option<QueryHit>,
}
#[derive(Clone, Debug)]
pub struct Publication {
    pub pass_id: u32,
    pub index: u32,
    pub before: Snapshot,
    pub after: Snapshot,
    pub backface: u8,
}
#[derive(Clone, Debug)]
pub struct Report {
    pub point: Position,
    pub normal: Vec3,
    pub direction: Vec3,
    pub distance: f32, // remaining requested distance, NOT hit distance
    pub source_id: u32,
    pub record: usize,
    pub owner_identity: u64,
    pub actor: ActorHandle,
    pub shape: ShapeHandle,
}
/// Parallel metadata emitted at the report site, preserving exact pass/tag roles.
#[derive(Clone, Copy, Debug)]
pub struct ReportMetadata {
    pub pass_id: u32,
    pub cache_tag: u32,
    pub global_index: u32,
}
#[derive(Clone, Copy, Debug)]
pub struct IterationSemantics {
    pub counter: u16,
    pub obstacle: u32,
    pub local_contact: Vec3,
    pub world_contact: Vec3,
    pub handles: Handles,
}
#[derive(Clone, Debug)]
pub struct Trace {
    pub state: Snapshot,
    pub collision_flags: u8,
    pub iterations: Vec<IterationRequest>,
    pub returns: Vec<IterationReturn>,
    pub queries: Vec<Query>,
    pub publications: Vec<Publication>,
    pub reports: Vec<Report>,
    pub report_metadata: Vec<ReportMetadata>,
    pub transforms: Vec<Pose>,
    pub iteration_semantics: Vec<IterationSemantics>,
    pub common: Option<CommonPublication>,
}

fn xy(a: Vec3, b: Vec3) -> f32 {
    (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]
}
fn yx(a: Vec3, b: Vec3) -> f32 {
    (a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]
}
fn length(v: Vec3) -> f32 {
    yx(v, v).sqrt()
}
fn unit(v: Vec3) -> Vec3 {
    let n = length(v);
    if n > 0. {
        let r = 1. / n;
        v.map(|x| x * r)
    } else {
        v
    }
}
fn finite(values: impl IntoIterator<Item = f32>) -> Result<(), Error> {
    if values.into_iter().all(f32::is_finite) {
        Ok(())
    } else {
        Err(Error::NonFiniteArithmetic)
    }
}
fn bounded(x: f32) -> bool {
    x.is_finite() && (x == 0. || x.is_normal()) && x.abs() <= 10000.
}
fn normal_or_zero(x: f32) -> bool {
    x.is_finite() && (x == 0. || x.is_normal())
}
fn double_normal_or_zero(x: f64) -> bool {
    x.is_finite() && (x == 0. || x.is_normal())
}
fn snapshot(s: &cct_slope::State, m: &MeshState) -> Snapshot {
    Snapshot {
        position: s.position,
        flags: s.flags,
        tag: s.tag,
        cache: m.cache,
        down: s.down_normal,
        side: s.side_normal,
        heights: [m.triangle_min, s.triangle_height, s.bound_height],
        contact_height: s.contact_height,
    }
}
fn state_finite(s: &State) -> bool {
    s.scheduler.position.into_iter().all(f64::is_finite)
        && s.scheduler
            .down_normal
            .into_iter()
            .chain(s.scheduler.side_normal)
            .chain([
                s.scheduler.triangle_height,
                s.scheduler.bound_height,
                s.scheduler.contact_height,
                s.mesh.triangle_min,
            ])
            .chain(s.mesh.local_contact)
            .chain(s.mesh.world_contact)
            .all(normal_or_zero)
}
fn double_xy(a: Vec3, b: Position) -> f64 {
    (f64::from(a[0]) * b[0] + f64::from(a[1]) * b[1]) + f64::from(a[2]) * b[2]
}
fn double_yx(a: Vec3, b: Position) -> f64 {
    (f64::from(a[1]) * b[1] + f64::from(a[0]) * b[0]) + f64::from(a[2]) * b[2]
}
// LAB preflight of possibly consumed scheduler projections. This guard is
// broader than the native branch admission; no native rejection is inferred.
fn scheduler_arithmetic(
    s: &cct_slope::State,
    settings: cct_slope::Settings,
    bottom: f64,
) -> Result<(), Error> {
    finite([
        xy(s.down_normal, settings.up),
        yx(s.down_normal, settings.up),
        xy(s.side_normal, settings.up),
        yx(s.side_normal, settings.up),
        (f64::from(s.triangle_height) - bottom) as f32,
        (f64::from(s.bound_height) - bottom) as f32,
    ])?;
    if !bottom.is_finite()
        || !double_xy(settings.up, s.position).is_finite()
        || !double_yx(settings.up, s.position).is_finite()
    {
        return Err(Error::NonFiniteArithmetic);
    }
    Ok(())
}
/// Original composed pose arithmetic; no generic quaternion/matrix replacement.
fn pose(world: Pose, local: Pose) -> Pose {
    let [x, y, z, w, px, py, pz] = world;
    let [lx, ly, lz, lw, lpx, lpy, lpz] = local;
    let tx = lpx + lpx;
    let ty = lpy + lpy;
    let tz = lpz + lpz;
    let q = [
        ((lx * w + lw * x) + lz * y) - ly * z,
        ((lw * y + w * ly) + lx * z) - x * lz,
        ((w * lz + lw * z) + x * ly) - lx * y,
        ((w * lw - lx * x) - y * ly) - lz * z,
    ];
    let dot = (ty * y + tx * x) + tz * z;
    let scale = w * w - 0.5;
    let p = [
        ((tz * y - ty * z) * w + scale * tx) + (dot * x + px),
        ((tx * z - tz * x) * w + py) + (scale * ty + dot * y),
        ((ty * x - tx * y) * w + pz) + (scale * tz + dot * z),
    ];
    [q[0], q[1], q[2], q[3], p[0], p[1], p[2]]
}
fn inverse_point(point: Position, p: Pose) -> Vec3 {
    let [x, y, z, w, px, py, pz] = p;
    let tx = (point[0] as f32 - px) + (point[0] as f32 - px);
    let ty = (point[1] as f32 - py) + (point[1] as f32 - py);
    let tz = (point[2] as f32 - pz) + (point[2] as f32 - pz);
    let scale = w * w - 0.5;
    let dot = (y * ty + x * tx) + z * tz;
    [
        (tx * scale - (y * tz - z * ty) * w) + x * dot,
        (ty * scale - (z * tx - x * tz) * w) + y * dot,
        (tz * scale - (x * ty - y * tx) * w) + z * dot,
    ]
}
struct Engine<'a, 'b> {
    mesh: &'a mut MeshState,
    scene: &'a PreparedScene<'b>,
    capsule: Capsule,
    settings: Settings,
    direct: bool,
    scheduler_bottom: f64,
    error: Option<Error>,
    trace: Trace,
}
impl Engine<'_, '_> {
    fn dispatch(
        &mut self,
        s: &cct_slope::State,
        center: Position,
        direction: Vec3,
        mut maximum: f32,
    ) -> Result<Option<QueryHit>, Error> {
        let tag = s.tag as usize;
        let mut winner = None;
        for (index, r) in self.scene.records.iter().enumerate() {
            let out = capsule_mesh::cached_mesh_query(&capsule_mesh::CachedMeshInput {
                triangles: self.scene.triangles,
                original_ids: self.scene.source_ids,
                quaternion: self.capsule.rotation,
                radius: self.capsule.radius,
                height: self.capsule.height,
                center,
                offset: r.offset,
                direction,
                max_distance: maximum,
                triangle_offset: r.start,
                triangle_count: r.count,
                cached_index: self.mesh.cache[tag],
                pass_id: s.tag,
                overlap: false,
                precise: true,
            })
            .map_err(|e| match e {
                capsule_mesh::MeshError::NonFiniteArithmetic => Error::NonFiniteArithmetic,
                _ => Error::QueryFailed,
            })?;
            self.mesh.cache[tag] = out.cached_index;
            if let Some(hit) = out.hit {
                maximum = hit.distance;
                winner = Some(QueryHit {
                    point: hit.world_point,
                    normal: hit.normal,
                    distance: hit.distance,
                    global_index: hit.global_index,
                    original_index: hit.original_index,
                    record: index,
                });
                if maximum <= 0. {
                    break;
                }
            }
        }
        Ok(winner)
    }
    fn iteration(
        &mut self,
        s: &mut cct_slope::State,
        r: cct_slope::Iteration,
    ) -> Result<cct_slope::Outcome, Error> {
        let up = self.settings.scheduler.up;
        let offset = self.settings.contact_offset;
        let original = r.displacement;
        let mut position = s.position;
        let mut target: Position = std::array::from_fn(|i| position[i] + f64::from(original[i]));
        let mut budget = r.budget;
        let mut attempted = false;
        let mut count = 0_u32;
        let mut repeated = 0_u32;
        let mut previous = [0.; 3];
        // Splitter clears these before DOWN even for an all-zero request.
        if r.pass == 2 && !self.direct {
            self.mesh.obstacle = u32::MAX;
            self.mesh.handles = Handles {
                actor: None,
                shape: None,
            };
        }
        self.trace.iterations.push(IterationRequest {
            pass_id: r.pass,
            budget: r.budget,
            delta: original,
            side: r.side,
            minimum: r.minimum,
            state: snapshot(s, self.mesh),
        });
        if original.into_iter().any(|x| x != 0.) {
            s.flags &= !0x34;
            self.mesh.handles = Handles {
                actor: None,
                shape: None,
            };
            self.mesh.obstacle = u32::MAX;
            while budget != 0 {
                self.mesh.counter = self.mesh.counter.wrapping_add(1);
                budget -= 1;
                let delta: Vec3 = std::array::from_fn(|i| (target[i] - position[i]) as f32);
                let distance = length(delta);
                finite(delta.into_iter().chain([distance]))?;
                if distance <= r.minimum {
                    break;
                }
                let reciprocal = 1. / distance;
                let direction = delta.map(|v| v * reciprocal);
                let forward = xy(direction, original);
                finite(direction.into_iter().chain([forward]))?;
                if forward <= 0. {
                    break;
                }
                attempted = true;
                let maximum = distance + offset;
                finite([maximum])?;
                let before = self.mesh.cache;
                let hit = self.dispatch(s, position, direction, maximum)?;
                self.trace.queries.push(Query {
                    center: position,
                    direction,
                    maximum,
                    pass_tag: s.tag,
                    cache: before,
                    result: QueryResult {
                        cache: self.mesh.cache,
                        hit,
                    },
                });
                let Some(hit) = hit else {
                    position = target;
                    break;
                };
                let before = snapshot(s, self.mesh);
                let mut publication = mesh_publication::State {
                    flags: s.flags,
                    down_normal: s.down_normal,
                    side_normal: s.side_normal,
                    triangle_min: self.mesh.triangle_min,
                    triangle_height: s.triangle_height,
                    bound_height: s.bound_height,
                };
                let origin = self.scene.cache_origin;
                let out = mesh_publication::publish(
                    &mut publication,
                    &mesh_publication::Input {
                        pass_id: r.pass,
                        object_word: 6,
                        nonwalkable_mode: self.settings.scheduler.nonwalkable_mode,
                        triangle_index: hit.global_index,
                        triangles: self.scene.triangles,
                        up,
                        cache_bounds: [
                            origin[0], origin[1], origin[2], origin[0], origin[1], origin[2],
                        ],
                        object_bounds: self.scene.object_bounds,
                        prevent_ceiling_slide: self.settings.scheduler.prevent_ceiling_slide,
                    },
                )
                .map_err(|e| match e {
                    mesh_publication::Error::NonFiniteArithmetic => Error::NonFiniteArithmetic,
                    _ => Error::PublicationFailed,
                })?;
                s.flags = publication.flags;
                s.down_normal = publication.down_normal;
                s.side_normal = publication.side_normal;
                self.mesh.triangle_min = publication.triangle_min;
                s.triangle_height = publication.triangle_height;
                s.bound_height = publication.bound_height;
                self.trace.publications.push(Publication {
                    pass_id: r.pass,
                    index: hit.global_index,
                    before,
                    after: snapshot(s, self.mesh),
                    backface: u8::from(out.backface),
                });
                if r.pass == 2 {
                    self.mesh.handles = Handles {
                        actor: Some(ActorHandle),
                        shape: Some(ShapeHandle),
                    };
                    let p = pose(self.scene.actor_pose, self.scene.shape_pose);
                    finite(p)?;
                    self.trace.transforms.push(p);
                    self.mesh.world_contact = hit.point.map(|x| x as f32);
                    self.mesh.local_contact = inverse_point(hit.point, p);
                    finite(
                        self.mesh
                            .world_contact
                            .into_iter()
                            .chain(self.mesh.local_contact),
                    )?;
                }
                if r.pass != 3 {
                    self.trace.report_metadata.push(ReportMetadata {
                        pass_id: r.pass,
                        cache_tag: s.tag,
                        global_index: hit.global_index,
                    });
                    self.trace.reports.push(Report {
                        point: hit.point,
                        normal: hit.normal,
                        direction,
                        distance,
                        source_id: hit.original_index,
                        record: hit.record,
                        owner_identity: 0x12345678,
                        shape: ShapeHandle,
                        actor: ActorHandle,
                    });
                    if r.pass == 2 && self.scene.behavior & 2 != 0 && count == 0 {
                        budget += 9;
                    }
                }
                count += 1;
                s.contact_height = xy(hit.point.map(|x| x as f32), up);
                finite([s.contact_height])?;
                if hit.distance > offset {
                    let advance = hit.distance - offset;
                    position =
                        std::array::from_fn(|i| position[i] + f64::from(direction[i] * advance));
                }
                let mut normal = hit.normal;
                if out.backface
                    || (s.flags & 2 != 0 && self.settings.scheduler.nonwalkable_mode != 1)
                {
                    let dot = xy(normal, up);
                    normal = unit(std::array::from_fn(|i| normal[i] - up[i] * dot));
                    finite(normal)?;
                }
                let normal_up = xy(hit.normal, up);
                finite([normal_up])?;
                let mut lifted = false;
                let custom = self.settings.custom_response;
                if custom
                    && s.flags & 2 != 0
                    && r.pass == 1
                    && normal_up > self.settings.scheduler.slope_limit
                {
                    let remain: Vec3 = std::array::from_fn(|i| (target[i] - position[i]) as f32);
                    let q = yx(remain, up);
                    let vertical = up.map(|u| q * u);
                    let room = self.settings.scheduler.step_offset - xy(vertical, vertical).sqrt();
                    finite(remain.into_iter().chain(vertical).chain([room]))?;
                    if room > 0. {
                        let tangent: Vec3 = std::array::from_fn(|i| remain[i] - vertical[i]);
                        let candidate =
                            (yx(tangent, tangent) * (1. - normal_up * normal_up)).sqrt();
                        finite([candidate])?;
                        let lift = if candidate < room { candidate } else { room };
                        target = std::array::from_fn(|i| target[i] + f64::from(lift * up[i]));
                        lifted = true;
                    }
                }
                if !lifted {
                    target = cct_response::response(
                        target,
                        position,
                        direction,
                        normal,
                        0.,
                        1.,
                        s.flags & 0x40 != 0,
                    );
                    if custom {
                        let away: Vec3 =
                            std::array::from_fn(|i| (position[i] - hit.point[i]) as f32);
                        let square = yx(away, away);
                        finite(away.into_iter().chain([square]))?;
                        let bias = if square > 0. {
                            let reciprocal = 1. / square.sqrt();
                            away.map(|x| reciprocal * x)
                        } else {
                            [0.; 3]
                        };
                        let q = yx(up, bias);
                        let bias = unit(std::array::from_fn(|i| bias[i] - up[i] * q));
                        finite(bias)?;
                        let scale = offset * 0.2_f32;
                        target = std::array::from_fn(|i| {
                            target[i] + f64::from((scale * bias[i]) * (repeated as f32))
                        });
                        let similarity = yx(bias, previous);
                        finite([similarity])?;
                        if similarity > 0.99_f32 {
                            repeated = repeated.wrapping_add(1);
                        }
                        previous = bias;
                    }
                }
                if !target.into_iter().chain(position).all(f64::is_finite) {
                    return Err(Error::NonFiniteArithmetic);
                }
            }
            s.position = position;
        }
        self.trace.iteration_semantics.push(IterationSemantics {
            counter: self.mesh.counter,
            obstacle: self.mesh.obstacle,
            local_contact: self.mesh.local_contact,
            world_contact: self.mesh.world_contact,
            handles: self.mesh.handles,
        });
        self.trace.returns.push(IterationReturn {
            pass_id: r.pass,
            attempted,
            count,
            state: snapshot(s, self.mesh),
        });
        if !self.direct {
            scheduler_arithmetic(s, self.settings.scheduler, self.scheduler_bottom)?;
        }
        Ok(cct_slope::Outcome {
            attempted,
            collisions: count,
        })
    }
}
impl cct_slope::Backend for Engine<'_, '_> {
    fn prepare(&mut self, s: &mut cct_slope::State, _: Vec3, _: Vec3) {
        self.scheduler_bottom = double_xy(self.settings.scheduler.up, s.position)
            - f64::from(self.settings.scheduler.extent);
        if self.error.is_none() {
            self.error =
                scheduler_arithmetic(s, self.settings.scheduler, self.scheduler_bottom).err();
        }
    }
    fn iterate(&mut self, s: &mut cct_slope::State, r: cct_slope::Iteration) -> cct_slope::Outcome {
        // cct_slope's bounded callback ABI has no Result. These provisional
        // outcomes are discarded on a latched error before any public commit.
        if self.error.is_some() {
            return cct_slope::Outcome {
                attempted: false,
                collisions: 0,
            };
        }
        match self.iteration(s, r) {
            Ok(out) => out,
            Err(e) => {
                self.error = Some(e);
                cct_slope::Outcome {
                    attempted: false,
                    collisions: 0,
                }
            }
        }
    }
}
fn validate(
    s: &State,
    common: &CommonPublication,
    scene: &PreparedScene<'_>,
    capsule: Capsule,
    settings: Settings,
    request: Request,
    admission: CommonAdmission,
) -> Result<(), Error> {
    if !state_finite(s)
        || !s.scheduler.position.into_iter().all(double_normal_or_zero)
        || s.scheduler.tag > 2
        || !scene.cache_origin.into_iter().all(double_normal_or_zero)
        || !scene.object_bounds.into_iter().all(normal_or_zero)
        || !scene
            .actor_pose
            .into_iter()
            .chain(scene.shape_pose)
            .all(bounded)
        || !scene
            .triangles
            .iter()
            .flatten()
            .flatten()
            .all(|x| bounded(*x))
        || scene.source_ids.len() != scene.triangles.len()
        || scene.records.is_empty()
        || scene.records.len() > 3
        || !capsule
            .rotation
            .into_iter()
            .chain([capsule.radius, capsule.height])
            .all(bounded)
        || capsule.radius < 0.
        || capsule.height < 0.
        || !settings
            .scheduler
            .up
            .into_iter()
            .chain([
                settings.scheduler.step_offset,
                settings.scheduler.min_distance,
                settings.scheduler.slope_limit,
                settings.scheduler.extent,
                settings.contact_offset,
            ])
            .all(bounded)
        || settings.contact_offset < 0.
        || settings.scheduler.step_offset < 0.
        || settings.scheduler.min_distance < 0.
    {
        return Err(Error::InvalidInput);
    }
    let q = capsule.rotation;
    let q_squared = ((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3];
    if (q_squared - 1.).abs() > 0.001 {
        return Err(Error::InvalidInput);
    }
    if settings.scheduler.standing || !settings.scheduler.slope_enabled {
        return Err(Error::UnsupportedDomain);
    }
    for record in scene.records {
        if record
            .start
            .checked_add(record.count)
            .is_none_or(|end| end as usize > scene.triangles.len())
            || !record.offset.into_iter().all(double_normal_or_zero)
        {
            return Err(Error::InvalidInput);
        }
    }
    let d = match request {
        Request::Iterate {
            displacement,
            side,
            budget,
            minimum,
            pass_id,
        } => {
            if budget > 10
                || pass_id > 3
                || !side.into_iter().chain([minimum]).all(bounded)
                || minimum < 0.
            {
                return Err(Error::InvalidInput);
            }
            displacement
        }
        Request::Split { displacement } => displacement,
        Request::Common { displacement, dt } => {
            if !normal_or_zero(dt)
                || !double_normal_or_zero(common.numeric.elapsed)
                || !common.numeric.pending.into_iter().all(bounded)
            {
                return Err(Error::InvalidInput);
            }
            if (matches!(admission, CommonAdmission::Base)
                && (common.handles.actor.is_some()
                    || common.handles.shape.is_some()
                    || s.mesh.handles.actor.is_some()
                    || s.mesh.handles.shape.is_some()))
                || s.mesh.obstacle != u32::MAX
            {
                return Err(Error::UnsupportedDomain);
            }
            displacement
        }
    };
    if !d.into_iter().all(bounded) {
        return Err(Error::InvalidInput);
    }
    if !matches!(request, Request::Iterate { .. }) {
        let bottom = double_xy(settings.scheduler.up, s.scheduler.position)
            - f64::from(settings.scheduler.extent);
        scheduler_arithmetic(&s.scheduler, settings.scheduler, bottom)?;
    }
    Ok(())
}
#[derive(Clone, Copy)]
enum CommonAdmission {
    Base,
    StaticPrefix,
}
#[derive(Clone, Debug)]
pub struct StaticCommonTrace {
    pub numerical: Trace,
    pub support_prefix: crate::mesh_static_support::Prefix,
}
/// Reviewed input-only static previous-contact prefix and prepared Common chain.
/// Persistent Common identities validate before distinct local output slots
/// initialize. Support/motion/state/publication all remain unchanged on failure.
pub fn run_static_common(
    state: &mut State,
    common: &mut CommonPublication,
    motion: &mut crate::mesh_static_support::Motion,
    scene: &PreparedScene<'_>,
    capsule: Capsule,
    settings: Settings,
    displacement: Vec3,
    dt: f32,
    support: crate::mesh_static_support::StaticSupport<'_>,
) -> Result<StaticCommonTrace, Error> {
    let request = Request::Common { displacement, dt };
    validate(
        state,
        common,
        scene,
        capsule,
        settings,
        request,
        CommonAdmission::StaticPrefix,
    )?;
    if !common.numeric.position.into_iter().all(f64::is_finite) {
        return Err(Error::InvalidInput);
    }
    let prefix = crate::mesh_static_support::prefix(
        support,
        common.handles,
        *motion,
        displacement,
        common.numeric.pending,
    )?;
    let mut next = *state;
    let mut next_common = *common;
    next_common.handles = prefix.handles;
    let next_motion = crate::mesh_static_support::Motion {
        velocity: prefix.velocity,
        moving: prefix.moving,
    };
    // Original Common141EE3DDA/141EE3E07 initializes local output pointers,
    // distinct from the C+130/C+118 pair already observed by the prefix above.
    next.mesh.handles = Handles {
        actor: None,
        shape: None,
    };
    let numerical = run_internal(
        &mut next,
        &mut next_common,
        scene,
        capsule,
        settings,
        request,
        CommonAdmission::StaticPrefix,
    )?;
    *state = next;
    *common = next_common;
    *motion = next_motion;
    Ok(StaticCommonTrace {
        numerical,
        support_prefix: prefix,
    })
}
/// Execute only the fixed prepared-static model domain. All caller-owned state
/// is staged and committed together on success. Native retry restores position
/// alone inside that temporary transaction; arbitrary previous Common handles
/// remain explicitly unsupported, independently from mesh output handles.
pub fn run(
    state: &mut State,
    common: &mut CommonPublication,
    scene: &PreparedScene<'_>,
    capsule: Capsule,
    settings: Settings,
    request: Request,
) -> Result<Trace, Error> {
    run_internal(
        state,
        common,
        scene,
        capsule,
        settings,
        request,
        CommonAdmission::Base,
    )
}
fn run_internal(
    state: &mut State,
    common: &mut CommonPublication,
    scene: &PreparedScene<'_>,
    capsule: Capsule,
    settings: Settings,
    request: Request,
    admission: CommonAdmission,
) -> Result<Trace, Error> {
    validate(state, common, scene, capsule, settings, request, admission)?;
    let mut next = *state;
    let mut next_common = *common;
    next.scheduler.collision_flags = 0;
    if matches!(request, Request::Common { .. }) {
        next.mesh.counter = 0;
    }
    let initial = snapshot(&next.scheduler, &next.mesh);
    let mut engine = Engine {
        mesh: &mut next.mesh,
        scene,
        capsule,
        settings,
        direct: matches!(request, Request::Iterate { .. }),
        scheduler_bottom: 0.,
        error: None,
        trace: Trace {
            state: initial,
            collision_flags: 0,
            iterations: Vec::new(),
            returns: Vec::new(),
            queries: Vec::new(),
            publications: Vec::new(),
            reports: Vec::new(),
            report_metadata: Vec::new(),
            transforms: Vec::new(),
            iteration_semantics: Vec::new(),
            common: None,
        },
    };
    match request {
        Request::Iterate {
            displacement,
            side,
            budget,
            minimum,
            pass_id,
        } => {
            engine.iteration(
                &mut next.scheduler,
                cct_slope::Iteration {
                    displacement,
                    side,
                    budget,
                    minimum,
                    pass: pass_id,
                },
            )?;
        }
        Request::Split { displacement } => cct_slope::split(
            &mut next.scheduler,
            &settings.scheduler,
            displacement,
            &mut engine,
        )
        .map_err(|_| Error::NonFiniteArithmetic)?,
        Request::Common { displacement, dt } => {
            cct_slope::common(
                &mut next.scheduler,
                &settings.scheduler,
                displacement,
                &mut next_common.numeric,
                dt,
                &mut engine,
            )
            .map_err(|_| Error::NonFiniteArithmetic)?;
            next_common.handles = engine.mesh.handles;
        }
    }
    if let Some(error) = engine.error {
        return Err(error);
    }
    engine.trace.state = snapshot(&next.scheduler, engine.mesh);
    engine.trace.collision_flags = next.scheduler.collision_flags;
    if matches!(request, Request::Common { .. }) {
        engine.trace.common = Some(next_common);
    }
    let trace = engine.trace;
    if !state_finite(&next) {
        return Err(Error::NonFiniteArithmetic);
    }
    *state = next;
    *common = next_common;
    Ok(trace)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn state() -> State {
        State {
            scheduler: cct_slope::State {
                position: [0., 2., 0.],
                flags: 0,
                tag: 0,
                down_normal: [0., 1., 0.],
                side_normal: [1., 0., 0.],
                triangle_height: 0.,
                bound_height: 0.,
                contact_height: 0.,
                collision_flags: 0,
            },
            mesh: MeshState {
                cache: [0; 3],
                triangle_min: -0.,
                counter: 65535,
                obstacle: 0,
                local_contact: [-0., 2., -3.],
                world_contact: [123., -45., 0.],
                handles: Handles {
                    actor: Some(ActorHandle),
                    shape: Some(ShapeHandle),
                },
            },
        }
    }
    fn settings() -> Settings {
        Settings {
            scheduler: cct_slope::Settings {
                up: [0., 1., 0.],
                step_offset: 0.3,
                min_distance: 1e-6,
                nonwalkable_mode: 0,
                standing: false,
                prevent_ceiling_slide: false,
                slope_enabled: true,
                slope_limit: 0.7,
                constrain: false,
                extent: 0.9,
            },
            contact_offset: 0.02,
            custom_response: true,
        }
    }
    fn capsule() -> Capsule {
        Capsule {
            radius: 0.3,
            height: 1.2,
            rotation: [
                0.,
                0.,
                std::f32::consts::FRAC_1_SQRT_2,
                std::f32::consts::FRAC_1_SQRT_2,
            ],
        }
    }
    fn common() -> CommonPublication {
        CommonPublication {
            numeric: cct_slope::Publication {
                elapsed: 12.125,
                pending: [0.; 3],
                position: [0.; 3],
                collision_flags: 0,
            },
            handles: Handles {
                actor: None,
                shape: None,
            },
        }
    }
    fn scene<'a>(t: &'a [Triangle], ids: &'a [u32], records: &'a [Record]) -> PreparedScene<'a> {
        PreparedScene {
            triangles: t,
            source_ids: ids,
            records,
            cache_origin: [0.; 3],
            object_bounds: [-10., -10., -10., 10., 10., 10.],
            actor_pose: [0., 0., 0., 1., 0., 0., 0.],
            shape_pose: [0., 0., 0., 1., 0., 0., 0.],
            behavior: 0,
        }
    }
    fn state_bits(s: &State) -> Vec<u64> {
        let mut v = s.scheduler.position.map(f64::to_bits).to_vec();
        v.extend(
            s.scheduler
                .down_normal
                .into_iter()
                .chain(s.scheduler.side_normal)
                .chain([
                    s.scheduler.triangle_height,
                    s.scheduler.bound_height,
                    s.scheduler.contact_height,
                    s.mesh.triangle_min,
                ])
                .chain(s.mesh.local_contact)
                .chain(s.mesh.world_contact)
                .map(|x| u64::from(x.to_bits())),
        );
        v.extend([
            u64::from(s.scheduler.flags),
            u64::from(s.scheduler.tag),
            u64::from(s.scheduler.collision_flags),
            u64::from(s.mesh.counter),
            u64::from(s.mesh.obstacle),
            u64::from(s.mesh.handles.actor.is_some()),
            u64::from(s.mesh.handles.shape.is_some()),
        ]);
        v.extend(s.mesh.cache.map(u64::from));
        v
    }
    #[test]
    fn retained_counter_and_zero_request_handles_are_distinct_from_down_setup() {
        let rec = [Record {
            start: 0,
            count: 0,
            offset: [0.; 3],
        }];
        let sc = scene(&[], &[], &rec);
        let mut s = state();
        let mut c = common();
        let before = state_bits(&s);
        let out = run(
            &mut s,
            &mut c,
            &sc,
            capsule(),
            settings(),
            Request::Iterate {
                displacement: [0.; 3],
                side: [0.; 3],
                budget: 10,
                minimum: 1e-6,
                pass_id: 2,
            },
        )
        .unwrap();
        assert_eq!(state_bits(&s), before);
        assert_eq!(out.iteration_semantics[0].counter, 65535);
        assert!(!out.returns[0].attempted);
        run(
            &mut s,
            &mut c,
            &sc,
            capsule(),
            settings(),
            Request::Iterate {
                displacement: [1e-7, 0., 0.],
                side: [0.; 3],
                budget: 1,
                minimum: 1e-6,
                pass_id: 1,
            },
        )
        .unwrap();
        assert_eq!(s.mesh.counter, 0);
        assert_eq!(
            s.mesh.handles,
            Handles {
                actor: None,
                shape: None
            }
        );
        assert_eq!(s.mesh.obstacle, u32::MAX);
    }
    #[test]
    fn common_prior_publication_handles_cannot_bypass_mesh_output_gate() {
        let rec = [Record {
            start: 0,
            count: 0,
            offset: [0.; 3],
        }];
        let sc = scene(&[], &[], &rec);
        let mut s = state();
        let mut c = common();
        s.mesh.handles = Handles {
            actor: None,
            shape: None,
        };
        s.mesh.obstacle = u32::MAX;
        c.handles.actor = Some(ActorHandle);
        let before = state_bits(&s);
        let old = c;
        assert!(matches!(
            run(
                &mut s,
                &mut c,
                &sc,
                capsule(),
                settings(),
                Request::Common {
                    displacement: [0.; 3],
                    dt: 0.016
                }
            ),
            Err(Error::UnsupportedDomain)
        ));
        assert_eq!(state_bits(&s), before);
        assert_eq!(c, old);
    }
    #[test]
    fn retained_projection_overflow_and_subnormal_fail_atomically_without_height_cap() {
        let rec = [Record {
            start: 0,
            count: 0,
            offset: [0.; 3],
        }];
        let sc = scene(&[], &[], &rec);
        let mut s = state();
        let mut c = common();
        s.scheduler.down_normal = [f32::MAX; 3];
        s.scheduler.flags = 4;
        let mut cfg = settings();
        cfg.scheduler.up = [1.; 3];
        let before = state_bits(&s);
        let old = c;
        assert!(matches!(
            run(
                &mut s,
                &mut c,
                &sc,
                capsule(),
                cfg,
                Request::Split {
                    displacement: [0.; 3]
                }
            ),
            Err(Error::NonFiniteArithmetic)
        ));
        assert_eq!(state_bits(&s), before);
        assert_eq!(c, old);
        s = state();
        s.scheduler.side_normal[0] = f32::from_bits(1);
        let before = state_bits(&s);
        assert!(matches!(
            run(
                &mut s,
                &mut c,
                &sc,
                capsule(),
                settings(),
                Request::Split {
                    displacement: [0.; 3]
                }
            ),
            Err(Error::InvalidInput)
        ));
        assert_eq!(state_bits(&s), before);
        s = state();
        s.scheduler.triangle_height = 1e6;
        s.scheduler.bound_height = 1e6;
        s.mesh.world_contact = [1e6; 3];
        assert!(
            run(
                &mut s,
                &mut c,
                &sc,
                capsule(),
                settings(),
                Request::Split {
                    displacement: [0.; 3]
                }
            )
            .is_ok()
        );
    }
    #[test]
    fn publication_failure_after_real_query_does_not_commit_cache_or_trace_state() {
        let t = [[[-10., 0., -10.], [0., 0., 10.], [10., 0., -10.]]];
        let rec = [Record {
            start: 0,
            count: 1,
            offset: [0.; 3],
        }];
        let mut sc = scene(&t, &[1001], &rec);
        sc.object_bounds = [f32::MAX; 6];
        let mut s = state();
        s.mesh.cache = [123; 3];
        let before = state_bits(&s);
        let mut c = common();
        let old = c;
        let mut cfg = settings();
        cfg.scheduler.up = [1.; 3];
        let out = run(
            &mut s,
            &mut c,
            &sc,
            capsule(),
            cfg,
            Request::Iterate {
                displacement: [0., -3., 0.],
                side: [0.; 3],
                budget: 10,
                minimum: 1e-6,
                pass_id: 2,
            },
        );
        assert!(matches!(out, Err(Error::NonFiniteArithmetic)));
        assert_eq!(state_bits(&s), before);
        assert_eq!(c, old);
    }
    fn common_bits(c: &CommonPublication) -> Vec<u64> {
        let mut v = c.numeric.position.map(f64::to_bits).to_vec();
        v.push(c.numeric.elapsed.to_bits());
        v.extend(c.numeric.pending.map(|x| u64::from(x.to_bits())));
        v.extend([
            u64::from(c.numeric.collision_flags),
            u64::from(c.handles.actor.is_some()),
            u64::from(c.handles.shape.is_some()),
        ]);
        v
    }
    #[test]
    fn static_common_prefix_retains_persistent_pair_before_local_outputs_initialize() {
        use crate::mesh_static_support as p;
        let memberships = [p::ShapeMembership::PreviousShape];
        let support = p::StaticSupport {
            shapes: &memberships,
            scene_state: p::SceneState::State1,
            scene_null: false,
            scene_other: false,
            buffer_bits: 0,
            flags: 2,
            buffer_flags: 0,
        };
        let records = [Record {
            start: 0,
            count: 0,
            offset: [0.; 3],
        }];
        let sc = scene(&[], &[], &records);
        let mut s = state();
        s.mesh.obstacle = u32::MAX;
        let mut c = common();
        c.handles = s.mesh.handles;
        let persistent = c.handles;
        let mut motion = p::Motion {
            velocity: [-0., 7., -8.],
            moving: 255,
        };
        let out = run_static_common(
            &mut s,
            &mut c,
            &mut motion,
            &sc,
            capsule(),
            settings(),
            [0.; 3],
            0.016,
            support,
        )
        .unwrap();
        assert_eq!(out.support_prefix.handles, persistent);
        assert_eq!(
            out.support_prefix.events.last(),
            Some(&p::Event::StaticMotion)
        );
        assert_eq!(
            c.handles,
            Handles {
                actor: None,
                shape: None
            }
        );
        assert_eq!(s.mesh.handles, c.handles);
        assert_eq!(motion.velocity.map(f32::to_bits), [0; 3]);
        assert_eq!(motion.moving, 0);
        assert_eq!(memberships, [p::ShapeMembership::PreviousShape]);
    }
    #[test]
    fn static_common_failed_publication_preserves_motion_state_and_persistent_pair_bits() {
        use crate::mesh_static_support as p;
        let memberships = [p::ShapeMembership::PreviousShape];
        let support = p::StaticSupport {
            shapes: &memberships,
            scene_state: p::SceneState::State1,
            scene_null: false,
            scene_other: false,
            buffer_bits: 64,
            flags: 0,
            buffer_flags: 2,
        };
        let t = [[[-10., 0., -10.], [0., 0., 10.], [10., 0., -10.]]];
        let records = [Record {
            start: 0,
            count: 1,
            offset: [0.; 3],
        }];
        let mut sc = scene(&t, &[1001], &records);
        sc.object_bounds = [f32::MAX; 6];
        let mut s = state();
        s.mesh.obstacle = u32::MAX;
        s.mesh.cache = [123; 3];
        let mut c = common();
        c.handles = s.mesh.handles;
        c.numeric.pending = [-0.; 3];
        c.numeric.position = [-0., 123., -456.];
        let mut motion = p::Motion {
            velocity: [-0., 7., -8.],
            moving: 255,
        };
        let before_state = state_bits(&s);
        let before_common = common_bits(&c);
        let before_motion = (motion.velocity.map(f32::to_bits), motion.moving);
        let mut cfg = settings();
        cfg.scheduler.up = [1.; 3];
        let mut control_scene = scene(&t, &[1001], &records);
        control_scene.object_bounds = [-10., -10., -10., 10., 10., 10.];
        let (mut control_state, mut control_common, mut control_motion) = (s, c, motion);
        let control = run_static_common(
            &mut control_state,
            &mut control_common,
            &mut control_motion,
            &control_scene,
            capsule(),
            cfg,
            [0., -3., 0.],
            0.016,
            support,
        )
        .unwrap();
        assert!(
            control
                .numerical
                .queries
                .iter()
                .any(|q| q.result.hit.is_some())
        );
        assert!(!control.numerical.publications.is_empty());
        let error = run_static_common(
            &mut s,
            &mut c,
            &mut motion,
            &sc,
            capsule(),
            cfg,
            [0., -3., 0.],
            0.016,
            support,
        )
        .unwrap_err();
        assert_eq!(error, Error::NonFiniteArithmetic);
        assert_eq!(state_bits(&s), before_state);
        assert_eq!(common_bits(&c), before_common);
        assert_eq!(
            (motion.velocity.map(f32::to_bits), motion.moving),
            before_motion
        );
        assert_eq!(memberships, [p::ShapeMembership::PreviousShape]);
        // Initial malformed support/motion and previous-obstacle domain also
        // reject before any staged prefix can become a public state change.
        motion.velocity[0] = f32::from_bits(1);
        assert!(matches!(
            run_static_common(
                &mut s,
                &mut c,
                &mut motion,
                &sc,
                capsule(),
                settings(),
                [0.; 3],
                0.016,
                support
            ),
            Err(Error::InvalidInput)
        ));
        assert_eq!(state_bits(&s), before_state);
        assert_eq!(common_bits(&c), before_common);
        motion.velocity[0] = -0.;
        s.mesh.obstacle = 1;
        let before_state = state_bits(&s);
        assert!(matches!(
            run_static_common(
                &mut s,
                &mut c,
                &mut motion,
                &sc,
                capsule(),
                settings(),
                [0.; 3],
                0.016,
                support
            ),
            Err(Error::UnsupportedDomain)
        ));
        assert_eq!(state_bits(&s), before_state);
        assert_eq!(common_bits(&c), before_common);
    }
    #[test]
    fn static_common_second_call_uses_first_landing_persistent_and_local_outputs() {
        use crate::mesh_static_support as p;
        let memberships = [p::ShapeMembership::PreviousShape];
        let support = p::StaticSupport {
            shapes: &memberships,
            scene_state: p::SceneState::State1,
            scene_null: false,
            scene_other: false,
            buffer_bits: 0,
            flags: 2,
            buffer_flags: 0,
        };
        let t = [[[-10., 0., -10.], [0., 0., 10.], [10., 0., -10.]]];
        let records = [Record {
            start: 0,
            count: 1,
            offset: [0.; 3],
        }];
        let sc = scene(&t, &[1001], &records);
        let mut s = state();
        s.mesh.obstacle = u32::MAX;
        let mut c = common();
        let mut motion = p::Motion {
            velocity: [0.; 3],
            moving: 0,
        };
        let first = run_static_common(
            &mut s,
            &mut c,
            &mut motion,
            &sc,
            capsule(),
            settings(),
            [0., -3., 0.],
            0.016,
            support,
        )
        .unwrap();
        assert!(
            first
                .numerical
                .queries
                .iter()
                .any(|q| q.result.hit.is_some())
        );
        let persistent = Handles {
            actor: Some(ActorHandle),
            shape: Some(ShapeHandle),
        };
        assert_eq!(c.handles, persistent);
        assert_eq!(s.mesh.handles, persistent);
        let (mut base_state, mut base_common) = (s, c);
        assert!(matches!(
            run(
                &mut base_state,
                &mut base_common,
                &sc,
                capsule(),
                settings(),
                Request::Common {
                    displacement: [0.; 3],
                    dt: 0.016
                }
            ),
            Err(Error::UnsupportedDomain)
        ));
        let second = run_static_common(
            &mut s,
            &mut c,
            &mut motion,
            &sc,
            capsule(),
            settings(),
            [0.; 3],
            0.016,
            support,
        )
        .unwrap();
        assert_eq!(second.support_prefix.handles, persistent);
        assert_eq!(
            second.support_prefix.events.last(),
            Some(&p::Event::StaticMotion)
        );
        assert_eq!(memberships, [p::ShapeMembership::PreviousShape]);
    }
    #[test]
    fn subnormal_common_dt_rejects_atomically_before_static_prefix_or_base_scheduler() {
        use crate::mesh_static_support as p;
        let memberships = [p::ShapeMembership::PreviousShape];
        let support = p::StaticSupport {
            shapes: &memberships,
            scene_state: p::SceneState::State1,
            scene_null: false,
            scene_other: false,
            buffer_bits: 0,
            flags: 2,
            buffer_flags: 0,
        };
        let records = [Record {
            start: 0,
            count: 0,
            offset: [0.; 3],
        }];
        let sc = scene(&[], &[], &records);
        let mut s = state();
        s.mesh.obstacle = u32::MAX;
        let mut c = common();
        c.handles = s.mesh.handles;
        c.numeric.pending = [-0.; 3];
        let mut motion = p::Motion {
            velocity: [-0., 7., -8.],
            moving: 255,
        };
        let before_state = state_bits(&s);
        let before_common = common_bits(&c);
        let before_motion = (motion.velocity.map(f32::to_bits), motion.moving);
        assert!(matches!(
            run_static_common(
                &mut s,
                &mut c,
                &mut motion,
                &sc,
                capsule(),
                settings(),
                [0.; 3],
                f32::from_bits(1),
                support
            ),
            Err(Error::InvalidInput)
        ));
        assert_eq!(state_bits(&s), before_state);
        assert_eq!(common_bits(&c), before_common);
        assert_eq!(
            (motion.velocity.map(f32::to_bits), motion.moving),
            before_motion
        );
        s.mesh.handles = Handles {
            actor: None,
            shape: None,
        };
        c.handles = s.mesh.handles;
        let before_state = state_bits(&s);
        let before_common = common_bits(&c);
        assert!(matches!(
            run(
                &mut s,
                &mut c,
                &sc,
                capsule(),
                settings(),
                Request::Common {
                    displacement: [0.; 3],
                    dt: f32::from_bits(1)
                }
            ),
            Err(Error::InvalidInput)
        ));
        assert_eq!(state_bits(&s), before_state);
        assert_eq!(common_bits(&c), before_common);
        // Negative normal dt and signed zero retain their existing admission.
        assert!(
            run(
                &mut s,
                &mut c,
                &sc,
                capsule(),
                settings(),
                Request::Common {
                    displacement: [0.; 3],
                    dt: -0.016
                }
            )
            .is_ok()
        );
        assert!(
            run(
                &mut s,
                &mut c,
                &sc,
                capsule(),
                settings(),
                Request::Common {
                    displacement: [0.; 3],
                    dt: -0.
                }
            )
            .is_ok()
        );
        assert_eq!(memberships, [p::ShapeMembership::PreviousShape]);
    }
    #[test]
    fn subnormal_arithmetic_double_inputs_reject_atomically_without_coordinate_caps() {
        use crate::mesh_static_support as p;
        let memberships = [p::ShapeMembership::PreviousShape];
        let support = p::StaticSupport {
            shapes: &memberships,
            scene_state: p::SceneState::State1,
            scene_null: false,
            scene_other: false,
            buffer_bits: 0,
            flags: 2,
            buffer_flags: 0,
        };
        for lane in 0..4 {
            let mut records = [Record {
                start: 0,
                count: 0,
                offset: [0.; 3],
            }];
            if lane == 2 {
                records[0].offset[0] = f64::from_bits(1);
            }
            let mut sc = scene(&[], &[], &records);
            if lane == 1 {
                sc.cache_origin[0] = f64::from_bits(1);
            }
            let mut s = state();
            s.mesh.obstacle = u32::MAX;
            if lane == 0 {
                s.scheduler.position[0] = f64::from_bits(1);
            }
            let mut c = common();
            c.handles = s.mesh.handles;
            if lane == 3 {
                c.numeric.elapsed = f64::from_bits(1);
            }
            let mut motion = p::Motion {
                velocity: [-0., 7., -8.],
                moving: 255,
            };
            let before_state = state_bits(&s);
            let before_common = common_bits(&c);
            let before_motion = (motion.velocity.map(f32::to_bits), motion.moving);
            assert!(matches!(
                run_static_common(
                    &mut s,
                    &mut c,
                    &mut motion,
                    &sc,
                    capsule(),
                    settings(),
                    [0.; 3],
                    0.016,
                    support
                ),
                Err(Error::InvalidInput)
            ));
            assert_eq!(state_bits(&s), before_state);
            assert_eq!(common_bits(&c), before_common);
            assert_eq!(
                (motion.velocity.map(f32::to_bits), motion.moving),
                before_motion
            );
            s.mesh.handles = Handles {
                actor: None,
                shape: None,
            };
            c.handles = s.mesh.handles;
            let before_state = state_bits(&s);
            let before_common = common_bits(&c);
            assert!(matches!(
                run(
                    &mut s,
                    &mut c,
                    &sc,
                    capsule(),
                    settings(),
                    Request::Common {
                        displacement: [0.; 3],
                        dt: 0.016
                    }
                ),
                Err(Error::InvalidInput)
            ));
            assert_eq!(state_bits(&s), before_state);
            assert_eq!(common_bits(&c), before_common);
        }
        let records = [Record {
            start: 0,
            count: 0,
            offset: [1e200, -0., 0.],
        }];
        let mut sc = scene(&[], &[], &records);
        sc.cache_origin = [1e200, -0., 0.];
        let mut s = state();
        s.mesh.obstacle = u32::MAX;
        s.scheduler.position[0] = 1e200;
        let mut c = common();
        c.handles = s.mesh.handles;
        c.numeric.elapsed = -12.125;
        // Old output-only publication position is finite transport, not a consumed
        // arithmetic input; it is replaced with the final Common position.
        c.numeric.position = [f64::from_bits(1), -0., 0.];
        let mut motion = p::Motion {
            velocity: [-0., 7., -8.],
            moving: 255,
        };
        assert!(
            run_static_common(
                &mut s,
                &mut c,
                &mut motion,
                &sc,
                capsule(),
                settings(),
                [0.; 3],
                -0.,
                support
            )
            .is_ok()
        );
        assert_eq!(s.scheduler.position[0].to_bits(), 1e200_f64.to_bits());
        assert_eq!(memberships, [p::ShapeMembership::PreviousShape]);
    }
}
