//! LAB engine bridge to the reviewed immutable static Common mesh solver.
//! Full dt is forwarded for every reached Common entry. World owns the outer
//! transaction; internal native Common retry retains position-only rollback.
use crate::{
 cct_scene::Capsule, cct_slope,
 contact_finalizer::{self,FinalizerGlobals,FinalizerState,SecondaryMove},
 engine_move::{EngineCallbacks,EngineMoveGlobals,EngineMoveState},
 engine_scene::AdjustmentContext,
 mesh_iteration::{self,CommonPublication,Handles,MeshState,Report,ReportMetadata,State},
 mesh_static_support::{Motion,SceneState,ShapeMembership,StaticSupport},
 request_adjustment::{self,AdjustmentInput},static_terrain::{StaticTerrain,TerrainError},
};
use serde::{Serialize,Deserialize};
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum MeshSceneError {Domain,Identity,Geometry,Terrain(TerrainError),Numerical(mesh_iteration::Error),Receiver}
impl std::fmt::Display for MeshSceneError {fn fmt(&self,f:&mut std::fmt::Formatter<'_>)->std::fmt::Result {write!(f,"{self:?}")}}
impl std::error::Error for MeshSceneError {}
#[derive(Clone,Debug,Serialize,Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeshRuntime {
 pub geometry_revision:u64,pub geometry_identity:Vec<u32>,pub capsule_radius:f32,pub cylinder_height:f32,pub capsule_rotation:[f32;4],
 pub state:State,pub common:CommonPublication,pub motion:Motion,
}
fn f(x:f32)->bool {x.is_finite()&&(x==0.||x.is_normal())&&x.abs()<=10000.}
fn d(x:f64)->bool {x.is_finite()&&(x==0.||x.is_normal())}
impl MeshRuntime {
 /// Authored LAB initialization. Invalid cache indices request the reviewed
 /// no-cache search. These values are not recovered controller constructors.
 pub fn new(terrain:&StaticTerrain,center:[f64;3],capsule:Capsule)->Result<Self,MeshSceneError> {
  terrain.validate().map_err(MeshSceneError::Terrain)?;
  let none=Handles{actor:None,shape:None};
  let runtime=Self{geometry_revision:terrain.revision(),geometry_identity:terrain.identity_words(),capsule_radius:capsule.radius,cylinder_height:capsule.height,capsule_rotation:capsule.rotation,
   state:State{scheduler:cct_slope::State{position:center,flags:0,tag:0,down_normal:[0.;3],side_normal:[0.;3],triangle_height:0.,bound_height:0.,contact_height:0.,collision_flags:0},
    mesh:MeshState{cache:[u32::MAX;3],triangle_min:0.,counter:0,obstacle:u32::MAX,local_contact:[0.;3],world_contact:[0.;3],handles:none}},
   common:CommonPublication{numeric:cct_slope::Publication{elapsed:0.,pending:[0.;3],position:center,collision_flags:0},handles:none},motion:Motion{velocity:[0.;3],moving:0}};
  runtime.validate(terrain,capsule,center)?;Ok(runtime)
 }
 pub fn validate(&self,terrain:&StaticTerrain,capsule:Capsule,center:[f64;3])->Result<(),MeshSceneError> {
  if self.geometry_revision!=terrain.revision()||self.geometry_identity!=terrain.identity_words()||self.capsule_radius.to_bits()!=capsule.radius.to_bits()||self.cylinder_height.to_bits()!=capsule.height.to_bits()||self.capsule_rotation.map(f32::to_bits)!=capsule.rotation.map(f32::to_bits) {return Err(MeshSceneError::Identity);}
  let s=&self.state.scheduler;let m=&self.state.mesh;let c=&self.common.numeric;
  if s.position.map(f64::to_bits)!=center.map(f64::to_bits)||!center.into_iter().all(d)||!c.position.into_iter().all(d)||!d(c.elapsed)||c.elapsed<0.
   ||s.tag>2||m.obstacle!=u32::MAX||!capsule.rotation.into_iter().chain([capsule.radius,capsule.height]).all(f)||capsule.radius<=0.||capsule.height<0.
   ||!s.down_normal.into_iter().chain(s.side_normal).chain([s.triangle_height,s.bound_height,s.contact_height,m.triangle_min]).chain(m.local_contact).chain(m.world_contact).chain(c.pending).chain(self.motion.velocity).all(f)
   ||[m.handles,self.common.handles].into_iter().any(|h|h.actor.is_none()&&h.shape.is_some()) {return Err(MeshSceneError::Domain);}
  let q=capsule.rotation;let length=((q[0]*q[0]+q[1]*q[1])+q[2]*q[2])+q[3]*q[3];
  if (length-1.).abs()>0.001 {return Err(MeshSceneError::Domain);}Ok(())
 }
}
#[derive(Clone,Debug)]
pub struct TypedMeshContact {pub report:Report,pub metadata:ReportMetadata}
pub struct EngineMeshScene<'a> {
 pub terrain:&'a StaticTerrain,pub runtime:MeshRuntime,pub capsule:Capsule,pub settings:mesh_iteration::Settings,
 pub finalizer_globals:FinalizerGlobals,pub slope_cosine:f32,pub split_vertical:bool,pub bridge_projection:[f32;4],
 pub adjustment:AdjustmentContext,pub metadata:bool,pub contacts:Vec<TypedMeshContact>,pub error:Option<MeshSceneError>,
 pub native_moves:u32,pub dispatch_dt_bits:Vec<u32>,pub behavior:u8,
}
impl<'a> EngineMeshScene<'a> {
 pub fn new(terrain:&'a StaticTerrain,runtime:MeshRuntime,capsule:Capsule,settings:mesh_iteration::Settings,finalizer_globals:FinalizerGlobals,slope_cosine:f32,adjustment:AdjustmentContext)->Result<Self,MeshSceneError> {
  runtime.validate(terrain,capsule,runtime.state.scheduler.position)?;
        let c = settings.scheduler;
        if !c.up.into_iter().chain([c.step_offset,c.min_distance,c.slope_limit,c.extent,settings.contact_offset]).all(f)
            || c.step_offset < 0. || c.min_distance < 0. || settings.contact_offset < 0.
            || c.standing || !c.slope_enabled {
            return Err(MeshSceneError::Domain);
        }

  if !slope_cosine.is_finite()||!(-1.0..=1.0).contains(&slope_cosine) {return Err(MeshSceneError::Domain);}
  Ok(Self{terrain,runtime,capsule,settings,finalizer_globals,slope_cosine,split_vertical:false,bridge_projection:[1.,0.,1.,0.],adjustment,metadata:false,contacts:Vec::new(),error:None,native_moves:0,dispatch_dt_bits:Vec::new(),behavior:0})
 }
 pub fn center(&self)->[f64;3] {self.runtime.state.scheduler.position}
 fn native_move(&mut self,receiver:&mut FinalizerState,request:[f32;3],dt:f32)->u8 {
  if self.error.is_some() {return 0;}
  let scene=self.terrain.prepared_scene(self.behavior);let shapes=[ShapeMembership::PreviousShape];
  let support=StaticSupport{shapes:&shapes,scene_state:SceneState::State1,scene_null:false,scene_other:false,buffer_bits:0,flags:2,buffer_flags:2};
  self.native_moves+=1;self.dispatch_dt_bits.push(dt.to_bits());
  let trace=match mesh_iteration::run_static_common(&mut self.runtime.state,&mut self.runtime.common,&mut self.runtime.motion,&scene,self.capsule,self.settings,request,dt,support) {
   Ok(t)=>t.numerical,Err(e)=>{self.error=Some(MeshSceneError::Numerical(e));return 0;}
  };
  if trace.reports.len()!=trace.report_metadata.len() {self.error=Some(MeshSceneError::Receiver);return 0;}
  for (report,metadata) in trace.reports.into_iter().zip(trace.report_metadata) {
   if metadata.pass_id==3||metadata.cache_tag>2||report.record!=0||metadata.global_index as usize>=self.terrain.triangles().len()
    ||self.terrain.source_ids()[metadata.global_index as usize]!=report.source_id
    ||!report.point.into_iter().map(|x|x as f32).chain(report.normal).all(f32::is_finite) {
    self.error=Some(MeshSceneError::Receiver);return 0;
   }
   receiver.reports.add_shape(report.point,report.normal,self.metadata);self.contacts.push(TypedMeshContact{report,metadata});
  }
  trace.collision_flags
 }
 fn bridge(&mut self,state:&mut FinalizerState,request:[f32;4],dt:f32)->[f32;4] {
        if !request.into_iter().all(f32::is_finite) || !dt.is_finite() || (dt != 0. && !dt.is_normal()) {
            self.error = Some(MeshSceneError::Domain);
            return [0.; 4];
        }
  let before=self.center();
  if request[..3].iter().any(|x|x.abs()>1e-6) {
   if self.split_vertical {
    state.mask=u32::from(self.native_move(state,[0.,request[1],0.],dt));
    let horizontal=std::array::from_fn::<_,3,_>(|i|request[i]*self.bridge_projection[i]);
    if horizontal.into_iter().any(|x|x.abs()>1e-6)&&self.error.is_none() {state.mask|=u32::from(self.native_move(state,horizontal,dt));}
   }else {state.mask=u32::from(self.native_move(state,[request[0],request[1],request[2]],dt));}
  }
  std::array::from_fn(|i|if i<3 {(self.center()[i]-before[i])as f32}else {0.})
 }
}
impl EngineCallbacks for EngineMeshScene<'_> {
 fn optional_adjustment(&mut self,state:&mut EngineMoveState,_:&mut EngineMoveGlobals,request:&mut [f32;4],dt:f32) {
  if self.error.is_some(){return;}
  match request_adjustment::adjust(AdjustmentInput{request:*request,dt,value_3c8:self.adjustment.value_3c8,normal:request_adjustment::select_normal(&state.contact.reports,self.adjustment.default_normal),cached_lower:self.adjustment.cached_lower,upper:self.adjustment.upper,projection_mask:self.adjustment.projection_mask,split_vertical:state.split_vertical}) {
   Ok(a)=>{*request=a.request;state.split_vertical=a.split_vertical;},Err(_)=>self.error=Some(MeshSceneError::Domain),
  }
 }
 fn move_request(&mut self,state:&mut EngineMoveState,_:&mut EngineMoveGlobals,request:&mut [f32;4],dt:f32)->[f32;4] {self.split_vertical=state.split_vertical!=0;self.bridge(&mut state.contact,*request,dt)}
 fn debug_observer(&mut self,_:&mut EngineMoveState,_:&mut EngineMoveGlobals,_:&mut [f32;4],_:[f32;4],_:f32) {}
 fn finalize(&mut self,state:&mut EngineMoveState,_:&mut EngineMoveGlobals,_:&mut [f32;4],dt:f32) {
  if self.error.is_some(){return;}
  self.split_vertical=state.split_vertical!=0;let mut globals=self.finalizer_globals;
  contact_finalizer::finalize(&mut state.contact,&mut globals,self.slope_cosine,dt,self);self.finalizer_globals=globals;
 }
}
impl SecondaryMove for EngineMeshScene<'_> {
 fn set_step(&mut self,value:f32,_:&mut FinalizerState,_:&mut FinalizerGlobals) {self.settings.scheduler.step_offset=value;}
 fn move_request(&mut self,request:[f32;4],dt:f32,state:&mut FinalizerState,_:&mut FinalizerGlobals)->[f32;4] {self.bridge(state,request,dt)}
}

#[cfg(test)]
mod tests {
 use super::*;
 fn capsule()->Capsule {Capsule{radius:0.3,height:1.2,rotation:[0.,0.,std::f32::consts::FRAC_1_SQRT_2,std::f32::consts::FRAC_1_SQRT_2]}}
 fn settings()->mesh_iteration::Settings {mesh_iteration::Settings{scheduler:cct_slope::Settings{up:[0.,1.,0.],step_offset:0.3,min_distance:1e-6,nonwalkable_mode:0,standing:false,prevent_ceiling_slide:false,slope_enabled:true,slope_limit:0.7,constrain:false,extent:0.9},contact_offset:0.02,custom_response:true}}
 fn globals()->FinalizerGlobals {FinalizerGlobals{zero:[0.;4],up:[0.,1.,0.,0.],fallback:[1.,0.,0.,0.]}}
 fn receiver()->FinalizerState {FinalizerState{reports:crate::contact_reports::ContactReports::default(),working_normal:[0.;4],other_normal:[0.;4],normal_y:0.,cached_y:0.,defer:false,mask:0,suppress:false,scalar:-1.,age:0.3,timer:0.3,position_y:1.1,step:0.3,late_flag:false,native_controller_present:true}}
 fn scene(t:&StaticTerrain)->EngineMeshScene<'_> {EngineMeshScene::new(t,MeshRuntime::new(t,[0.,2.,8.],capsule()).unwrap(),capsule(),settings(),globals(),0.7,AdjustmentContext{value_3c8:-1.,cached_lower:-0.866,upper:-0.0087,projection_mask:[1.,0.,1.,0.],default_normal:[0.;4]}).unwrap()}
 #[test] fn landing_support_survives_actual_next_call_and_serde_roundtrip() {
  let terrain=StaticTerrain::course();let mut s=scene(&terrain);let mut receiver=receiver();receiver.reports.gap_7e0=[0xa5;16];let dt=1./120.;
  s.bridge(&mut receiver,[0.,-4.,0.,0.],dt);assert_eq!(s.error,None);
  assert!(s.runtime.common.handles.shape.is_some());assert!(s.runtime.state.mesh.handles.shape.is_some());
  let before=serde_json::to_string(&s.runtime).unwrap();s.runtime=serde_json::from_str(&before).unwrap();
  assert_eq!(serde_json::to_string(&s.runtime).unwrap(),before);
  s.bridge(&mut receiver,[0.25,-0.01,0.,0.],dt);assert_eq!(s.error,None);
  assert!(s.runtime.common.handles.shape.is_some());assert_eq!(receiver.reports.gap_7e0,[0xa5;16]);
  assert_eq!(s.dispatch_dt_bits,vec![dt.to_bits();2]);assert_eq!(s.runtime.common.numeric.elapsed,(f64::from(dt)+f64::from(dt)));
  assert!(s.contacts.iter().all(|c|c.metadata.pass_id!=3 && terrain.source_ids()[c.metadata.global_index as usize]==c.report.source_id));
 }
 #[test] fn split_passes_forward_full_dt_and_insignificant_request_skips() {
  let terrain=StaticTerrain::course();let mut s=scene(&terrain);let mut receiver=receiver();s.split_vertical=true;let dt=1./120.;
  s.bridge(&mut receiver,[0.1,-0.1,0.,0.],dt);assert_eq!(s.error,None);assert_eq!(s.native_moves,2);assert_eq!(s.dispatch_dt_bits,vec![dt.to_bits();2]);
  let elapsed=s.runtime.common.numeric.elapsed;let mask=receiver.mask;
  s.bridge(&mut receiver,[1e-7,0.,0.,0.],dt);assert_eq!(s.native_moves,2);assert_eq!(receiver.mask,mask);assert_eq!(s.runtime.common.numeric.elapsed.to_bits(),elapsed.to_bits());
 }
 #[test] fn actual_finalizer_secondary_common_receives_dt_with_zero_step() {
  let terrain=StaticTerrain::course();let mut s=scene(&terrain);let mut receiver=receiver();
  receiver.reports.add_shape([0.,1.,8.],[0.6,0.8,0.],false);receiver.mask=4;receiver.scalar=-10.;
  let dt=0.1;let mut globals=globals();contact_finalizer::finalize(&mut receiver,&mut globals,0.9,dt,&mut s);
  assert_eq!(s.error,None);assert_eq!(s.native_moves,1);assert_eq!(s.dispatch_dt_bits,[dt.to_bits()]);assert_eq!(s.runtime.common.numeric.elapsed,f64::from(dt));assert_eq!(s.settings.scheduler.step_offset,receiver.step);
 }
 #[test] fn required_handle_roles_do_not_silently_reset_on_missing_keys() {
  let handles=Handles{actor:Some(crate::mesh_iteration::ActorHandle),shape:Some(crate::mesh_iteration::ShapeHandle)};
  let v=serde_json::to_value(handles).unwrap();assert_eq!(serde_json::from_value::<Handles>(v).unwrap(),handles);
  for text in ["{}",r#"{"actor":null}"#,r#"{"shape":null}"#,r#"{"actor":"fixture_shape","shape":"fixture_actor"}"#] {assert!(serde_json::from_str::<Handles>(text).is_err());}
  assert_eq!(serde_json::from_str::<Handles>(r#"{"actor":null,"shape":null}"#).unwrap(),Handles{actor:None,shape:None});
 }
 #[test] fn asset_and_capsule_binding_rejects_alias_without_clearing_handles() {
  let terrain=StaticTerrain::course();let s=scene(&terrain);let mut raw=serde_json::to_value(&terrain).unwrap();raw["revision"]=serde_json::json!(60);
  let other:StaticTerrain=serde_json::from_value(raw).unwrap();assert_eq!(s.runtime.validate(&other,capsule(),s.center()),Err(MeshSceneError::Identity));
  let mut c=capsule();c.rotation=[0.,0.,0.,1.];assert_eq!(s.runtime.validate(&terrain,c,s.center()),Err(MeshSceneError::Identity));
 }
}
