//! Authored LAB static triangle terrain, shared by every World geometry service.
//! Closed convex pieces and an explicit operating region make occupancy meaningful
//! for interiors and below-floor spawns. These are LAB policies, not native MTD.
use crate::{mesh_iteration::{PreparedScene, Record}, precise_capsule::Triangle, segment_triangle};
use glam::{DVec3, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
const EPS: f32 = 1e-5;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ConvexPiece { pub start: u32, pub count: u32 }
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Asset {
    revision: u64,
    triangles: Vec<Triangle>,
    source_ids: Vec<u32>,
    pieces: Vec<ConvexPiece>,
    allowed_min: [f32; 3],
    allowed_max: [f32; 3],
    safe_spawn: [f32; 3],
}
#[derive(Clone, Debug, Serialize)]
#[serde(transparent)]
pub struct StaticTerrain {
    asset: Asset,
    #[serde(skip)] records: Vec<Record>,
    #[serde(skip)] bounds: [f32; 6],
    #[serde(skip)] normals: Vec<Vec3>,
}
impl<'de> Deserialize<'de> for StaticTerrain {
    fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        Self::checked(Asset::deserialize(d)?).map_err(serde::de::Error::custom)
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TerrainError { InvalidAsset, InvalidGeometry, NonConvexOrOpen, InvalidDomain, MixedSolids }
impl std::fmt::Display for TerrainError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result { write!(f, "{self:?}") }
}
impl std::error::Error for TerrainError {}
#[derive(Clone, Copy, Debug)]
pub struct TerrainRayHit { pub distance: f32, pub normal: Vec3, pub source_id: u32 }
fn admissible(x: f32) -> bool { x.is_finite() && (x == 0. || x.is_normal()) && x.abs() <= 10000. }
fn key(v: [f32; 3]) -> [u32; 3] { v.map(|x| if x == 0. { 0 } else { x.to_bits() }) }
impl StaticTerrain {
    fn checked(asset: Asset) -> Result<Self, TerrainError> {
        if asset.revision == 0 || asset.triangles.is_empty() || asset.triangles.len() > 512
            || asset.source_ids.len() != asset.triangles.len() || asset.pieces.is_empty()
            || asset.pieces.len() > 32 || asset.source_ids.iter().collect::<BTreeSet<_>>().len() != asset.source_ids.len()
            || !asset.triangles.iter().flatten().flatten().copied().chain(asset.allowed_min).chain(asset.allowed_max).chain(asset.safe_spawn).all(admissible)
        { return Err(TerrainError::InvalidAsset); }
        let min = Vec3::from(asset.allowed_min); let max = Vec3::from(asset.allowed_max);
        if !min.cmplt(max).all() || !Vec3::from(asset.safe_spawn).cmpge(min).all() || !Vec3::from(asset.safe_spawn).cmple(max).all() { return Err(TerrainError::InvalidDomain); }
        let mut lo = Vec3::splat(f32::INFINITY); let mut hi = Vec3::splat(f32::NEG_INFINITY);
        let mut normals = Vec::new();
        for tri in &asset.triangles {
            let [a,b,c] = tri.map(Vec3::from); let cross = (b-a).cross(c-a);
            if !cross.is_finite() || cross.length_squared() <= 1e-10 { return Err(TerrainError::InvalidAsset); }
            let n = cross.normalize(); if !n.is_finite() { return Err(TerrainError::InvalidAsset); } normals.push(n);
            for v in [a,b,c] { lo = lo.min(v); hi = hi.max(v); }
        }
        let mut end = 0usize;
        for piece in &asset.pieces {
            let start = piece.start as usize;
            let next = start.checked_add(piece.count as usize).ok_or(TerrainError::InvalidAsset)?;
            if start != end || piece.count < 4 || next > asset.triangles.len() { return Err(TerrainError::InvalidAsset); }
            let tris = &asset.triangles[start..next];
            let vertices: Vec<DVec3> = tris.iter().flatten().map(|v| DVec3::from_array(v.map(f64::from))).collect();
            let mut edges = BTreeMap::<([u32;3],[u32;3]), (u32, i32)>::new();
            let mut volume = 0f64;
            for tri in tris {
                for (a,b) in [(tri[0],tri[1]),(tri[1],tri[2]),(tri[2],tri[0])] {
                    let a=key(a); let b=key(b); let (edge,sign)=if a<b { ((a,b),1) }else { ((b,a),-1) };
                    let v=edges.entry(edge).or_default();v.0+=1;v.1+=sign;
                }
                let [a,b,c] = tri.map(|v|DVec3::from_array(v.map(f64::from)));
                let n=(b-a).cross(c-a);let length=n.length();
                if vertices.iter().any(|v| n.dot(*v-a) > 1e-6*length) { return Err(TerrainError::NonConvexOrOpen); }
                volume+=a.dot(b.cross(c));
            }
            if edges.values().any(|&(count,balance)|count!=2||balance!=0) || !volume.is_finite() || volume<=1e-6 { return Err(TerrainError::NonConvexOrOpen); }
            end=next;
        }
        if end != asset.triangles.len() { return Err(TerrainError::InvalidAsset); }
        let count = asset.triangles.len() as u32;
        Ok(Self { asset, records: vec![Record {start:0,count,offset:[0.;3]}], bounds:[lo.x,lo.y,lo.z,hi.x,hi.y,hi.z], normals })
    }
    /// Revalidate raw authored data; derived services cannot be independently edited.
    pub fn validate(&self) -> Result<(), TerrainError> { Self::checked(self.asset.clone()).map(|_|()) }
    pub fn revision(&self)->u64 { self.asset.revision }
    pub fn safe_spawn(&self)->Vec3 { self.asset.safe_spawn.into() }
    pub fn triangles(&self)->&[Triangle] { &self.asset.triangles }
    pub fn source_ids(&self)->&[u32] { &self.asset.source_ids }
    /// Exact LAB asset binding, including all f32 bits; no hash collision/revision alias.
    pub fn identity_words(&self)->Vec<u32> {
        let a=&self.asset;
        let mut words=vec![a.revision as u32,(a.revision>>32)as u32,a.triangles.len()as u32,a.pieces.len()as u32];
        words.extend(a.triangles.iter().flatten().flatten().map(|x|x.to_bits()));words.extend(&a.source_ids);
        for p in &a.pieces {words.extend([p.start,p.count]);}
        words.extend(a.allowed_min.into_iter().chain(a.allowed_max).chain(a.safe_spawn).map(f32::to_bits)); words
    }
    pub fn prepared_scene(&self, behavior:u8)->PreparedScene<'_> {
        PreparedScene { triangles:&self.asset.triangles,source_ids:&self.asset.source_ids,records:&self.records,cache_origin:[0.;3],object_bounds:self.bounds,actor_pose:[0.,0.,0.,1.,0.,0.,0.],shape_pose:[0.,0.,0.,1.,0.,0.,0.],behavior }
    }
    /// Clip the centerline against the strictly interior halfspaces of each convex
    /// piece, covering interior crossings even when both endpoints are outside.
    fn axis_inside(&self,p:Vec3,q:Vec3)->bool {
        self.asset.pieces.iter().any(|piece| {
            let mut low=0f32;let mut high=1f32;
            for i in piece.start as usize..(piece.start+piece.count)as usize {
                let n=self.normals[i];let a=Vec3::from(self.asset.triangles[i][0]);
                let d=n.dot(p-a)+EPS;let v=n.dot(q-p);
                if v==0. {if d>0. {return false;}}
                else if v>0. {high=high.min(-d/v);}else {low=low.max(-d/v);}
                if low>high {return false;}
            }
            low<=high && high>=0. && low<=1.
        })
    }
    pub fn can_occupy(&self,feet:Vec3,radius:f32,height:f32)->bool {
        if !feet.is_finite() || !radius.is_finite() || !height.is_finite() || radius<=0. || height<2.*radius { return false; }
        let min=Vec3::from(self.asset.allowed_min);let max=Vec3::from(self.asset.allowed_max);
        let body_min=feet-Vec3::new(radius,0.,radius);let body_max=feet+Vec3::new(radius,height,radius);
        if !(body_min+Vec3::splat(EPS)).cmpge(min).all() || !(body_max-Vec3::splat(EPS)).cmple(max).all() {return false;}
        let p=feet+Vec3::Y*radius;let q=feet+Vec3::Y*(height-radius);
        if self.axis_inside(p,q) {return false;}
        let square=(radius-EPS).max(0.).powi(2);
        self.asset.triangles.iter().all(|tri| segment_triangle::witness(p.to_array(),q.to_array(),*tri).is_some_and(|w|w.distance>=square))
    }
    /// LAB torso proximity. Normals near horizontal planes are excluded; nearest
    /// distance wins, and asset order deterministically resolves exact ties.
    pub fn wall_contact(&self,feet:Vec3,radius:f32,height:f32,reach:f32)->Option<Vec3> {
        if !feet.is_finite() || !radius.is_finite() || radius<=0. || !height.is_finite() || height<2.*radius || !reach.is_finite() || reach<0. {return None;}
        let p=feet+Vec3::Y*(height*0.2);let q=feet+Vec3::Y*(height*0.85);let mut best=reach;let mut normal=None;
        for (i,tri) in self.asset.triangles.iter().enumerate() {
            if self.normals[i].y.abs()>=0.25 {continue;}
            let w=segment_triangle::witness(p.to_array(),q.to_array(),*tri)?;let gap=w.distance.sqrt()-radius;
            let delta=Vec3::new(w.segment[0]-w.triangle[0],0.,w.segment[2]-w.triangle[2]);
            if gap>=-0.001 && gap<=best && delta.length_squared()>0. { best=gap;normal=delta.try_normalize(); }
        }
        normal
    }
    /// Explicit LAB two-sided triangle ray, source-ID tie order. This is neither
    /// the native filtered targeting query nor a sphere/clearance sweep.
    pub fn ray_hit(&self,origin:Vec3,direction:Vec3,max:f32)->Result<Option<TerrainRayHit>,TerrainError> {
        if !origin.is_finite() || !direction.is_finite() || !max.is_finite() || max<0. || (direction.length_squared()-1.).abs()>0.001 {return Err(TerrainError::InvalidDomain);}
        let o=origin.as_dvec3();let d=direction.as_dvec3();let mut best:Option<TerrainRayHit>=None;
        for (i,tri) in self.asset.triangles.iter().enumerate() {
            let [a,b,c]=tri.map(|v|Vec3::from(v).as_dvec3());let e=b-a;let f=c-a;let h=d.cross(f);let det=e.dot(h);
            if det.abs()<1e-12 {continue;}
            let inv=1./det;let s=o-a;let u=s.dot(h)*inv;if !(0.0..=1.0).contains(&u) {continue;}
            let cross=s.cross(e);let v=d.dot(cross)*inv;if v<0. || u+v>1. {continue;}
            let t=f.dot(cross)*inv;if t<0. || t>f64::from(max) {continue;}
            let distance=t as f32;let source_id=self.asset.source_ids[i];
            if best.is_none_or(|old|distance<old.distance || (distance==old.distance && source_id<old.source_id)) { best=Some(TerrainRayHit{distance,normal:self.normals[i],source_id}); }
        }
        Ok(best)
    }
    pub fn ray_distance(&self,origin:Vec3,direction:Vec3,max:f32)->Option<f32> {self.ray_hit(origin,direction,max).ok().flatten().map(|h|h.distance)}
    pub fn aim_target(&self,origin:Vec3,direction:Vec3,extent:f32)->Option<Vec3> {
        if !origin.is_finite() || !extent.is_finite() || extent<=0. || origin.abs().max_element()>extent {return None;}
        let direction=direction.try_normalize()?;let mut max=extent;
        for i in 0..3 {if direction[i]!=0. {max=max.min((extent.copysign(direction[i])-origin[i])/direction[i]);}}
        let hit=self.ray_hit(origin,direction,max).ok()?;
        Some(origin+direction*hit.map_or(max,|h|h.distance-0.1))
    }
    /// LAB longitudinal ray backoff, not proof of camera radius clearance.
    pub fn camera_fraction(&self,target:Vec3,desired:Vec3,clearance:f32)->Result<f32,TerrainError> {
        if !target.is_finite() || !desired.is_finite() || !clearance.is_finite() || clearance<0. {return Err(TerrainError::InvalidDomain);}
        let delta=desired-target;let length=delta.length();if length==0. {return Ok(1.);}
        let hit=self.ray_hit(target,delta/length,length)?;
        Ok(hit.map_or(1.,|h|((h.distance-clearance).max(0.)/length).min(1.)))
    }
    pub fn course()->Self {
        let mut triangles=Vec::new();let mut pieces=Vec::new();
        let mut add=|vertices:Vec<Vec3>,faces:Vec<Vec<usize>>| {
            let start=triangles.len()as u32;let center=vertices.iter().copied().sum::<Vec3>()/vertices.len()as f32;
            for mut face in faces {
                let [a,b,c]=[vertices[face[0]],vertices[face[1]],vertices[face[2]]];
                if (b-a).cross(c-a).dot(a-center)<0. {face.reverse();}
                for i in 1..face.len()-1 {triangles.push([vertices[face[0]].to_array(),vertices[face[i]].to_array(),vertices[face[i+1]].to_array()]);}
            }
            pieces.push(ConvexPiece{start,count:triangles.len()as u32-start});
        };
        let box_faces=vec![vec![0,1,3,2],vec![4,6,7,5],vec![0,4,5,1],vec![2,3,7,6],vec![0,2,6,4],vec![1,5,7,3]];
        let verts=|min:Vec3,max:Vec3| (0..8).map(|i|Vec3::new(if i&1==0 {min.x}else {max.x},if i&2==0 {min.y}else {max.y},if i&4==0 {min.z}else {max.z})).collect::<Vec<_>>();
        add(verts(Vec3::new(-30.,-2.,-30.),Vec3::new(30.,0.,30.)),box_faces.clone());
        for (x,z,length,height) in [(0.,-10.,8.,2.),(9.,-10.,3.,4.)] {
            add(vec![Vec3::new(x-2.,0.,z),Vec3::new(x+2.,0.,z),Vec3::new(x-2.,0.,z+length),Vec3::new(x+2.,0.,z+length),Vec3::new(x-2.,height,z+length),Vec3::new(x+2.,height,z+length)],vec![vec![0,2,3,1],vec![0,1,5,4],vec![0,4,2],vec![1,3,5],vec![2,4,5,3]]);
        }
        let rotation=glam::Quat::from_rotation_y(0.35);let wall=verts(Vec3::new(-0.15,-2.,-5.),Vec3::new(0.15,2.,5.)).into_iter().map(|v|rotation*v+Vec3::new(-6.,2.,-6.)).collect();
        add(wall,box_faces.clone());
        add(verts(Vec3::new(-2.,1.2,12.),Vec3::new(2.,2.,20.)),box_faces.clone());
        add(verts(Vec3::new(9.,0.,5.),Vec3::new(14.,2.,9.)),box_faces);
        let source_ids=(0..triangles.len()as u32).collect();
        Self::checked(Asset{revision:59,triangles,source_ids,pieces,allowed_min:[-29.,0.,-29.],allowed_max:[29.,100.,29.],safe_spawn:[0.,0.04,8.]}).expect("authored convex course")
    }
}

#[cfg(test)]
mod tests {
 use super::*;
 #[test] fn course_roundtrip_keeps_exact_geometry_and_rejects_open_piece() {
  let t=StaticTerrain::course();t.validate().unwrap();let mut value=serde_json::to_value(&t).unwrap();
  let decoded:StaticTerrain=serde_json::from_value(value.clone()).unwrap();assert_eq!(t.identity_words(),decoded.identity_words());
  value["triangles"].as_array_mut().unwrap().pop();value["source_ids"].as_array_mut().unwrap().pop();
  assert!(serde_json::from_value::<StaticTerrain>(value).is_err());
 }
 #[test] fn shell_tangency_interior_and_below_floor_are_distinct() {
  let t=StaticTerrain::course();assert!(t.can_occupy(Vec3::new(0.,0.,8.),0.3,1.8));
  assert!(!t.can_occupy(Vec3::new(0.,-0.01,8.),0.3,1.8));
  assert!(!t.can_occupy(Vec3::new(0.,-5.,8.),0.3,1.8));
  assert!(!t.can_occupy(Vec3::new(11.,0.1,7.),0.3,1.8));
  assert!(t.can_occupy(Vec3::new(0.,0.01,16.),0.3,0.9));
  assert!(!t.can_occupy(Vec3::new(0.,0.01,16.),0.3,1.8));
 }
 #[test] fn two_sided_ray_camera_and_aim_use_same_ceiling() {
  let t=StaticTerrain::course();let origin=Vec3::new(0.,0.5,16.);
  let ray=t.ray_hit(origin,Vec3::Y,5.).unwrap().unwrap();assert!((ray.distance-0.7).abs()<1e-5);
  let aim=t.aim_target(origin,Vec3::Y,5000.).unwrap();assert!((aim.y-1.1).abs()<1e-5);
  let fraction=t.camera_fraction(origin,origin+Vec3::Y*2.,0.15).unwrap();assert!((fraction-0.275).abs()<1e-5);
  assert!(t.wall_contact(Vec3::new(0.,0.01,8.),0.3,1.8,0.5).is_none());
 }
 #[test] fn convex_axis_clip_rejects_crossing_and_fully_embedded_segments() {
  let t=StaticTerrain::course();assert!(t.axis_inside(Vec3::new(11.,1.,7.),Vec3::new(11.,1.5,7.)));
  assert!(t.axis_inside(Vec3::new(8.,1.,7.),Vec3::new(15.,1.,7.)));
  assert!(!t.axis_inside(Vec3::new(8.,3.,7.),Vec3::new(15.,3.,7.)));
 }
}
