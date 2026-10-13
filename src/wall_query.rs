//! Finite static LAB provider for the recovered paired wall selector.
//! Motion endpoints, compass orientation, static roles and next-dispatch delivery
//! are supplied bindings. This is not native callback or numerical equivalence.
use crate::{precise_capsule, world::{World, WorldTerrain}};
use glam::Vec3;
use serde::{Deserialize, Serialize};

pub const NORMAL_Y_LIMIT: f32 = f32::from_bits(0x3ed8616b);
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct Probe {
    pub metric: f32,
    pub normal: Vec3,
    pub point: Vec3,
    pub object: Option<u32>,
}
impl Default for Probe {
    fn default() -> Self { Self {metric:f32::MAX,normal:Vec3::ZERO,point:Vec3::ZERO,object:None} }
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct Query {
    pub identity: Vec<u32>,
    pub records: [Probe;17],
    pub base_radius: f32,
    pub query_radius: f32,
    pub reach: f32,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq)]
pub struct Contact { pub normal:Vec3, pub point:Vec3, pub pair:u8, pub score:f32 }
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Selection { Pending, NoHit, Hit(Contact) }

pub fn identity(world:&World)->Vec<u32> {
    match &world.terrain {
        WorldTerrain::Boxes => { let mut v=vec![0,world.solids.len() as u32];
            for s in &world.solids {v.extend(s.min.to_array().into_iter().chain(s.max.to_array()).map(f32::to_bits));} v }
        WorldTerrain::Mesh{asset} => {let mut v=vec![1];v.extend(asset.identity_words());v}
    }
}
impl Query {
    pub fn valid_for(&self,world:&World)->bool {
        self.identity==identity(world) && self.base_radius.is_finite() && self.base_radius>0.
        && self.query_radius.to_bits()==(0.9*self.base_radius).to_bits()
        && self.reach.to_bits()==(self.base_radius/0.9).to_bits()
        && self.records.iter().all(|p|p.metric.is_finite()&&p.normal.is_finite()&&p.point.is_finite())
    }
}
pub fn select(records:&[Probe;17])->Selection {
    // Readiness veto applies even after a candidate, including record16.
    if records.iter().any(|p|!(p.metric>=0.)) {return Selection::Pending;}
    let mut winner:Option<(usize,f32)>=None;
    for i in 0..8 {
        let a=records[i*2];let b=records[i*2+1];
        if a.object.is_none() || b.object.is_none() || a.metric>1. || b.metric>1.
            || !(a.normal.y<NORMAL_Y_LIMIT && b.normal.y<NORMAL_Y_LIMIT) {continue;}
        // MINSS(second,first): first record's bits win equality; later pairs win score ties.
        let score=if b.metric<a.metric {b.metric}else{a.metric};
        if winner.is_some_and(|(_,best)|score>best) {continue;}
        winner=Some((i,score));
    }
    // Normalize only AFTER selection. A cancelling winning pair does not fall
    // back to an earlier candidate. Native robust scaling is expressed mathematically.
    let Some((i,score))=winner else {return Selection::NoHit;};
    let mut n=(records[i*2].normal+records[i*2+1].normal)*0.5;n.y=0.;
    let scale=n.abs().max_element();
    if !scale.is_finite() || scale==0. {return Selection::NoHit;}
    n=(n/scale).normalize_or_zero();
    if !n.is_finite() || n.abs().cmple(Vec3::splat(1e-6)).all() {return Selection::NoHit;}
    // Static geometry is not BaseAvatar: no record16 Avatar fallback.
    Selection::Hit(Contact{normal:n,point:records[i*2].point,pair:i as u8,score})
}

fn sweep(world:&World,origin:Vec3,radius:f32,direction:Vec3,reach:f32)->Result<Probe,&'static str> {
    let mut best:Option<(f32,Vec3,Option<Vec3>,u32)>=None;
    match &world.terrain {
        WorldTerrain::Boxes => for (i,s) in world.solids.iter().enumerate() {
            if !s.size().cmpgt(Vec3::ZERO).all() {return Err("degenerate static box");}
            let h=precise_capsule::capsule_box(origin.to_array(),origin.to_array(),radius,
                [0.,0.,0.,1.],s.center().to_array(),(s.size()*0.5).to_array(),direction.to_array(),reach,0);
            if h.hit {
                let n=h.normal.ok_or("missing box normal")?;
                if best.is_none_or(|b|h.distance<=b.0) {best=Some((h.distance,n.into(),h.point.map(Vec3::from),i as u32));}
            }
        },
        WorldTerrain::Mesh{asset} => {
            if let Some(h)=precise_capsule::collection_sweep(asset.triangles(),origin.to_array(),radius,
                direction.to_array(),reach,precise_capsule::CollectionOptions{check_initial:true,..Default::default()}) {
                best=Some((h.distance,h.normal.into(),h.point.map(Vec3::from),asset.source_ids()[h.index]));
            }
        }
    }
    let Some((d,mut n,p,object))=best else {return Ok(Probe::default());};
    if !d.is_finite() || d<0. || d>reach || !n.is_finite() {return Err("unsupported sweep result");}
    let point=if d==0. {n=-direction*reach;origin+direction*radius}
        else {p.ok_or("missing nonzero contact point")?};
    if !point.is_finite() {return Err("nonfinite contact point");}
    Ok(Probe{metric:d/reach,normal:n,point,object:Some(object)})
}
pub fn submit(world:&World,feet:Vec3,height:f32,r:f32,forward:Vec3)->Result<Query,&'static str> {
    if world.validate().is_err() || !feet.is_finite() || !height.is_finite() || !r.is_finite()
        || r<=0. || height<2.*r || !forward.is_finite() {return Err("unsupported query inputs");}
    let query_radius=0.9*r;let reach=r/0.9;
    let a=feet+Vec3::Y*r;let b=feet+Vec3::Y*(height-r);
    let mut f=Vec3::new(forward.x,0.,forward.z).normalize_or_zero();if f==Vec3::ZERO {f=Vec3::NEG_Z;}
    let right=f.cross(Vec3::Y);let mut records=[Probe::default();17];
    // LAB compass order. Native currentDirections/quaternion are not reconstructed.
    for i in 0..8 {let angle=i as f32*std::f32::consts::FRAC_PI_4;
        let d=(f*angle.cos()+right*angle.sin()).normalize();
        records[i*2]=sweep(world,a,query_radius,d,reach)?;
        records[i*2+1]=sweep(world,b,query_radius,d,reach)?;
    }
    records[16]=sweep(world,a,query_radius,-Vec3::Y,reach)?;
    Ok(Query{identity:identity(world),records,base_radius:r,query_radius,reach})
}
