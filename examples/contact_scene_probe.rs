//! Private supplied static-scene comparison transport. No World/controller policy.
use movement_lab::{cct_policy::{self,Settings},cct_scene::{Capsule,Collider,StaticBoxes}};
use serde::Deserialize;
use serde_json::json;
use std::io::{self,BufRead};
#[derive(Deserialize)]
struct BoxInput {center:[u32;3],extents:[u32;3]}
#[derive(Deserialize)]
struct Input {center:[u64;3],displacement:[u32;3],capsule_q:[u32;4],radius:u32,cylinder_height:u32,total_height:u32,contact_offset:u32,step_offset:u32,min_distance:u32,boxes:Vec<BoxInput>,behavior:u8,metadata:bool}
fn main()->Result<(),Box<dyn std::error::Error>> {
 for line in io::stdin().lock().lines() {
  let c:Input=serde_json::from_str(&line?)?;
  let boxes:Vec<_>=c.boxes.iter().map(|b|Collider{center:b.center.map(f32::from_bits),extents:b.extents.map(f32::from_bits)}).collect();
  let capsule=Capsule{radius:f32::from_bits(c.radius),height:f32::from_bits(c.cylinder_height),rotation:c.capsule_q.map(f32::from_bits)};
  let settings=Settings{up:[0.,1.,0.],contact_offset:f32::from_bits(c.contact_offset),step_offset:f32::from_bits(c.step_offset),min_distance:f32::from_bits(c.min_distance),overlap_recovery:true,prevent_ceiling_slide:false,normalize_components:false,custom_response:true,walk_experiment:false,standing_on_moving:false,nonwalkable_mode:0,slope_limit:0.7};
  let mut scene=StaticBoxes{capsule,boxes:&boxes,contact_offset:settings.contact_offset,overlap_recovery:true,behavior:c.behavior,recovery_scan_limit:16,reports:Vec::new(),contact_reports:{let mut r=movement_lab::contact_reports::ContactReports::from_record(&[0x55;85]);r.reset();r},report_metadata:c.metadata};
  let out=match cct_policy::move_with_scene(c.center.map(f64::from_bits),c.displacement.map(f32::from_bits),&settings,&mut scene) {
   Ok(m)=>{
    let half=f32::from_bits(c.total_height)*0.5;
    let feet=[m.position[0] as f32,(m.position[1]-f64::from(half)) as f32,m.position[2] as f32];
    json!({"position":m.position.map(f64::to_bits),"feet":feet.map(f32::to_bits),"collision_flags":m.collision_flags,"contact_record":scene.contact_reports.to_record().iter().map(|b|format!("{b:02x}")).collect::<String>(),"iterations":m.passes.iter().enumerate().map(|(i,p)|json!({"pass_id":i,"position":p.position.map(f64::to_bits),"attempted":p.attempted,"collisions":p.collisions,"passes":p.iterations})).collect::<Vec<_>>()})
   },
   Err(e)=>json!({"error":format!("{e:?}")}),
  };
  println!("{}",serde_json::to_string(&out)?);
 }
 Ok(())
}
