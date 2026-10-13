use super::*;
use crate::{Solid,wall_query::{self,Selection,Probe},wall_traversal::{Provider,Kind,ResultKind,OutcomeKind,JumpActions}};
fn floor()->World {World::boxes(vec![Solid::new(Vec3::new(-50.,-2.,-100.),Vec3::new(50.,0.,100.))])}
fn course()->World {let mut w=floor();w.solids.push(Solid::new(Vec3::new(0.5,0.,-80.),Vec3::new(2.,40.,80.)));w}
fn airborne()->Player {let mut p=Player::new(Vec3::new(0.,3.,0.));p.jump_budget_initialized=true;p}
fn dt()->f32 {MAX_SUBSTEP}
fn release(i:Input)->Input {Input{jump:false,jump_held:false,..i}}
fn held()->Input {Input{jump:true,jump_held:true,movement:Vec3::X,camera_forward:Vec3::X,..Default::default()}}
fn preset()->Tuning {serde_json::from_str(include_str!("../presets/static-tenno-bullet.json")).unwrap()}

#[test]
fn real_probe_radius_reach_and_late_readiness_veto() {
    let w=World::boxes(vec![Solid::new(Vec3::new(0.58,-3.,-4.),Vec3::new(2.,10.,4.))]);
    let q=wall_query::submit(&w,Vec3::ZERO,1.8,0.3,Vec3::X).unwrap();
    assert_eq!(q.query_radius,(0.9f32*0.3));assert_eq!(q.reach,0.3/0.9);
    assert!(matches!(wall_query::select(&q.records),Selection::Hit(_)),"R/.9 reaches this gap; R does not");
    let mut records=q.records;records[14].metric=-1.;assert_eq!(wall_query::select(&records),Selection::Pending);
    records=q.records;records[16].metric=f32::NAN;assert_eq!(wall_query::select(&records),Selection::Pending);
    records=q.records;for r in &mut records {r.normal.y=wall_query::NORMAL_Y_LIMIT;}
    assert_eq!(wall_query::select(&records),Selection::NoHit);
}
#[test]
fn selector_ties_choose_later_pair_and_floor_is_not_avatar() {
    let mut r=[Probe::default();17];
    for p in &mut r[..16] {*p=Probe{metric:0.5,normal:-Vec3::X,point:Vec3::ZERO,object:Some(0)};}
    let Selection::Hit(hit)=wall_query::select(&r) else {panic!()};assert_eq!(hit.pair,7);
    r=[Probe::default();17];r[16]=Probe{metric:0.,normal:Vec3::Y,point:Vec3::ZERO,object:Some(1)};
    assert_eq!(wall_query::select(&r),Selection::NoHit);
}
#[test]
fn actual_initial_query_then_release_consumes_once_and_does_not_rearm() {
    let mut p=airborne();let t=preset();let w=course();let i=held();
    p.step(&i,dt(),&w,&t);assert_eq!(p.wall_traversal.launches,0);assert!(p.wall_traversal.held);
    assert_eq!(p.wall_traversal.ticket.as_ref().unwrap().kind,Kind::Initial);
    p.step(&release(i),dt(),&w,&t);assert_eq!(p.wall_traversal.launches,1);assert!(!p.wall_traversal.held);
    let submissions=p.wall_traversal.submitted;
    for _ in 0..300 {p.step(&release(i),dt(),&w,&t);}
    assert_eq!(p.wall_traversal.submitted,submissions);assert_eq!(p.wall_traversal.launches,1);
}
#[test]
fn legacy_held_transition_releases_without_another_pulse() {
    let mut p=airborne();let mut t=preset();t.wall_traversal.provider=Provider::Pending;let w=course();
    p.step(&held(),dt(),&w,&t);p.step(&Input{jump:false,..held()},dt(),&w,&t);
    assert!(p.wall_traversal.held);p.step(&release(held()),dt(),&w,&t);
    assert!(!p.wall_traversal.held);assert!(p.wall_traversal.ticket.is_some());
    p.step(&release(held()),dt(),&w,&t);assert!(!p.wall_traversal.held);
}
#[test]
fn refusal_is_handled_and_retry_nohit_never_spends_ordinary_budget() {
    let mut p=airborne();let mut t=preset();t.wall_traversal.provider=Provider::Refused;let w=floor();
    p.step(&held(),dt(),&w,&t);assert_eq!(p.jumps_remaining,2);assert!(p.wall_traversal.ticket.is_none());
    assert!(p.wall_traversal.outcomes.iter().any(|o|o.result==ResultKind::WallHandled));
    t.wall_traversal.provider=Provider::Static;p.wall_traversal.retry_timer=0.;
    p.step(&Input{jump:false,..held()},dt(),&w,&t);
    assert_eq!(p.wall_traversal.ticket.as_ref().unwrap().kind,Kind::Retry);
    p.step(&Input{jump:false,..held()},dt(),&w,&t);assert_eq!(p.jumps_remaining,2);
    assert!(!p.wall_traversal.outcomes.iter().any(|o|o.kind==OutcomeKind::InitialFallback));
}
#[test]
fn rejected_wall_admission_publishes_vector_and_timers_without_refill() {
    let mut p=airborne();let mut t=preset();t.wall_traversal.actor_admits_wall=false;let w=course();
    p.jumps_remaining=0;p.bullet_jumps_left=0;p.air_rolls_left=0;
    p.step(&held(),dt(),&w,&t);p.step(&Input{jump:false,..held()},dt(),&w,&t);
    assert!(p.wall_traversal.published_vector.is_some());assert!(p.wall_jump_timer<f32::MAX);
    assert_eq!(p.wall_jump_timer,p.wall_traversal.retry_timer);
    assert_eq!((p.jumps_remaining,p.bullet_jumps_left,p.air_rolls_left),(0,0,0));
    assert_eq!(p.wall_traversal.launches,0);
}
#[test]
fn coincident_nohit_fallback_and_unrelated_crouched_edge_attempt_both() {
    let w=floor();let t=preset();let mut p=airborne();
    let a=Input{jump:true,jump_held:true,jump_edge_id:Some(0),jump_actions:JumpActions{
        authoritative:true,first_id:0,active_bits:1,crouch_bits:0,len:1},..Default::default()};
    p.step(&a,dt(),&w,&t);assert_eq!(p.jumps_remaining,2);
    let b=Input{jump:true,crouch:true,jump_edge_id:Some(2),jump_actions:JumpActions{
        authoritative:true,first_id:1,active_bits:2,crouch_bits:2,len:2},..Default::default()};
    p.step(&b,dt(),&w,&t);
    let o=&p.wall_traversal.outcomes;
    assert_eq!(o.len(),2);assert_eq!((o[0].kind,o[0].id,o[0].crouch,o[0].result),
        (OutcomeKind::InitialFallback,Some(0),false,ResultKind::Accepted));
    assert_eq!((o[1].kind,o[1].id,o[1].crouch,o[1].result),
        (OutcomeKind::Ordinary,Some(2),true,ResultKind::Accepted));
    assert_eq!((p.jumps_remaining,p.bullet_jumps_left),(0,0));
}
#[test]
fn deferred_requests_keep_last_writer_not_an_invented_pending_gate() {
    let w=floor();let t=preset();let mut p=Player::new(Vec3::ZERO);p.grounded=true;
    p.jump_budget_initialized=true;p.jump_lifecycle.request_deferred_height(0.3).unwrap();
    let (a,_,_)=p.ordinary_jump_request(&Input::default(),false,&w,&t);
    let mut later=t;later.jump_height=2.;
    let (b,_,_)=p.ordinary_jump_request(&Input::default(),false,&w,&later);
    assert_eq!((a,b),(ResultKind::Accepted,ResultKind::Accepted));
    assert_eq!(p.jump_lifecycle.deferred_height,2.);assert_eq!(p.jumps_remaining,0);
    assert!(p.jump_lifecycle.deferred_pending);assert_eq!(p.jump_lifecycle.properties.staged,1);
    assert_eq!(p.jump_lifecycle.properties.evaluated,0);
}
#[test]
fn ground_deferred_status_preserves_retry_and_signed_boundaries() {
    let w=floor();let t=preset();let mut p=Player::new(Vec3::ZERO);p.grounded=true;
    p.step(&Input{jump:true,jump_held:true,..Default::default()},dt(),&w,&t);
    assert!(p.wall_traversal.retry_timer<f32::MAX);
    let mut p=airborne();p.wall_traversal.held=true;p.wall_traversal.retry_timer=dt()*0.5;
    p.wall_traversal.legacy_held=true;
    p.step(&Input{jump:false,..held()},dt(),&w,&t);
    assert_eq!(p.wall_traversal.retry_timer,-dt()*0.5);assert_eq!(p.wall_traversal.ticket.as_ref().unwrap().kind,Kind::Retry);
}
#[test]
fn held_trajectories_use_real_up_and_both_side_branches() {
    let w=course();let t=preset();
    for (name,m) in [("up",Vec3::X),("positive",Vec3::new(0.25,0.,0.9682458)),("negative",Vec3::new(0.25,0.,-0.9682458))] {
        let mut p=airborne();let mut max_y=p.position.y;let mut subtypes=std::collections::BTreeSet::new();
        for tick in 0..1200 {
            p.step(&Input{jump:tick==0,jump_held:true,movement:m,camera_forward:m,..Default::default()},dt(),&w,&t);
            max_y=max_y.max(p.position.y);if p.wall_jump_subtype!=0{subtypes.insert(p.wall_jump_subtype);}
            assert!(p.wall_traversal.error.is_none(),"{name}: {:?}",p.wall_traversal.error);
        }
        eprintln!("HELD90 {name} launches={} peak={max_y} position={:?} subtypes={subtypes:?}",p.wall_traversal.launches,p.position);
        assert!(p.wall_traversal.launches>=2,"{name} continuation");
        if name=="up" {assert!(max_y>4.);assert!(subtypes.contains(&2));}
        else {assert!(p.position.z.abs()>4.);assert_eq!(p.position.z.signum(),m.z.signum());assert!(subtypes.contains(&3)||subtypes.contains(&4));}
    }
}
#[test]
fn changed_geometry_rejects_pending_resume_without_rebinding() {
    let mut p=airborne();let w=course();let t=preset();p.step(&held(),dt(),&w,&t);
    let saved=serde_json::to_string(&p).unwrap();let mut q:Player=serde_json::from_str(&saved).unwrap();
    q.step(&Input{jump:false,..held()},dt(),&floor(),&t);
    assert!(q.wall_traversal.error.is_some());assert_eq!(q.wall_traversal.launches,0);assert!(q.wall_traversal.ticket.is_some());
}

#[test]
fn mixed_press_associations_survive_delayed_edge_and_refused_first_request() {
    for provider in [Provider::Static,Provider::Refused] {
        let mut p=airborne();let w=course();let mut t=preset();t.wall_traversal.provider=provider;
        let first=Input{jump:true,jump_edge_id:Some(0),jump_actions:JumpActions{
            authoritative:true,first_id:0,len:4,active_bits:5,crouch_bits:4},..Default::default()};
        p.step(&first,dt(),&w,&t);assert_eq!(p.jumps_remaining,2);
        assert!(!p.wall_traversal.held);assert_eq!(p.wall_traversal.dispositions.len(),1);
        let second=Input{jump:true,crouch:true,jump_edge_id:Some(2),jump_actions:JumpActions{
            authoritative:true,first_id:4,..Default::default()},..Default::default()};
        p.step(&second,dt(),&w,&t);assert!(p.wall_traversal.error.is_none());
        assert!(p.wall_traversal.dispositions.is_empty());assert_eq!(p.bullet_jumps_left,0);
        assert!(p.wall_traversal.outcomes.iter().any(|o|o.kind==OutcomeKind::Ordinary && o.id==Some(2) && o.result==ResultKind::Accepted));
        assert_eq!(p.jumps_remaining,if provider==Provider::Static {0}else{1});
    }
}
#[test]
fn two_wall_handled_presses_do_not_leak_a_delayed_double() {
    let mut p=airborne();let w=course();let t=preset();
    let a=Input{jump:true,jump_edge_id:Some(0),jump_actions:JumpActions{
        authoritative:true,first_id:0,len:4,active_bits:5,crouch_bits:0},..Default::default()};
    p.step(&a,dt(),&w,&t);assert_eq!(p.wall_traversal.submitted,1);
    p.step(&Input{jump:true,jump_edge_id:Some(2),jump_actions:JumpActions{
        authoritative:true,first_id:4,..Default::default()},..Default::default()},dt(),&w,&t);
    assert_eq!(p.wall_traversal.launches,1);assert_eq!(p.jumps_remaining,1);
    assert!(!p.wall_traversal.outcomes.iter().any(|o|o.kind==OutcomeKind::Ordinary));
}
#[test]
fn selector_does_not_replace_cancelling_winner_with_earlier_pair() {
    let mut r=[Probe::default();17];
    for (i,n) in [(0,-Vec3::X),(1,-Vec3::X),(14,Vec3::X),(15,-Vec3::X)] {
        r[i]=Probe{metric:0.5,normal:n,point:Vec3::ZERO,object:Some(0)};
    }
    assert_eq!(wall_query::select(&r),Selection::NoHit);
    r[15].normal=Vec3::X*1e-20;r[14].normal=Vec3::X*1e-20;
    let Selection::Hit(c)=wall_query::select(&r) else {panic!()};assert_eq!(c.pair,7);assert_eq!(c.normal,Vec3::X);
}
#[test]
fn coincident_second_attempt_uses_actual_remaining_budget_and_roll_gates() {
    for roll in [false,true] {
        let mut p=airborne();let w=floor();let t=preset();p.jumps_remaining=1;
        p.step(&held(),dt(),&w,&t);
        if roll {p.roll_remaining=0.5;}
        p.step(&Input{jump:true,crouch:true,..Default::default()},dt(),&w,&t);
        let o=&p.wall_traversal.outcomes;
        assert_eq!(o.iter().filter(|o|matches!(o.kind,OutcomeKind::InitialFallback|OutcomeKind::Ordinary)).count(),2);
        assert_eq!(o[0].result,if roll {ResultKind::Roll}else{ResultKind::Accepted});
        assert_eq!(o[1].result,if roll {ResultKind::Roll}else{ResultKind::Budget});
        assert_eq!(p.bullet_jumps_left,1);assert_eq!(p.jumps_remaining,if roll {1}else{0});
    }
}
#[test]
fn no_wall_ordinary_then_double_records_exact_staged_phase_and_no_hold_retries() {
    let w=floor();let t=preset();let mut p=Player::new(Vec3::ZERO);p.grounded=true;
    p.step(&Input{jump:true,..Default::default()},dt(),&w,&t);
    assert!(!p.grounded);assert_eq!(p.jumps_remaining,1);
    p.step(&Input::default(),dt(),&w,&t);
    p.step(&Input{jump:true,movement:Vec3::X,..Default::default()},dt(),&w,&t);
    assert_eq!(p.jumps_remaining,1);assert_eq!(p.wall_traversal.ticket.as_ref().unwrap().kind,Kind::Initial);
    p.step(&Input{movement:Vec3::X,..Default::default()},dt(),&w,&t);
    assert_eq!(p.jumps_remaining,0);assert_eq!(p.jump_style,JumpStyle::Double);
    assert!(p.wall_traversal.outcomes.iter().any(|o|o.kind==OutcomeKind::InitialFallback&&o.result==ResultKind::Accepted));
    let requests=p.wall_traversal.submitted;
    for _ in 0..20 {p.step(&Input::default(),dt(),&w,&t);}
    assert_eq!(p.wall_traversal.submitted,requests);assert_eq!(p.jumps_remaining,0);
}
#[test]
fn competing_dispatcher_gate_preserves_both_clocks_and_pending_service() {
    let mut p=airborne();let mut t=preset();let w=course();p.step(&held(),dt(),&w,&t);
    t.wall_traversal.competing_move=true;p.wall_jump_timer=0.2;p.wall_traversal.retry_timer=0.3;
    p.step(&Input{jump:false,..held()},dt(),&w,&t);
    assert_eq!(p.wall_jump_timer,0.2);assert_eq!(p.wall_traversal.retry_timer,0.3);
    assert!(p.wall_traversal.ticket.is_some());assert_eq!(p.wall_traversal.launches,0);
}
#[test]
fn wrong_associated_posture_and_malformed_packet_are_explicit_errors() {
    let w=floor();let t=preset();let mut p=airborne();
    p.step(&Input{jump_actions:JumpActions{authoritative:true,first_id:0,len:1,active_bits:1,crouch_bits:1},..Default::default()},dt(),&w,&t);
    p.step(&Input{jump:true,jump_edge_id:Some(0),jump_actions:JumpActions{authoritative:true,first_id:1,..Default::default()},..Default::default()},dt(),&w,&t);
    assert!(p.wall_traversal.error.as_ref().unwrap().contains("posture"));assert_eq!(p.jumps_remaining,2);
    let before=p.position;
    p.step(&Input{jump_actions:JumpActions{authoritative:true,len:65,..Default::default()},..Default::default()},dt(),&w,&t);
    assert!(p.wall_traversal.error.is_some());assert_eq!(p.position,before);
}
#[test]
fn triangle_provider_uses_selected_mesh_and_initial_overlap_callback_is_explicit() {
    let asset=crate::static_terrain::StaticTerrain::course();let world=World::mesh(asset).unwrap();
    let n=glam::Quat::from_rotation_y(0.35)*Vec3::X;
    let feet=Vec3::new(-6.,1.,-6.)-n*0.5;
    let query=wall_query::submit(&world,feet,1.8,0.3,n).unwrap();
    let Selection::Hit(c)=wall_query::select(&query.records) else {panic!("actual triangle wall must be selected");};
    assert!(c.normal.dot(-n)>0.999);assert!(query.valid_for(&world));assert!(!query.valid_for(&course()));
    let q=wall_query::submit(&course(),Vec3::new(0.4,3.,0.),1.8,0.3,Vec3::X).unwrap();
    let p=q.records[0];assert_eq!(p.metric,0.);assert_eq!(p.normal,-Vec3::X*q.reach);
    assert_eq!(p.point,Vec3::new(0.4,3.3,0.)+Vec3::X*q.query_radius);
}
#[test]
fn launch_rejection_does_not_rewrite_or_double_age_retry_clock() {
    let w=course();let t=preset();let mut p=airborne();p.step(&held(),dt(),&w,&t);
    p.wall_jump_timer=0.2;p.wall_traversal.retry_timer=0.3;p.jumps_remaining=0;
    p.step(&Input{jump:false,..held()},dt(),&w,&t);
    assert_eq!(p.wall_jump_timer,0.2-dt());assert_eq!(p.wall_traversal.retry_timer,0.3);
    assert_eq!(p.jumps_remaining,0);assert_eq!(p.wall_traversal.launches,0);
    assert_eq!(p.wall_traversal.outcomes.last().unwrap().result,ResultKind::LaunchTimer);
    p.step(&Input{jump:false,..held()},dt(),&w,&t);
    assert_eq!(p.wall_traversal.retry_timer,0.3-dt());
}

#[test]
fn grounded_held_press_transitions_to_real_repeated_wall_launches() {
    let w=course();let t=preset();let mut p=Player::new(Vec3::new(0.,0.04,0.));
    for _ in 0..120 {p.step(&Input::default(),dt(),&w,&t);}
    assert!(p.grounded);let start=p.position;
    for tick in 0..1200 {
        p.step(&Input{jump:tick==0,jump_held:true,movement:Vec3::X,camera_forward:Vec3::X,..Default::default()},dt(),&w,&t);
        assert!(p.wall_traversal.error.is_none());
    }
    assert!(p.wall_traversal.launches>=2);assert!(p.position.y>start.y+4.);
    eprintln!("GROUNDED_HELD90 launches={} start={:?} end={:?}",p.wall_traversal.launches,start,p.position);
}

#[test]
fn evaluated_property_one_rejects_actual_ordinary_admission_without_writes() {
    let w=floor();let t=preset();
    for crouch in [false,true] {
        let mut p=airborne();p.jump_lifecycle.properties.evaluated=1;
        p.jump_lifecycle.properties.staged=0;let before=serde_json::to_string(&p).unwrap();
        let result=p.ordinary_jump_request(&held(),crouch,&w,&t);
        assert_eq!(result,(ResultKind::EvaluatedProperty,false,false));
        assert_eq!(serde_json::to_string(&p).unwrap(),before);
    }
}
#[test]
fn evaluated_property_one_rejects_wall_reset_after_original_caller_publications() {
    let t=preset();let mut p=airborne();p.jump_lifecycle.properties.evaluated=1;
    p.jumps_remaining=0;p.bullet_jumps_left=0;p.air_rolls_left=0;
    let old_y=p.velocity.y;let lifecycle=p.jump_lifecycle.clone();
    let result=p.perform_wall_launch(&held(),-Vec3::X,&t);
    assert_eq!(result,(ResultKind::EvaluatedProperty,false));
    assert_eq!((p.jumps_remaining,p.bullet_jumps_left,p.air_rolls_left),(0,0,0));
    assert_eq!(p.velocity.y.to_bits(),old_y.to_bits());
    assert_eq!(serde_json::to_string(&p.jump_lifecycle).unwrap(),serde_json::to_string(&lifecycle).unwrap());
    assert!(p.wall_traversal.published_vector.is_some());assert!(p.wall_jump_timer<f32::MAX);
    assert_eq!(p.wall_jump_timer,p.wall_traversal.retry_timer);assert_eq!(p.wall_traversal.launches,0);
    let mut allowed=airborne();allowed.jump_lifecycle.request_deferred_height(0.3).unwrap();
    assert_eq!(allowed.jump_lifecycle.properties.evaluated,0);
    assert_eq!(allowed.perform_wall_launch(&held(),-Vec3::X,&t).0,ResultKind::Accepted);
    assert!(allowed.jump_lifecycle.deferred_pending);assert_eq!(allowed.jump_lifecycle.deferred_height,0.3);
}
