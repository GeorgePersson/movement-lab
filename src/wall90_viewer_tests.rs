// Included in the actual InputPlugin/TimePlugin scheduler fixture.
#[test]
fn ordered_wall_hold_release_and_mixed_press_transport90() {
    let (mut a,w,_)=app();
    {
        let mut lab=a.world_mut().resource_mut::<Lab>();
        lab.world=World::boxes(vec![movement_lab::Solid::new(CoreVec3::new(-50.,-2.,-100.),CoreVec3::new(50.,0.,100.)),
            movement_lab::Solid::new(CoreVec3::new(0.5,0.,-80.),CoreVec3::new(2.,40.,80.))]);
        lab.tuning=serde_json::from_str(include_str!("../presets/static-tenno-bullet.json")).unwrap();
        lab.player=Player::new(CoreVec3::new(0.,3.,0.));lab.previous=lab.player.position;
    }
    a.world_mut().resource_mut::<Orbit>().yaw=std::f32::consts::FRAC_PI_2;
    batch(&mut a,w,&[(KeyCode::KeyW,true),(KeyCode::Space,true)]);
    // A single one-second render delta is clamped by Time<Virtual>. Advance
    // 120 actual frames/fixed ticks; keep the default clamp unchanged.
    assert!(a.world().resource::<Time<Virtual>>().max_delta()<Duration::from_secs(1));
    for _ in 0..120 {ticks(&mut a,1);}
    assert_eq!(a.world().resource::<Trace>().after.len(),120);
    {let lab=a.world().resource::<Lab>();assert!(lab.player.wall_traversal().held);
        assert!(lab.player.wall_traversal().launches>=2);assert!(lab.player.wall_traversal().error.is_none());}
    key(&mut a,w,KeyCode::Space,false,false);ticks(&mut a,1);
    let launches=a.world().resource::<Lab>().player.wall_traversal().launches;
    for _ in 0..120 {ticks(&mut a,1);}
    let lab=a.world().resource::<Lab>();assert!(!lab.player.wall_traversal().held);
    assert_eq!(lab.player.wall_traversal().launches,launches);assert!(lab.player.wall_traversal().error.is_none());
}
#[test]
fn ordered_action_chunks_and_press_associations_do_not_drop_after64_events90() {
    let (mut a,w,_)=app();
    a.world_mut().resource_mut::<Lab>().tuning.wall_traversal.wall_enabled=false;
    for i in 0..65 {
        key(&mut a,w,KeyCode::ControlLeft,i%3==0,false);
        key(&mut a,w,KeyCode::Space,true,false);key(&mut a,w,KeyCode::Space,false,false);
    }
    advance(&mut a,Duration::ZERO);
    assert_eq!(a.world().resource::<Lab>().jump_edges.action_len(),130);
    assert_eq!(queued(&a).len(),65);
    ticks(&mut a,1);assert_eq!(a.world().resource::<Lab>().jump_edges.action_len(),66);
    ticks(&mut a,1);assert_eq!(a.world().resource::<Lab>().jump_edges.action_len(),2);
    ticks(&mut a,1);assert_eq!(a.world().resource::<Lab>().jump_edges.action_len(),0);
    for _ in 0..62 {ticks(&mut a,1);assert!(a.world().resource::<Lab>().player.wall_traversal().error.is_none());}
    let lab=a.world().resource::<Lab>();assert!(lab.jump_edges.pending_snapshot().is_empty());
    assert!(lab.player.wall_traversal().dispositions.is_empty());assert_eq!(lab.player.wall_traversal().next_id,Some(130));
}
#[test]
fn controls_action_clear_preserve_matrix_and_id_continuity90() {
    for mode in ["start","cancel","air_reject","budget_reject","manual","reset","focus"] {
        let (mut a,w,_)=app();
        batch(&mut a,w,&[(KeyCode::Space,true),(KeyCode::Space,false)]);advance(&mut a,Duration::ZERO);
        {
            let mut lab=a.world_mut().resource_mut::<Lab>();
            if mode=="cancel"||mode=="manual" {lab.combo.start();}
            if mode=="air_reject" {lab.player.grounded=false;}
            if mode=="budget_reject" {lab.player.jumps_remaining=0;}
        }
        if !matches!(mode,"manual"|"reset"|"focus") {key(&mut a,w,KeyCode::KeyG,true,false);}
        if mode=="reset" {key(&mut a,w,KeyCode::KeyR,true,false);key(&mut a,w,KeyCode::KeyG,true,false);}
        if mode=="focus" {a.world_mut().write_message(KeyboardFocusLost);}
        batch(&mut a,w,&[(KeyCode::ControlLeft,true),(KeyCode::Space,true),(KeyCode::Space,false),(KeyCode::ControlLeft,false)]);
        advance(&mut a,Duration::ZERO);
        let lab=a.world().resource::<Lab>();
        let actions=lab.jump_edges.action_snapshot();let edges=lab.jump_edges.pending_snapshot();
        match mode {
            "start"|"cancel"=>{assert_eq!(edges,vec![true],"{mode}");assert_eq!(actions.iter().map(|x|x.0).collect::<Vec<_>>(),vec![2,3],"{mode}");},
            "air_reject"|"budget_reject"|"manual"=>{assert_eq!(edges,vec![false,true],"{mode}");assert_eq!(actions.len(),4,"{mode}");},
            _=>{assert!(edges.is_empty(),"{mode}");assert!(actions.is_empty(),"{mode}");assert!(!lab.player.wall_traversal().held);assert!(lab.player.wall_traversal().ticket.is_none());}
        }
        assert!(!lab.combo.is_active(),"fresh manual action or cancellation resolves helper: {mode}");
        if matches!(mode,"reset"|"focus") {
            key(&mut a,w,KeyCode::KeyR,false,false);key(&mut a,w,KeyCode::KeyG,false,false);
            key(&mut a,w,KeyCode::Space,true,false);advance(&mut a,Duration::ZERO);
            let ids=a.world().resource::<Lab>().jump_edges.action_snapshot();
            assert_eq!(ids.len(),1);assert!(ids[0].0>=2,"ID must not reset: {mode}");
            ticks(&mut a,1);assert!(a.world().resource::<Lab>().player.wall_traversal().error.is_none());
        }
    }
}
