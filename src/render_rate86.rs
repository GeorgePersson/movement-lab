//! Pass86 isolated observation harness. Candidate86 fixed-step aim ownership.
use super::*;
use bevy::input::{ButtonState, InputPlugin, InputSystems};
use bevy::input::keyboard::{Key, KeyboardInput, NativeKey};
use bevy::input::mouse::{MouseButtonInput, MouseMotion};
use bevy::time::{TimePlugin, TimeUpdateStrategy};
use bevy::gizmos::{gizmos::GizmoStorage, config::{DefaultGizmoConfigGroup, GizmoConfigStore, GizmoConfig}};
use serde_json::{json, Value};
use std::time::Duration;

#[derive(Resource, Default)]
struct Clock86 { frame: usize, wall_ns: u64, present: bool }
#[derive(Resource, Default)]
struct Trace86 { ticks: Vec<Value>, frames: Vec<Value>, before: Value }
fn focus_marker86(_keys: Res<ButtonInput<KeyCode>>) {}
fn before86(lab: Res<Lab>, camera: Single<&Transform, With<FollowCamera>>, mut trace: ResMut<Trace86>) {
    trace.before=json!({"queued":lab.jump_edges.pending_snapshot(),"input":lab.input,
        "combo":lab.combo.stage_label(),"camera":core_vec(camera.translation)});
}
fn after86(audit: Res<AimAudit86>, lab: Res<Lab>, fixed: Res<Time<Fixed>>, clock: Res<Clock86>, mut trace: ResMut<Trace86>) {
    let before=trace.before.clone();let tick=trace.ticks.len()+1;
    trace.ticks.push(json!({"tick":tick,"frame":clock.frame,"wall_ns":clock.wall_ns,
        "fixed_ns":fixed.elapsed().as_nanos() as u64,"before":before,
        "position":lab.player.position,"velocity":lab.player.combined_velocity(),
        "consumed_input":audit.input,"aim_origin":audit.origin,"aim_boom":audit.boom,
        "player_json":serde_json::to_string(&lab.player).unwrap(),
        "position_bits":lab.player.position.to_array().map(f32::to_bits),
        "velocity_bits":lab.player.combined_velocity().to_array().map(f32::to_bits),
        "state":lab.player.state,"grounded":lab.player.grounded,"jumps":lab.player.jumps_remaining,
        "bullets":lab.player.bullet_jumps_left,"rolls":lab.player.air_rolls_left,
        "combo":lab.combo.stage_label(),"aim_target":lab.input.aim_target}));
}
fn should_present86(clock: Res<Clock86>) -> bool { clock.present }
fn frame86(lab: Res<Lab>, clock: Res<Clock86>, fixed: Res<Time<Fixed>>,
    camera: Single<&Transform, With<FollowCamera>>, orbit: Res<Orbit>, mut trace: ResMut<Trace86>) {
    let ticks=trace.ticks.len();trace.frames.push(json!({"frame":clock.frame,"wall_ns":clock.wall_ns,
        "ticks":ticks,"overstep":fixed.overstep_fraction(),"position":lab.player.position,
        "camera":core_vec(camera.translation),"yaw":orbit.yaw,"pitch":orbit.pitch}));
}
fn clear_gizmos86(mut storage: ResMut<GizmoStorage<DefaultGizmoConfigGroup, ()>>) { storage.clear(); }
fn app86(auto: bool, look: bool) -> (App,Entity) {
    let (mut app,window,_)=viewer_tests::controls_app();
    {
        let mut lab=app.world_mut().resource_mut::<Lab>();
        lab.tuning=serde_json::from_str(include_str!("../presets/static-tenno-bullet.json")).unwrap();
        let mut player=Player::new(lab.world.spawn());
        player.step(&neutral_input(),0.1,&lab.world,&lab.tuning);
        player.wall_transport_boundary();
        assert!(player.grounded);lab.previous=player.position;lab.player=player;lab.show_trail=false;
    }
    { let mut orbit=app.world_mut().resource_mut::<Orbit>();orbit.pitch=0.32;orbit.captured=look; }
    app.world_mut().resource_mut::<RunOptions>().combo=auto;
    app.world_mut().spawn((Avatar,Transform::IDENTITY));
    let mut configs=GizmoConfigStore::default();configs.insert(GizmoConfig::default(),DefaultGizmoConfigGroup);
    app.insert_resource(configs).init_resource::<GizmoStorage<DefaultGizmoConfigGroup,()>>();
    app.add_plugins((InputPlugin,TimePlugin));
    app.insert_resource(Time::<Fixed>::from_hz(120.));
    app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::ZERO));
    app.insert_resource(Clock86{present:true,..default()}).init_resource::<Trace86>().init_resource::<AimAudit86>();
    app.add_systems(PreUpdate,focus_marker86.in_set(bevy::ui::UiSystems::Focus).after(InputSystems));
    app.add_systems(FixedUpdate,(before86,step,after86).chain());
    app.add_systems(Update,(presentation.run_if(should_present86),frame86).chain());
    app.add_systems(Last,clear_gizmos86);
    app.update(); // identicalzero-time camera initialization, no fixed step
    assert!(app.world().resource::<Trace86>().ticks.is_empty());
    *app.world_mut().resource_mut::<Trace86>()=Trace86::default();
    (app,window)
}
#[derive(Clone,Debug)]
enum Event86 { Key(KeyCode,bool), Aim(bool), Motion(f32,f32) }
fn events86(scenario:&str)->Vec<(u64,Event86)> {
    use Event86::*;
    let ms=|n:u64|n*1_000_000;
    let mut events=match scenario {
        "sprint"=>vec![(0,Key(KeyCode::KeyW,true)),(0,Key(KeyCode::ShiftLeft,true)),
            (ms(2733),Key(KeyCode::KeyW,false)),(ms(2733),Key(KeyCode::ShiftLeft,false))],
        "manual"|"manual_look"=>vec![(0,Key(KeyCode::KeyW,true)),(0,Key(KeyCode::ShiftLeft,true)),
            (ms(560),Key(KeyCode::ControlLeft,true)),(ms(711),Key(KeyCode::Space,true)),
            (ms(718),Key(KeyCode::Space,false)),(ms(750),Key(KeyCode::ControlLeft,false)),
            (ms(1021),Key(KeyCode::Space,true)),(ms(1028),Key(KeyCode::Space,false)),
            (ms(1200),Aim(true)),(ms(2720),Aim(false)),(ms(2900),Key(KeyCode::KeyW,false)),
            (ms(2900),Key(KeyCode::ShiftLeft,false)),(ms(3110),Key(KeyCode::KeyQ,true)),(ms(3120),Key(KeyCode::KeyQ,false))],
        "key_combo"=>vec![(ms(173),Key(KeyCode::KeyG,true)),(ms(177),Key(KeyCode::KeyG,false))],
        "auto_combo"|"auto_frozen_camera"=>vec![],
        _=>panic!("unknownscenario")
    };
    if scenario=="manual_look" { events.extend([(ms(333),Motion(18.,-9.)),(ms(702),Motion(-31.,12.)),(ms(1004),Motion(7.,-4.))]); }
    events.sort_by_key(|e|e.0);events
}
fn send86(app:&mut App,window:Entity,event:&Event86) {
    let state=|b|if b{ButtonState::Pressed}else{ButtonState::Released};
    match event {
        Event86::Key(code,down)=>{app.world_mut().write_message(KeyboardInput{key_code:*code,
            logical_key:Key::Unidentified(NativeKey::Unidentified),state:state(*down),text:None,repeat:false,window});},
        Event86::Aim(down)=>{app.world_mut().write_message(MouseButtonInput{button:MouseButton::Right,state:state(*down),window});},
        Event86::Motion(x,y)=>{app.world_mut().write_message(MouseMotion{delta:Vec2::new(*x,*y)});}
    }
}
fn run86(scenario:&str,rate:u64)->Value {
    let (mut app,window)=app86(scenario.starts_with("auto_"),scenario=="manual_look");
    if scenario=="auto_frozen_camera" {app.world_mut().resource_mut::<Clock86>().present=false;}
    let events=events86(scenario);let mut index=0;let mut delivered=Vec::new();
    let total=5_000_000_000u64;let mut wall=0;let mut frame=0;let irregular=[4,29,7,17,41,3,11];
    while wall<total {
        frame+=1;
        let next=if rate==0{wall+irregular[(frame-1)%irregular.len()]*1_000_000}else{(frame as u64)*1_000_000_000/rate};
        let next=next.min(total);assert!(next>wall);
        {let mut clock=app.world_mut().resource_mut::<Clock86>();clock.frame=frame;clock.wall_ns=next;}
        while index<events.len()&&events[index].0<=next {
            let (at,event)=&events[index];send86(&mut app,window,event);
            delivered.push(json!({"index":index,"event":format!("{event:?}"),"scheduled_ns":at,"delivered_ns":next,
                "frame":frame,"delay_ns":next-at}));index+=1;
        }
        app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_nanos(next-wall)));app.update();wall=next;
    }
    let trace=app.world().resource::<Trace86>();assert_eq!(trace.ticks.len(),600);
    assert_eq!(app.world().resource::<Time<Virtual>>().elapsed().as_nanos(),u128::from(total));
    assert!(trace.ticks.iter().all(|t|t["position"].as_array().unwrap().iter().all(|x|x.as_f64().unwrap().is_finite())));
    json!({"scenario":scenario,"render_hz":rate,"irregular_ms":if rate==0{Some(irregular)}else{None},
        "wall_ns":total,"fixed_timestep_ns":app.world().resource::<Time<Fixed>>().timestep().as_nanos() as u64,
        "events":delivered,"ticks":trace.ticks,"frames":trace.frames})
}
#[test]
fn real_scheduler_render_cadence86() {
    let output=PathBuf::from(std::env::var("WF86_OUTPUT_DIR").expect("set new outputdirectory"));
    std::fs::create_dir_all(&output).unwrap();
    for scenario in ["sprint","manual","manual_look","key_combo","auto_combo","auto_frozen_camera"] {
        for rate in [30,60,120,144,240,0] {
            let report=run86(scenario,rate);let path=output.join(format!("{scenario}-{rate}.json"));assert!(!path.exists());
            std::fs::write(path,serde_json::to_vec(&report).unwrap()).unwrap();
            println!("PASS86 observed {scenario} render={rate} fixedticks=600");
        }
    }
    let baseline: Value=serde_json::from_slice(&std::fs::read(output.join("auto_combo-120.json")).unwrap()).unwrap();
    for scenario in ["auto_combo","auto_frozen_camera"] {
        for rate in [30,60,120,144,240,0] {
            let report: Value=serde_json::from_slice(&std::fs::read(output.join(format!("{scenario}-{rate}.json"))).unwrap()).unwrap();
            for (a,b) in baseline["ticks"].as_array().unwrap().iter().zip(report["ticks"].as_array().unwrap()) {
                assert_eq!(fixed_identity86(a),fixed_identity86(b),"auto identity {scenario} rate {rate} tick {}",a["tick"]);
            }
        }
    }
}

#[derive(Resource, Default)]
pub(super) struct AimAudit86 {
    pub input: MovementInput,
    pub origin: CoreVec3,
    pub boom: CoreVec3,
}
fn bits(v: CoreVec3) -> [u32;3] { v.to_array().map(f32::to_bits) }
fn output86(name: &str, value: &Value) {
    let p=PathBuf::from(std::env::var("WF86_OUTPUT_DIR").unwrap()).join(name);
    std::fs::create_dir_all(p.parent().unwrap()).unwrap();assert!(!p.exists());
    std::fs::write(p,serde_json::to_vec(value).unwrap()).unwrap();
}
// Overrides only the test's final fixed-tick input stream, after actual controls.
// This is separate from the 36 unchanged wall-clock event-delivery experiments.
fn fixed_samples86(mut lab: ResMut<Lab>, mut orbit: ResMut<Orbit>, fixed: Res<Time<Fixed>>) {
    let n=fixed.elapsed().as_nanos()/fixed.timestep().as_nanos();
    orbit.yaw=(n as f32)*0.001;orbit.pitch=0.25+(n as f32)*0.0002;
    lab.jump_edges.clear_pending();
    lab.input=MovementInput{movement:CoreVec3::NEG_Z,camera_forward:core_vec(forward(&orbit)),
        sprint:true,crouch:(35..=40).contains(&n),jump:n==40||n==78,aim:n>80,roll:false,aim_target:None,..Default::default()};
}
fn poison86(mut camera: Single<&mut Transform,With<FollowCamera>>, time: Res<Time>) {
    camera.translation=Vec3::new(10000.+time.elapsed_secs(),-7000.,-9000.);
    camera.rotation=Quat::from_rotation_x(2.5);
}
fn perturb_run86(mode: &str, rate: u64) -> Value {
    let (mut app,_)=app86(false,false);
    app.add_systems(FixedUpdate,fixed_samples86.before(before86));
    if mode=="omitted" {app.world_mut().resource_mut::<Clock86>().present=false;}
    if mode=="repeated" {app.add_systems(Update,presentation.after(frame86));}
    if mode=="poisoned" {
        app.add_systems(PreUpdate,poison86.before(controls));
        app.add_systems(Update,poison86.after(frame86));
    }
    let total=1_500_000_000u64;let mut wall=0;let mut frame=0;
    while wall<total {
        frame+=1;let next=((frame as u64)*1_000_000_000/rate).min(total);
        {let mut c=app.world_mut().resource_mut::<Clock86>();c.frame=frame;c.wall_ns=next;}
        app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_nanos(next-wall)));
        app.update();wall=next;
    }
    let t=app.world().resource::<Trace86>();assert_eq!(t.ticks.len(),180);
    json!({"mode":mode,"rate":rate,"ticks":t.ticks,"frames":t.frames})
}
fn fixed_identity86(t: &Value)->Value {
    json!([t["player_json"],t["position_bits"],t["velocity_bits"],
        // JSON strings distinguish signed zero for every serialized input/vector.
        serde_json::to_string(&t["consumed_input"]).unwrap(),
        serde_json::to_string(&t["aim_origin"]).unwrap(),serde_json::to_string(&t["aim_boom"]).unwrap()])
}
#[test]
fn fixed_input_survives_omitted_repeated_poisoned_and_varied_presentation86() {
    let baseline=perturb_run86("normal",120);output86("perturb-normal-120.json",&baseline);
    assert_ne!(baseline["ticks"][0]["aim_origin"],baseline["ticks"][100]["aim_origin"]);
    assert_ne!(baseline["frames"][0]["camera"],baseline["frames"][100]["camera"]);
    for mode in ["normal","omitted","repeated","poisoned"] {
        for rate in [30,144,240] {
            let report=perturb_run86(mode,rate);output86(&format!("perturb-{mode}-{rate}.json"),&report);
            for (a,b) in baseline["ticks"].as_array().unwrap().iter().zip(report["ticks"].as_array().unwrap()) {
                assert_eq!(fixed_identity86(a),fixed_identity86(b),"mode {mode} rate {rate} tick {}",a["tick"]);
            }
        }
    }
}
fn single_step_app86(world: World, feet: CoreVec3, crouched: bool, input: MovementInput) -> App {
    let (mut app,_,_)=viewer_tests::controls_app();
    {let mut lab=app.world_mut().resource_mut::<Lab>();lab.world=world;lab.player=Player::new(feet);
     lab.player.crouched=crouched;lab.input=input;lab.previous=CoreVec3::splat(999.);}
    app.world_mut().resource_mut::<Orbit>().pitch=0.;
    let mut time=Time::<Fixed>::from_hz(120.);time.advance_by(time.timestep());app.insert_resource(time);
    app.init_resource::<AimAudit86>();app
}
fn execute86(app: &mut App) {
    app.world_mut().run_system_cached(step).unwrap();
}
fn box_world86() -> World {
    use movement_lab::world::Solid;
    World::boxes(vec![
        Solid::new(CoreVec3::new(-20.,-2.,-20.),CoreVec3::new(20.,0.,20.)),
        Solid::new(CoreVec3::new(-3.,0.,2.),CoreVec3::new(3.,4.,3.)),
        Solid::new(CoreVec3::new(-3.,0.,-5.),CoreVec3::new(3.,4.,-4.)),
    ])
}
#[test]
fn obstruction_backoff_stance_mixed_ray_and_reset86() {
    let world=box_world86();let feet=CoreVec3::new(0.,0.05,0.);
    let mut observations=Vec::new();
    for crouched in [false,true] {
        // Input requests opposite stance, checking origin uses current pre-step height.
        let input=MovementInput{camera_forward:CoreVec3::NEG_Z,crouch:!crouched,jump:true,..neutral_input()};
        let mut app=single_step_app86(world.clone(),feet,crouched,input);
        let lab=app.world().resource::<Lab>();let h=lab.player.height(&lab.tuning);
        let expected_origin=CoreVec3::new(0.,feet.y+h*0.7,1.7125); // (2-.15)-5.5*.025
        let (_,origin)=camera_placement(&world,render_vec(feet),h,Vec3::NEG_Z);
        assert!(core_vec(origin).distance(expected_origin)<1e-6);
        let expected_target=CoreVec3::new(0.,expected_origin.y,-3.9);
        let mut expected_input=input;expected_input.aim_target=world.aim_target(core_vec(origin),input.camera_forward,5000.);
        assert!(expected_input.aim_target.unwrap().distance(expected_target)<1e-5);
        let mut expected=lab.player.clone();expected.step(&expected_input,app.world().resource::<Time<Fixed>>().delta_secs(),&world,&lab.tuning);
        execute86(&mut app);
        assert_eq!(serde_json::to_string(&expected).unwrap(),serde_json::to_string(&app.world().resource::<Lab>().player).unwrap());
        assert_eq!(bits(app.world().resource::<AimAudit86>().origin),bits(core_vec(origin)));
        observations.push(json!({"crouched":crouched,"origin":origin.to_array(),"target":expected_input.aim_target}));
        // Reset/teleport changes authoritative feet; previous and camera are stale.
        app.world_mut().resource_mut::<Lab>().player.respawn(CoreVec3::new(6.,0.05,0.));
        execute86(&mut app);assert!(app.world().resource::<AimAudit86>().origin.x>5.9);
    }
    // Ray direction intentionally differs from the orbit boom.
    let input=MovementInput{camera_forward:CoreVec3::X,..neutral_input()};
    let mut app=single_step_app86(World::boxes(vec![]),feet,false,input);execute86(&mut app);
    let a=app.world().resource::<AimAudit86>();assert_eq!(bits(a.boom),bits(CoreVec3::NEG_Z));
    let target=a.input.aim_target.unwrap();assert!((target.x-a.origin.x-5000.).abs()<0.001);
    assert_eq!(target.z.to_bits(),a.origin.z.to_bits());
    observations.push(json!({"mixed_ray_origin":a.origin,"mixed_ray_target":target}));
    output86("geometry-box-stance-reset.json",&json!(observations));
}
#[test]
fn triangle_obstruction_retains_longitudinal_backoff86() {
    let asset=movement_lab::static_terrain::StaticTerrain::course();let world=World::mesh(asset).unwrap();
    // Authored low tunnel occupies y1.2..2,z12..20. Camera ray rises into ceiling.
    let feet=CoreVec3::new(0.,0.05,16.);let h=0.7;let fwd=Vec3::NEG_Y;
    let (target,origin)=camera_placement(&world,render_vec(feet),h,fwd);
    assert!((target.y-0.54).abs()<1e-6);assert!((origin.y-1.05).abs()<1e-5);
    assert!((origin.y-target.y-5.5).abs()>1.);
    let hit=world.aim_target(core_vec(origin),CoreVec3::Y,5000.).unwrap();assert!((hit.y-1.1).abs()<1e-5);
    // Actual Fixed step uses mesh geometry and current crouched stance at this location.
    let input=MovementInput{camera_forward:CoreVec3::Y,..neutral_input()};
    let mut app=single_step_app86(world.clone(),feet,true,input);
    app.world_mut().resource_mut::<Orbit>().pitch=-std::f32::consts::FRAC_PI_2;
    let lab=app.world().resource::<Lab>();let (_,expected)=camera_placement(&world,render_vec(feet),lab.player.height(&lab.tuning),forward(app.world().resource::<Orbit>()));
    execute86(&mut app);let audit=app.world().resource::<AimAudit86>();
    assert_eq!(bits(audit.origin),bits(core_vec(expected)));
    assert_eq!(audit.input.aim_target,world.aim_target(core_vec(expected),CoreVec3::Y,5000.));
    output86("geometry-mesh.json",&json!({"target":target.to_array(),"origin":origin.to_array(),"hit":hit,"step_origin":audit.origin,"step_aim":audit.input.aim_target}));
}
#[test]
fn final_helper_aim_precedes_view_restore86() {
    let mut app=single_step_app86(box_world86(),CoreVec3::new(0.,3.,0.),false,neutral_input());
    let dt=app.world().resource::<Time<Fixed>>().delta_secs();
    let mut combo=Combo::default();combo.start();
    // Construct final genuine Combo::input pulse by executing its state machine.
    loop {
        let mut next=combo.clone();let grounded=matches!(combo.stage_label(),"Sprint"|"Slide"|"Bullet jump");
        let input=next.input(dt,grounded,CoreVec3::NEG_Z).unwrap();
        if !next.is_active() {assert!(input.aim);break;}
        combo=next;
    }
    {let mut lab=app.world_mut().resource_mut::<Lab>();lab.combo=combo;lab.combo_forward=CoreVec3::NEG_Z;lab.player.grounded=false;}
    {let mut orbit=app.world_mut().resource_mut::<Orbit>();orbit.pitch=0.32;orbit.yaw=0.23;orbit.combo_previous_pitch=Some(-0.4);}
    let lab=app.world().resource::<Lab>();let expected_boom=forward(app.world().resource::<Orbit>());
    let (_,expected)=camera_placement(&lab.world,render_vec(lab.player.position),lab.player.height(&lab.tuning),expected_boom);
    execute86(&mut app);
    let audit=app.world().resource::<AimAudit86>();assert!(audit.input.aim);
    assert_eq!(bits(audit.origin),bits(core_vec(expected)));assert_eq!(bits(audit.boom),bits(core_vec(expected_boom)));
    assert_eq!(app.world().resource::<Orbit>().pitch,-0.4);assert!(!app.world().resource::<Lab>().input.aim);
    let lab=app.world().resource::<Lab>();let (_,restored)=camera_placement(&lab.world,render_vec(lab.previous),lab.player.height(&lab.tuning),forward(app.world().resource::<Orbit>()));
    assert_ne!(bits(audit.origin),bits(core_vec(restored)));
    let final_pulse=json!({"consumed":audit.input,"origin":audit.origin,"boom":audit.boom,"restored_pitch":-0.4,"yaw":0.23});
    let resumed=core_vec(forward(app.world().resource::<Orbit>()));
    assert_eq!(bits(app.world().resource::<Lab>().input.camera_forward),bits(resumed));
    let mut following=Vec::new();
    for _ in 0..3 {
        let lab=app.world().resource::<Lab>();
        let (_,expected_origin)=camera_placement(&lab.world,render_vec(lab.player.position),lab.player.height(&lab.tuning),render_vec(resumed));
        let expected_target=lab.world.aim_target(core_vec(expected_origin),resumed,5000.);
        execute86(&mut app); // no controls/PreUpdate/render between these fixed steps
        let audit=app.world().resource::<AimAudit86>();
        assert_eq!(bits(audit.input.camera_forward),bits(resumed));
        assert_eq!(bits(audit.origin),bits(core_vec(expected_origin)));
        assert_eq!(audit.input.aim_target,expected_target);
        assert_eq!(audit.input.movement,CoreVec3::ZERO);
        assert!(!audit.input.jump&&!audit.input.roll&&!audit.input.aim&&!audit.input.sprint&&!audit.input.crouch);
        following.push(json!({"input":audit.input,"origin":audit.origin,"boom":audit.boom}));
    }
    output86("final-helper.json",&json!({"final_pulse":final_pulse,"following_fixed_steps_without_controls":following}));
}

#[derive(Resource)]
struct WallStream90 {tick:usize,direction:CoreVec3}
fn wall_stream90(mut lab:ResMut<Lab>,mut stream:ResMut<WallStream90>) {
    let n=stream.tick;stream.tick+=1;
    // Explicit same-fixed-tick diagnostic input. Raw event delivery is separately
    // tested by InputPlugin integration; this does not claim device timing parity.
    lab.input=MovementInput{movement:stream.direction,camera_forward:stream.direction,
        jump:n==0,jump_held:n<600,..Default::default()};
}
fn wall_run90(direction:CoreVec3,rate:u64)->Value {
    let (mut app,_)=app86(false,false);
    {
        let mut lab=app.world_mut().resource_mut::<Lab>();
        lab.world=World::boxes(vec![movement_lab::Solid::new(CoreVec3::new(-50.,-2.,-100.),CoreVec3::new(50.,0.,100.)),
            movement_lab::Solid::new(CoreVec3::new(0.5,0.,-80.),CoreVec3::new(2.,40.,80.))]);
        lab.player=Player::new(CoreVec3::new(0.,3.,0.));lab.previous=lab.player.position;
    }
    app.world_mut().resource_mut::<Orbit>().yaw=direction.x.atan2(-direction.z);
    app.insert_resource(WallStream90{tick:0,direction});
    app.add_systems(FixedUpdate,wall_stream90.before(step).after(before86));
    let mut elapsed=0u64;let mut frame=0usize;let total=6_000_000_000u64;let irregular=[4,29,7,17,41,3,11];
    while elapsed<total {
        frame+=1;let next=(if rate==0 {elapsed+irregular[(frame-1)%7]*1_000_000}
            else {frame as u64*1_000_000_000/rate}).min(total);
        {let mut c=app.world_mut().resource_mut::<Clock86>();c.frame=frame;c.wall_ns=next;}
        app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::from_nanos(next-elapsed)));app.update();elapsed=next;
    }
    let trace=app.world().resource::<Trace86>();assert_eq!(trace.ticks.len(),720);
    let held_end:Player=serde_json::from_str(trace.ticks[599]["player_json"].as_str().unwrap()).unwrap();
    let end=app.world().resource::<Lab>();
    assert!(held_end.wall_traversal().launches>=2);assert!(!end.player.wall_traversal().held);
    assert!(end.player.wall_traversal().launches<=held_end.wall_traversal().launches+1);
    assert!(trace.ticks.iter().all(|t|serde_json::from_str::<Player>(t["player_json"].as_str().unwrap()).unwrap().wall_traversal().error.is_none()));
    json!({"direction":direction,"render_hz":rate,"ticks":trace.ticks,"frames":trace.frames,
        "scope":"same fixed tick input, actual Bevy scheduling and presentation; LAB query substep latency"})
}
#[test]
fn held_up_and_both_across_release_are_render_independent90() {
    for (name,direction) in [("up",CoreVec3::X),("positive",CoreVec3::new(0.25,0.,0.9682458)),("negative",CoreVec3::new(0.25,0.,-0.9682458))] {
        let baseline=wall_run90(direction,120);output86(&format!("wall90-{name}-120.json"),&baseline);
        for rate in [30,60,144,240,0] {
            let other=wall_run90(direction,rate);output86(&format!("wall90-{name}-{rate}.json"),&other);
            for (a,b) in baseline["ticks"].as_array().unwrap().iter().zip(other["ticks"].as_array().unwrap()) {
                assert_eq!(fixed_identity86(a),fixed_identity86(b),"{name} render {rate} tick {}",a["tick"]);
            }
        }
    }
}
