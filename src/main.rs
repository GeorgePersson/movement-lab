use bevy::{
    asset::RenderAssetUsages,
    input::mouse::AccumulatedMouseMotion,
    mesh::PrimitiveTopology,
    prelude::*,
    render::view::screenshot::{Screenshot, save_to_disk},
    window::{
        CursorGrabMode, CursorOptions, MonitorSelection, PresentMode, PrimaryWindow, WindowMode,
        WindowResizeConstraints,
    },
};
use glam::Vec3 as CoreVec3;
use movement_lab::combo::Combo;
use movement_lab::{Input as MovementInput, Player, Tuning, World};
use std::{collections::VecDeque, path::PathBuf};

mod jump_edges;
use jump_edges::JumpEdges;

const SPAWN: CoreVec3 = CoreVec3::new(0.0, 0.05, 8.0);

#[derive(Resource)]
struct Lab {
    player: Player,
    world: World,
    tuning: Tuning,
    previous: CoreVec3,
    input: MovementInput,
    seconds: f32,
    trail: VecDeque<CoreVec3>,
    show_trail: bool,
    tuning_path: PathBuf,
    notice: String,
    combo: Combo,
    combo_forward: CoreVec3,
    jump_edges: JumpEdges,
}

impl Lab {
    fn take_step_input(&mut self) -> MovementInput {
        let mut input = self.input;
        // Physical controls mark their stream authoritative. Programmatic combo/
        // diagnostic samples remain the explicitly separate legacy adapter.
        if input.jump_actions.authoritative || self.jump_edges.is_pending() || self.jump_edges.action_len()>0 {
            input.jump_actions=self.jump_edges.packet();input.jump_held=self.jump_edges.held();
            input.jump_edge_id=None;
            if let Some(edge)=self.jump_edges.take_delivered(input.jump_actions.first_id+input.jump_actions.len as u64) {
                input.jump=true;input.crouch=edge.crouch;input.jump_edge_id=Some(edge.id);
            }
        }
        self.input.jump = false;
        self.input.roll = false;
        input
    }
}

#[derive(Resource)]
struct Orbit {
    yaw: f32,
    pitch: f32,
    captured: bool,
    combo_previous_pitch: Option<f32>,
}

#[derive(Resource, Default)]
struct RunOptions {
    demo: bool,
    fullscreen: bool,
    duration: Option<f32>,
    capture: Option<PathBuf>,
    captured: bool,
    combo: bool,
    combo_started: bool,
    mesh_course: bool,
}

#[derive(Component)]
struct Avatar;
#[derive(Component)]
struct FollowCamera;
#[derive(Component)]
struct Telemetry;
#[derive(Component)]
struct ComboButton;
#[derive(Component)]
struct ComboButtonText;
#[derive(Component)]
struct ComboStatus;

fn neutral_input() -> MovementInput {
    MovementInput {
        movement: CoreVec3::ZERO,
        camera_forward: CoreVec3::NEG_Z,
        aim_target: None,
        jump: false,
        crouch: false,
        sprint: false,
        aim: false,
        roll: false,
        ..Default::default()
    }
}

// The prepared mesh path cannot resolve initial overlap. Apply this capability
// policy after loading tuning, both on startup and on F5, without changing any
// supported numeric settings or relaxing World's rejection of unsupported calls.
fn apply_world_tuning_policy(world: &World, tuning: &mut Tuning) -> Option<&'static str> {
    if world.mesh_terrain().is_some() && tuning.collision.overlap_recovery {
        tuning.collision.overlap_recovery = false;
        Some("Mesh course: overlap recovery disabled (unsupported)")
    } else {
        None
    }
}

fn main() {
    let mut options = RunOptions::default();
    let mut tuning_path = PathBuf::from("presets/static-tenno-bullet.json");
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--world" => {
                options.mesh_course = match args.next().as_deref() {
                    Some("graybox") => false,
                    Some("mesh-course") => true,
                    _ => panic!("--world needs graybox or mesh-course"),
                }
            }
            "--demo" => options.demo = true,
            "--combo" => options.combo = true,
            "--fullscreen" => options.fullscreen = true,
            "--duration" => {
                options.duration = Some(
                    args.next()
                        .expect("--duration needs seconds")
                        .parse()
                        .expect("invalid duration"),
                );
            }
            "--capture" => {
                options.capture = Some(args.next().expect("--capture needs a path").into())
            }
            "--tuning" => tuning_path = args.next().expect("--tuning needs a path").into(),
            "--help" | "-h" => {
                println!(
                    "movement-lab [--world graybox|mesh-course] [--fullscreen] [--demo | --combo] [--duration SECONDS] [--capture PATH.png] [--tuning PATH.json]"
                );
                return;
            }
            _ => panic!("Unknown argument: {arg}; use --help"),
        }
    }
    let mut tuning: Tuning = match std::fs::read_to_string(&tuning_path) {
        Ok(text) => serde_json::from_str(&text)
            .unwrap_or_else(|e| panic!("Invalid tuning {}: {e}", tuning_path.display())),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
            let tuning = Tuning::default();
            if let Err(e) =
                std::fs::write(&tuning_path, serde_json::to_string_pretty(&tuning).unwrap())
            {
                eprintln!("Could not save default tuning: {e}");
            }
            tuning
        }
        Err(e) => panic!("Cannot read tuning {}: {e}", tuning_path.display()),
    };
    assert!(
        tuning.is_valid(),
        "Tuning values must be finite and satisfy geometry/duration bounds"
    );
    let window_mode = if options.fullscreen {
        WindowMode::BorderlessFullscreen(MonitorSelection::Current)
    } else {
        WindowMode::Windowed
    };
    let world = if options.mesh_course {
        World::mesh(movement_lab::static_terrain::StaticTerrain::course())
            .expect("checked triangle course")
    } else {
        World::graybox()
    };
    let policy_notice = apply_world_tuning_policy(&world, &mut tuning);
    let startup_notice = match policy_notice {
        Some(notice) => format!("{notice} | Click to look around | Tab releases the cursor"),
        None => "Click to look around | Tab releases the cursor".into(),
    };
    let spawn = world.spawn();
    assert!(
        world.can_occupy(spawn, tuning.radius, tuning.standing_height),
        "selected spawn must fit capsule"
    );
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.025, 0.045, 0.065)))
        .insert_resource(GlobalAmbientLight {
            color: Color::srgb(0.7, 0.85, 1.0),
            brightness: 260.0,
            ..default()
        })
        .insert_resource(Time::<Fixed>::from_hz(120.0))
        .insert_resource(Orbit {
            yaw: 0.0,
            pitch: -0.14,
            captured: false,
            combo_previous_pitch: None,
        })
        .insert_resource(Lab {
            player: Player::new(spawn),
            world,
            tuning,
            previous: spawn,
            input: neutral_input(),
            seconds: 0.0,
            trail: VecDeque::new(),
            show_trail: true,
            tuning_path,
            notice: startup_notice,
            combo: Combo::default(),
            combo_forward: CoreVec3::NEG_Z,
            jump_edges: JumpEdges::default(),
        })
        .insert_resource(options)
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Movement Lab — Parkour Research".into(),
                resolution: (1280, 800).into(),
                mode: window_mode,
                resize_constraints: WindowResizeConstraints {
                    min_width: 800.0,
                    min_height: 600.0,
                    ..default()
                },
                present_mode: PresentMode::AutoVsync,
                ..default()
            }),
            ..default()
        }))
        .add_systems(Startup, setup)
        .add_systems(PreUpdate, controls.after(bevy::ui::UiSystems::Focus))
        .add_systems(FixedUpdate, step)
        .add_systems(
            Update,
            (presentation, telemetry, combo_ui, capture_and_exit).chain(),
        )
        .run();
}

fn render_vec(v: CoreVec3) -> Vec3 {
    Vec3::new(v.x, v.y, v.z)
}
fn core_vec(v: Vec3) -> CoreVec3 {
    CoreVec3::new(v.x, v.y, v.z)
}
fn forward(orbit: &Orbit) -> Vec3 {
    Vec3::new(
        orbit.yaw.sin() * orbit.pitch.cos(),
        orbit.pitch.sin(),
        -orbit.yaw.cos() * orbit.pitch.cos(),
    )
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    lab: Res<Lab>,
) {
    let ground = materials.add(StandardMaterial {
        base_color: Color::srgb(0.14, 0.20, 0.24),
        perceptual_roughness: 0.92,
        ..default()
    });
    let wall = materials.add(StandardMaterial {
        base_color: Color::srgb(0.28, 0.39, 0.44),
        perceptual_roughness: 0.8,
        ..default()
    });
    let ledge = materials.add(StandardMaterial {
        base_color: Color::srgb(0.22, 0.52, 0.52),
        perceptual_roughness: 0.7,
        ..default()
    });
    for solid in &lab.world.solids {
        let size = render_vec(solid.max - solid.min);
        let center = render_vec((solid.max + solid.min) * 0.5);
        let material = if solid.max.y <= 0.1 {
            ground.clone()
        } else if size.y > 3.0 {
            wall.clone()
        } else {
            ledge.clone()
        };
        commands.spawn((
            Mesh3d(meshes.add(Cuboid::from_size(size))),
            MeshMaterial3d(material),
            Transform::from_translation(center),
        ));
    }
    if let Some(asset) = lab.world.mesh_terrain() {
        // Exact shared collision triangles, including source winding; flat normals.
        let mut positions = Vec::<[f32; 3]>::new();
        let mut normals = Vec::<[f32; 3]>::new();
        for triangle in asset.triangles() {
            let a = Vec3::from(triangle[0]);
            let b = Vec3::from(triangle[1]);
            let c = Vec3::from(triangle[2]);
            let normal = (b - a).cross(c - a).normalize().to_array();
            positions.extend_from_slice(triangle);
            normals.extend_from_slice(&[normal; 3]);
        }
        let mut mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
        commands.spawn((
            Mesh3d(meshes.add(mesh)),
            MeshMaterial3d(ledge.clone()),
            Transform::default(),
        ));
    }
    commands.spawn((
        Avatar,
        Mesh3d(meshes.add(Capsule3d::new(0.3, 1.2))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.22, 0.96, 0.80),
            metallic: 0.2,
            perceptual_roughness: 0.35,
            ..default()
        })),
        Transform::from_translation(render_vec(lab.world.spawn()) + Vec3::Y * 0.9),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 12000.0,
            ..default()
        },
        Transform::from_xyz(10.0, 18.0, 7.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        FollowCamera,
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.5, 14.0).looking_at(render_vec(lab.world.spawn()), Vec3::Y),
    ));

    commands
        .spawn((Node {
            position_type: PositionType::Absolute,
            left: px(28),
            top: px(24),
            flex_direction: FlexDirection::Column,
            row_gap: px(5),
            ..default()
        },))
        .with_children(|p| {
            p.spawn((
                Text::new("MOVEMENT / LAB"),
                TextFont {
                    font_size: FontSize::Px(28.0),
                    ..default()
                },
                TextColor(Color::srgb(0.35, 1.0, 0.86)),
            ));
            p.spawn((
                Text::new("PARKOUR RESEARCH  /  UNMEASURED TUNING"),
                TextFont {
                    font_size: FontSize::Px(12.0),
                    ..default()
                },
                TextColor(Color::srgb(0.65, 0.78, 0.83)),
            ));
        });
    commands.spawn((
        Telemetry,
        Text::new(""),
        TextFont {
            font_size: FontSize::Px(17.0),
            ..default()
        },
        TextColor(Color::srgb(0.86, 0.95, 0.98)),
        Node {
            position_type: PositionType::Absolute,
            left: px(28),
            top: px(108),
            ..default()
        },
    ));
    commands.spawn((
        Text::new("WASD  move     Shift  sprint     Ctrl / C  slide\nSpace  jump / double jump     Ctrl + Space  bullet jump\nWall jumps: hold Space + move toward/along a wall\nRight mouse  aim glide / wall latch     Alt / Q  roll\nG  full combo / cancel     R  reset     F2  trail\nF5  reload tuning     Tab  cursor     Esc  exit"),
        TextFont { font_size: FontSize::Px(15.0), ..default() },
        TextColor(Color::srgb(0.8, 0.89, 0.93)),
        Node { position_type: PositionType::Absolute, left: px(28), bottom: px(28), ..default() },
    ));
    commands
        .spawn((Node {
            position_type: PositionType::Absolute,
            right: px(28),
            top: px(24),
            width: px(270),
            flex_direction: FlexDirection::Column,
            row_gap: px(10),
            ..default()
        },))
        .with_children(|p| {
            p.spawn((
                ComboButton,
                Button,
                Node {
                    width: percent(100),
                    height: px(48),
                    justify_content: JustifyContent::Center,
                    align_items: AlignItems::Center,
                    border_radius: BorderRadius::all(px(8)),
                    ..default()
                },
                BackgroundColor(Color::srgb(0.12, 0.48, 0.40)),
            ))
            .with_children(|b| {
                b.spawn((
                    ComboButtonText,
                    Text::new("Run full combo   [G]"),
                    TextFont {
                        font_size: FontSize::Px(18.),
                        ..default()
                    },
                    TextColor(Color::WHITE),
                ));
            });
            p.spawn((
                Text::new("Sprint > slide > bullet jump\n> double jump > glide"),
                TextFont {
                    font_size: FontSize::Px(15.),
                    ..default()
                },
                TextColor(Color::srgb(0.8, 0.89, 0.93)),
            ));
            p.spawn((
                ComboStatus,
                Text::new(
                    "Start on the ground. Uses upward aim.\nMovement keys cancel. Tab frees mouse.",
                ),
                TextFont {
                    font_size: FontSize::Px(13.),
                    ..default()
                },
                TextColor(Color::srgb(0.65, 0.78, 0.83)),
            ));
        });
    commands.spawn((
        Text::new("+"),
        TextFont {
            font_size: FontSize::Px(22.0),
            ..default()
        },
        TextColor(Color::srgba(0.8, 1.0, 0.93, 0.7)),
        Node {
            position_type: PositionType::Absolute,
            left: percent(50),
            top: percent(50),
            ..default()
        },
    ));
}

fn controls(
    keys: Res<ButtonInput<KeyCode>>,
    mut keyboard_events: MessageReader<bevy::input::keyboard::KeyboardInput>,
    mut focus_losses: MessageReader<bevy::input::keyboard::KeyboardFocusLost>,
    primary_window: Single<Entity, With<PrimaryWindow>>,
    mouse: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    mut window: Single<(&Window, &mut CursorOptions), With<PrimaryWindow>>,
    mut orbit: ResMut<Orbit>,
    mut lab: ResMut<Lab>,
    options: Res<RunOptions>,
    mut exit: MessageWriter<AppExit>,
    combo_button: Single<Ref<Interaction>, With<ComboButton>>,
) {
    let lost_focus = focus_losses.read().count() != 0;
    let accepts_input = window.0.focused && !lost_focus && !options.demo;
    if !accepts_input { lab.jump_edges.reset(); }
    let prior_edge_count = lab.jump_edges.pending_len();
    let prior_action_count=lab.jump_edges.action_len();
    for event in keyboard_events.read() {
        if accepts_input && event.window == *primary_window {
            lab.jump_edges.record(event.key_code, event.state, event.repeat);
        }
    }
    // Keep new manual edges separate while helper transitions clear older
    // pending inputs. Public viewer behavior gives fresh manual input priority
    // after G/button handling; the same-frame chord must not disappear there.
    let mut frame_jump_edges = lab.jump_edges.detach_since(prior_edge_count);
    let mut frame_wall_actions=lab.jump_edges.detach_actions_since(prior_action_count);
    let over_button = **combo_button != Interaction::None;
    let button_pressed = combo_button.is_changed() && **combo_button == Interaction::Pressed;
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
    if keys.just_pressed(KeyCode::Tab) || ((!window.0.focused || lost_focus) && orbit.captured) {
        orbit.captured = false;
        window.1.grab_mode = CursorGrabMode::None;
        window.1.visible = true;
    } else if mouse.just_pressed(MouseButton::Left) && window.0.focused && !over_button {
        orbit.captured = true;
        window.1.grab_mode = CursorGrabMode::Locked;
        window.1.visible = false;
    }
    if orbit.captured {
        if motion.delta != Vec2::ZERO {
            // An intentional look adjustment replaces the temporary guided view.
            orbit.combo_previous_pitch = None;
        }
        orbit.yaw += motion.delta.x * 0.0022;
        orbit.pitch = (orbit.pitch - motion.delta.y * 0.0022).clamp(-1.35, 1.35);
        lab.notice = "Sprint > slide > bullet jump > double jump > aim glide".into();
    }
    if keys.just_pressed(KeyCode::KeyR) {
        frame_jump_edges.clear();
        frame_wall_actions.clear();
        lab.combo.cancel();
        restore_combo_view(&mut orbit);
        lab.jump_edges.reset();
        lab.input = neutral_input();
        let spawn = lab.world.spawn();
        lab.player.respawn(spawn);
        lab.previous = spawn;
        lab.trail.clear();
        lab.input.jump_actions.authoritative=true;
        lab.notice = "Reset to start".into();
    }
    if keys.just_pressed(KeyCode::F2) {
        lab.show_trail = !lab.show_trail;
    }
    if keys.just_pressed(KeyCode::F5) {
        match std::fs::read_to_string(&lab.tuning_path)
            .map_err(|e| e.to_string())
            .and_then(|s| serde_json::from_str::<Tuning>(&s).map_err(|e| e.to_string()))
        {
            Ok(mut tuning)
                if tuning.is_valid()
                    && lab.world.can_occupy(
                        lab.player.position,
                        tuning.radius,
                        lab.player.height(&tuning),
                    ) =>
            {
                let policy_notice = apply_world_tuning_policy(&lab.world, &mut tuning);
                lab.tuning = tuning;
                lab.notice = match policy_notice {
                    Some(notice) => format!("Tuning reloaded | {notice}"),
                    None => "Tuning reloaded".into(),
                };
            }
            Ok(_) => {
                lab.notice = "Tuning rejected: invalid values or current hull would overlap".into();
            }
            Err(error) => {
                eprintln!("Tuning reload failed: {error}");
                lab.notice = "Tuning reload failed; keeping previous values".into();
            }
        }
    }
    if options.demo {
        return;
    }
    if keys.just_pressed(KeyCode::KeyR) {return;}
    if !window.0.focused || lost_focus {
        let interrupted_combo = lab.combo.is_active();
        lab.combo.cancel();
        if interrupted_combo {
            lab.notice = "Combo cancelled - window lost focus".into();
        }
        restore_combo_view(&mut orbit);
        lab.jump_edges.reset();
        lab.player.cancel_wall_input();
        lab.input = neutral_input();
        lab.input.jump_actions.authoritative=true;
        return;
    }
    if keys.just_pressed(KeyCode::KeyG) || button_pressed {
        toggle_combo(&mut lab, &mut orbit);
    }
    if lab.combo.is_active()
        && (lab.jump_edges.is_pending() || !frame_jump_edges.is_empty() || keys.any_pressed([
            KeyCode::KeyW,
            KeyCode::KeyA,
            KeyCode::KeyS,
            KeyCode::KeyD,
            KeyCode::Space,
            KeyCode::KeyC,
            KeyCode::ControlLeft,
            KeyCode::AltLeft,
            KeyCode::KeyQ,
        ]) || mouse.pressed(MouseButton::Right))
    {
        lab.combo.cancel();
        restore_combo_view(&mut orbit);
        lab.player.wall_transport_boundary();
        lab.notice = "Combo cancelled - manual control".into();
    }
    let fwd = Vec3::new(orbit.yaw.sin(), 0.0, -orbit.yaw.cos());
    let right = fwd.cross(Vec3::Y);
    let x = (keys.pressed(KeyCode::KeyD) as i32 - keys.pressed(KeyCode::KeyA) as i32) as f32;
    let z = (keys.pressed(KeyCode::KeyW) as i32 - keys.pressed(KeyCode::KeyS) as i32) as f32;
    lab.input.movement = core_vec((right * x + fwd * z).clamp_length_max(1.0));
    lab.input.camera_forward = core_vec(forward(&orbit));
    // The final fixed-step sample resolves aim from authoritative player state.
    lab.input.aim_target = None;
    lab.input.crouch = keys.pressed(KeyCode::ControlLeft) || keys.pressed(KeyCode::KeyC);
    lab.input.sprint = keys.pressed(KeyCode::ShiftLeft);
    lab.input.aim = mouse.pressed(MouseButton::Right);
    // Keep edges until a fixed step consumes them, even at >120 render FPS.
    // Jump press posture and multiple press edges come from ordered raw
    // KeyboardInput messages above, independently of final held-key state.
    lab.jump_edges.append_pending(frame_jump_edges);
    lab.jump_edges.append_actions(frame_wall_actions);
    lab.input.jump_actions.authoritative=true;
    lab.input.roll |= keys.just_pressed(KeyCode::AltLeft) || keys.just_pressed(KeyCode::KeyQ);
}

fn toggle_combo(lab: &mut Lab, orbit: &mut Orbit) {
    if lab.combo.is_active() {
        lab.jump_edges.clear_pending();
        lab.player.cancel_wall_input();
        lab.combo.cancel();
        restore_combo_view(orbit);
        lab.input = neutral_input();
        lab.notice = "Combo cancelled".into();
    } else if !lab.player.grounded {
        lab.notice = "Land first, then press G for the combo".into();
    } else if lab.player.jumps_remaining < 2 || lab.player.bullet_jumps_left == 0 {
        lab.notice = "Combo needs two jumps and a bullet jump. Press R to reset.".into();
    } else {
        lab.jump_edges.clear_pending();
        lab.player.cancel_wall_input();
        // Guided input aid, not recovered animation timing or aim behavior.
        orbit.combo_previous_pitch = Some(orbit.pitch);
        orbit.pitch = orbit.pitch.max(0.32);
        lab.combo_forward = core_vec(forward(orbit));
        lab.combo.start();
        lab.input = neutral_input();
        lab.notice = "Full combo running - G or movement keys cancel".into();
    }
}

fn restore_combo_view(orbit: &mut Orbit) {
    if let Some(pitch) = orbit.combo_previous_pitch.take() {
        orbit.pitch = pitch;
    }
}

fn combo_ui(
    lab: Res<Lab>,
    mut button: Single<(&Interaction, &mut BackgroundColor), With<ComboButton>>,
    mut label: Single<&mut Text, With<ComboButtonText>>,
    mut status: Single<&mut Text, (With<ComboStatus>, Without<ComboButtonText>)>,
) {
    label.0 = if lab.combo.is_active() {
        "Cancel combo   [G]"
    } else {
        "Run full combo   [G]"
    }
    .into();
    status.0 = if lab.combo.is_active() {
        format!(
            "Now: {}\nG or movement keys cancel.",
            lab.combo.stage_label()
        )
    } else {
        "Start on the ground. Uses upward aim.\nMovement keys cancel. Tab frees mouse.".into()
    };
    button.1.0 = if *button.0 == Interaction::Hovered {
        Color::srgb(0.17, 0.62, 0.51)
    } else if lab.combo.is_active() {
        Color::srgb(0.38, 0.29, 0.15)
    } else {
        Color::srgb(0.12, 0.48, 0.40)
    };
}

fn step(
    mut lab: ResMut<Lab>,
    time: Res<Time<Fixed>>,
    mut options: ResMut<RunOptions>,
    mut orbit: ResMut<Orbit>,
    #[cfg(test)] mut aim_audit: Option<ResMut<render_rate86::AimAudit86>>,
) {
    let dt = time.delta_secs();
    let old = lab.seconds;
    lab.seconds += dt;
    if options.combo && !options.combo_started && lab.player.grounded {
        toggle_combo(&mut lab, &mut orbit);
        options.combo_started = true;
    }
    // Keep boom orientation independent of the selected demo/combo ray. Capture
    // before a finishing helper restores the manual view for the next tick.
    let aim_view_forward = forward(&orbit);
    if options.demo {
        let now = lab.seconds;
        let hit = |t: f32| old < t && now >= t;
        lab.input = MovementInput {
            movement: CoreVec3::NEG_Z,
            camera_forward: CoreVec3::new(0.0, 0.32, -0.9474),
            aim_target: None,
            sprint: true,
            crouch: (1.4..1.9).contains(&now),
            jump: hit(1.6) || hit(2.5),
            aim: (2.7..4.3).contains(&now),
            roll: hit(4.6),
            ..Default::default()
        };
    }
    let combo_was_active = lab.combo.is_active();
    if lab.combo.is_active() {
        let grounded = lab.player.grounded;
        let direction = lab.combo_forward;
        let prior_stage = lab.combo.stage_label();
        lab.input = lab
            .combo
            .input(dt, grounded, direction)
            .unwrap_or_else(neutral_input);
        if options.combo && lab.combo.stage_label() != prior_stage {
            eprintln!(
                "COMBO_STAGE {} -> {} at {:.3}s",
                prior_stage,
                lab.combo.stage_label(),
                lab.seconds
            );
        }
        if !lab.combo.is_active() {
            restore_combo_view(&mut orbit);
            lab.notice = "Combo finished - press G to run it again after landing".into();
        }
    }
    lab.previous = lab.player.position;
    let combo_finished = combo_was_active && !lab.combo.is_active();
    let mut sampled_input = lab.take_step_input();
    if options.demo || combo_was_active {
        sampled_input.jump_actions=Default::default();sampled_input.jump_edge_id=None;
        sampled_input.jump_held=false;
    }
    // LAB phase policy: pre-step feet and current stance, not interpolated render
    // state or the stance requested by this input. Native matrix binding is open.
    let (_, aim_origin) = camera_placement(
        &lab.world,
        render_vec(lab.player.position),
        lab.player.height(&lab.tuning),
        aim_view_forward,
    );
    sampled_input.aim_target = lab.world.aim_target(
        core_vec(aim_origin), sampled_input.camera_forward, 5000.,
    );
    lab.input.aim_target = sampled_input.aim_target;
    #[cfg(test)]
    if let Some(audit) = aim_audit.as_mut() {
        audit.input = sampled_input;
        audit.origin = core_vec(aim_origin);
        audit.boom = core_vec(aim_view_forward);
    }
    // Disjoint borrowing makes the core independent from Bevy ECS.
    let Lab {
        player,
        world,
        tuning,
        input,
        ..
    } = &mut *lab;
    player.step(&sampled_input, dt, world, tuning);
    if combo_finished {
        player.wall_transport_boundary();
        // Hand off the restored view at this fixed tick. Waiting for controls
        // would leave catch-up ticks on neutral_input's unrelated NEG_Z ray.
        *input = MovementInput {
            camera_forward: core_vec(forward(&orbit)),
            jump_actions: movement_lab::wall_traversal::JumpActions{authoritative:true,..Default::default()},
            ..neutral_input()
        };
    }
    if lab.player.position.y < -20.0 {
        lab.combo.cancel();
        restore_combo_view(&mut orbit);
        let spawn = lab.world.spawn();
        lab.player.respawn(spawn);
        lab.previous = spawn;
    }
    let position = lab.player.position;
    if lab
        .trail
        .back()
        .is_none_or(|last| last.distance_squared(position) > 0.01)
    {
        lab.trail.push_back(position);
        if lab.trail.len() > 360 {
            lab.trail.pop_front();
        }
    }
}

// Shared LAB orbit geometry. Boxes use expanded AABBs; mesh clearance is a
// longitudinal ray backoff. Callers explicitly choose simulation or render feet.
fn camera_placement(world: &World, feet: Vec3, height: f32, view_forward: Vec3) -> (Vec3, Vec3) {
    let target = feet + Vec3::Y * (height * 0.7);
    let desired = target - view_forward * 5.5;
    let delta = desired - target;
    let fraction = world.camera_fraction(core_vec(target), core_vec(desired), 0.15);
    (target, target + delta * fraction)
}

fn presentation(
    lab: Res<Lab>,
    orbit: Res<Orbit>,
    fixed: Res<Time<Fixed>>,
    mut avatar: Single<&mut Transform, (With<Avatar>, Without<FollowCamera>)>,
    mut camera: Single<&mut Transform, (With<FollowCamera>, Without<Avatar>)>,
    mut gizmos: Gizmos,
) {
    let feet = render_vec(
        lab.previous
            .lerp(lab.player.position, fixed.overstep_fraction()),
    );
    let height = lab.player.height(&lab.tuning);
    avatar.translation = feet + Vec3::Y * height * 0.5;
    avatar.scale.y = height / 1.8;
    avatar.scale.x = lab.tuning.radius / 0.3;
    avatar.scale.z = lab.tuning.radius / 0.3;
    let (target, origin) = camera_placement(&lab.world, feet, height, forward(&orbit));
    camera.translation = origin;
    camera.look_at(target, Vec3::Y);
    for n in -12..=12 {
        let k = n as f32 * 2.0;
        let color = Color::srgba(0.23, 0.39, 0.43, 0.5);
        gizmos.line(Vec3::new(-24.0, 0.014, k), Vec3::new(24.0, 0.014, k), color);
        gizmos.line(Vec3::new(k, 0.014, -24.0), Vec3::new(k, 0.014, 24.0), color);
    }
    if lab.show_trail {
        let mut previous = None;
        for point in &lab.trail {
            let point = render_vec(*point) + Vec3::Y * 0.15;
            if let Some(last) = previous {
                gizmos.line(last, point, Color::srgb(0.2, 0.9, 0.72));
            }
            previous = Some(point);
        }
    }
}

fn telemetry(lab: Res<Lab>, mut text: Single<&mut Text, With<Telemetry>>, time: Res<Time>) {
    let v = lab.player.combined_velocity();
    text.0 = format!(
        "{:?}\n\nSPEED  {:5.1}     VERTICAL  {:+5.1}\nHEIGHT {:5.1}     {}\n\nJUMPS  {}    BULLET  {}    ROLL  {}\nGLIDE  {:3.0}%\n\n{}\n{:.0} FPS",
        lab.player.state,
        CoreVec3::new(v.x, 0.0, v.z).length(),
        v.y,
        lab.player.position.y,
        if lab.player.grounded {
            "GROUNDED"
        } else {
            "AIRBORNE"
        },
        lab.player.jumps_remaining,
        lab.player.bullet_jumps_left,
        lab.player.air_rolls_left,
        lab.player.aim_resource * 100.0,
        lab.notice,
        1.0 / time.delta_secs().max(0.001)
    );
}

fn capture_and_exit(
    mut commands: Commands,
    time: Res<Time>,
    mut options: ResMut<RunOptions>,
    mut exit: MessageWriter<AppExit>,
) {
    let elapsed = time.elapsed_secs();
    if !options.captured && elapsed > 4.0 {
        if let Some(path) = &options.capture {
            if let Some(parent) = path.parent() {
                let _ = std::fs::create_dir_all(parent);
            }
            commands
                .spawn(Screenshot::primary_window())
                .observe(save_to_disk(path.clone()));
            options.captured = true;
        }
    }
    if options
        .duration
        .is_some_and(|duration| elapsed >= duration.max(6.0))
    {
        exit.write(AppExit::Success);
    }
}

#[cfg(test)]
mod viewer_tests {
    use super::*;

    pub(super) fn controls_app() -> (App, Entity, Entity) {
        let world = World::graybox();
        let tuning = Tuning::default();
        let mut player = Player::new(SPAWN);
        player.step(&neutral_input(), 0.1, &world, &tuning);
        player.wall_transport_boundary();
        assert!(player.grounded);
        let mut app = App::new();
        app.insert_resource(Lab {
            previous: player.position,
            player,
            world,
            tuning,
            input: neutral_input(),
            seconds: 0.,
            trail: VecDeque::new(),
            show_trail: true,
            tuning_path: PathBuf::new(),
            notice: String::new(),
            combo: Combo::default(),
            combo_forward: CoreVec3::NEG_Z,
            jump_edges: JumpEdges::default(),
        });
        app.insert_resource(Orbit {
            yaw: 0.,
            pitch: -0.14,
            captured: false,
            combo_previous_pitch: None,
        });
        app.init_resource::<ButtonInput<KeyCode>>();
        app.init_resource::<ButtonInput<MouseButton>>();
        app.init_resource::<AccumulatedMouseMotion>();
        app.init_resource::<RunOptions>();
        app.add_message::<AppExit>();
        app.world_mut().spawn((FollowCamera, Transform::IDENTITY));
        let window = app
            .world_mut()
            .spawn((
                PrimaryWindow,
                Window {
                    focused: true,
                    ..default()
                },
                CursorOptions::default(),
            ))
            .id();
        let button = app.world_mut().spawn((ComboButton, Interaction::None)).id();
        app.add_message::<bevy::input::keyboard::KeyboardInput>();
        app.add_message::<bevy::input::keyboard::KeyboardFocusLost>();
        app.add_systems(PreUpdate, controls.after(bevy::ui::UiSystems::Focus));
        (app, window, button)
    }

    #[test]
    fn button_starts_combo_without_capturing_pointer_and_g_cancels() {
        let (mut app, window, button) = controls_app();
        *app.world_mut().get_mut::<Interaction>(button).unwrap() = Interaction::Pressed;
        app.world_mut()
            .resource_mut::<ButtonInput<MouseButton>>()
            .press(MouseButton::Left);
        app.update();
        assert!(app.world().resource::<Lab>().combo.is_active());
        assert!(!app.world().resource::<Orbit>().captured);
        assert_eq!(
            app.world().get::<CursorOptions>(window).unwrap().grab_mode,
            CursorGrabMode::None
        );
        assert!(app.world().resource::<Lab>().combo_forward.y >= 0.31);
        app.world_mut()
            .resource_mut::<ButtonInput<MouseButton>>()
            .reset_all();
        *app.world_mut().get_mut::<Interaction>(button).unwrap() = Interaction::None;
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyG);
        app.update();
        assert!(!app.world().resource::<Lab>().combo.is_active());
        assert_eq!(app.world().resource::<Orbit>().pitch, -0.14);
    }

    #[test]
    fn keyboard_combo_cancels_on_manual_control_or_focus_loss() {
        let (mut app, window, _) = controls_app();
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyG);
        app.update();
        assert!(app.world().resource::<Lab>().combo.is_active());
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .reset_all();
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyW);
        app.update();
        assert!(!app.world().resource::<Lab>().combo.is_active());
        assert_eq!(
            app.world().resource::<Lab>().input.movement,
            CoreVec3::NEG_Z
        );
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .reset_all();
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyG);
        app.update();
        assert!(app.world().resource::<Lab>().combo.is_active());
        app.world_mut().get_mut::<Window>(window).unwrap().focused = false;
        app.update();
        assert!(!app.world().resource::<Lab>().combo.is_active());
        assert_eq!(app.world().resource::<Lab>().input.movement, CoreVec3::ZERO);
    }

    #[test]
    fn short_ctrl_space_chord_survives_release_before_fixed_step_once() {
        let (mut app, window, _) = controls_app();
        let mut keys = app.world_mut().resource_mut::<ButtonInput<KeyCode>>();
        keys.press(KeyCode::ControlLeft);
        keys.press(KeyCode::Space);
        drop(keys);
        for key in [KeyCode::ControlLeft, KeyCode::Space] {
            app.world_mut().write_message(bevy::input::keyboard::KeyboardInput {
                key_code: key,
                logical_key: bevy::input::keyboard::Key::Unidentified(
                    bevy::input::keyboard::NativeKey::Unidentified),
                state: bevy::input::ButtonState::Pressed,
                text: None,
                repeat: false,
                window,
            });
        }
        app.update();
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .reset_all();
        app.update(); // another render update before a simulation step
        // A rejected airborne combo request must not erase the queued chord.
        app.world_mut().resource_mut::<Lab>().player.grounded = false;
        app.world_mut()
            .resource_mut::<ButtonInput<KeyCode>>()
            .press(KeyCode::KeyG);
        app.update();
        app.world_mut().resource_mut::<Lab>().player.grounded = true;
        let mut lab = app.world_mut().resource_mut::<Lab>();
        assert!(!lab.input.crouch);
        let input = lab.take_step_input();
        assert!(input.jump && input.crouch);
        let Lab {
            player,
            world,
            tuning,
            ..
        } = &mut *lab;
        player.step(&input, 1. / 120., world, tuning);
        assert_eq!(
            player.bullet_jumps_left, 0,
            "must perform bullet jump, not ordinary jump"
        );
        let next = lab.take_step_input();
        assert!(!next.jump && !next.crouch);
    }
}

// Headless integration: real InputPlugin/TimePlugin/Main/Fixed scheduler and
// production controls/step. UI Focus is a named marker, NOT real UiPlugin.
#[cfg(test)]
mod input_timing_integration {
    use super::*;
    use bevy::input::{InputPlugin, InputSystems, ButtonState};
    use bevy::input::keyboard::{KeyboardInput, KeyboardFocusLost, Key, NativeKey};
    use bevy::time::{TimePlugin, TimeUpdateStrategy};
    use std::time::Duration;

    #[derive(Clone, Debug)]
    struct Observed {
        queued: Vec<bool>,
        jumps: u8,
        bullets: u8,
        combo: bool,
    }
    fn observed(lab: &Lab) -> Observed {
        Observed { queued: lab.jump_edges.pending_snapshot(),
            jumps: lab.player.jumps_remaining, bullets: lab.player.bullet_jumps_left,
            combo: lab.combo.is_active() }
    }
    #[derive(Resource, Default)]
    struct Trace { phases: Vec<&'static str>, before: Vec<Observed>, after: Vec<Observed> }
    fn ui_focus_marker(keys: Res<ButtonInput<KeyCode>>, mut trace: ResMut<Trace>) {
        // Reading the InputPlugin result orders this marker after the real input
        // fold. It does NOT simulate UI hit testing or mouse interaction.
        let _ = keys.just_pressed(KeyCode::Space);
        trace.phases.push("UI Focus marker");
    }
    fn after_controls(mut trace: ResMut<Trace>) { trace.phases.push("controls"); }
    fn before_step(lab: Res<Lab>, mut trace: ResMut<Trace>) {
        trace.phases.push("Fixed before"); trace.before.push(observed(&lab));
    }
    fn after_step(lab: Res<Lab>, mut trace: ResMut<Trace>) {
        trace.phases.push("Fixed after"); trace.after.push(observed(&lab));
    }
    fn update_marker(mut trace: ResMut<Trace>) { trace.phases.push("Update"); }
    fn app() -> (App, Entity, Entity) {
        // Reuses only resource/entities and production registration. The real
        // InputPlugin below owns held-key and edge derivation in these tests.
        let (mut app, window, button) = viewer_tests::controls_app();
        // Controlled upward look isolates transport from immediate floor contact.
        app.world_mut().resource_mut::<Orbit>().pitch = 0.32;
        {
            let mut cameras = app.world_mut().query_filtered::<&mut Transform, With<FollowCamera>>();
            cameras.single_mut(app.world_mut()).unwrap().translation = Vec3::new(0., 1.8, 12.);
        }
        app.add_plugins((InputPlugin, TimePlugin));
        app.init_resource::<Trace>();
        app.insert_resource(Time::<Fixed>::from_hz(120.));
        app.insert_resource(TimeUpdateStrategy::ManualDuration(Duration::ZERO));
        app.add_systems(PreUpdate,
            ui_focus_marker.in_set(bevy::ui::UiSystems::Focus).after(InputSystems));
        app.add_systems(PreUpdate, after_controls.after(controls));
        app.add_systems(FixedUpdate, (before_step, step, after_step).chain());
        app.add_systems(Update, update_marker);
        app.update(); // Initialize real time with zero elapsed and no input.
        *app.world_mut().resource_mut::<Trace>() = Trace::default();
        (app, window, button)
    }
    fn key(app: &mut App, window: Entity, code: KeyCode, down: bool, repeat: bool) {
        app.world_mut().write_message(KeyboardInput { key_code: code,
            logical_key: Key::Unidentified(NativeKey::Unidentified),
            state: if down { ButtonState::Pressed } else { ButtonState::Released },
            text: None, repeat, window });
    }
    fn batch(app: &mut App, window: Entity, keys: &[(KeyCode, bool)]) {
        for &(code, down) in keys { key(app, window, code, down, false); }
    }
    fn advance(app: &mut App, duration: Duration) {
        app.insert_resource(TimeUpdateStrategy::ManualDuration(duration)); app.update();
    }
    fn ticks(app: &mut App, count: u32) {
        let d = app.world().resource::<Time<Fixed>>().timestep() * count;
        advance(app, d);
    }
    fn queued(app: &App) -> Vec<bool> { app.world().resource::<Lab>().jump_edges.pending_snapshot() }

    #[test]
    fn released_chord_survives_no_fixed_frames_then_launches_once() {
        let (mut app, w, _) = app();
        batch(&mut app, w, &[(KeyCode::ControlLeft,true),(KeyCode::Space,true),
            (KeyCode::Space,false),(KeyCode::ControlLeft,false)]);
        advance(&mut app, Duration::ZERO);
        assert_eq!(queued(&app), [true]);
        assert!(app.world().resource::<Trace>().before.is_empty());
        // Another real InputPlugin frame clears transient ButtonInput edges.
        advance(&mut app, Duration::ZERO);
        assert_eq!(queued(&app), [true]);
        ticks(&mut app, 1);
        let trace = app.world().resource::<Trace>();
        assert_eq!(trace.before.len(), 1); assert_eq!(trace.before[0].queued, [true]);
        assert_eq!((trace.after[0].jumps,trace.after[0].bullets),(1,0));
        assert!(trace.after[0].queued.is_empty());
        let phases = &trace.phases[trace.phases.len()-5..];
        assert_eq!(phases, ["UI Focus marker","controls","Fixed before","Fixed after","Update"]);
        ticks(&mut app, 1);
        let trace = app.world().resource::<Trace>();
        assert!(trace.before[1].queued.is_empty());
        assert_eq!((trace.after[1].jumps,trace.after[1].bullets),(1,0));
    }

    #[test]
    fn raw_space_before_ctrl_is_not_reinterpreted_as_bullet() {
        let (mut app,w,_) = app();
        batch(&mut app,w,&[(KeyCode::Space,true),(KeyCode::ControlLeft,true)]);
        ticks(&mut app,1);
        let trace = app.world().resource::<Trace>();
        assert_eq!(trace.before[0].queued,[false]);
        assert_eq!(trace.after[0].bullets,1,"earlier Space posture must stay ordinary");
        assert_eq!(trace.after[0].jumps,1);
    }

    #[test]
    fn distinct_rapid_presses_are_consumed_in_separate_catchup_ticks() {
        let (mut app,w,_) = app();
        // Bullet followed by ordinary double jump in one render event batch.
        batch(&mut app,w,&[(KeyCode::ControlLeft,true),(KeyCode::Space,true),
            (KeyCode::Space,false),(KeyCode::ControlLeft,false),
            (KeyCode::Space,true),(KeyCode::Space,false)]);
        ticks(&mut app,3);
        let trace = app.world().resource::<Trace>();
        assert_eq!(trace.before.len(),3);
        assert_eq!(trace.before[0].queued,[true,false]);
        assert_eq!(trace.before[1].queued,[false]);
        assert!(trace.before[2].queued.is_empty());
        assert_eq!((trace.after[0].jumps,trace.after[0].bullets),(1,0));
        assert_eq!((trace.after[1].jumps,trace.after[1].bullets),(0,0));
        assert_eq!((trace.after[2].jumps,trace.after[2].bullets),(0,0));
    }

    #[test]
    fn repeats_duplicates_other_windows_and_focus_loss_have_no_stale_edges() {
        let (mut app,w,_) = app();
        let other = app.world_mut().spawn_empty().id();
        key(&mut app,other,KeyCode::Space,true,false);
        key(&mut app,w,KeyCode::Space,true,true);
        advance(&mut app,Duration::ZERO); assert!(queued(&app).is_empty());
        batch(&mut app,w,&[(KeyCode::ControlLeft,true),(KeyCode::Space,true),
            (KeyCode::Space,true)]);
        key(&mut app,w,KeyCode::Space,true,true);
        advance(&mut app,Duration::ZERO); assert_eq!(queued(&app),[true]);
        // KeyboardFocusLost is a unit message, globally releases cached input.
        // The production app has one primary window; no nonexistent window
        // field is invented for this Bevy message.
        app.world_mut().write_message(KeyboardFocusLost);
        advance(&mut app,Duration::ZERO); assert!(queued(&app).is_empty());
        key(&mut app,w,KeyCode::Space,true,false);
        advance(&mut app,Duration::ZERO); assert_eq!(queued(&app),[false]);
        app.world_mut().get_mut::<Window>(w).unwrap().focused=false;
        advance(&mut app,Duration::ZERO); assert!(queued(&app).is_empty());
    }

    #[test]
    fn reset_clears_edges_airborne_combo_rejection_preserves_manual_jump() {
        let (mut app,w,_) = app();
        batch(&mut app,w,&[(KeyCode::ControlLeft,true),(KeyCode::Space,true),
            (KeyCode::Space,false),(KeyCode::ControlLeft,false)]);
        advance(&mut app,Duration::ZERO); assert_eq!(queued(&app),[true]);
        key(&mut app,w,KeyCode::KeyR,true,false);
        advance(&mut app,Duration::ZERO); assert!(queued(&app).is_empty());
        key(&mut app,w,KeyCode::KeyR,false,false);
        key(&mut app,w,KeyCode::Space,true,false);
        app.world_mut().resource_mut::<Lab>().player.grounded=false;
        key(&mut app,w,KeyCode::KeyG,true,false);
        advance(&mut app,Duration::ZERO);
        assert_eq!(queued(&app),[false]);
        assert!(!app.world().resource::<Lab>().combo.is_active());
        assert!(app.world().resource::<Lab>().notice.contains("Land first"));
    }

    #[test]
    fn released_manual_jump_cancels_combo_and_is_not_overwritten() {
        let (mut app,w,_) = app();
        key(&mut app,w,KeyCode::KeyG,true,false);
        advance(&mut app,Duration::ZERO);
        assert!(app.world().resource::<Lab>().combo.is_active());
        batch(&mut app,w,&[(KeyCode::KeyG,false),(KeyCode::ControlLeft,true),
            (KeyCode::Space,true),(KeyCode::Space,false),(KeyCode::ControlLeft,false)]);
        ticks(&mut app,1);
        let trace=app.world().resource::<Trace>();
        assert!(!trace.before[0].combo); assert_eq!(trace.before[0].queued,[true]);
        assert_eq!((trace.after[0].jumps,trace.after[0].bullets),(1,0));
    }

    #[test]
    fn admitted_roll_rejects_jump_once_and_transport_never_retries() {
        let (mut app,w,_) = app();
        batch(&mut app,w,&[(KeyCode::AltLeft,true),(KeyCode::ControlLeft,true),
            (KeyCode::Space,true),(KeyCode::Space,false),(KeyCode::ControlLeft,false),
            (KeyCode::AltLeft,false)]);
        ticks(&mut app,1);
        let trace=app.world().resource::<Trace>();
        assert_eq!(trace.before[0].queued,[true]);
        assert!(trace.after[0].queued.is_empty());
        assert_eq!((trace.after[0].jumps,trace.after[0].bullets),(2,1));
        for _ in 0..60 { ticks(&mut app,1); } // past .42s without virtual-time clamp
        let trace=app.world().resource::<Trace>();
        assert!(trace.before[1..].iter().all(|s|s.queued.is_empty()));
        assert!(trace.after.iter().all(|s|s.jumps==2 && s.bullets==1));
    }
    #[test]
    fn g_start_and_fresh_space_same_frame_keeps_manual_jump_in_either_event_order() {
        for order in [
            [(KeyCode::KeyG,true),(KeyCode::Space,true)],
            [(KeyCode::Space,true),(KeyCode::KeyG,true)],
        ] {
            let (mut app,w,_) = app();
            batch(&mut app,w,&order);
            ticks(&mut app,1);
            let trace=app.world().resource::<Trace>();
            assert!(!trace.before[0].combo,"manual input supersedes the helper");
            assert_eq!(trace.before[0].queued,[false]);
            assert_eq!((trace.after[0].jumps,trace.after[0].bullets),(1,1));
            assert!(trace.after[0].queued.is_empty());
        }
    }

    #[test]
    fn g_cancel_and_fresh_space_same_frame_still_dispatches_manual_jump() {
        let (mut app,w,_) = app();
        key(&mut app,w,KeyCode::KeyG,true,false);
        advance(&mut app,Duration::ZERO);
        assert!(app.world().resource::<Lab>().combo.is_active());
        key(&mut app,w,KeyCode::KeyG,false,false);
        advance(&mut app,Duration::ZERO);
        batch(&mut app,w,&[(KeyCode::KeyG,true),(KeyCode::Space,true)]);
        ticks(&mut app,1);
        let trace=app.world().resource::<Trace>();
        assert!(!trace.before[0].combo); assert_eq!(trace.before[0].queued,[false]);
        assert_eq!((trace.after[0].jumps,trace.after[0].bullets),(1,1));
    }

    #[test]
    fn button_start_and_fully_released_fresh_chord_dispatches_manual_bullet() {
        let (mut app,w,button) = app();
        // Interaction is supplied explicitly: this tests controls priority,
        // not real UiPlugin pointer hit testing or a physical button click.
        *app.world_mut().get_mut::<Interaction>(button).unwrap()=Interaction::Pressed;
        batch(&mut app,w,&[(KeyCode::ControlLeft,true),(KeyCode::Space,true),
            (KeyCode::Space,false),(KeyCode::ControlLeft,false)]);
        ticks(&mut app,1);
        let trace=app.world().resource::<Trace>();
        assert!(!trace.before[0].combo); assert_eq!(trace.before[0].queued,[true]);
        assert_eq!((trace.after[0].jumps,trace.after[0].bullets),(1,0));
        assert!(!app.world().resource::<Orbit>().captured);
    }

    #[test]
    fn helper_transition_discards_stale_prior_edges_but_retains_only_fresh_edges() {
        for fresh in [false,true] {
            let (mut app,w,_) = app();
            batch(&mut app,w,&[(KeyCode::Space,true),(KeyCode::Space,false)]);
            advance(&mut app,Duration::ZERO);
            assert_eq!(queued(&app),[false],"prior frame ordinary pulse is pending");
            key(&mut app,w,KeyCode::KeyG,true,false);
            if fresh {
                batch(&mut app,w,&[(KeyCode::ControlLeft,true),(KeyCode::Space,true),
                    (KeyCode::Space,false),(KeyCode::ControlLeft,false)]);
            }
            ticks(&mut app,1);
            let trace=app.world().resource::<Trace>();
            if fresh {
                assert!(!trace.before[0].combo);
                assert_eq!(trace.before[0].queued,[true],"old ordinary pulse must be cleared");
                assert_eq!((trace.after[0].jumps,trace.after[0].bullets),(1,0));
            } else {
                assert!(trace.before[0].combo);
                assert!(trace.before[0].queued.is_empty());
                assert_eq!((trace.after[0].jumps,trace.after[0].bullets),(2,1));
            }
        }
    }
    #[test]
    fn released_c_and_overlap_modifier_releases_preserve_crouch_at_space_edge() {
        for events in [
            vec![(KeyCode::KeyC,true),(KeyCode::Space,true),
                (KeyCode::Space,false),(KeyCode::KeyC,false)],
            vec![(KeyCode::ControlLeft,true),(KeyCode::KeyC,true),
                (KeyCode::KeyC,false),(KeyCode::Space,true),
                (KeyCode::Space,false),(KeyCode::ControlLeft,false)],
            vec![(KeyCode::ControlLeft,true),(KeyCode::KeyC,true),
                (KeyCode::ControlLeft,false),(KeyCode::Space,true),
                (KeyCode::Space,false),(KeyCode::KeyC,false)],
        ] {
            let (mut app,w,_) = app(); batch(&mut app,w,&events); ticks(&mut app,1);
            let trace=app.world().resource::<Trace>();
            assert_eq!(trace.before[0].queued,[true]);
            assert_eq!((trace.after[0].jumps,trace.after[0].bullets),(1,0));
        }
    }

    #[test]
    fn focus_reset_modifier_repeat_resynchronizes_without_repeated_space_pulse() {
        let (mut app,w,_) = app();
        key(&mut app,w,KeyCode::KeyC,true,false);
        advance(&mut app,Duration::ZERO);
        app.world_mut().write_message(KeyboardFocusLost);
        advance(&mut app,Duration::ZERO);
        key(&mut app,w,KeyCode::KeyC,true,true);
        key(&mut app,w,KeyCode::Space,true,true);
        advance(&mut app,Duration::ZERO);
        assert!(queued(&app).is_empty(),"Space auto-repeat is not a fresh jump");
        key(&mut app,w,KeyCode::Space,true,false);
        ticks(&mut app,1);
        let trace=app.world().resource::<Trace>();
        assert_eq!(trace.before[0].queued,[true]);
        assert_eq!((trace.after[0].jumps,trace.after[0].bullets),(1,0));
    }
    include!("wall90_viewer_tests.rs");
}

// Isolated pass86 scheduling and camera ownership validation.
#[cfg(test)]
mod render_rate86;

#[cfg(test)]
mod mesh_reload85;
