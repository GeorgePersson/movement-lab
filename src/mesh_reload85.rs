//! Actual controls -> F5 file load -> production step regression; no renderer.
use super::*;
use std::{
    sync::atomic::{AtomicUsize, Ordering},
    time::Duration,
};
static NEXT: AtomicUsize = AtomicUsize::new(0);
struct Fixture(PathBuf);
impl Fixture {
    fn new(text: &str) -> Self {
        let path = std::env::temp_dir().join(format!(
            "movement-lab-reload85-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::create_dir(&path).unwrap();
        std::fs::write(path.join("tuning.json"), text).unwrap();
        Self(path)
    }
    fn path(&self) -> PathBuf {
        self.0.join("tuning.json")
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}
fn preset() -> &'static str {
    include_str!("../presets/static-tenno-bullet.json")
}
fn harness(mesh: bool, path: PathBuf) -> App {
    let (mut app, _, _) = viewer_tests::controls_app();
    let world = if mesh {
        World::mesh(movement_lab::static_terrain::StaticTerrain::course()).unwrap()
    } else {
        World::graybox()
    };
    let mut tuning: Tuning = serde_json::from_str(preset()).unwrap();
    apply_world_tuning_policy(&world, &mut tuning);
    let mut player = Player::new(world.spawn());
    for _ in 0..60 {
        player.step(&neutral_input(), 1. / 120., &world, &tuning);
    }
    assert!(player.grounded);
    {
        let mut lab = app.world_mut().resource_mut::<Lab>();
        lab.previous = player.position;
        lab.player = player;
        lab.world = world;
        lab.tuning = tuning;
        lab.tuning_path = path;
    }
    let mut time = Time::<Fixed>::from_hz(120.);
    time.advance_by(Duration::from_secs_f64(1. / 120.));
    app.insert_resource(time);
    app.add_systems(FixedUpdate, step);
    app
}
fn reload(app: &mut App) {
    app.world_mut()
        .resource_mut::<ButtonInput<KeyCode>>()
        .press(KeyCode::F5);
    app.update();
    app.world_mut()
        .resource_mut::<ButtonInput<KeyCode>>()
        .reset_all();
}
fn walk(app: &mut App) -> f32 {
    let before = app.world().resource::<Lab>().player.position;
    app.world_mut()
        .resource_mut::<ButtonInput<KeyCode>>()
        .press(KeyCode::KeyW);
    app.update();
    for _ in 0..60 {
        app.world_mut().run_schedule(FixedUpdate);
    }
    let after = app.world().resource::<Lab>().player.position;
    CoreVec3::new(after.x - before.x, 0., after.z - before.z).length()
}
#[test]
fn default_preset_f5_preserves_mesh_movement() {
    let fixture = Fixture::new(preset());
    let mut control = harness(true, fixture.path());
    let mut reloaded = harness(true, fixture.path());
    assert!(
        serde_json::from_str::<Tuning>(preset())
            .unwrap()
            .collision
            .overlap_recovery
    );
    reload(&mut reloaded);
    let enabled = reloaded
        .world()
        .resource::<Lab>()
        .tuning
        .collision
        .overlap_recovery;
    let notice = reloaded.world().resource::<Lab>().notice.clone();
    let control_distance = walk(&mut control);
    let reloaded_distance = walk(&mut reloaded);
    let report = serde_json::json!({"overlap_recovery_after_f5":enabled,"notice":notice,"control_distance":control_distance,"reloaded_distance":reloaded_distance});
    println!("RELOAD85 {report}");
    if let Ok(out) = std::env::var("WF85_OUTPUT_DIR") {
        std::fs::create_dir_all(&out).unwrap();
        std::fs::write(
            PathBuf::from(out).join("mesh-default-reload.json"),
            serde_json::to_string_pretty(&report).unwrap(),
        )
        .unwrap();
    }
    assert!(
        control_distance > 0.5,
        "control must actually move: {report}"
    );
    assert!(
        reloaded_distance > 0.5,
        "F5 must not freeze mesh movement: {report}"
    );
    assert!(!enabled);
    assert!((control_distance - reloaded_distance).abs() < 0.00001);
}

#[test]
fn startup_and_reload_normalize_only_unsupported_mesh_overlap_policy() {
    for mesh in [false, true] {
        for requested_overlap in [false, true] {
            let mut requested: Tuning = serde_json::from_str(preset()).unwrap();
            requested.walk_speed = 4.25;
            requested.gravity = 21.5;
            requested.collision.contact_offset = 0.013;
            requested.collision.step_offset = 0.31;
            requested.collision.engine.slope_cosine = 0.8;
            requested.collision.overlap_recovery = requested_overlap;
            assert!(requested.is_valid());
            let original = serde_json::to_value(&requested).unwrap();
            let original_text = serde_json::to_string(&requested).unwrap();
            let fixture = Fixture::new(&original_text);
            let mut app = harness(mesh, fixture.path());
            let notice =
                apply_world_tuning_policy(&app.world().resource::<Lab>().world, &mut requested);
            let changed = mesh && requested_overlap;
            assert_eq!(notice.is_some(), changed);
            let mut expected = original.clone();
            if changed {
                expected["collision"]["overlap_recovery"] = serde_json::json!(false);
            }
            assert_eq!(
                serde_json::to_value(&requested).unwrap(),
                expected,
                "startup must preserve every other serialized setting"
            );
            reload(&mut app);
            let lab = app.world().resource::<Lab>();
            assert_eq!(
                serde_json::to_value(&lab.tuning).unwrap(),
                expected,
                "F5 must match startup and preserve numeric values"
            );
            assert_eq!(
                lab.notice
                    .contains("overlap recovery disabled (unsupported)"),
                changed
            );
            assert!(lab.notice.starts_with("Tuning reloaded"));
            assert_eq!(
                std::fs::read_to_string(fixture.path()).unwrap(),
                original_text,
                "normalization must not rewrite source JSON bytes"
            );
            assert!(
                walk(&mut app) > 0.5,
                "accepted tuning must remain movable (mesh={mesh},overlap={requested_overlap})"
            );
        }
    }
}

#[test]
fn rejected_and_failed_f5_leave_previous_tuning_unchanged() {
    for mesh in [false, true] {
        let fixture = Fixture::new("not JSON");
        let mut app = harness(mesh, fixture.path());
        let before = serde_json::to_value(&app.world().resource::<Lab>().tuning).unwrap();
        reload(&mut app);
        assert!(
            app.world()
                .resource::<Lab>()
                .notice
                .starts_with("Tuning reload failed")
        );
        assert_eq!(
            serde_json::to_value(&app.world().resource::<Lab>().tuning).unwrap(),
            before
        );
        let mut invalid: Tuning = serde_json::from_str(preset()).unwrap();
        invalid.radius = -1.;
        std::fs::write(fixture.path(), serde_json::to_string(&invalid).unwrap()).unwrap();
        reload(&mut app);
        assert!(
            app.world()
                .resource::<Lab>()
                .notice
                .starts_with("Tuning rejected")
        );
        assert_eq!(
            serde_json::to_value(&app.world().resource::<Lab>().tuning).unwrap(),
            before
        );
    }
    // Existing low tunnel admits crouching but rejects increasing that hull.
    let fixture = Fixture::new(preset());
    let mut app = harness(true, fixture.path());
    {
        let mut lab = app.world_mut().resource_mut::<Lab>();
        lab.player.position = CoreVec3::new(0., 0.04, 16.);
        lab.player.crouched = true;
        assert!(lab.world.can_occupy(
            lab.player.position,
            lab.tuning.radius,
            lab.player.height(&lab.tuning)
        ));
    }
    let before = serde_json::to_value(&app.world().resource::<Lab>().tuning).unwrap();
    let mut too_tall: Tuning = serde_json::from_str(preset()).unwrap();
    too_tall.crouch_height = 1.3;
    assert!(too_tall.is_valid());
    std::fs::write(fixture.path(), serde_json::to_string(&too_tall).unwrap()).unwrap();
    reload(&mut app);
    assert!(
        app.world()
            .resource::<Lab>()
            .notice
            .starts_with("Tuning rejected")
    );
    assert_eq!(
        serde_json::to_value(&app.world().resource::<Lab>().tuning).unwrap(),
        before
    );
}
