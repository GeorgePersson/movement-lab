//! Actual Player -> prepared triangle scene -> retained publication -> slope tests.
use super::*;
fn tuning() -> Tuning {
    let mut t: Tuning =
        serde_json::from_str(include_str!("../presets/static-tenno-bullet.json")).unwrap();
    t.collision.overlap_recovery = false;
    t.slide_boost = 0.;
    t.slide_end_speed = 0.;
    t
}
fn supported(world: &World, t: &Tuning, ramp: bool) -> Player {
    let mut p = Player::new(if ramp {
        Vec3::new(0., 1.12, -6.)
    } else {
        world.spawn()
    });
    p.crouched = true;
    p.previously_crouch = true;
    for _ in 0..240 {
        p.step(
            &Input {
                crouch: true,
                ..Default::default()
            },
            MAX_SUBSTEP,
            world,
            t,
        );
    }
    assert!(p.grounded, "failed to settle: {:?}", p.position);
    assert_eq!(p.engine_contact.mask & 4, 4);
    let n = p.engine_contact.working_normal;
    if ramp {
        assert!(n[1] > 0.9 && n[2] < -0.2, "wrong published normal {n:?}");
    } else {
        assert!(n[1] > 0.9999);
    }
    p
}
#[test]
fn downhill_uphill_and_flat_trajectories_use_published_support() {
    let w = World::mesh(crate::static_terrain::StaticTerrain::course()).unwrap();
    let t = tuning();
    let ramp = supported(&w, &t, true);
    let flat = supported(&w, &t, false);
    let mut rows = Vec::new();
    for (name, mut p, speed) in [
        ("downhill", ramp.clone(), -3.),
        ("uphill", ramp.clone(), 3.),
        ("flat", flat, 3.),
    ] {
        p.velocity = Vec3::Z * speed;
        p.slide_requested = true;
        let start = p.position;
        let initial_normal = p.engine_contact.working_normal;
        let mut samples = Vec::new();
        for i in 0..120 {
            p.step(
                &Input {
                    crouch: true,
                    ..Default::default()
                },
                MAX_SUBSTEP,
                &w,
                &t,
            );
            assert!(p.position.is_finite());
            assert!(w.can_occupy(p.position, t.radius, p.height(&t)));
            assert!(p.grounded, "{name} lost support at{i}");
            assert_eq!(p.engine_contact.mask & 4, 4);
            assert_eq!(p.slide_velocity(), Vec3::ZERO);
            if i % 12 == 0 {
                samples.push(serde_json::json!({"tick":i,"p":p.position.to_array(),"base":p.velocity.to_array(),"normal":p.engine_contact.working_normal,"mask":p.engine_contact.mask}));
            }
        }
        rows.push(serde_json::json!({"case":name,"delta":(p.position-start).to_array(),"final_base":p.velocity.to_array(),"initial_normal":initial_normal,"samples":samples}));
    }
    println!("SLOPE88 {}", serde_json::to_string(&rows).unwrap());
    if let Ok(out) = std::env::var("WF88_OUTPUT_DIR") {
        std::fs::create_dir_all(&out).unwrap();
        std::fs::write(
            std::path::PathBuf::from(out).join("trajectories.json"),
            serde_json::to_string_pretty(&rows).unwrap(),
        )
        .unwrap();
    }
    let down = rows[0]["delta"][2].as_f64().unwrap();
    let up = rows[1]["delta"][2].as_f64().unwrap();
    let flat = rows[2]["delta"][2].as_f64().unwrap();
    assert!(down < -1.5 && up > 0. && up < -down && flat > 0.);
    assert!(rows[0]["final_base"][2].as_f64().unwrap() < -3.);
    assert!(rows[1]["final_base"][2].as_f64().unwrap() < 3.);
    assert!(rows[2]["final_base"][2].as_f64().unwrap() < 3.);
}
#[test]
fn binding_uses_working_normal_and_mask_not_other_normal_or_actor_grounding() {
    let w = World::mesh(crate::static_terrain::StaticTerrain::course()).unwrap();
    let t = tuning();
    let mut p = supported(&w, &t, true);
    p.velocity = Vec3::NEG_Z * 3.;
    p.slide_requested = true;
    let mut other = p.clone();
    other.engine_contact.other_normal = [1., 0., 0., 0.];
    other.engine_contact.normal_y = -1.;
    p.step(
        &Input {
            crouch: true,
            ..Default::default()
        },
        MAX_SUBSTEP,
        &w,
        &t,
    );
    other.step(
        &Input {
            crouch: true,
            ..Default::default()
        },
        MAX_SUBSTEP,
        &w,
        &t,
    );
    assert_eq!(p.velocity, other.velocity);
    assert_eq!(p.position, other.position);
    // Actor ground may be true with no bit4; current admission is LAB, and the
    // selector must take ordinary input when actual native-role mask lacksbit4.
    let input = Input {
        crouch: true,
        movement: Vec3::X,
        ..Default::default()
    };
    let mut missing = supported(&w, &t, true);
    missing.velocity = Vec3::NEG_Z * 3.;
    missing.slide_requested = true;
    missing.engine_contact.mask = 0;
    let mut zero = missing.clone();
    zero.engine_contact.mask = 4;
    zero.engine_contact.working_normal = [0.; 4];
    missing.step(&input, MAX_SUBSTEP, &w, &t);
    zero.step(&input, MAX_SUBSTEP, &w, &t);
    assert!(missing.velocity.x > 0.);
    assert_eq!(missing.velocity, zero.velocity);
}
#[test]
fn slope_config_roundtrip_and_zero_turn_bypass_preserve_ownership() {
    let mut value: serde_json::Value =
        serde_json::from_str(include_str!("../presets/static-tenno-bullet.json")).unwrap();
    value
        .as_object_mut()
        .unwrap()
        .remove("slide_slope_acceleration");
    let legacy: Tuning = serde_json::from_value(value).unwrap();
    assert_eq!(legacy.slide_slope_acceleration, 6.);
    for bad in [-1., f32::NAN, f32::INFINITY] {
        assert!(
            !Tuning {
                slide_slope_acceleration: bad,
                ..legacy
            }
            .is_valid()
        );
    }
    let w = World::mesh(crate::static_terrain::StaticTerrain::course()).unwrap();
    let mut t = tuning();
    t.ground_accel_drag = Some(0.);
    let mut p = supported(&w, &t, true);
    p.slide_requested = true;
    p.velocity = Vec3::NEG_Z * 3.;
    p.slide_impulse.replace(Vec3::X * 2., 10.);
    p.step(
        &Input {
            crouch: true,
            movement: Vec3::Z * 0.5,
            ..Default::default()
        },
        MAX_SUBSTEP,
        &w,
        &t,
    );
    assert_eq!(horizontal(p.velocity), Vec3::Z * 3.);
    assert_eq!(p.slide_velocity(), Vec3::X * 2.);
}

#[test]
fn active_slope_changes_only_base_before_nonzero_slide_and_roll_composition() {
    let w = World::mesh(crate::static_terrain::StaticTerrain::course()).unwrap();
    let t = tuning();
    let mut base = supported(&w, &t, true);
    base.velocity = Vec3::NEG_Z * 3.;
    base.slide_requested = true;
    let mut composed = base.clone();
    composed.slide_impulse.replace(Vec3::X * 2., 10.);
    composed.roll_impulse.replace(Vec3::NEG_Z, 4.);
    let input = Input {
        crouch: true,
        ..Default::default()
    };
    base.step(&input, MAX_SUBSTEP, &w, &t);
    composed.step(&input, MAX_SUBSTEP, &w, &t);
    assert!(!base.last_world_move_failed && !composed.last_world_move_failed);
    assert!(
        base.velocity.z < -3.,
        "actual nonzero slope acceleration must execute"
    );
    assert_eq!(
        composed.velocity, base.velocity,
        "additive records must not enter base slope kernel"
    );
    assert_eq!(composed.slide_velocity(), Vec3::X * 2.);
    assert_eq!(composed.roll_velocity(), Vec3::NEG_Z);
    assert_eq!(
        composed.engine_contact.last_request_base,
        horizontal(composed.velocity) + composed.slide_velocity() + composed.roll_velocity()
    );
    assert!(composed.position.x > base.position.x);
    assert!(composed.position.z < base.position.z);
}

#[test]
fn preaging_gravity_product_and_deferred_actor_air_are_separate_bindings() {
    let w = World::mesh(crate::static_terrain::StaticTerrain::course()).unwrap();
    let t = tuning();
    let input = Input {
        crouch: true,
        ..Default::default()
    };
    let make = || {
        let mut p = supported(&w, &t, true);
        p.velocity = Vec3::NEG_Z * 3.;
        p.slide_requested = true;
        p
    };
    let mut boundary = make();
    boundary.bullet_gravity_remaining = 0.495;
    boundary.wall_gravity_remaining = -1.;
    boundary.wall_gravity_scale = 1.;
    for expected_path in [
        crate::slide_slope::Path::ZeroTarget,
        crate::slide_slope::Path::Downhill,
    ] {
        let n = boundary.engine_contact.working_normal;
        let g = -t.gravity
            * crate::gravity::modifier_factor(
                t.bullet_jump_gravity_scale,
                boundary.bullet_gravity_remaining,
                t.gravity_modifier_fade_window,
            );
        let selected = crate::slide_slope::select(
            boundary.engine_contact.mask,
            true,
            Vec3::new(n[0], n[1], n[2]),
            g,
            t.slide_slope_acceleration,
            Vec3::ZERO,
            t.ground_acceleration,
            t.ground_accel_drag.unwrap(),
        );
        assert_eq!(selected.path, expected_path, "pre-aging gate boundary");
        let expected = accelerate_velocity(
            horizontal(boundary.velocity),
            selected.target,
            MAX_SUBSTEP,
            VelocityTuning {
                acceleration: selected.acceleration,
                turn_rate: selected.turn_rate,
                deceleration: t.slide_drag,
                max_speed: t.walk_speed,
                angle_cutoff_degrees: 110.,
            },
        );
        let before = boundary.bullet_gravity_remaining;
        boundary.step(&input, MAX_SUBSTEP, &w, &t);
        assert!(!boundary.last_world_move_failed);
        assert_eq!(horizontal(boundary.velocity), expected);
        assert_eq!(boundary.bullet_gravity_remaining, before - MAX_SUBSTEP);
    }
    // Each modifier alone crosses the gate, but their product does not.
    let mut product = make();
    product.bullet_gravity_remaining = 0.25;
    product.wall_gravity_remaining = -1.;
    product.wall_gravity_scale = 0.03;
    let mut bullet_only = product.clone();
    bullet_only.wall_gravity_scale = 1.;
    let mut wall_only = product.clone();
    wall_only.bullet_gravity_remaining = 0.;
    let before = product.velocity.z;
    product.step(&input, MAX_SUBSTEP, &w, &t);
    bullet_only.step(&input, MAX_SUBSTEP, &w, &t);
    wall_only.step(&input, MAX_SUBSTEP, &w, &t);
    assert!(product.velocity.z > before, "product below gate must brake");
    assert!(
        bullet_only.velocity.z < before && wall_only.velocity.z < before,
        "both named modifiers must participate in the threshold gate"
    );
    // Native mask4 does not imply actor grounded: cached-air/defer can retainair.
    let mut deferred = make();
    deferred.grounded = false;
    deferred.engine_contact.defer = true;
    deferred.jump_lifecycle.reset.cached_air = true;
    assert_eq!(deferred.engine_contact.mask & 4, 4);
    assert!(deferred.engine_contact.air(true, &t.collision.engine));
    let mut ordinary = deferred.clone();
    ordinary.slide_requested = false;
    deferred.step(&input, MAX_SUBSTEP, &w, &t);
    ordinary.step(&input, MAX_SUBSTEP, &w, &t);
    assert_eq!(
        horizontal(deferred.velocity),
        horizontal(ordinary.velocity),
        "retained support must not override LAB actor-air admission"
    );
}
