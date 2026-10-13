//! Small engine-independent integration example.
use glam::Vec3;
use movement_lab::{Input, Player, Tuning, World};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let tuning: Tuning = serde_json::from_str(include_str!(
        "../presets/static-tenno-bullet.json"
    ))?;
    let world = World::graybox();
    let mut player = Player::new(Vec3::new(0.0, 0.05, 5.0));
    let dt = 1.0 / 120.0;
    for _ in 0..30 {
        player.step(&Input::default(), dt, &world, &tuning);
    }
    let input = Input {
        movement: Vec3::NEG_Z,
        camera_forward: Vec3::new(0.0, 0.6, -0.8),
        sprint: true,
        crouch: true,
        jump: true,
        ..Default::default()
    };
    player.step(&input, dt, &world, &tuning);
    println!("{:?}: {:?}", player.state, player.position);
    Ok(())
}
