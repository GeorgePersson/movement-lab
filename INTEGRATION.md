# Integrating the controller

Add the cloned repository as a local dependency (adjust the path):

```toml
[dependencies]
movement-lab = { path = "../movement-lab" }
glam = "0.30"
serde_json = "1"
```

The crate is imported as `movement_lab`. Keep the viewer feature disabled for
engine-independent use. The controller uses `glam` 0.30 vectors; convert explicitly
when your engine uses another vector type/version.

For the following snippet in your own `src/main.rs`, first copy this repository's
`presets/` directory into your application's root. A path dependency does not
copy runtime configuration into the consuming application. The bundled example
already resolves the preset inside this repository.

```rust
use glam::Vec3;
use movement_lab::{Input, Player, Tuning, World};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let tuning: Tuning = serde_json::from_str(include_str!(
        "../presets/static-tenno-bullet.json"
    ))?;
    let world = World::graybox();
    let mut player = Player::new(Vec3::new(0.0, 0.05, 5.0));
    let dt = 1.0 / 120.0;

    // Establish contact before issuing a jump.
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
```

This snippet is also available as `examples/integration_start.rs`:

```sh
cargo run --locked --example integration_start
```

## Input and time

- Coordinates are Y-up; `movement` is a world-space analog direction with Y=0
  for ordinary movement and wall controls. Wall paths can reject nonzero Y.
- Supply a finite, normalized `camera_forward` including pitch. `aim_target`
  optionally supplies a world-space aim-query result.
- `jump` and `roll` are press edges, consumed on one simulation step. Do not
  repeatedly set them true just because the key remains held.
- `jump_held` is separate held state used for wall traversal. The viewer's
  `src/jump_edges.rs` preserves ordered presses/releases and crouch snapshots
  across render frames and fixed steps. Follow that adapter when integrating
  real input rather than dropping multiple events into one Boolean.
- Feed simulation-owned camera state into the controller. Interpolated render
  transforms must not change the next step's input.
- Use a stable fixed step, initially 1/120 s. The implementation subdivides
  steps, but arbitrary step partitioning is not bitwise equivalent.
- On focus loss/helper cancellation, clear queued input, cancel any active
  `Combo`, and call `player.cancel_wall_input()` to cancel pending held-wall state.
  Use `Player::respawn` for respawn, and clear the external input queue too.

`combo::Combo` produces inputs for the full demonstration sequence; it does not
force the player's position, physics or resource counters. Its delays are authored
demo timings. Only start the demo with grounded support.

## Collision and engine adaptation

`Player::step` currently accepts the provided `World`; it is not a generic engine
physics trait. Start with `World::graybox()` or the supported box/mesh constructors
in `src/world.rs`, then adapt that layer deliberately to your engine's queries.
Position is the capsule's feet origin, not the render mesh center.

Retain the separate velocity channels, deferred impulses, grounded/support state,
launch budgets and wall request lifecycle when adapting. Treat asynchronous wall
query delivery and actor/provider roles as explicit interfaces; the current
one-substep delivery rule is a laboratory policy. Unsupported mesh queries can
reject a move; inspect `Player::last_world_move_failed()` rather than treating
failure as a successful zero displacement.

For a mesh world, set `tuning.collision.overlap_recovery = false` before stepping.
The viewer applies this capability policy on startup and reload, but a direct
library caller must apply it too; the mesh backend does not support recovery from
initial overlap.

Load `presets/static-tenno-bullet.json` for the selected baseline. Its adjacent
provenance file distinguishes source-backed constants and assumptions. Tune
against your own project's scale and collision setup; no gameplay-calibrated fit
is claimed. `tuning.json` is a separate approximate baseline.
