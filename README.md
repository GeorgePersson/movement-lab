# Movement Lab

A reusable Rust movement controller and an optional Bevy third-person playground.
Includes walking, sprinting, crouching, sliding, jumps, bullet jumps, double jumps,
rolling, aim glide, wall latching, held wall jumps, landing and movement chaining.
Press **G** or click **Run full combo** to demonstrate sprint → slide → bullet jump
→ double jump → aim glide.

This is an original prototype guided by partial offline movement research.
It is not a verified recreation of Warframe: no calibrated gameplay comparison
has been completed. Some collision, configuration, timing and actor policies are
explicit laboratory assumptions. Source comments and preset provenance record
those boundaries. No original game binaries, assets or decompiler projects are
included, and no game installation or account is needed.

## Build and run

Install Rust with Cargo. The accepted baseline was tested with Rust 1.99.0 on
Linux x86-64. The lockfile pins dependencies; Bevy is optional for the library.

```sh
cargo test --locked --lib --tests --examples
cargo run --locked --features viewer --bin movement-lab -- --fullscreen
```

The viewer needs a graphics-capable desktop and the native build dependencies
for Bevy's enabled features. On Linux this includes development libraries for
udev and the X11/Wayland windowing backends. Windows and macOS have not been
validated in this snapshot.

Run commands from this repository's root so the preset paths resolve. Examples:

```sh
cargo run --locked --features viewer --bin movement-lab -- --combo
cargo run --locked --features viewer --bin movement-lab -- --world mesh-course
python3 scripts/test.py
cargo run --locked --example trajectory -- bullet-up 120 trace.json presets/static-tenno-bullet.json
```

The Python 3 test wrapper runs all targets with the viewer enabled and supplies a
fresh temporary output directory for camera/tuning observation tests. It prints
the directory path and preserves the observations for inspection. Running the
viewer tests directly requires `WF86_OUTPUT_DIR` pointing at a fresh directory.

## Controls

| Input | Action |
| --- | --- |
| WASD / mouse | Move / look |
| Left Shift | Sprint |
| C or Left Ctrl | Crouch / slide |
| Space | Jump; crouch + Space for bullet jump; airborne press for double jump |
| Hold Space + move toward/along a wall | Wall jumps |
| Right mouse | Aim glide / wall latch when eligible |
| Q or Left Alt | Roll |
| G or Run full combo button | Full combo demonstration |
| R | Reset |
| F5 | Reload tuning |
| Tab | Release pointer; left-click the window to capture it again |
| Escape | Quit |

Click the window to capture the pointer. The mesh course disables unsupported
overlap recovery on startup and reload; the viewer displays that policy.

## Use the movement code in another project

The core library has no Bevy dependency unless the `viewer` feature is enabled.
See [INTEGRATION.md](INTEGRATION.md) for the controller API, input timing,
collision adapter boundaries and an example. Start with `src/controller.rs`,
`src/bullet.rs`, `src/combo.rs` and `src/world.rs`.

## Snapshot and validation

`SOURCE-SNAPSHOT.json` authenticates the 92 source/configuration files copied
unchanged from accepted prototype 96. That baseline passed 400 distinct tests,
an independent rerun and 72 retained observation comparisons. Those results do
not establish gameplay fidelity or bitwise independence across simulation rates.
The default viewer uses fixed 120 Hz simulation; its controller also subdivides
large steps using laboratory timing policy. Full simulation-rate equivalence and
live original-game interfaces remain open.
