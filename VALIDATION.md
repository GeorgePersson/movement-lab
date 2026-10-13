# Export validation

Checked on 2026-10-13 (Pacific/Auckland), Linux x86-64, Rust 1.99.0.

- All 92 accepted prototype-96 source/config files match `SOURCE-SNAPSHOT.json`.
- Fresh library/integration/example build: 368 tests pass.
- Fresh full-feature test run through `scripts/test.py`: 400 tests pass, zero failures.
- `examples/integration_start.rs` builds and runs, entering `BulletJump`.
- Independent export review found no private binaries, machine-specific paths or obvious secrets; its documentation corrections are included.

The initial direct viewer-test command failed six observation tests because
`WF86_OUTPUT_DIR` was unset. The test wrapper supplies fresh output directories;
the full rerun passes. No baseline movement source was changed to obtain this result.

The baseline viewer had three successful rendered captures before export.
This export did not repeat graphical or physical input testing. Gameplay fit
is unvalidated, and arbitrary simulation-rate equivalence remains open.
