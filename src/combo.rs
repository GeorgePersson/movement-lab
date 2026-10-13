//! One-button LAB demonstration inputs. These convenience delays are not native
//! animation timing. This helper never edits Player physics, budgets or tuning.

use crate::Input;
use glam::Vec3;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum Stage {
    #[default]
    Idle,
    Sprint,
    Slide,
    Bullet,
    BeforeDouble,
    Double,
    BeforeGlide,
    Glide,
}

#[derive(Clone, Debug, Default)]
pub struct Combo {
    stage: Stage,
    elapsed: f32,
}

impl Combo {
    /// Restart the sequence. The viewer requires grounded support before calling
    /// this; input() also cancels if support is lost before the bullet pulse.
    pub fn start(&mut self) {
        self.stage = Stage::Sprint;
        self.elapsed = 0.;
    }

    pub fn cancel(&mut self) {
        self.stage = Stage::Idle;
        self.elapsed = 0.;
    }

    pub fn is_active(&self) -> bool {
        self.stage != Stage::Idle
    }

    pub fn stage_label(&self) -> &'static str {
        match self.stage {
            Stage::Idle => "Ready",
            Stage::Sprint => "Sprint",
            Stage::Slide => "Slide",
            Stage::Bullet => "Bullet jump",
            Stage::BeforeDouble => "Coast to double jump",
            Stage::Double => "Double jump",
            Stage::BeforeGlide => "Coast to glide",
            Stage::Glide => "Aim glide",
        }
    }

    /// Sample once per simulation step, normally the viewer's fixed 120 Hz.
    /// Press edges occur on exactly one returned input each. Durations round up
    /// to whole calls; no long frame can collapse two jump presses into one input.
    /// Invalid time/camera samples emit no input and leave the sequence unchanged.
    /// The caller may fill aim_target after its world query.
    pub fn input(&mut self, dt: f32, grounded: bool, camera_forward: Vec3) -> Option<Input> {
        if !self.is_active() || !dt.is_finite() || dt <= 0. || !camera_forward.is_finite() {
            return None;
        }
        if matches!(self.stage, Stage::Sprint | Stage::Slide | Stage::Bullet) {
            if !grounded {
                self.cancel();
                return None;
            }
        } else if grounded {
            // The grounded value on the bullet pulse itself is pre-step support,
            // so it must not cancel that pulse. Only subsequent samples end early.
            self.cancel();
            return None;
        }
        let mut input = Input {
            movement: Vec3::new(camera_forward.x, 0., camera_forward.z).normalize_or_zero(),
            camera_forward,
            ..Default::default()
        };
        match self.stage {
            Stage::Idle => unreachable!(),
            Stage::Sprint => {
                input.sprint = true;
                self.advance(dt, 0.55, Stage::Slide);
            }
            Stage::Slide => {
                input.sprint = true;
                input.crouch = true;
                self.advance(dt, 0.15, Stage::Bullet);
            }
            Stage::Bullet => {
                input.sprint = true;
                input.crouch = true;
                input.jump = true;
                self.stage = Stage::BeforeDouble;
                self.elapsed = 0.;
            }
            Stage::BeforeDouble => self.advance(dt, 0.30, Stage::Double),
            Stage::Double => {
                input.jump = true;
                self.stage = Stage::BeforeGlide;
                self.elapsed = 0.;
            }
            Stage::BeforeGlide => self.advance(dt, 0.12, Stage::Glide),
            Stage::Glide => {
                input.aim = true;
                self.advance(dt, 1.5, Stage::Idle);
            }
        }
        Some(input)
    }

    fn advance(&mut self, dt: f32, duration: f32, next: Stage) {
        // Clamp before addition so even finite f32::MAX cannot overflow elapsed.
        self.elapsed += dt.min(duration);
        if self.elapsed >= duration {
            self.stage = next;
            self.elapsed = 0.;
        }
    }
}
