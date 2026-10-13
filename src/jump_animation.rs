//! Original bounded jump-animation graph for explicitly supplied LAB inputs.
//!
//! This models semantic state changes and three queued movement events. It is
//! not a clip player: no clocks, blend delays, velocity tests, wall/parry/holster
//! branches, or automatic graph admission are inferred. The caller supplies an
//! admitted evaluation and the clip ACTIVE predicate, promotes properties first,
//! then drains returned events after evaluation. Other animation event fanout,
//! non-jump property writes and item cleanup are omitted LAB effects.

use crate::{
    jump_graph::{JumpBinding, JumpValue, WriteMode},
    jump_lifecycle::JumpProperties,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum JumpNode {
    #[default]
    Initial,
    Jumping,
    JumpingB,
    NormalJump,
    TriggerJump,
    Falling,
    LandingCheck,
    Landing,
    SlideLanding,
    Terminate,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum JumpStyle {
    #[default]
    Normal,
    Double,
    Bullet,
}

/// Supplied evaluated properties in the bounded no-wall/no-parry/no-holster domain.
/// ACTIVE is caller policy, not a recovered duration or physical air predicate.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct GraphInputs {
    pub style: JumpStyle,
    pub aiming: bool,
    pub active_clip: bool,
    /// Supplied IV_MOVE_STYLE==SLIDE for the landing check.
    pub sliding: bool,
}

/// Movement-relevant event identities, emitted after node entry modifications.
/// Start and Wait retain ordering but their native fanout is not modeled here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JumpEvent {
    Start,
    Impulse,
    Wait,
}

/// Persistent semantic graph state. Activation/restart is an explicit LAB policy.
/// Evaluation never launches physics, consumes pending height, or refills budgets.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct LabJumpAnimation {
    node: JumpNode,
    #[serde(skip)]
    trace: Vec<JumpNode>,
    #[serde(skip)]
    limit_reached: bool,
}

impl LabJumpAnimation {
    pub const fn node(&self) -> JumpNode {
        self.node
    }

    /// Last evaluation's initial node followed by each entered semantic node.
    pub fn transition_trace(&self) -> &[JumpNode] {
        &self.trace
    }

    /// True after ten transitions, even if a further evaluation would hold.
    pub const fn transition_limit_reached(&self) -> bool {
        self.limit_reached
    }

    /// Chosen LAB activation on an accepted request. Preserve active nodes so
    /// retriggers use their ordered rules; only a terminated graph restarts.
    pub fn restart_for_lab_request(&mut self) {
        if self.node == JumpNode::Terminate {
            self.node = JumpNode::Initial;
            self.trace.clear();
            self.limit_reached = false;
        }
    }

    /// Run at most ten semantic transitions, using current evaluated properties.
    /// No implicit promotion occurs. Entry modifications use mode1 and therefore
    /// always change staged, while an external binding can retain evaluated.
    /// Unsupported evaluated values produce no transition or property mutation.
    ///
    /// Landing item-cleanup intermediaries and their events are omitted: leaving
    /// Landing/SlideLanding returns directly to Initial. The ten-transition bound
    /// applies to this subset, not a promise of an identical native node count.
    pub fn evaluate(
        &mut self,
        properties: &mut JumpProperties,
        binding: &mut JumpBinding,
        inputs: GraphInputs,
    ) -> Vec<JumpEvent> {
        self.trace.clear();
        self.trace.push(self.node);
        self.limit_reached = false;
        let mut events = Vec::new();
        for _ in 0..10 {
            let Ok(value) = JumpValue::try_from(properties.evaluated) else {
                break;
            };
            let Some(next) = self.next(value, inputs) else {
                return events;
            };
            self.node = next;
            self.trace.push(next);
            match next {
                JumpNode::Jumping | JumpNode::JumpingB => {
                    binding.write(properties, JumpValue::Jumping, WriteMode::RespectBinding);
                }
                JumpNode::Falling => {
                    binding.write(properties, JumpValue::Falling, WriteMode::RespectBinding);
                    events.push(JumpEvent::Wait);
                }
                JumpNode::NormalJump => events.push(JumpEvent::Start),
                JumpNode::TriggerJump => events.push(JumpEvent::Impulse),
                _ => {}
            }
        }
        self.limit_reached = self.trace.len() == 11;
        events
    }

    fn next(&self, value: JumpValue, inputs: GraphInputs) -> Option<JumpNode> {
        use JumpNode as N;
        use JumpValue as V;
        match self.node {
            N::Initial => Some(match value {
                V::Start | V::Jumping => N::Jumping,
                V::Falling => N::Falling,
                V::Idle | V::Landing => N::Terminate,
            }),
            N::Jumping | N::JumpingB => {
                // Preserve rule order: normal/double routes precede aim; ACTIVE
                // precedes landing. These nonloop nodes have no landing override.
                if inputs.style == JumpStyle::Normal {
                    Some(N::NormalJump)
                } else if value == V::Start {
                    Some(if self.node == N::Jumping {
                        N::JumpingB
                    } else {
                        N::Jumping
                    })
                } else if self.node == N::Jumping && inputs.style == JumpStyle::Double {
                    Some(N::JumpingB)
                } else if value == V::Falling || inputs.aiming {
                    Some(N::Falling)
                } else if inputs.active_clip {
                    None
                } else if value == V::Landing {
                    Some(N::Terminate)
                } else {
                    Some(N::TriggerJump)
                }
            }
            N::NormalJump => Some(N::TriggerJump),
            N::TriggerJump => Some(N::Falling),
            N::Falling => match value {
                V::Landing => Some(N::LandingCheck),
                V::Falling => None,
                _ => Some(N::Initial),
            },
            N::LandingCheck => Some(if inputs.sliding {
                N::SlideLanding
            } else {
                N::Landing
            }),
            N::Landing | N::SlideLanding => {
                if value != V::Landing {
                    Some(N::Initial)
                } else if inputs.active_clip {
                    None
                } else {
                    Some(N::Terminate)
                }
            }
            N::Terminate => None,
        }
    }
}
