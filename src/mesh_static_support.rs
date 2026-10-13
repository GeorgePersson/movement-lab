//! Input-only prefix for the reviewed supplied static previous-contact domain.
//! Membership, scene relation and flag storage are immutable fixture inputs.
//! Static raw actor type6 is fixed; moving support and object lifetime are open.
use crate::mesh_iteration::{Error, Handles};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ShapeMembership {
    PreviousShape,
    OtherFixtureShape,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SceneState {
    State0,
    State1,
    State2,
    State3,
}
impl SceneState {
    pub fn from_raw(value: u8) -> Result<Self, Error> {
        match value {
            0 => Ok(Self::State0),
            1 => Ok(Self::State1),
            2 => Ok(Self::State2),
            3 => Ok(Self::State3),
            _ => Err(Error::InvalidInput),
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub struct StaticSupport<'a> {
    pub shapes: &'a [ShapeMembership],
    pub scene_state: SceneState,
    pub scene_null: bool,
    pub scene_other: bool,
    /// Bit40 selects the flags byte's buffer, independently of supplied poses.
    /// Bit04 is not admitted here; pose buffering is a separate fixture input.
    pub buffer_bits: u8,
    pub flags: u8,
    pub buffer_flags: u8,
}
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Motion {
    pub velocity: [f32; 3],
    pub moving: u8,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    Count,
    Enumerate(u32),
    Scene,
    Flags,
    StaticMotion,
    DisabledProbe,
}
#[derive(Clone, Debug)]
pub struct Prefix {
    pub events: Vec<Event>,
    /// Persistent identities immediately after native validation, before the
    /// distinct Common local output slots initialize and numerical passes run.
    pub handles: Handles,
    pub velocity: [f32; 3],
    pub moving: u8,
    pub request: [f32; 3],
}
pub fn validate(
    support: StaticSupport<'_>,
    previous: Handles,
    motion: Motion,
) -> Result<(), Error> {
    if support.shapes.len() > 64
        || motion
            .velocity
            .into_iter()
            .any(|x| !x.is_finite() || (x != 0. && !x.is_normal()) || x.abs() > 10000.)
    {
        return Err(Error::InvalidInput);
    }
    if support.buffer_bits & 4 != 0 || (previous.actor.is_none() && previous.shape.is_some()) {
        return Err(Error::UnsupportedDomain);
    }
    Ok(())
}
pub fn prefix(
    support: StaticSupport<'_>,
    previous: Handles,
    motion: Motion,
    displacement: [f32; 3],
    pending: [f32; 3],
) -> Result<Prefix, Error> {
    validate(support, previous, motion)?;
    if displacement
        .into_iter()
        .chain(pending)
        .any(|x| !x.is_finite() || (x != 0. && !x.is_normal()) || x.abs() > 10000.)
    {
        return Err(Error::InvalidInput);
    }
    let request = std::array::from_fn(|i| displacement[i] + pending[i]);
    if !request.into_iter().all(f32::is_finite) {
        return Err(Error::NonFiniteArithmetic);
    }
    let mut handles = previous;
    let mut events = Vec::new();
    if handles.actor.is_some() && handles.shape.is_some() {
        events.push(Event::Count);
        let mut found = false;
        for (index, membership) in support.shapes.iter().enumerate() {
            events.push(Event::Enumerate(index as u32));
            if *membership == ShapeMembership::PreviousShape {
                found = true;
                break;
            }
        }
        let mut valid = found;
        if found {
            events.push(Event::Scene);
            valid = matches!(support.scene_state, SceneState::State1 | SceneState::State2)
                && !support.scene_null
                && !support.scene_other;
            if valid {
                events.push(Event::Flags);
                let flags = if support.buffer_bits & 64 != 0 {
                    support.buffer_flags
                } else {
                    support.flags
                };
                valid = flags & 2 != 0;
            }
        }
        if !valid {
            handles = Handles {
                actor: None,
                shape: None,
            };
        }
    }
    events.push(if handles.shape.is_some() {
        Event::StaticMotion
    } else {
        Event::DisabledProbe
    });
    // Actual type6 static-motion / flag-disabled query paths clear both fields.
    Ok(Prefix {
        events,
        handles,
        velocity: [0.; 3],
        moving: 0,
        request,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh_iteration::{ActorHandle, ShapeHandle};
    #[test]
    fn membership_stops_at_first_match_and_buffered_flags_select_independently() {
        let shapes = [
            ShapeMembership::OtherFixtureShape,
            ShapeMembership::PreviousShape,
            ShapeMembership::PreviousShape,
        ];
        let s = StaticSupport {
            shapes: &shapes,
            scene_state: SceneState::State2,
            scene_null: false,
            scene_other: false,
            buffer_bits: 64,
            flags: 0,
            buffer_flags: 2,
        };
        let handles = Handles {
            actor: Some(ActorHandle),
            shape: Some(ShapeHandle),
        };
        let motion = Motion {
            velocity: [-0., 7., -8.],
            moving: 255,
        };
        let p = prefix(s, handles, motion, [1., -0., 0.], [0.25, -0., 0.]).unwrap();
        assert_eq!(p.handles, handles);
        assert_eq!(
            p.events,
            [
                Event::Count,
                Event::Enumerate(0),
                Event::Enumerate(1),
                Event::Scene,
                Event::Flags,
                Event::StaticMotion
            ]
        );
        assert_eq!(p.velocity.map(f32::to_bits), [0; 3]);
        assert_eq!(
            p.request.map(f32::to_bits),
            [1.25_f32.to_bits(), (-0_f32).to_bits(), 0]
        );
        assert_eq!(
            motion.velocity.map(f32::to_bits),
            [(-0_f32).to_bits(), 7_f32.to_bits(), (-8_f32).to_bits()]
        );
        assert_eq!(
            shapes,
            [
                ShapeMembership::OtherFixtureShape,
                ShapeMembership::PreviousShape,
                ShapeMembership::PreviousShape
            ]
        );
    }
    #[test]
    fn invalid_scene_skips_flags_and_partial_actor_identity_is_retained() {
        let shapes = [ShapeMembership::PreviousShape];
        let s = StaticSupport {
            shapes: &shapes,
            scene_state: SceneState::State1,
            scene_null: false,
            scene_other: true,
            buffer_bits: 0,
            flags: 2,
            buffer_flags: 2,
        };
        let motion = Motion {
            velocity: [1.; 3],
            moving: 1,
        };
        let p = prefix(
            s,
            Handles {
                actor: Some(ActorHandle),
                shape: Some(ShapeHandle),
            },
            motion,
            [0.; 3],
            [0.; 3],
        )
        .unwrap();
        assert_eq!(
            p.events,
            [
                Event::Count,
                Event::Enumerate(0),
                Event::Scene,
                Event::DisabledProbe
            ]
        );
        assert_eq!(
            p.handles,
            Handles {
                actor: None,
                shape: None
            }
        );
        let partial = Handles {
            actor: Some(ActorHandle),
            shape: None,
        };
        let p = prefix(s, partial, motion, [0.; 3], [0.; 3]).unwrap();
        assert_eq!(p.events, [Event::DisabledProbe]);
        assert_eq!(p.handles, partial);
    }
    #[test]
    fn sixty_four_members_support_early_and_last_match_without_extra_enumeration() {
        let mut shapes = [ShapeMembership::OtherFixtureShape; 64];
        shapes[63] = ShapeMembership::PreviousShape;
        let handles = Handles {
            actor: Some(ActorHandle),
            shape: Some(ShapeHandle),
        };
        let motion = Motion {
            velocity: [0.; 3],
            moving: 0,
        };
        let make = |shapes| StaticSupport {
            shapes,
            scene_state: SceneState::State1,
            scene_null: false,
            scene_other: false,
            buffer_bits: 0,
            flags: 2,
            buffer_flags: 0,
        };
        let p = prefix(make(&shapes), handles, motion, [0.; 3], [0.; 3]).unwrap();
        assert_eq!(
            p.events
                .iter()
                .filter(|e| matches!(e, Event::Enumerate(_)))
                .count(),
            64
        );
        assert_eq!(p.events[64], Event::Enumerate(63));
        let mut early_shapes = shapes;
        early_shapes[0] = ShapeMembership::PreviousShape;
        let p = prefix(make(&early_shapes), handles, motion, [0.; 3], [0.; 3]).unwrap();
        assert_eq!(
            p.events,
            [
                Event::Count,
                Event::Enumerate(0),
                Event::Scene,
                Event::Flags,
                Event::StaticMotion
            ]
        );
    }
}
