use movement_lab::{
    jump_graph::{BindingError, JumpBinding, JumpValue, WriteMode},
    jump_lifecycle::JumpProperties,
};

#[test]
fn stage_only_and_mode_one_have_different_publication() {
    let mut properties = JumpProperties {
        evaluated: 3,
        staged: 0,
    };
    let mut binding = JumpBinding::default();
    binding.write(&mut properties, JumpValue::Start, WriteMode::StageOnly);
    assert_eq!(
        properties,
        JumpProperties {
            evaluated: 3,
            staged: 1
        }
    );
    binding.write(
        &mut properties,
        JumpValue::Jumping,
        WriteMode::RespectBinding,
    );
    assert_eq!(
        properties,
        JumpProperties {
            evaluated: 2,
            staged: 2
        }
    );
}

#[test]
fn nested_owners_share_first_evaluated_capture_until_last_release() {
    let mut properties = JumpProperties {
        evaluated: 1,
        staged: 2,
    };
    let mut binding = JumpBinding::default();
    binding.acquire(&properties).unwrap();
    assert_eq!(binding.captured_value(), Some(JumpValue::Start));
    binding.write(
        &mut properties,
        JumpValue::Falling,
        WriteMode::RespectBinding,
    );
    properties.evaluated = 4; // independent supplied phase, not a new acquisition
    binding.acquire(&properties).unwrap();
    assert_eq!(binding.lock_count(), 2);
    assert_eq!(binding.captured_value(), Some(JumpValue::Start));
    binding.promote(&mut properties).unwrap();
    assert_eq!(
        properties,
        JumpProperties {
            evaluated: 1,
            staged: 3
        }
    );
    assert!(binding.release());
    binding.promote(&mut properties).unwrap();
    assert_eq!(properties.evaluated, 1);
    assert!(binding.release());
    assert_eq!(properties.evaluated, 1, "release does not publish staged");
    assert_eq!(binding.captured_value(), None);
    binding.promote(&mut properties).unwrap();
    assert_eq!(properties.evaluated, 3);
    assert!(!binding.release());
}

#[test]
fn leave_synchronization_precedes_modification_and_release() {
    let mut properties = JumpProperties {
        evaluated: 1,
        staged: 2,
    };
    let mut binding = JumpBinding::default();
    binding.acquire(&properties).unwrap();
    binding.synchronize(&mut properties).unwrap();
    assert_eq!(binding.captured_value(), Some(JumpValue::Jumping));
    assert_eq!(binding.lock_count(), 1);
    binding.write(
        &mut properties,
        JumpValue::Falling,
        WriteMode::RespectBinding,
    );
    assert_eq!(
        properties,
        JumpProperties {
            evaluated: 2,
            staged: 3
        }
    );
    binding.release();
    assert_eq!(
        properties,
        JumpProperties {
            evaluated: 2,
            staged: 3
        }
    );
    binding.promote(&mut properties).unwrap();
    assert_eq!(properties.evaluated, 3);
}

#[test]
fn entry_modification_then_acquisition_captures_new_value_unless_already_locked() {
    for preexisting_lock in [false, true] {
        let mut properties = JumpProperties {
            evaluated: 1,
            staged: 1,
        };
        let mut binding = JumpBinding::default();
        if preexisting_lock {
            binding.acquire(&properties).unwrap();
        }
        binding.write(
            &mut properties,
            JumpValue::Jumping,
            WriteMode::RespectBinding,
        );
        binding.acquire(&properties).unwrap();
        assert_eq!(properties.staged, 2);
        assert_eq!(
            binding.captured_value(),
            Some(if preexisting_lock {
                JumpValue::Start
            } else {
                JumpValue::Jumping
            })
        );
        assert_eq!(binding.lock_count(), if preexisting_lock { 2 } else { 1 });
    }
}

#[test]
fn mode_two_replaces_capture_without_changing_ownership() {
    let mut properties = JumpProperties {
        evaluated: 1,
        staged: 2,
    };
    let mut binding = JumpBinding::default();
    binding.acquire(&properties).unwrap();
    binding.acquire(&properties).unwrap();
    binding.write(
        &mut properties,
        JumpValue::Landing,
        WriteMode::ReplaceBinding,
    );
    assert_eq!(
        properties,
        JumpProperties {
            evaluated: 4,
            staged: 4
        }
    );
    assert_eq!(binding.lock_count(), 2);
    assert_eq!(binding.captured_value(), Some(JumpValue::Landing));
    binding.write(&mut properties, JumpValue::Idle, WriteMode::StageOnly);
    binding.promote(&mut properties).unwrap();
    assert_eq!(
        properties,
        JumpProperties {
            evaluated: 4,
            staged: 0
        }
    );
}

#[test]
fn synchronization_without_binding_does_not_create_one() {
    let mut properties = JumpProperties {
        evaluated: 0,
        staged: 3,
    };
    let mut binding = JumpBinding::default();
    binding.synchronize(&mut properties).unwrap();
    assert_eq!(properties.evaluated, 3);
    assert_eq!(binding.lock_count(), 0);
    assert_eq!(binding.captured_value(), None);
}

#[test]
fn invalid_raw_property_reads_are_atomic_and_do_not_affect_legacy_storage() {
    let mut binding = JumpBinding::default();
    let mut properties = JumpProperties {
        evaluated: 255,
        staged: 1,
    };
    assert_eq!(
        binding.acquire(&properties),
        Err(BindingError::UnsupportedValue(255))
    );
    assert_eq!(binding, JumpBinding::default());
    binding.write(&mut properties, JumpValue::Jumping, WriteMode::StageOnly);
    assert_eq!(
        properties.evaluated, 255,
        "an unwritten raw channel stays intact"
    );
    properties.staged = 200;
    let before = properties;
    assert_eq!(
        binding.synchronize(&mut properties),
        Err(BindingError::UnsupportedValue(200))
    );
    assert_eq!(
        binding.promote(&mut properties),
        Err(BindingError::UnsupportedValue(200))
    );
    assert_eq!(properties, before);
}

#[test]
fn serde_defaults_and_round_trip_preserve_nested_capture() {
    assert_eq!(
        serde_json::from_str::<JumpBinding>("{}").unwrap(),
        JumpBinding::default()
    );
    #[derive(serde::Deserialize)]
    struct LegacyOwner {
        #[serde(default)]
        binding: JumpBinding,
    }
    assert_eq!(
        serde_json::from_str::<LegacyOwner>("{}").unwrap().binding,
        JumpBinding::default()
    );
    let mut binding = JumpBinding::default();
    let properties = JumpProperties {
        evaluated: 2,
        staged: 3,
    };
    binding.acquire(&properties).unwrap();
    binding.acquire(&properties).unwrap();
    let restored: JumpBinding =
        serde_json::from_str(&serde_json::to_string(&binding).unwrap()).unwrap();
    assert_eq!(restored, binding);
    assert_eq!(restored.captured_value(), Some(JumpValue::Jumping));
}

#[test]
fn acquisition_overflow_is_rejected_instead_of_native_counter_wrap() {
    let mut binding: JumpBinding =
        serde_json::from_str(r#"{"count":4294967295,"captured":"Falling"}"#).unwrap();
    let before = binding;
    assert_eq!(
        binding.acquire(&JumpProperties::default()),
        Err(BindingError::CountOverflow)
    );
    assert_eq!(binding, before);
    let mut absent: JumpBinding =
        serde_json::from_str(r#"{"count":0,"captured":"Landing"}"#).unwrap();
    assert!(!absent.release());
    assert_eq!(absent.captured_value(), None);
}
