//! Recovered finite-domain hit-report accumulator, independent of grounding.
//!
//! Reported point Y selects the primary. Latest report flags are separate from
//! that contact. Reset clears validity/flags while retaining both vectors.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Contact {
    pub point: [f32; 4],
    pub normal: [f32; 4],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactReports {
    pub primary: Contact,
    pub secondary: Contact,
    pub primary_valid: u8,
    pub secondary_valid: u8,
    pub latest_was_controller: u8,
    pub reset_byte_813: u8,
    pub latest_metadata: u8,
    /// Untouched gap between primary and secondary in the comparison record.
    pub gap_7e0: [u8; 16],
}

impl Default for ContactReports {
    fn default() -> Self {
        Self::from_record(&[0; 85])
    }
}

impl ContactReports {
    pub fn has_primary(&self) -> bool {
        self.primary_valid != 0
    }

    pub fn has_secondary(&self) -> bool {
        self.secondary_valid != 0
    }

    pub fn reset(&mut self) {
        self.primary_valid = 0;
        self.secondary_valid = 0;
        self.latest_was_controller = 0;
        self.reset_byte_813 = 0;
        self.latest_metadata = 0;
    }

    /// No directional normal filter or ground classification is performed.
    pub fn add(&mut self, contact: Contact, was_controller: bool, metadata: bool) {
        if !self.has_primary() {
            self.primary = contact;
            self.primary_valid = 1;
        } else {
            if self.primary.point[1] >= contact.point[1] {
                self.secondary = self.primary;
                self.primary = contact;
            } else {
                self.secondary = contact;
            }
            self.secondary_valid = 1;
        }
        self.latest_was_controller = u8::from(was_controller);
        self.latest_metadata = u8::from(metadata);
    }

    /// Native shape report converts its double world point to float and pads W.
    /// The metadata predicate is supplied; its material/role meaning is unbound.
    pub fn add_shape(&mut self, point: [f64; 3], normal: [f32; 3], metadata: bool) {
        self.add(
            Contact {
                point: [point[0] as f32, point[1] as f32, point[2] as f32, 0.0],
                normal: [normal[0], normal[1], normal[2], 0.0],
            },
            false,
            metadata,
        );
    }

    /// Exact transport of the sparse engine record at offsets 0x7C0..=0x814.
    pub fn from_record(record: &[u8; 85]) -> Self {
        fn vector(record: &[u8; 85], offset: usize) -> [f32; 4] {
            std::array::from_fn(|i| {
                let j = offset + i * 4;
                f32::from_le_bytes(record[j..j + 4].try_into().unwrap())
            })
        }
        Self {
            primary: Contact {
                point: vector(record, 0),
                normal: vector(record, 16),
            },
            secondary: Contact {
                point: vector(record, 48),
                normal: vector(record, 64),
            },
            gap_7e0: record[32..48].try_into().unwrap(),
            primary_valid: record[80],
            secondary_valid: record[81],
            latest_was_controller: record[82],
            reset_byte_813: record[83],
            latest_metadata: record[84],
        }
    }

    pub fn to_record(&self) -> [u8; 85] {
        let mut record = [0; 85];
        for (offset, vector) in [
            (0, self.primary.point),
            (16, self.primary.normal),
            (48, self.secondary.point),
            (64, self.secondary.normal),
        ] {
            for (i, value) in vector.into_iter().enumerate() {
                record[offset + i * 4..offset + i * 4 + 4].copy_from_slice(&value.to_le_bytes());
            }
        }
        record[32..48].copy_from_slice(&self.gap_7e0);
        record[80..85].copy_from_slice(&[
            self.primary_valid,
            self.secondary_valid,
            self.latest_was_controller,
            self.reset_byte_813,
            self.latest_metadata,
        ]);
        record
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn contact(y: f32, normal: [f32; 4]) -> Contact {
        Contact {
            point: [0., y, 0., 0.],
            normal,
        }
    }

    #[test]
    fn reset_retains_vectors_and_gap_but_clears_all_five_bytes() {
        let before = [0x55; 85];
        let mut state = ContactReports::from_record(&before);
        state.reset();
        assert_eq!(&state.to_record()[..80], &before[..80]);
        assert_eq!(&state.to_record()[80..], &[0; 5]);
        assert!(!state.has_primary());
        assert!(!state.has_secondary());
    }

    #[test]
    fn tie_replaces_primary_and_latest_flags_do_not_follow_primary() {
        let mut state = ContactReports::default();
        let floor = contact(1., [0., 1., 0., 0.]);
        let wall = contact(1., [1., 0., 0., 0.]);
        let ceiling = contact(4., [0., -1., 0., 0.]);
        state.add(floor, true, true);
        state.add(wall, false, false);
        assert_eq!(state.primary, wall);
        assert_eq!(state.secondary, floor);
        state.add(ceiling, true, false);
        assert_eq!(state.primary, wall);
        assert_eq!(state.secondary, ceiling);
        assert_eq!(state.latest_was_controller, 1);
        assert_eq!(state.latest_metadata, 0);
    }

    #[test]
    fn arbitrary_record_bits_roundtrip_and_shape_point_is_narrowed() {
        let record = std::array::from_fn(|i| (i * 47) as u8);
        assert_eq!(ContactReports::from_record(&record).to_record(), record);
        let mut state = ContactReports::default();
        state.add_shape([0.123456789, -0.0, 2.], [0., -1., 0.], true);
        assert_eq!(
            state.primary.point[0].to_bits(),
            (0.123456789_f64 as f32).to_bits()
        );
        assert_eq!(state.primary.point[1].to_bits(), (-0.0_f32).to_bits());
        assert_eq!(state.primary.normal, [0., -1., 0., 0.]);
    }
}
