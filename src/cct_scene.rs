//! Static axis-aligned laboratory scene adapter for the recovered CCT policy.
//!
//! Scene enumeration uses supplied box order, a zero cache origin and identity
//! actor/local box rotations. These are explicit LAB scene choices. Query and
//! penetration arithmetic are recovered; this is not native scene filtering,
//! broadphase caching, moving actors or actor grounding. Shape contact reports
//! feed the recovered accumulator, with explicitly supplied metadata.

use crate::{capsule_mtd, cct_overlap, cct_policy, contact_reports::ContactReports, precise_sweep};
use cct_policy::{BoxHit, Error, Pass, Position, Scene, Vector};

#[derive(Clone, Copy, Debug)]
pub struct Capsule {
    pub radius: f32,
    /// Cylinder length, excluding the two hemispherical caps.
    pub height: f32,
    pub rotation: [f32; 4],
}

#[derive(Clone, Copy, Debug)]
pub struct Collider {
    pub center: Vector,
    pub extents: Vector,
}

pub struct StaticBoxes<'a> {
    pub capsule: Capsule,
    pub boxes: &'a [Collider],
    pub contact_offset: f32,
    pub overlap_recovery: bool,
    /// Supplied shape behavior callback result; current native selection unknown.
    pub behavior: u8,
    /// Explicit LAB liveness guard for native recovery's potentially looping scan.
    pub recovery_scan_limit: usize,
    pub reports: Vec<(Pass, BoxHit)>,
    /// Engine report state survives until its caller explicitly resets it.
    pub contact_reports: ContactReports,
    /// Supplied actor metadata predicate, separate from geometry and walkability.
    pub report_metadata: bool,
}

impl StaticBoxes<'_> {
    /// LAB boundary guards, including the empty-scene case. The native geometry
    /// routines assume valid supplied poses; they do not establish these checks.
    fn validate_geometry(&self, center: Position) -> Result<(), Error> {
        let bounded = |x: f32| x.is_finite() && x.abs() <= precise_sweep::MAX_COMPONENT;
        let q = self.capsule.rotation;
        let q_squared = ((q[0] * q[0] + q[1] * q[1]) + q[2] * q[2]) + q[3] * q[3];
        if !center
            .into_iter()
            .all(|x| x.is_finite() && x.abs() <= f64::from(precise_sweep::MAX_COMPONENT))
            || !q
                .into_iter()
                .chain([
                    self.capsule.radius,
                    self.capsule.height,
                    self.contact_offset,
                ])
                .all(bounded)
            || self.capsule.radius < 0.
            || self.capsule.height < 0.
            || self.contact_offset < 0.
            || (q_squared - 1.).abs() > 0.001
            || self.boxes.iter().any(|b| {
                !b.center.into_iter().chain(b.extents).all(bounded)
                    || b.extents.into_iter().any(|x| x <= 0.)
            })
        {
            return Err(Error::InvalidInput);
        }
        Ok(())
    }
}

/// Common sweep wrapper's ordered capsule endpoint construction. Its X grouping
/// differs from the penetration wrapper, so do not reuse MTD's endpoint helper.
pub fn query_endpoints(q: [f32; 4], position: Vector, half_height: f32) -> (Vector, Vector) {
    let [x, y, z, w] = q;
    let twice_w = w * 2.;
    let twice_x = x * 2.;
    let delta = [
        ((twice_w * w - 1.) + twice_x * x) * half_height,
        (twice_w * z + twice_x * y) * half_height,
        (twice_x * z - twice_w * y) * half_height,
    ];
    (
        std::array::from_fn(|i| delta[i] + position[i]),
        std::array::from_fn(|i| position[i] - delta[i]),
    )
}

impl Scene for StaticBoxes<'_> {
    fn prepare(&mut self, center: Position, _: Vector, _: Vector) -> Result<(), Error> {
        // All supplied immutable boxes are enumerated on each query. No claim
        // that a native temporal cache is empty or has these scene contents.
        self.validate_geometry(center)
    }

    fn query(
        &mut self,
        center: Position,
        direction: Vector,
        limit: f32,
        _: Pass,
    ) -> Result<Option<BoxHit>, Error> {
        self.validate_geometry(center)?;
        let norm = (direction[1] * direction[1] + direction[0] * direction[0])
            + direction[2] * direction[2];
        if !direction.into_iter().all(f32::is_finite)
            || (norm - 1.).abs() > 0.001
            || !limit.is_finite()
            || limit <= 0.
            || limit > precise_sweep::MAX_COMPONENT
        {
            return Err(Error::InvalidInput);
        }
        let (a, b) = query_endpoints(
            self.capsule.rotation,
            center.map(|x| x as f32),
            self.capsule.height * 0.5,
        );
        let mut best = limit;
        let mut winner = None;
        for (index, collider) in self.boxes.iter().enumerate() {
            let result = precise_sweep::sweep(&precise_sweep::SweepInput {
                a,
                b,
                radius: self.capsule.radius,
                box_q: [0., 0., 0., 1.],
                box_p: collider.center,
                extents: collider.extents,
                direction,
                distance: best,
                flags: 0x503,
            })
            .map_err(|_| Error::QueryFailed)?;
            if !result.hit
                || result.distance >= best
                || (result.distance == 0. && !self.overlap_recovery)
            {
                continue;
            }
            best = result.distance;
            winner = Some(BoxHit {
                distance: result.distance,
                normal: result.normal.ok_or(Error::InvalidQueryResult)?,
                point: if result.flags.is_some_and(|f| f & 1 != 0) {
                    Some(
                        result
                            .point
                            .ok_or(Error::InvalidQueryResult)?
                            .map(f64::from),
                    )
                } else {
                    None
                },
                shape_id: u32::try_from(index).map_err(|_| Error::InvalidInput)?,
            });
            // Static actor type6 is eligible for the native early zero winner.
            if best <= 0. {
                break;
            }
        }
        Ok(winner)
    }

    fn report(&mut self, hit: &BoxHit, _: Vector, _: f32, pass: Pass) -> Result<u8, Error> {
        self.contact_reports.add_shape(
            hit.point.ok_or(Error::InvalidQueryResult)?,
            hit.normal,
            self.report_metadata,
        );
        self.reports.push((pass, hit.clone()));
        Ok(self.behavior)
    }

    fn recover_overlap(&mut self, center: Position) -> Result<Position, Error> {
        self.validate_geometry(center)?;
        let records: Vec<_> = self
            .boxes
            .iter()
            .map(|b| cct_overlap::BoxRecord {
                record_type: 3,
                actor_type: 6,
                actor_flags: 0,
                extents: b.extents,
                actor_position: b.center,
                local_position: [0.; 3],
            })
            .collect();
        let recovery = cct_overlap::recover(
            center.map(|x| x as f32),
            self.capsule.rotation,
            self.capsule.radius,
            self.capsule.height,
            self.contact_offset,
            &records,
            self.recovery_scan_limit,
            |query| {
                let result = capsule_mtd::capsule_box_mtd(&capsule_mtd::MtdInput {
                    radius: query.radius,
                    half_height: query.half_height,
                    capsule_rotation: query.capsule_q,
                    capsule_position: query.center,
                    box_rotation: [0., 0., 0., 1.],
                    box_position: query.box_center,
                    extents: query.box_extents,
                });
                Ok(result.map(|m| cct_overlap::Penetration {
                    normal: m.normal,
                    depth: m.depth,
                }))
            },
        )
        .map_err(|e| match e {
            cct_overlap::Error::ScanLimit => Error::RecoveryScanLimit,
            cct_overlap::Error::NonFiniteArithmetic => Error::NonFiniteArithmetic,
            cct_overlap::Error::InvalidInput => Error::InvalidInput,
            cct_overlap::Error::QueryFailed => Error::QueryFailed,
        })?;
        Ok(recovery.position.map(f64::from))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn empty_scene_still_rejects_invalid_geometry_and_directions() {
        let mut s = StaticBoxes {
            capsule: Capsule {
                radius: 0.3,
                height: 1.2,
                rotation: [0., 0., 0., 1.],
            },
            boxes: &[],
            contact_offset: 0.02,
            overlap_recovery: true,
            behavior: 4,
            recovery_scan_limit: 16,
            reports: Vec::new(),
            contact_reports: ContactReports::default(),
            report_metadata: false,
        };
        assert!(
            s.query([0.; 3], [1., 0., 0.], 1., Pass::Side)
                .unwrap()
                .is_none()
        );
        s.capsule.rotation = [0.; 4];
        assert!(matches!(
            s.query([0.; 3], [1., 0., 0.], 1., Pass::Side),
            Err(Error::InvalidInput)
        ));
        s.capsule.rotation = [0., 0., 0., 1.];
        s.capsule.height = -1.;
        assert!(matches!(
            s.prepare([0.; 3], [1., 0., 0.], [1., 0., 0.]),
            Err(Error::InvalidInput)
        ));
        s.capsule.height = 1.2;
        assert!(matches!(
            s.query([0.; 3], [0.; 3], 1., Pass::Side),
            Err(Error::InvalidInput)
        ));
        assert!(matches!(
            s.query([0.; 3], [1., 0., 0.], f32::INFINITY, Pass::Side),
            Err(Error::InvalidInput)
        ));
    }
}
