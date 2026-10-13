//! Recovered mesh contact publication at a supplied internal iteration boundary.
//!
//! Triangle cache, raw object word, saved-register inputs and the object's bounds
//! service are supplied. This fragment stops before actor transforms, reports and
//! further iteration. It does not discover objects or execute their bounds query.
//! Finite normal/zero arithmetic is the compared domain; LAB guards additionally
//! reject unreadable indices and nonfinite/overflowing arithmetic atomically.

use crate::precise_capsule::{Triangle, Vec3};

pub const OBJECT_BOUNDS_MARGIN: f32 = f32::from_bits(0x3f81_47ae);

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct State {
    pub flags: u32,
    pub down_normal: Vec3,
    pub side_normal: Vec3,
    pub triangle_min: f32,
    pub triangle_height: f32,
    pub bound_height: f32,
}

pub struct Input<'a> {
    pub pass_id: u32,
    /// Original raw word at object+8. Only the observed comparison to six is used.
    pub object_word: u16,
    pub nonwalkable_mode: u32,
    pub triangle_index: u32,
    pub triangles: &'a [Triangle],
    pub up: Vec3,
    /// Cached double min[3], max[3]; narrowing happens after their midpoint sum.
    pub cache_bounds: [f64; 6],
    /// Supplied object.v48 output min[3], max[3], with the requested margin applied
    /// by that service. No additional margin inflation is performed here.
    pub object_bounds: [f32; 6],
    pub prevent_ceiling_slide: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Outcome {
    /// The fragment's incoming DIL is supplied as zero. It can change on side
    /// contact only when prevent_ceiling_slide is set and dot(normal,up)<0.
    pub backface: bool,
    /// Some(margin) records the supplied object-bounds service request.
    pub bounds_request: Option<f32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    InvalidInput,
    NonFiniteArithmetic,
}

fn xy(a: Vec3, b: Vec3) -> f32 {
    (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]
}
fn yx(a: Vec3, b: Vec3) -> f32 {
    (a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]
}
fn finite(s: &State) -> bool {
    s.down_normal
        .into_iter()
        .chain(s.side_normal)
        .chain([s.triangle_min, s.triangle_height, s.bound_height])
        .all(f32::is_finite)
}
fn normal(triangle: Triangle) -> Result<Vec3, Error> {
    let ab: Vec3 = std::array::from_fn(|i| triangle[1][i] - triangle[0][i]);
    let ac: Vec3 = std::array::from_fn(|i| triangle[2][i] - triangle[0][i]);
    let n = [
        ab[1] * ac[2] - ac[1] * ab[2],
        ab[2] * ac[0] - ac[2] * ab[0],
        ab[0] * ac[1] - ac[0] * ab[1],
    ];
    let length = yx(n, n).sqrt();
    if !ab.into_iter().chain(ac).chain(n).all(f32::is_finite) || !length.is_finite() {
        return Err(Error::NonFiniteArithmetic);
    }
    if length > 0. {
        let reciprocal = 1. / length;
        Ok(n.map(|x| x * reciprocal))
    } else {
        Ok(n)
    }
}

/// Apply the original fragment's flags/normals/heights, preserving untouched
/// fields. Any LAB validation/arithmetic failure leaves the caller state intact.
pub fn publish(state: &mut State, c: &Input<'_>) -> Result<Outcome, Error> {
    if !finite(state)
        || !c.up.into_iter().chain(c.object_bounds).all(f32::is_finite)
        || !c.cache_bounds.into_iter().all(f64::is_finite)
        || c.triangles
            .iter()
            .flatten()
            .flatten()
            .any(|x| !x.is_finite())
    {
        return Err(Error::InvalidInput);
    }
    let mut next = *state;
    let mut outcome = Outcome {
        backface: false,
        bounds_request: None,
    };
    let valid = c.triangle_index != u32::MAX;
    let down = c.pass_id == 2;
    let side = matches!(c.pass_id, 1 | 3);
    let eligible = valid
        && if down {
            c.object_word == 6 || c.nonwalkable_mode == 3
        } else {
            side && c.object_word == 6
        };
    if down {
        next.flags &= !0x30;
    }
    if eligible {
        let triangle = *c
            .triangles
            .get(c.triangle_index as usize)
            .ok_or(Error::InvalidInput)?;
        let n = normal(triangle)?;
        if down {
            next.flags |= 4;
            let height = triangle.map(|p| xy(p, c.up));
            let center =
                std::array::from_fn(|i| ((c.cache_bounds[i] + c.cache_bounds[i + 3]) * 0.5) as f32);
            let base = yx(center, c.up);
            if !height
                .into_iter()
                .chain(center)
                .chain([base])
                .all(f32::is_finite)
            {
                return Err(Error::NonFiniteArithmetic);
            }
            // Original MINSS/MAXSS use their second operand on equality, including
            // opposite signed zeros: B against A, then the result against C.
            let maximum = if height[1] > height[0] {
                height[1]
            } else {
                height[0]
            };
            let maximum = if maximum > height[2] {
                maximum
            } else {
                height[2]
            };
            let minimum = if height[1] < height[0] {
                height[1]
            } else {
                height[0]
            };
            let minimum = if minimum < height[2] {
                minimum
            } else {
                height[2]
            };
            next.triangle_min = minimum + base;
            next.triangle_height = maximum + base;
            next.down_normal = n;
            next.bound_height = xy(
                [c.object_bounds[3], c.object_bounds[4], c.object_bounds[5]],
                c.up,
            );
            outcome.bounds_request = Some(OBJECT_BOUNDS_MARGIN);
        } else {
            next.flags |= 8;
            next.side_normal = n;
            if c.prevent_ceiling_slide {
                let facing = xy(n, c.up);
                if !facing.is_finite() {
                    return Err(Error::NonFiniteArithmetic);
                }
                outcome.backface = facing < 0.;
            }
        }
    }
    if !finite(&next) {
        return Err(Error::NonFiniteArithmetic);
    }
    *state = next;
    Ok(outcome)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn state() -> State {
        State {
            flags: 0xff,
            down_normal: [0.2, 0.3, 0.4],
            side_normal: [0.5, 0.6, 0.7],
            triangle_min: -9.,
            triangle_height: 8.,
            bound_height: 7.,
        }
    }
    fn input(triangles: &[Triangle]) -> Input<'_> {
        Input {
            pass_id: 2,
            object_word: 6,
            nonwalkable_mode: 0,
            triangle_index: 0,
            triangles,
            up: [0., 1., 0.],
            cache_bounds: [-1., -2., -3., 4., 5., 6.],
            object_bounds: [-2., -3., -4., 5., 6., 7.],
            prevent_ceiling_slide: true,
        }
    }
    #[test]
    fn down_clears_flags_without_reading_ineligible_triangle() {
        let mut s = state();
        let before = s;
        let mut c = input(&[]);
        c.object_word = 5;
        assert_eq!(
            publish(&mut s, &c).unwrap(),
            Outcome {
                backface: false,
                bounds_request: None
            }
        );
        assert_eq!(s.flags, before.flags & !0x30);
        assert_eq!(s.down_normal, before.down_normal);
        assert_eq!(s.triangle_height, before.triangle_height);
    }
    #[test]
    fn side_mode_three_does_not_bypass_object_gate() {
        let tris = [[[0., 0., 0.], [1., 0., 0.], [0., 0., 1.]]];
        let mut c = input(&tris);
        c.pass_id = 3;
        c.nonwalkable_mode = 3;
        let mut s = state();
        assert!(publish(&mut s, &c).unwrap().backface);
        c.object_word = 5;
        let mut s = state();
        let before = s;
        assert!(!publish(&mut s, &c).unwrap().backface);
        assert_eq!(s, before);
    }
    #[test]
    fn invalid_active_index_leaves_all_publication_fields_intact() {
        let mut s = state();
        let before = s;
        assert_eq!(publish(&mut s, &input(&[])), Err(Error::InvalidInput));
        assert_eq!(s, before);
    }
    #[test]
    fn cached_midpoint_narrows_after_double_sum() {
        let tris = [[[0., 0., 0.], [0., 0., 1.], [1., 0., 0.]]];
        let mut c = input(&tris);
        c.cache_bounds = [-1., 1_073_741_825., -3., 1., 1_073_741_953., 3.];
        let mut s = state();
        publish(&mut s, &c).unwrap();
        assert_eq!(s.triangle_height.to_bits(), (1_073_741_952_f32).to_bits());
    }
    #[test]
    fn side_projection_overflow_preserves_all_publication_fields() {
        let tris = [[[0., 0., 0.], [1., -1., 0.], [0., 1., -1.]]];
        let mut c = input(&tris);
        c.pass_id = 1;
        c.up = [f32::MAX; 3];
        let mut s = state();
        let before = s;
        assert_eq!(publish(&mut s, &c), Err(Error::NonFiniteArithmetic));
        assert_eq!(s, before);
    }
}
