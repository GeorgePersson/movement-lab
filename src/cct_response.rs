//! Original mixed-precision target-response equations recovered offline.
//!
//! Target/current positions use f64; reflected components, remaining length and
//! scaled displacements use f32. Inputs must be finite with moderate arithmetic.
//! This helper updates only a target. Contact selection, pass scheduling, velocity
//! changes and caller coefficient policy belong to the caller.

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    (a[1] * b[1] + a[0] * b[0]) + a[2] * b[2]
}
fn length(v: [f32; 3]) -> f32 {
    dot(v, v).sqrt()
}
fn unit(v: [f32; 3]) -> [f32; 3] {
    let len = length(v);
    if len > 0.0 {
        let inverse = 1.0 / len;
        v.map(|x| x * inverse)
    } else {
        v
    }
}

/// Return the new target; do not reinterpret this as a base-velocity projection.
/// Zero coefficients skip their addition entirely, preserving double signed zero.
#[allow(clippy::too_many_arguments)]
pub fn response(
    target: [f64; 3],
    current: [f64; 3],
    direction: [f32; 3],
    normal: [f32; 3],
    bump: f32,
    tangent: f32,
    normalize_components: bool,
) -> [f64; 3] {
    let projection = dot(direction, normal);
    let reflected = unit(std::array::from_fn(|i| {
        direction[i] - (normal[i] + normal[i]) * projection
    }));
    let projection = dot(normal, reflected);
    let normal_part = normal.map(|x| projection * x);
    let tangent_part = std::array::from_fn(|i| reflected[i] - normal_part[i]);
    let remaining = length(std::array::from_fn(|i| (target[i] - current[i]) as f32));
    let mut out = current;
    if bump != 0.0 {
        let component = if normalize_components {
            unit(normal_part)
        } else {
            normal_part
        };
        for i in 0..3 {
            out[i] += ((component[i] * bump) * remaining) as f64;
        }
    }
    if tangent != 0.0 {
        let component = if normalize_components {
            unit(tangent_part)
        } else {
            tangent_part
        };
        for i in 0..3 {
            out[i] += ((component[i] * tangent) * remaining) as f64;
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn zero_coefficients_preserve_double_current_bits() {
        let current = [-0.0, 1.0e15 + 0.125, 0.0];
        let output = response(
            [4.0, 5.0, 6.0],
            current,
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            -0.0,
            0.0,
            true,
        );
        assert_eq!(output.map(f64::to_bits), current.map(f64::to_bits));
    }
    #[test]
    fn remaining_difference_is_computed_before_f32_conversion() {
        let current = [1.0e15; 3];
        let output = response(
            [current[0] + 2.0, current[1], current[2]],
            current,
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            0.0,
            1.0,
            false,
        );
        assert_eq!(output, [current[0] + 2.0, current[1], current[2]]);
    }
    #[test]
    fn response_returns_target_without_input_velocity_mutation() {
        let direction = [1.0, 0.0, 0.0];
        assert_eq!(
            response(
                [2.0, 0.0, 0.0],
                [0.0; 3],
                direction,
                [1.0, 0.0, 0.0],
                0.0,
                1.0,
                false
            ),
            [0.0; 3]
        );
        assert_eq!(
            response(
                [2.0, 0.0, 0.0],
                [0.0; 3],
                direction,
                [1.0, 0.0, 0.0],
                1.0,
                0.0,
                false
            ),
            [-2.0, 0.0, 0.0]
        );
        assert_eq!(direction, [1.0, 0.0, 0.0]);
    }
}
