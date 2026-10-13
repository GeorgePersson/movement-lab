//! Finite supplied-state request adjustment. Cached bounds and projection mask
//! are explicit inputs; their native first-use initialization is not simulated.
//! RSQRTPS uses the executing x86_64 CPU's estimate, not a portable sqrt substitute.
use crate::contact_reports::ContactReports;
#[derive(Clone, Copy, Debug)]
pub struct AdjustmentInput {
    pub request: [f32; 4],
    pub dt: f32,
    pub value_3c8: f32,
    pub normal: [f32; 4],
    pub cached_lower: f32,
    pub upper: f32,
    pub projection_mask: [f32; 4],
    pub split_vertical: u8,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AdjustmentResult {
    pub request: [f32; 4],
    pub split_vertical: u8,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AdjustmentError {
    NonFinite,
    UnsupportedTarget,
}
pub fn select_normal(reports: &ContactReports, default_normal: [f32; 4]) -> [f32; 4] {
    if reports.primary_valid == 0 {
        default_normal
    } else if reports.secondary_valid != 0 {
        reports.secondary.normal
    } else {
        reports.primary.normal
    }
}
fn dot(a: [f32; 4], b: [f32; 4]) -> f32 {
    (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]
}
pub fn adjust(input: AdjustmentInput) -> Result<AdjustmentResult, AdjustmentError> {
    if !input
        .request
        .into_iter()
        .chain(input.normal)
        .chain(input.projection_mask)
        .chain([input.dt, input.value_3c8, input.cached_lower, input.upper])
        .all(f32::is_finite)
    {
        return Err(AdjustmentError::NonFinite);
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        let _ = input;
        Err(AdjustmentError::UnsupportedTarget)
    }
    #[cfg(target_arch = "x86_64")]
    {
        let mut out = AdjustmentResult {
            request: input.request,
            split_vertical: input.split_vertical,
        };
        if input.request[1] < input.dt
            || input.value_3c8 <= 1.0
            || input.normal[1] < input.cached_lower
            || input.normal[1] > input.upper
        {
            return Ok(out);
        }
        let hn = std::array::from_fn(|i| input.normal[i] * input.projection_mask[i]);
        let hd = std::array::from_fn(|i| input.request[i] * input.projection_mask[i]);
        let s = dot(hn, hn);
        if !s.is_finite() {
            return Err(AdjustmentError::NonFinite);
        }
        if s <= 1e-5 {
            return Ok(out);
        }
        // SSE is mandatory on x86_64; preserve the original estimate instruction.
        let e = unsafe {
            use std::arch::x86_64::*;
            _mm_cvtss_f32(_mm_rsqrt_ps(_mm_set1_ps(s)))
        };
        let correction = 3.0 - ((e * s) * e);
        let n: [f32; 4] = std::array::from_fn(|i| correction * ((e * hn[i]) * 0.5));
        let tangent = [f32::from_bits(n[2].to_bits() ^ 0x80000000), 0.0, n[0], 0.0];
        if dot(n, hd) < 0.0 {
            let d = dot(hd, tangent);
            out.request[0] = d * tangent[0];
            out.request[2] = d * tangent[2];
        }
        out.request[1] =
            (0.0 - (tangent[0] * input.normal[2] - n[0] * input.normal[0])) * input.request[1];
        out.split_vertical = 1;
        if !out.request.into_iter().all(f32::is_finite) {
            return Err(AdjustmentError::NonFinite);
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn input() -> AdjustmentInput {
        AdjustmentInput {
            request: [-1.0, 1.0, -1.0, -0.0],
            dt: 0.01,
            value_3c8: 2.0,
            normal: [0.6, -0.8, 0.8, 0.0],
            cached_lower: -1.0,
            upper: 0.0,
            projection_mask: [1.0, 0.0, 1.0, 0.0],
            split_vertical: 7,
        }
    }
    #[test]
    fn selection_requires_primary_validity() {
        let mut r = ContactReports::default();
        r.primary.normal = [1.0; 4];
        r.secondary.normal = [2.0; 4];
        r.secondary_valid = 1;
        assert_eq!(select_normal(&r, [3.0; 4]), [3.0; 4]);
        r.primary_valid = 1;
        assert_eq!(select_normal(&r, [3.0; 4]), [2.0; 4]);
        r.secondary_valid = 0;
        assert_eq!(select_normal(&r, [3.0; 4]), [1.0; 4]);
    }
    #[test]
    fn rejected_request_preserves_flag_and_all_bits() {
        let mut i = input();
        i.dt = 2.0;
        let o = adjust(i).unwrap();
        assert_eq!(o.request.map(f32::to_bits), i.request.map(f32::to_bits));
        assert_eq!(o.split_vertical, 7);
        i = input();
        i.value_3c8 = 1.0;
        assert_eq!(adjust(i).unwrap().split_vertical, 7);
    }
    #[test]
    fn zero_projection_preserves_state() {
        let mut i = input();
        i.normal = [0.0, -0.5, 0.0, 0.0];
        let o = adjust(i).unwrap();
        assert_eq!(o.split_vertical, 7);
        assert_eq!(o.request.map(f32::to_bits), i.request.map(f32::to_bits));
    }
    #[test]
    fn inward_projection_sets_flag_preserving_w() {
        let i = input();
        let o = adjust(i).unwrap();
        assert_eq!(o.split_vertical, 1);
        assert_eq!(o.request[3].to_bits(), i.request[3].to_bits());
        assert!(o.request[0].abs() < 1.0 && o.request[2].abs() < 1.0);
    }
    #[test]
    fn rejects_nonfinite_input() {
        let mut i = input();
        i.normal[0] = f32::NAN;
        assert_eq!(adjust(i), Err(AdjustmentError::NonFinite));
    }
}
