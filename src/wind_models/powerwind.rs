use crate::WindModel;
use crate::types::{Vector3,Float};

/// Built-in [WindModel] to represent a [wind profile power law](https://en.wikipedia.org/wiki/Wind_profile_power_law)
pub struct PowerWind<T: Float> {
    u_r: T,
    z_r: T,
    alpha: T,
    bearing: T,
}

impl<T: Float> PowerWind<T> {
    /// Create a new [PowerWind] model with specified parameters
    /// 
    /// # Arguments
    /// 
    /// * `u_r` - Reference wind speed (m·s<sup>-1</sup>) 
    /// * `z_r` - Reference wind height (m)
    /// * `bearing` - The bearing for the calculated wind vector (deg)
    /// * `alpha` - Power law exponent
    pub fn new_with_alpha(u_r: T, z_r: T, bearing: T, alpha: T) -> Self {
        PowerWind { u_r, z_r, bearing, alpha }
    }
    
    /// Create a new [PowerWind] model with specoified parameters and typical alpha
    /// 
    /// The alpha parameter will be set to `0.143`
    /// 
    /// # Arguments
    /// 
    /// * `u_r` - Reference wind speed (m·s<sup>-1</sup>) 
    /// * `z_r` - Reference wind height (m)
    /// * `bearing` - The bearing for the calculated wind vector (deg)
    pub fn new(u_r: T, z_r: T, bearing: T) -> Self {
        let alpha_typical = T::from(0.143).unwrap();
        PowerWind::new_with_alpha(u_r, z_r, bearing, alpha_typical)
    }
}

impl<T: Float> WindModel<T> for PowerWind<T> {
    fn get_wind(&self, position: &Vector3<T>) -> Vector3<T> {
        let velocity = self.u_r * <T as num_traits::Float>::powf(position.z / self.z_r,self.alpha);
        let bearing_rad = self.bearing.to_radians();
        Vector3::new(
            velocity * <T as num_traits::Float>::cos(bearing_rad),
            velocity * <T as num_traits::Float>::sin(bearing_rad),
            T::zero())
    }
    
    fn step(&mut self, _delta_t: T) {}
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_powercalc() {
        use approx::assert_relative_eq;
        
        const U_R: f64 = 10.0;
        const Z_R: f64 = 10.0;
        const ALPHA: f64 = 0.143;
        
        let wind_model = PowerWind::new(U_R,Z_R,0.0);
        
        for height_idx in 0..20 {
            let height = height_idx as f64 * 0.1;
            
            let expected_result = U_R * (height/Z_R).powf(ALPHA);
        
            let wind = wind_model.get_wind(&Vector3::new(0.0,0.0,height));
            assert_relative_eq!(wind.x,expected_result);
        }
    }
}