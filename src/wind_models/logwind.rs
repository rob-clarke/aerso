use crate::WindModel;
use crate::types::{Vector3,Float};

/// Built-in [WindModel] to represent a [log wind profile](https://en.wikipedia.org/wiki/Log_wind_profile)
pub struct LogWind<T: Float> {
    d: T,
    z0: T,
    u_star: T,
    bearing: T,
}

impl<T: Float> LogWind<T> {
    /// Create a new LogWind with specified parameters
    /// 
    /// # Arguments
    /// 
    /// * `d` - Zero plane displacement (m)
    /// * `z0` - Surface roughness (m)
    /// * `u_star` - Friction velocity (m·s<sup>-1</sup>)
    /// * `bearing` - The bearing for the calculated wind vector (deg)
    pub fn new(d: T, z0: T, u_star: T, bearing: T) -> Self {
        LogWind {
            d,
            z0,
            u_star,
            bearing,
        }
    }
}

impl<T: Float> WindModel<T> for LogWind<T> {
    fn get_wind(&self, position: &Vector3<T>) -> Vector3<T> {
        let k = T::from(0.41).unwrap();
        let velocity = self.u_star/k * <T as num_traits::Float>::ln((position.z - self.d) / self.z0);
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
    fn test_logcalc() {
        
    }
}
