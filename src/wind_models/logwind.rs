use crate::{Vector3,WindModel};
use crate::types::Float;

pub struct LogWind<T: Float> {
    d: T,
    z0: T,
    u_star: T,
    bearing: T,
}

impl<T: Float> LogWind<T> {
    
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
