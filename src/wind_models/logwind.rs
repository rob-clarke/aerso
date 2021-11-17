use crate::{Vector3,WindModel};
use crate::types::Real;

pub struct LogWind<T: Real> {
    d: T,
    z0: T,
    u_star: T,
    bearing: T,
}

impl<T: Real> LogWind<T> {
    const K: T = T::from(0.41);
    
    pub fn new(d: T, z0: T, u_star: T, bearing: T) -> Self {
        LogWind {
            d,
            z0,
            u_star,
            bearing,
        }
    }
}

impl<T: Real> WindModel<T> for LogWind<T> {
    fn get_wind(&self, position: &Vector3<T>) -> Vector3<T> {
        let velocity = self.u_star/Self::K * ((position.z - self.d) / self.z0).ln();
        let bearing_rad = self.bearing.to_radians();
        Vector3::new(
            velocity * bearing_rad.cos(),
            velocity * bearing_rad.sin(),
            T::zero())
    }
    
    fn step(&mut self, _delta_t: T) {}
}

#[test]
fn test_logcalc() {
    
}
