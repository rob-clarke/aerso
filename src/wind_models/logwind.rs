use crate::{Real,Vector3,WindModel};

pub struct LogWind {
    d: Real,
    z0: Real,
    u_star: Real,
    bearing: Real,
}

impl LogWind {
    pub fn new(d: Real, z0: Real, u_star: Real, bearing: Real) -> LogWind {
        LogWind {
            d,
            z0,
            u_star,
            bearing,
        }
    }
}

impl WindModel for LogWind {
    fn get_wind(&self, position: &Vector3) -> Vector3 {
        const K: Real = 0.41;
        let velocity = self.u_star/K * ((position.z - self.d) / self.z0).ln();
        let bearing_rad = self.bearing.to_radians();
        Vector3::new(
            velocity * bearing_rad.cos(),
            velocity * bearing_rad.sin(),
            0.0)
    }
    
    fn step(&mut self, _delta_t: Real) {}
}

#[test]
fn test_logcalc() {
    
}
