use crate::{Real,Vector3,WindModel};

pub struct PowerWind {
    u_r: Real,
    z_r: Real,
    alpha: Real,
    bearing: Real,
}

impl PowerWind {
    const ALPHA_TYPICAL: Real = 0.143;
    
    pub fn new_with_alpha(u_r: Real, z_r: Real, bearing: Real, alpha: Real) -> PowerWind {
        PowerWind { u_r, z_r, bearing, alpha }
    }
    
    pub fn new(u_r: Real, z_r: Real, bearing: Real) -> PowerWind {
        PowerWind::new_with_alpha(u_r, z_r, bearing, PowerWind::ALPHA_TYPICAL)
    }
}

impl WindModel for PowerWind {
    fn get_wind(&self, position: &Vector3) -> Vector3 {
        let velocity = self.u_r * (position.z / self.z_r).powf(self.alpha);
        let bearing_rad = self.bearing.to_radians();
        Vector3::new(
            velocity * bearing_rad.cos(),
            velocity * bearing_rad.sin(),
            0.0)
    }
    
    fn step(&mut self, _delta_t: Real) {}
}

#[test]
fn test_powercalc() {
    use approx::assert_relative_eq;
    
    const U_R: Real = 10.0;
    const Z_R: Real = 10.0;
    const ALPHA: Real = 0.143;
    
    let wind_model = PowerWind::new(U_R,Z_R,0.0);
    
    for height_idx in 0..20 {
        let height = height_idx as Real * 0.1;
        
        let expected_result = U_R * (height/Z_R).powf(ALPHA);
    
        let wind = wind_model.get_wind(&Vector3::new(0.0,0.0,height));
        assert_relative_eq!(wind.x,expected_result);
    }
}
