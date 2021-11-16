use crate::{Real,Vector3,WindModel};

pub struct ConstantWind {
    wind: Vector3,
}

impl ConstantWind {
    pub fn new(wind: Vector3) -> ConstantWind {
        ConstantWind {
            wind,
        }
    }
}

impl WindModel for ConstantWind {
    fn get_wind(&self, _position: &Vector3) -> Vector3 {
        self.wind
    }
    
    fn step(&mut self, _delta_t: Real) {}
}

#[test]
fn test_constant() {
    // Create random wind
    // Test output is the same
}