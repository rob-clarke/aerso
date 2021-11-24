use crate::WindModel;
use crate::types::{Vector3,Float};

/// Built-in [WindModel] to represent a constant wind
pub struct ConstantWind<T: Float> {
    wind: Vector3<T>,
}

impl<T: Float> ConstantWind<T> {
    /// Create a new ConstantWind model with `wind`
    /// 
    /// # Arguments
    /// 
    /// * `wind` - The wind direction vector (N,E,D)
    pub fn new(wind: Vector3<T>) -> Self {
        ConstantWind {
            wind,
        }
    }
}

impl<T: Float> WindModel<T> for ConstantWind<T> {
    fn get_wind(&self, _position: &Vector3<T>) -> Vector3<T> {
        self.wind
    }
    
    fn step(&mut self, _delta_t: T) {}
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn test_constant() {
        // Create random wind
        // Test output is the same
    }
}