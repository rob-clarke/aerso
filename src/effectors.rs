use crate::{AeroBody,AirState,WindModel,DensityModel};
use crate::types::{Vector3,Frame,Force,Torque};

use crate::types::{Float,DefaultFloatRepr};

/// Interface to an aerodynamic effect
pub trait AeroEffect<I: Copy = [DefaultFloatRepr;4], T: Float = DefaultFloatRepr> {
    fn get_effect(&self, airstate: AirState<T>, rates: Vector3<T>, inputstate: I) -> (Force<T>,Torque<T>);
}

use crate::wind_models::ConstantWind;
use crate::aero::StandardDensity;

pub struct AffectedBody<I: Copy = [DefaultFloatRepr;4], T: Float = DefaultFloatRepr, W: WindModel<T> = ConstantWind<T>, D: DensityModel<T> = StandardDensity> {
    pub body: AeroBody<T,W,D>,
    pub effectors: Vec<Box<dyn AeroEffect<I,T>>>,
}

impl<I: Copy, T: Float, W: WindModel<T>, D: DensityModel<T>> AffectedBody<I,T,W,D> {
    
    pub fn step(&mut self, delta_t: T, inputstate: I) {
        let airstate = self.body.get_airstate();
        let rates = self.body.rates();
        let ft_pairs = self.effectors.iter().map(|e| e.get_effect(airstate,rates,inputstate) );
        
        let mut forces = Vec::<Force<T>>::with_capacity(self.effectors.len());
        let mut torques = Vec::<Torque<T>>::with_capacity(self.effectors.len());
        for (f,t) in ft_pairs {
            forces.push(f);
            torques.push(t);
        }
        
        self.body.step(&forces,&torques,delta_t);
    }
   
    /// Get body acceleration in previous timestep
    pub fn acceleration(&self) -> Vector3<T> {
        self.body.acceleration()
    }
    
}

use crate::types::{UnitQuaternion,StateVector,StateView};
impl<T: Float, W: WindModel<T>, D: DensityModel<T>, I: Copy> StateView<T> for AffectedBody<I,T,W,D> {
    fn position(&self) -> Vector3<T> {
        self.body.position()
    }
    
    fn velocity_in_frame(&self, frame: Frame) -> Vector3<T> {
        self.body.velocity_in_frame(frame)
    }
    
    fn attitude(&self) -> UnitQuaternion<T> {
        self.body.attitude()
        }
    
    fn rates_in_frame(&self, frame: Frame) -> Vector3<T> {
        self.body.rates_in_frame(frame)
    }
    
    fn statevector(&self) -> StateVector<T> {
        self.body.statevector()
    }
}