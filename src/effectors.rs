use crate::{Real,Vector3,Force,Torque,AeroBody,Frame,AirState,WindModel,DensityModel};

/// Interface to an aerodynamic effect
pub trait AeroEffect<T: Copy> {
    fn get_effect(&self, airstate: AirState, rates: Vector3, inputstate: T) -> (Force,Torque);
}

pub struct AffectedBody<T: Copy, W: WindModel, D: DensityModel> {
    pub body: AeroBody<W,D>,
    pub effectors: Vec<Box<dyn AeroEffect<T>>>,
}

impl<T: Copy,W: WindModel, D: DensityModel> AffectedBody<T,W,D> {
    
   pub fn step(&mut self, delta_t: Real, inputstate: T) {
       let airstate = self.body.get_airstate();
       let rates = self.body.rates();
       let ft_pairs = self.effectors.iter().map(|e| e.get_effect(airstate,rates,inputstate) );
       
       let mut forces = Vec::<Force>::with_capacity(self.effectors.len());
       let mut torques = Vec::<Torque>::with_capacity(self.effectors.len());
       for (f,t) in ft_pairs {
           forces.push(f);
           torques.push(t);
       }
       
       self.body.step(&forces,&torques,delta_t);
   }
    
}

use crate::{StateView,StateVector,UnitQuaternion};
impl<T: Copy, W: WindModel, D: DensityModel> StateView for AffectedBody<T,W,D> {
    fn position(&self) -> Vector3 {
        self.body.position()
    }
    
    fn velocity_in_frame(&self, frame: Frame) -> Vector3 {
        self.body.velocity_in_frame(frame)
    }
    
    fn attitude(&self) -> UnitQuaternion {
        self.body.attitude()
        }
    
    fn rates_in_frame(&self, frame: Frame) -> Vector3 {
        self.body.rates_in_frame(frame)
    }
    
    fn statevector(&self) -> StateVector {
        self.body.statevector()
    }
}