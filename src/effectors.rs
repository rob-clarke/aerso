use crate::{Vector3,Force,Torque,AeroBody,Frame,AirState,WindModel,DensityModel};
use crate::types::Float;

/// Interface to an aerodynamic effect
pub trait AeroEffect<T: Float,I: Copy> {
    fn get_effect(&self, airstate: AirState<T>, rates: Vector3<T>, inputstate: I) -> (Force<T>,Torque<T>);
}

pub struct AffectedBody<T: Float, W: WindModel<T>, D: DensityModel<T>, I: Copy> {
    pub body: AeroBody<T,W,D>,
    pub effectors: Vec<Box<dyn AeroEffect<T,I>>>,
}

impl<T: Float, W: WindModel<T>, D: DensityModel<T>, I: Copy> AffectedBody<T,W,D,I> {
    
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
    
}

use crate::{StateView,StateVector,UnitQuaternion};
impl<T: Float, W: WindModel<T>, D: DensityModel<T>, I: Copy> StateView<T> for AffectedBody<T,W,D,I> {
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