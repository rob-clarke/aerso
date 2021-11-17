extern crate nalgebra as na;
use crate::types::{Real,Vector3,Matrix3,UnitQuaternion,Frame,StateVector,StateView,Force,Torque};

/// Represent a 6DoF body affected by gravity
pub struct Body<T: Real> {
    /// Mass of body (kg)
    mass: T,
    /// Inertia matrix
    inertia: Matrix3<T>,
    /// Inertia inverse
    inertia_inverse: Matrix3<T>,
    /// 13-dimensional state vector
    pub statevector: StateVector<T>,
}


impl<T: Real> Body<T> {
    
    const GRAVITY_VECTOR: Vector3<T> = Vector3::new(T::zero(),T::zero(),T::from(physical_constants::STANDARD_ACCELERATION_OF_GRAVITY));

    /// Create a new instance of Body with `mass` and `inertia` in specified state 
    pub fn new(mass: T, inertia: Matrix3<T>, position: Vector3<T>, velocity: Vector3<T>, attitude: UnitQuaternion<T>, rates: Vector3<T>) -> Self {
        let statevector = StateVector::from_vec(vec![
            position[0], position[1], position[2],
            velocity[0], velocity[1], velocity[2],
            attitude[3], attitude[0], attitude[1], attitude[2],
            rates[0],    rates[1],    rates[2]
            ]);
        Body::new_from_statevector(mass,inertia,statevector)
    }
    
    /// Create a new instance of Body with `mass` and `inertia` in specified state
    /// statevector is made of [position,body_velocity,attitude_quaternion,body_axis_rates]
    pub fn new_from_statevector(mass: T, inertia: Matrix3<T>, statevector: StateVector<T>) -> Self {
        let inertia_inverse = match inertia.try_inverse() {
            Some(inverted) => inverted,
            None => { panic!("Unable to invert inertia matrix") }
        };
        Body {
            mass,
            inertia,
            inertia_inverse,
            statevector,
        }
    }
    
    /// Construct the Direction Cosine Matrix (DCM) from the state attitude
    /// Transforms quantites from the world frame to the body frame
    pub fn get_dcm(state: &StateVector<T>) -> Matrix3<T> {
        let q = state.fixed_rows::<4>(6);
        let q0 = q[0]; let q02 = q0.powi(2);
        let q1 = q[1]; let q12 = q1.powi(2);
        let q2 = q[2]; let q22 = q2.powi(2);
        let q3 = q[3]; let q32 = q3.powi(2);
        
        let two: T = 2.0.into();
        
        Matrix3::<T>::new(
            q02+q12-q22-q32,   two*(q1*q2+q0*q3), two*(q1*q3-q0*q2),
            two*(q1*q2-q0*q3), q02-q12+q22-q32,   two*(q2*q3+q0*q1),
            two*(q1*q3+q0*q2), two*(q2*q3-q0*q1), q02-q12-q22+q32
        )
    }
    
    /// Construct the inverse DCM
    /// Transforms quantities from the body frame to the world frame
    pub fn get_dcm_body(state: &StateVector<T>) -> Matrix3<T> {
        Body::get_dcm(state).transpose()
    }
    
    /// Calculate the state derivative
    /// state: 13-dimensional state vector to get derivative about
    /// forces: Vector of applied forces, both world and body frame
    /// torques: Vector of applied torques, both world and body frame
    /// 
    /// NB: Gravity is included by default
    fn get_derivative(&self, state: &StateVector<T>, forces: &Vec<Force<T>>, torques: &Vec<Torque<T>>) -> StateVector<T> {
        let mut world_forces = Self::GRAVITY_VECTOR * self.mass;
        let mut body_forces = Vector3::zeros();
        for force in forces {
            match force.frame {
                Frame::World => { world_forces += force.force },
                Frame::Body  => { body_forces += force.force },
            }
        }
        
        let mut world_torques = Vector3::zeros();
        let mut body_torques = Vector3::zeros();
        for torque in torques {
            match torque.frame {
                Frame::World => { world_torques += torque.torque },
                Frame::Body  => { body_torques += torque.torque },
            }
        }
        
        let position_dot = Body::get_dcm_body(&state) * state.velocity();
        let velocity_dot = state.velocity().cross(&state.rates()) + ( (Body::get_dcm(&state) * world_forces) + body_forces ) * (T::from(1.0)/self.mass);
        
        let o_x = state.rates()[0];
        let o_y = state.rates()[1];
        let o_z = state.rates()[2];
        let qdot_matrix = na::Matrix4::<T>::new(
            T::zero(), -o_x,       -o_y,       -o_z,
            o_x,        T::zero(),  o_z,       -o_y,
            o_y,       -o_z,        T::zero(),  o_x,
            o_z,        o_y,       -o_x,        T::zero()
            );
        
        let attitude_dot = qdot_matrix * state.fixed_rows::<4>(6) * T::from(0.5);
        let rates_dot = self.inertia_inverse * (Body::get_dcm(&state) * world_torques + body_torques - (self.inertia * state.rates()).cross(&state.rates()) );
        
        StateVector::from_vec(vec![
            position_dot[0], position_dot[1], position_dot[2],
            velocity_dot[0], velocity_dot[1], velocity_dot[2],
            attitude_dot[0], attitude_dot[1], attitude_dot[2], attitude_dot[3],
            rates_dot[0],    rates_dot[1],    rates_dot[2]
            ])
        
    }
    
    /// Propagate the state vector by delta_t under the supplied forces and torques
    ///
    /// Uses 4th-order Runge-Kutta integration
    /// 
    /// forces: Vector of applied forces, both world and body frame
    /// torques: Vector of applied torques, both world and body frame
    /// deltaT: Timestep (s)
    ///
    /// NB: Gravity is included by default
    pub fn step(&mut self, forces: &Vec<Force<T>>, torques: &Vec<Torque<T>>, delta_t: T) {
        let k1 = self.get_derivative(&self.statevector, &forces, &torques);
        let k2 = self.get_derivative(&(self.statevector + k1 * delta_t/T::from(2.0)), &forces, &torques);
        let k3 = self.get_derivative(&(self.statevector + k2 * delta_t/T::from(2.0)), &forces, &torques);
        let k4 = self.get_derivative(&(self.statevector + k3 * delta_t),            &forces, &torques);
        
        self.statevector = self.statevector + (k1 + k2*T::from(2.0) + k3*T::from(2.0) + k4) * delta_t/T::from(6.0);
    }
    
}

/// Add state vector helpers to the Body struct
impl<T: Real> StateView<T> for Body<T> {
    fn position(&self) -> Vector3<T> {
        self.statevector.position()
    }
    
    fn velocity_in_frame(&self, frame: Frame) -> Vector3<T> {
        self.statevector.velocity_in_frame(frame)
    }
    
    fn attitude(&self) -> UnitQuaternion<T> {
        self.statevector.attitude()
        }
    
    fn rates_in_frame(&self, frame: Frame) -> Vector3<T> {
        self.statevector.rates_in_frame(frame)
    }
    
    fn statevector(&self) -> StateVector<T> {
        self.statevector
    }
}
