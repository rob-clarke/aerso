extern crate nalgebra as na;

use crate::types::{Vector3,Matrix3,UnitQuaternion,Frame,StateVector,StateView,Force,Torque};
use crate::types::{Float,DefaultFloatRepr};

// Integrating Rotations using Non-Unit Quaternions
// https://par.nsf.gov/servlets/purl/10097724

/// Represent a 6DoF body affected by gravity
#[derive(Copy,Clone)]
pub struct Body<T: Float = DefaultFloatRepr> {
    /// Mass of body (kg)
    mass: T,
    /// Inertia matrix
    inertia: Matrix3<T>,
    /// Inertia inverse
    inertia_inverse: Matrix3<T>,
    /// 13-dimensional state vector
    /// 
    /// Statevector is formed of \[position,velocity(body),attitude_quaternion(i,j,k,w),axis_rates(body)\]
    statevector: StateVector<T>,
    /// Body frame acceleration of vehicle during last step
    acceleration: Vector3<T>,
}


impl<T: Float> Body<T> {
    /// Create a new instance of Body with `mass` and `inertia` at the origin
    pub fn new_at_origin(mass: T, inertia: Matrix3<T>) -> Self {
        Body::new(mass, inertia, Vector3::zeros(), Vector3::zeros(), UnitQuaternion::from_euler_angles(T::zero(),T::zero(),T::zero()), Vector3::zeros())
    }
    
    /// Create a new instance of Body with `mass` and `inertia` in specified state 
    pub fn new(mass: T, inertia: Matrix3<T>, position: Vector3<T>, velocity: Vector3<T>, attitude: UnitQuaternion<T>, rates: Vector3<T>) -> Self {
        let statevector = StateVector::from_vec(vec![
            position[0], position[1], position[2],
            velocity[0], velocity[1], velocity[2],
            attitude[0], attitude[1], attitude[2], attitude[3],
            rates[0],    rates[1],    rates[2]
            ]);
        Body::new_from_statevector(mass,inertia,statevector)
    }
    
    /// Create a new instance of Body with `mass` and `inertia` in specified state
    /// 
    /// statevector is made of \[position,velocity(body),attitude_quaternion(i,j,k,w),axis_rates(body)\]
    pub fn new_from_statevector(mass: T, inertia: Matrix3<T>, statevector: StateVector<T>) -> Self {
        if mass <= T::zero() {
            panic!("Mass must be >= 0.0")
        }
        
        let inertia_inverse = match inertia.try_inverse() {
            Some(inverted) => inverted,
            None => { panic!("Unable to invert inertia matrix") }
        };
        Body {
            mass,
            inertia,
            inertia_inverse,
            statevector,
            acceleration: Vector3::<T>::new(T::zero(),T::zero(),T::zero()),
        }
    }
    
    /// Construct the Direction Cosine Matrix (DCM) from the state attitude
    /// 
    /// Transforms quantites from the world frame to the body frame
    /// 
    /// Note that this is not a struct method. Usage:
    /// ```
    /// # use aerso::Body;
    /// # use aerso::types::{Matrix3,Vector3,StateView};
    /// # let body = Body::new_at_origin(1.0,Matrix3::identity());
    /// let dcm = Body::get_dcm(&body.statevector());
    /// ```
    /// 
    /// # Arguments
    /// 
    /// * `state` - The statevector to calculate the DCM for
    pub fn get_dcm(state: &StateVector<T>) -> Matrix3<T> {
        // Don't use attitude here to avoid unnecessary square root call
        let q = state.fixed_rows::<4>(6);
        let q0 = q[3]; let q02 = <T as num_traits::Float>::powi(q0,2); // Real part (w)
        let q1 = q[0]; let q12 = <T as num_traits::Float>::powi(q1,2); // i
        let q2 = q[1]; let q22 = <T as num_traits::Float>::powi(q2,2); // j
        let q3 = q[2]; let q32 = <T as num_traits::Float>::powi(q3,2); // k
        
        let two: T = T::from_f64(2.0).unwrap();
        
        // NB: This appears as the transpose of Eq. (13) from the referenced paper
        // This matches the convention of the Stengel notes
        Matrix3::<T>::new(
            q02+q12-q22-q32,   two*(q1*q2+q0*q3), two*(q1*q3-q0*q2),
            two*(q1*q2-q0*q3), q02-q12+q22-q32,   two*(q2*q3+q0*q1),
            two*(q1*q3+q0*q2), two*(q2*q3-q0*q1), q02-q12-q22+q32
        ) * <T as num_traits::Float>::recip(q02+q12+q22+q32)
    }
    
    /// Construct the inverse DCM
    /// 
    /// Transforms quantities from the body frame to the world frame
    ///
    /// Note that this is not a struct method. Usage:
    /// ```
    /// # use aerso::Body;
    /// # use aerso::types::{Matrix3,Vector3,StateView};
    /// # let body = Body::new_at_origin(1.0,Matrix3::identity());
    /// let dcm_body = Body::get_dcm_body(&body.statevector());
    /// ```
    /// 
    /// # Arguments
    /// 
    /// * `state` - The statevector to calculate the DCM for
    pub fn get_dcm_body(state: &StateVector<T>) -> Matrix3<T> {
        Body::get_dcm(state).transpose()
    }
    
    /// Calculate the state derivative
    /// 
    /// NB: Gravity is included by default
    /// 
    /// # Arguments
    /// * `state` - 13-dimensional state vector to get derivative about
    /// * `forces` - Vector of applied forces, both world and body frame
    /// * `torques` - Vector of applied torques, both world and body frame
    pub fn get_derivative(&self, state: &StateVector<T>, forces: &[Force<T>], torques: &[Torque<T>]) -> StateVector<T> {
        let gravity_accel: Vector3<T> = Vector3::new(
            T::zero(),
            T::zero(),
            T::from(physical_constants::STANDARD_ACCELERATION_OF_GRAVITY).unwrap()
            );
        let mut world_forces = gravity_accel * self.mass;
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
        
        let dcm = Body::get_dcm(state);
        let dcm_body = dcm.transpose();

        let position_dot = dcm_body * state.velocity();
        let velocity_dot = state.velocity().cross(&state.rates()) + ( (dcm * world_forces) + body_forces ) * <T as num_traits::Float>::recip(self.mass);
        
        let o_x = state.rates()[0];
        let o_y = state.rates()[1];
        let o_z = state.rates()[2];
        // NB: This matrix appears different from Eq. (21) in the reference due to quaternion ordering
        // The paper uses [w,i,j,k] but nalgebra uses [i,j,k,w]
        // Hence row/column 1 is shifted to row/column 4
        let qdot_matrix = na::Matrix4::<T>::new(
            T::zero(), o_z,       -o_y,       o_x,
            -o_z,      T::zero(), o_x,        o_y,
             o_y,      -o_x,      T::zero(),  o_z,
            -o_x,      -o_y,      -o_z,       T::zero()
            );
        
        // NB: Quaternion does not remain normalised throughout integration
        let q = state.fixed_rows::<4>(6); // Don't use attitude here to avoid uneccesary square root
        #[cfg(not(feature="constrain-qnorm-drift"))]
        let attitude_dot = qdot_matrix * q * T::from(0.5).unwrap();
        #[cfg(feature="constrain-qnorm-drift")]
        let attitude_dot = {
            // Use Eq. (23) from the paper to set the free parameter to constrain the drift of the quaternion norm
            let qnorm_sqd = <T as num_traits::Float>::powi(q[0],2) + 
                        <T as num_traits::Float>::powi(q[1],2) +
                        <T as num_traits::Float>::powi(q[2],2) +
                        <T as num_traits::Float>::powi(q[3],2);
            let k = T::from_f64(0.001).unwrap();
            let c = k * (T::one() - qnorm_sqd);
            qdot_matrix * q * T::from(0.5).unwrap() + q * c
            };
        
            let rates_dot = self.inertia_inverse * (dcm * world_torques + body_torques - (self.inertia * state.rates()).cross(&state.rates()) );
        
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
    /// NB: Gravity is included by default
    ///
    /// # Arguments
    /// 
    /// * `forces` - Vector of applied forces, both world and body frame
    /// * `torques` - Vector of applied torques, both world and body frame
    /// * `delta_t` - Timestep (s)
    pub fn step(&mut self, forces: &[Force<T>], torques: &[Torque<T>], delta_t: T) {
        let k1 = self.get_derivative( &self.statevector,                                           forces, torques);
        let k2 = self.get_derivative(&(self.statevector + k1 * delta_t/T::from_f64(2.0).unwrap()), forces, torques);
        let k3 = self.get_derivative(&(self.statevector + k2 * delta_t/T::from_f64(2.0).unwrap()), forces, torques);
        let k4 = self.get_derivative(&(self.statevector + k3 * delta_t),                           forces, torques);
        
        // NB: k1 is a derivative so velocity -> velocity_dot -> acceleration
        self.acceleration = k1.velocity();
        self.statevector += (k1 + k2*T::from_f64(2.0).unwrap() + k3*T::from_f64(2.0).unwrap() + k4) * delta_t/T::from_f64(6.0).unwrap();
    }
    
    /// Get body-frame acceleration at the start of the previous timestep
    /// 
    /// The resultant acceleration is in body frame and is the coordinate acceleration.
    /// To turn this into proper acceleration as seen by an accelerometer, the acceleration of the
    /// frame needs to be added, in this case standard gravity:
    /// ```
    /// # use aerso::Body;
    /// # use aerso::types::{Matrix3,Vector3,StateView};
    /// # let body = Body::new_at_origin(1.0,Matrix3::identity());
    /// let acc_frame = Body::get_dcm(&body.statevector()) * Vector3::new(0.0,0.0,-9.81);
    /// let acc_proper = body.acceleration() + acc_frame;
    /// ```
    pub fn acceleration(&self) -> Vector3<T> {
        self.acceleration
    }
    
    /// Set the body statevector
    /// 
    /// Will also reset the body acceleration to zero
    /// 
    /// The statevector is in the order: \[position,velocity(body),attitude_quaternion(i,j,k,w),axis_rates(body)\]
    pub fn set_state(&mut self, new_state: StateVector<T>) {
        self.statevector = new_state;
        self.acceleration = Vector3::zeros();
    }
    
}

/// Add state vector helpers to the Body struct
impl<T: Float> StateView<T> for Body<T> {
    fn position(&self) -> Vector3<T> {
        self.statevector.position()
    }
    
    fn velocity_in_frame(&self, frame: Frame) -> Vector3<T> {
        let body_frame_velocity = self.statevector.velocity_in_frame(Frame::Body);
        match frame {
            Frame::Body => body_frame_velocity,
            Frame::World => Body::get_dcm_body(&self.statevector) * body_frame_velocity,
        }
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

#[cfg(test)]
mod test {
    use approx::assert_relative_eq;
    use super::*;

    #[test]
    #[should_panic]
    fn test_non_invertible_inertia() {
        let inertia = Matrix3::new(
            1.0,0.0,0.0,
            0.0,1.0,0.0,
            0.0,0.0,0.0);
        Body::new_at_origin(1.0,inertia);
    }

    #[test]
    #[should_panic]
    fn test_zero_mass() {
        let inertia = Matrix3::identity();
        Body::new_at_origin(0.0,inertia);
    }

    #[test]
    #[should_panic]
    fn test_negative_mass() {
        let inertia = Matrix3::identity();
        Body::new_at_origin(-1.0,inertia);
    }

    #[test]
    fn test_get_dcm() {
        let state = StateVector::from_vec(vec![
        0.0, 0.0, 0.0,
        1.0, 2.0, 3.0,
        0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0
        ]);
        
        let dcm = Body::get_dcm(&state);
        assert_relative_eq!(dcm[0],1.0);
        assert_relative_eq!(dcm[1],0.0);
        assert_relative_eq!(dcm[2],0.0);
        
        assert_relative_eq!(dcm[3],0.0);
        assert_relative_eq!(dcm[4],1.0);
        assert_relative_eq!(dcm[5],0.0);

        assert_relative_eq!(dcm[6],0.0);
        assert_relative_eq!(dcm[7],0.0);
        assert_relative_eq!(dcm[8],1.0);

        let inertia = Matrix3::identity();
        let mut body = Body::new_from_statevector(1.0,inertia,state);

        body.step(&[],&[],0.1);

        let attitude = body.attitude();
        assert_relative_eq!(attitude.i,0.0);
        assert_relative_eq!(attitude.j,0.0);
        assert_relative_eq!(attitude.k,0.0);
        assert_relative_eq!(attitude.w,1.0);

    }

    #[test]
    fn test_derivative() {
        let inertia = Matrix3::identity();
        let body = Body::new_at_origin(1.0,inertia);
        {
            let state = StateVector::from_vec(vec![
            0.0, 0.0, 0.0,
            1.0, 2.0, 3.0,
            0.0, 0.0, 0.0, 1.0,
            0.0, 0.0, 0.0
            ]);
        
            let derivative = body.get_derivative(&state,&[],&[]);
            assert_relative_eq!(derivative[0],1.0);
            assert_relative_eq!(derivative[1],2.0);
            assert_relative_eq!(derivative[2],3.0);
            
            assert_relative_eq!(derivative[3],0.0);
            assert_relative_eq!(derivative[4],0.0);
            assert_relative_eq!(derivative[5],physical_constants::STANDARD_ACCELERATION_OF_GRAVITY);

            assert_relative_eq!(derivative[6],0.0);
            assert_relative_eq!(derivative[7],0.0);
            assert_relative_eq!(derivative[8],0.0);
            assert_relative_eq!(derivative[9],0.0);

            assert_relative_eq!(derivative[10],0.0);
            assert_relative_eq!(derivative[11],0.0);
            assert_relative_eq!(derivative[12],0.0);
        }
        {
            let state = StateVector::from_vec(vec![
            0.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
            5.0, 0.0, 0.0
            ]);
        
            let derivative = body.get_derivative(&state,&[],&[]);
            assert_relative_eq!(derivative[0],1.0);
            assert_relative_eq!(derivative[1],0.0);
            assert_relative_eq!(derivative[2],0.0);
            
            assert_relative_eq!(derivative[3],0.0);
            assert_relative_eq!(derivative[4],0.0);
            assert_relative_eq!(derivative[5],physical_constants::STANDARD_ACCELERATION_OF_GRAVITY);

            assert_relative_eq!(derivative[6],2.5);
            assert_relative_eq!(derivative[7],0.0);
            assert_relative_eq!(derivative[8],0.0);
            assert_relative_eq!(derivative[9],0.0);

            assert_relative_eq!(derivative[10],0.0);
            assert_relative_eq!(derivative[11],0.0);
            assert_relative_eq!(derivative[12],0.0);
        }

    }

    #[test]
    fn test_torque() {
        let inertia = Matrix3::identity();
        let state = StateVector::from_vec(vec![
            0.0, 0.0, 0.0,
            1.0, 2.0, 3.0,
            0.0, 0.0, 0.0, 1.0,
            0.0, 0.0, 0.0
            ]);
        let mut body = Body::new_from_statevector(1.0, inertia, state);
        
        let torque = Torque::body(1.0, 0.0, 0.0);
        
        let derivative = body.get_derivative(&state,&[],&[torque]);
        assert_relative_eq!(derivative[0],1.0);
        assert_relative_eq!(derivative[1],2.0);
        assert_relative_eq!(derivative[2],3.0);
        
        assert_relative_eq!(derivative[3],0.0);
        assert_relative_eq!(derivative[4],0.0);
        assert_relative_eq!(derivative[5],physical_constants::STANDARD_ACCELERATION_OF_GRAVITY);

        assert_relative_eq!(derivative[6],0.0);
        assert_relative_eq!(derivative[7],0.0);
        assert_relative_eq!(derivative[8],0.0);
        assert_relative_eq!(derivative[9],0.0);

        // assert_relative_eq!(derivative[10],1.0);
        assert_relative_eq!(derivative[11],0.0);
        assert_relative_eq!(derivative[12],0.0);
        
        let dt = 0.1;
        body.step(&[], &[torque], dt);
        
        let state = body.statevector();
        assert_relative_eq!(state[0],1.0*0.1,max_relative = 0.00001);
        assert_relative_eq!(state[1],2.0*0.1,max_relative = 0.00001);
        assert_relative_eq!(
            state[2],
            3.0*0.1 + 0.5*dt.powi(2)*physical_constants::STANDARD_ACCELERATION_OF_GRAVITY,
            max_relative = 0.00001);
        
        // assert_relative_eq!(state[3],1.0,max_relative = 0.00001);
        // assert_relative_eq!(state[4],2.0,max_relative = 0.00001);
        // assert_relative_eq!(
        //     state[5],
        //     3.0 + dt*physical_constants::STANDARD_ACCELERATION_OF_GRAVITY,
        //     max_relative = 0.00001);

        assert_relative_eq!(state[6],0.00249999,max_relative = 0.00001);
        assert_relative_eq!(state[7],0.0);
        assert_relative_eq!(state[8],0.0);
        assert_relative_eq!(state[9],0.99999,max_relative = 0.00001);

        assert_relative_eq!(state[10],0.1,max_relative = 0.00001);
        assert_relative_eq!(state[11],0.0,max_relative = 0.00001);
        assert_relative_eq!(state[12],0.0,max_relative = 0.00001);

    }
    
}