extern crate nalgebra as na;

#[cfg(not(feature="single-precision"))]
pub type Real = f64;

#[cfg(feature="single-precision")]
pub type Real = f32;

pub type Vector3 = na::Vector3<Real>;
pub type Matrix3 = na::Matrix3<Real>;
pub type UnitQuaternion = na::UnitQuaternion<Real>;

pub type StateVector = na::SVector<Real,13>;

/// Represent reference frame of quantities
#[derive(Copy,Clone)]
pub enum Frame {
    World,
    Body
}

/// Represent a force in either body or world reference frame
#[derive(Copy,Clone)]
pub struct Force {
    pub force: Vector3,
    pub frame: Frame,
}

impl Force {
    fn new(x: Real, y: Real, z: Real, frame: Frame) -> Force {
        Force {
            force: Vector3::new(x,y,z),
            frame
        }
    }
    
    /// Create a world-referenced force
    pub fn world(x: Real,y: Real, z: Real) -> Force {
        Force::new(x,y,z,Frame::World)
    }
    
    /// Create a body-referenced force
    pub fn body(x: Real,y: Real, z: Real) -> Force {
        Force::new(x,y,z,Frame::Body)
    }
}

/// Represent a torque in either body or world reference frame
#[derive(Copy,Clone)]
pub struct Torque {
    pub torque: Vector3,
    pub frame: Frame,
}

impl Torque {
    fn new(x: Real, y: Real, z: Real, frame: Frame) -> Torque {
        Torque {
            torque: Vector3::new(x,y,z),
            frame
        }
    }
    
    /// Create a world-referenced torque
    pub fn world(x: Real,y: Real, z: Real) -> Torque {
        Torque::new(x,y,z,Frame::World)
    }
    
    /// Create a body-referenced force
    pub fn body(x: Real,y: Real, z: Real) -> Torque {
        Torque::new(x,y,z,Frame::Body)
    }
}

/// Trait to allow more meaningful access to the state vector
pub trait StateView {
    
    /// Return the world-frame position
    fn position(&self) -> Vector3;
    
    /// Return the velocity in the specified frame
    fn velocity_in_frame(&self, frame: Frame) -> Vector3;
    
    /// Return the body-frame velocity
    fn velocity(&self) -> Vector3 {
        self.velocity_in_frame(Frame::Body)
    }
    
    /// Return the attitude quaternion
    fn attitude(&self) -> UnitQuaternion;
    
    /// Return the axis rates in the specified frame
    fn rates_in_frame(&self, frame: Frame) -> Vector3;
    
    /// Return the body-frame axis rates
    fn rates(&self) -> Vector3 {
        self.rates_in_frame(Frame::Body)
    }
    
    /// Return the entire statevector
    fn statevector(&self) -> StateVector;
    
}

impl StateView for StateVector {
    fn position(&self) -> Vector3 {
        self.fixed_rows::<3>(0).into()
    }
    
    fn velocity_in_frame(&self, frame: Frame) -> Vector3 {
        match frame {
            Frame::Body => self.fixed_rows::<3>(3).into(),
            _ => unimplemented!(),
        }
    }
    
    fn attitude(&self) -> UnitQuaternion {
        let elements: na::Vector4<Real> = self.fixed_rows::<4>(6).into();
        UnitQuaternion::from_quaternion(elements.into())
        }
    
    fn rates_in_frame(&self, frame: Frame) -> Vector3 {
        match frame {
            Frame::Body => self.fixed_rows::<3>(10).into(),
            _ => unimplemented!(),
        }
    }
    
    fn statevector(&self) -> StateVector {
        *self
    }
    
}
