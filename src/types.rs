extern crate nalgebra as na;

pub type Vector3<T> = na::Vector3<T>;
pub type Matrix3<T> = na::Matrix3<T>;
pub type UnitQuaternion<T> = na::UnitQuaternion<T>;

pub type StateVector<T> = na::SVector<T,13>;

pub trait Float: num_traits::Float + num_traits::FromPrimitive + na::RealField {}
impl Float for f32 {}
impl Float for f64 {}

/// Represent reference frame of quantities
#[derive(Copy,Clone)]
pub enum Frame {
    World,
    Body
}

/// Represent a force in either body or world reference frame
#[derive(Copy,Clone)]
pub struct Force<T: Float> {
    pub force: Vector3<T>,
    pub frame: Frame,
}

impl<T: Float> Force<T> {
    fn new(x: T, y: T, z: T, frame: Frame) -> Self {
        Force {
            force: Vector3::new(x,y,z),
            frame
        }
    }
    
    /// Create a world-referenced force
    pub fn world(x: T, y: T, z: T) -> Self {
        Force::new(x,y,z,Frame::World)
    }
    
    /// Create a body-referenced force
    pub fn body(x: T,y: T, z: T) -> Self {
        Force::new(x,y,z,Frame::Body)
    }
}

/// Represent a torque in either body or world reference frame
#[derive(Copy,Clone)]
pub struct Torque<T: Float> {
    pub torque: Vector3<T>,
    pub frame: Frame,
}

impl<T: Float> Torque<T> {
    fn new(x: T, y: T, z: T, frame: Frame) -> Self {
        Torque {
            torque: Vector3::new(x,y,z),
            frame
        }
    }
    
    /// Create a world-referenced torque
    pub fn world(x: T,y: T, z: T) -> Self {
        Torque::new(x,y,z,Frame::World)
    }
    
    /// Create a body-referenced force
    pub fn body(x: T,y: T, z: T) -> Self {
        Torque::new(x,y,z,Frame::Body)
    }
}

/// Trait to allow more meaningful access to the state vector
pub trait StateView<T: Float> {
    
    /// Return the world-frame position
    fn position(&self) -> Vector3<T>;
    
    /// Return the velocity in the specified frame
    fn velocity_in_frame(&self, frame: Frame) -> Vector3<T>;
    
    /// Return the body-frame velocity
    fn velocity(&self) -> Vector3<T> {
        self.velocity_in_frame(Frame::Body)
    }
    
    /// Return the attitude quaternion
    fn attitude(&self) -> UnitQuaternion<T>;
    
    /// Return the axis rates in the specified frame
    fn rates_in_frame(&self, frame: Frame) -> Vector3<T>;
    
    /// Return the body-frame axis rates
    fn rates(&self) -> Vector3<T> {
        self.rates_in_frame(Frame::Body)
    }
    
    /// Return the entire statevector
    fn statevector(&self) -> StateVector<T>;
    
}

impl<T: Float> StateView<T> for StateVector<T> {
    fn position(&self) -> Vector3<T> {
        self.fixed_rows::<3>(0).into()
    }
    
    fn velocity_in_frame(&self, frame: Frame) -> Vector3<T> {
        match frame {
            Frame::Body => self.fixed_rows::<3>(3).into(),
            _ => unimplemented!(),
        }
    }
    
    fn attitude(&self) -> UnitQuaternion<T> {
        let elements: na::Vector4<T> = self.fixed_rows::<4>(6).into();
        UnitQuaternion::from_quaternion(elements.into())
        }
    
    fn rates_in_frame(&self, frame: Frame) -> Vector3<T> {
        match frame {
            Frame::Body => self.fixed_rows::<3>(10).into(),
            _ => unimplemented!(),
        }
    }
    
    fn statevector(&self) -> StateVector<T> {
        *self
    }
    
}
