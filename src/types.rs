extern crate nalgebra as na;

#[cfg(not(feature="single-precision"))]
/// Default float representation used in the library
/// 
/// Used when "single-precision" feature is not enabled
pub(crate) type DefaultFloatRepr = f64;

#[cfg(feature="single-precision")]
/// Default float representation used in the library
/// 
/// Used when "single-precision" feature is enabled
pub(crate) type DefaultFloatRepr = f32;

/// Alias for 3D vector representation
pub type Vector3<T = DefaultFloatRepr> = na::Vector3<T>;

/// Alias for 3x3 matrix representation
pub type Matrix3<T = DefaultFloatRepr> = na::Matrix3<T>;

/// Alias for unit quaternion representation
pub type UnitQuaternion<T = DefaultFloatRepr> = na::UnitQuaternion<T>;

/// Alias for 13-dimensional state vector representation
pub type StateVector<T = DefaultFloatRepr> = na::SVector<T,13>;

/// Trait to cover both single (`f32`) and double (`f64`) precision generic types
pub trait Float: num_traits::Float + num_traits::FromPrimitive + na::RealField {}
impl Float for f32 {}
impl Float for f64 {}

/// Represent reference frame of quantities
#[derive(Copy,Clone)]
pub enum Frame {
    /// Associated quantity is in world frame
    World,
    /// Associated quantity is in body frame
    Body
}

/// Represent a force in either body or world reference frame
#[derive(Copy,Clone)]
pub struct Force<T: Float = DefaultFloatRepr> {
    /// Vector representing the force
    pub force: Vector3<T>,
    /// Frame of the vector
    pub frame: Frame,
}

impl<T: Float> Force<T> {
    /// Create new force with components `x`,`y`,`z`, in `frame`
    fn new(x: T, y: T, z: T, frame: Frame) -> Self {
        Force {
            force: Vector3::new(x,y,z),
            frame
        }
    }
    
    /// Create a world-referenced force with components `x`,`y`,`z`
    pub fn world(x: T, y: T, z: T) -> Self {
        Force::new(x,y,z,Frame::World)
    }
    
    /// Create a world-referenced force with vec
    pub fn world_vec(vec: Vector3<T>) -> Self {
        Force::world(vec[0],vec[1],vec[2])
    }
    
    /// Create a body-referenced force with components `x`,`y`,`z`
    pub fn body(x: T,y: T, z: T) -> Self {
        Force::new(x,y,z,Frame::Body)
    }
    
    /// Create a body-referenced force with vec
    pub fn body_vec(vec: Vector3<T>) -> Self {
        Force::body(vec[0],vec[1],vec[2])
    }
    
}

/// Represent a torque in either body or world reference frame
#[derive(Copy,Clone)]
pub struct Torque<T: Float = DefaultFloatRepr> {
    /// Vector representing the torque
    pub torque: Vector3<T>,
    /// Frame of the torque
    pub frame: Frame,
}

impl<T: Float> Torque<T> {
    /// Create new torque with components `x`,`y`,`z`, in `frame`
    fn new(x: T, y: T, z: T, frame: Frame) -> Self {
        Torque {
            torque: Vector3::new(x,y,z),
            frame
        }
    }
    
    /// Create a world-referenced torque with components `x`,`y`,`z`
    pub fn world(x: T,y: T, z: T) -> Self {
        Torque::new(x,y,z,Frame::World)
    }
    
    /// Create a world-referenced torque with vec
    pub fn world_vec(vec: Vector3<T>) -> Self {
        Torque::world(vec[0],vec[1],vec[2])
    }
    
    /// Create a body-referenced torque with components `x`,`y`,`z`
    pub fn body(x: T,y: T, z: T) -> Self {
        Torque::new(x,y,z,Frame::Body)
    }
    
    /// Create a body-referenced torque with vec
    pub fn body_vec(vec: Vector3<T>) -> Self {
        Torque::body(vec[0],vec[1],vec[2])
    }
}

/// Trait to allow more meaningful access to the state vector
pub trait StateView<T: Float = DefaultFloatRepr> {
    
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
