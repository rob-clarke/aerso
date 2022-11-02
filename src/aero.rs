use crate::Body;
use crate::types::{Vector3,Matrix3,UnitQuaternion,Frame,Force,Torque,StateView};

use crate::types::{Float,DefaultFloatRepr};

/// Trait for general wind model
pub trait WindModel<T: Float = DefaultFloatRepr> {
    
    /// Return the current wind at the specified position in world frame coordinates
    /// Both vectors should be in North-East-Down frame
    fn get_wind(&self, position: &Vector3<T>) -> Vector3<T>;
    
    /// Advance time of the wind model by `delta_t` seconds
    fn step(&mut self, delta_t: T);
    
}

/// Trait for general density model
pub trait DensityModel<T: Float = DefaultFloatRepr> {

    /// Return the current density at the specified position (kg.m^-3)
    fn get_density(&self, position: &Vector3<T>) -> T;

}

/// Built-in [DensityModel] for constant ISA standard sea level density
/// 
/// This model does not vary density with altitude.
pub struct ConstantDensity;
impl ConstantDensity {
    const ISA_STANDARD_DENSITY: f64 = 1.225;
}
impl<T: Float> DensityModel<T> for ConstantDensity {
    fn get_density(&self, _position: &Vector3<T>) -> T {
        T::from(Self::ISA_STANDARD_DENSITY).unwrap()
    }
}

/// Built-in [DensityModel] for ISA standard density model
/// 
/// This model is valid up to 11km altitude
/// 
/// http://www-mdp.eng.cam.ac.uk/web/library/enginfo/aerothermal_dvd_only/aero/atmos/atmos.html
pub struct StandardDensity;
impl StandardDensity {
    const T_LR: f64 = 0.0065; // K/m
    const R: f64    = 287.0;  // m^2/s^2/K
    
    const ISA_SL_T: f64  = 288.15; // K
    const ISA_11K_T: f64 = 216.65; // K
    
    const ISA_SL_H: f64  =      0.0; // m
    const ISA_11K_H: f64 = 11_000.0; // m
    
    const ISA_SL_P: f64 = 101_325.0; // Pa
    
}
impl StandardDensity {
    fn interp<T: Float>(x: T, x_0: T, x_1: T, y_0: T, y_1: T) -> T {
        y_0 + (x-x_0)/(x_1-x_0)*(y_1-y_0)
    }
    fn get_standard_temperature<T: Float>(altitude: T) -> T {
        Self::interp(
            altitude,
            T::from(Self::ISA_SL_H).unwrap(), T::from(Self::ISA_11K_H).unwrap(),
            T::from(Self::ISA_SL_T).unwrap(), T::from(Self::ISA_11K_T).unwrap()
        )
    }
    fn get_standard_pressure<T: Float>(altitude: T) -> T {
        let t_0 = T::from(Self::ISA_SL_T).unwrap();
        let p_0 = T::from(Self::ISA_SL_P).unwrap();
        let t_a = Self::get_standard_temperature(altitude);
        let lr = T::from(Self::T_LR).unwrap();
        let r_dry = T::from(Self::R).unwrap();
        let g = T::from(physical_constants::STANDARD_ACCELERATION_OF_GRAVITY).unwrap();
        
        p_0 * num_traits::Float::powf( t_a/t_0, g/(lr*r_dry) )
    }
}
impl<T: Float> DensityModel<T> for StandardDensity {
    fn get_density(&self, position: &Vector3<T>) -> T {
        let altitude = -position[2];
        let r_dry = T::from(Self::R).unwrap();
        Self::get_standard_pressure(altitude) / (r_dry * Self::get_standard_temperature(altitude))
    }
}

/// Represent generic air state
#[derive(Clone,Copy)]
pub struct AirState<T: Float = DefaultFloatRepr> {
    /// Angle of attack (radians)
    pub alpha: T,
    /// Angle of sideslip (radians)
    pub beta: T,
    /// Airspeed (m·s<sup>-1</sup>) 
    pub airspeed: T,
    /// Dynamic pressure (Pa) (kg·m<sup>-1</sup>·s<sup>2</sup>)
    pub q: T,
}

/// Represent a body in an atmosphere
#[derive(Copy,Clone)]
pub struct AeroBody<T: Float = DefaultFloatRepr, W: WindModel<T> = ConstantWind<T>, D: DensityModel<T> = StandardDensity> {
    /// The underlying rigid body
    pub body: Body<T>,
    /// Optional wind model
    wind_model: W,
    /// Optional density model
    density_model: D,
}

use crate::wind_models::ConstantWind;
impl<T: Float> AeroBody<T,ConstantWind<T>,ConstantDensity> {
    /// Create an [AeroBody] with no wind and constant ISA standard sea-level density
    /// 
    /// # Arguments
    /// 
    /// * `body` - The kinematics body to use
    pub fn new(body: Body<T>) -> Self {
        let wind_model = crate::wind_models::ConstantWind::<T>::new(Vector3::new(T::zero(),T::zero(),T::zero()));
        Self::with_wind_model(body,wind_model)
    }
}

impl<T: Float, W: WindModel<T>> AeroBody<T,W,ConstantDensity> {
    /// Create an AeroBody with a [WindModel] and constant ISA standard sea-level density
    /// 
    /// # Arguments
    /// 
    /// * `body` - The kinematics body to use
    /// * `wind_model` - The [WindModel] to use
    pub fn with_wind_model(body: Body<T>, wind_model: W) -> Self {
        let density_model = ConstantDensity{};
        Self::with_density_model(body,wind_model,density_model)
    }
}

impl<T: Float, W: WindModel<T>, D: DensityModel<T>> AeroBody<T,W,D> {
    /// Create an AeroBody with a [WindModel] and a [DensityModel]
    ///
    /// # Arguments
    /// 
    /// * `body` - The kinematics body to use
    /// * `wind_model` - The [WindModel] to use
    /// * `density_model` - The [DensityModel] to use
    pub fn with_density_model(body: Body<T>, wind_model: W, density_model: D) -> Self {
        Self {
            body,
            wind_model,
            density_model,
        }
    }
    
    /// Return an [AirState] representing the equivalent aerodynamic state for `state`
    /// 
    /// The [AirState] includes the angles of attack (`alpha`) and sideslip (`beta`), the `airspeed` and the dynamic pressure, (`q`).
    /// 
    /// It is calculated using the supplied wind and density models.
    /// 
    /// # Arguments
    /// 
    /// * `state` - The state to use to calculate the airstate
    pub fn get_airstate_from_state(&self, state: &StateVector<T>) -> AirState<T> {
        let current_world_wind = self.wind_model.get_wind(&state.position());
        let current_body_wind = state.velocity() - Body::get_dcm(state) * current_world_wind;
        
        let u = current_body_wind[0];
        let v = current_body_wind[1];
        let w = current_body_wind[2];
        
        let u_sqd = <T as num_traits::Float>::powi(u,2);
        let v_sqd = <T as num_traits::Float>::powi(v,2);
        let w_sqd = <T as num_traits::Float>::powi(w,2);
        
        let airspeed = <T as num_traits::Float>::sqrt( u_sqd + v_sqd + w_sqd );
        
        let alpha = <T as num_traits::Float>::atan2(w,u);
        
        let beta = if airspeed != T::zero() { <T as num_traits::Float>::asin( v / airspeed ) } else { T::zero() };
        
        let q = T::from(0.5).unwrap() * self.density_model.get_density(&state.position()) * <T as num_traits::Float>::powi(airspeed,2);
        
        AirState {
            alpha,
            beta,
            airspeed,
            q,
        }
    }
    
    /// Return an [AirState] representing the current aerodynamic state of the body
    /// 
    /// The [AirState] includes the angles of attack (`alpha`) and sideslip (`beta`), the `airspeed` and the dynamic pressure, (`q`).
    /// 
    /// It is calculated using the supplied wind and density models.
    pub fn get_airstate(&self) -> AirState<T> {
        self.get_airstate_from_state(&self.statevector())
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
        self.body.get_derivative(state, forces, torques)
    }
    
    /// Propagate the body state and wind_model by `delta_t` under the supplied `forces` and `torques`
    /// 
    /// See the documentation for [Body::step] for further details
    pub fn step(&mut self, forces: &[Force<T>], torques: &[Torque<T>], delta_t: T) {
        self.body.step(forces, torques, delta_t);        
        self.wind_model.step(delta_t);
    }
    
    /// Get body-frame acceleration at the start of the previous timestep
    /// 
    /// See [Body::acceleration] for more details
    pub fn acceleration(&self) -> Vector3<T> {
        self.body.acceleration()
    }
    
    /// Set the statevector for the underlying [Body]
    /// 
    /// The statevector is in the order: \[position,velocity(body),attitude_quaternion(i,j,k,w),axis_rates(body)\]
    pub fn set_state(&mut self, new_state: StateVector<T>) {
        self.body.set_state(new_state);
    }
}

use crate::types::StateVector;
impl<W: WindModel<T>, D: DensityModel<T>, T: Float> StateView<T> for AeroBody<T,W,D> {
    
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

mod test {
    
    use super::*;

    extern crate rstest;
    use rstest::{fixture,rstest};

    #[fixture]
    fn body() -> Body<f64> {
        let initial_position = Vector3::zeros();
        let initial_velocity = Vector3::zeros();
        let initial_attitude = UnitQuaternion::from_euler_angles(0.0,0.0,0.0);
        let initial_rates = Vector3::zeros();
        
        Body::new( 1.0, Matrix3::identity(), initial_position, initial_velocity, initial_attitude, initial_rates)
    }

    #[rstest]
    fn test_zero(body: Body<f64>) {
        use approx::assert_relative_eq;
        
        let wind = Vector3::new(0.0,0.0,0.0);
        
        let wind_model = ConstantWind::new(wind);
        let vehicle = AeroBody::with_wind_model(body,wind_model);
        
        let airstate = vehicle.get_airstate();
        
        assert_relative_eq!(airstate.airspeed,0.0);
        assert_relative_eq!(airstate.q,0.0);
        assert_relative_eq!(airstate.alpha,0.0);
        assert_relative_eq!(airstate.beta,0.0);
    }

    #[rstest]
    fn test_headwind(body: Body<f64>) {
        use approx::assert_relative_eq;
        
        let wind = Vector3::new(-1.0,0.0,0.0);
        let wind_model = ConstantWind::new(wind);
        let vehicle = AeroBody::with_wind_model(body,wind_model);
        
        let airstate = vehicle.get_airstate();
        
        assert_relative_eq!(airstate.airspeed,1.0);
        assert_relative_eq!(airstate.q,0.5*ConstantDensity::ISA_STANDARD_DENSITY);
        assert_relative_eq!(airstate.alpha,0.0);
        assert_relative_eq!(airstate.beta,0.0);
    }

    #[rstest]
    fn test_highwind(body: Body<f64>) {
        use approx::assert_relative_eq;
        
        let wind = Vector3::new(-20.0,0.0,0.0);
        let wind_model = ConstantWind::new(wind);
        let vehicle = AeroBody::with_wind_model(body,wind_model);
        
        let airstate = vehicle.get_airstate();
        
        assert_relative_eq!(airstate.airspeed,20.0);
        assert_relative_eq!(airstate.q,0.5*ConstantDensity::ISA_STANDARD_DENSITY*400.0);
        assert_relative_eq!(airstate.alpha,0.0);
        assert_relative_eq!(airstate.beta,0.0);
    }

    #[rstest]
    fn test_tailwind(body: Body<f64>) {
        use approx::assert_relative_eq;
        
        let wind = Vector3::new(1.0,0.0,0.0);
        let wind_model = ConstantWind::new(wind);
        let vehicle = AeroBody::with_wind_model(body,wind_model);
        
        let airstate = vehicle.get_airstate();
        
        assert_relative_eq!(airstate.airspeed,1.0);
        assert_relative_eq!(airstate.q,0.5*ConstantDensity::ISA_STANDARD_DENSITY);
        assert_relative_eq!(airstate.alpha,180.0f64.to_radians());
        assert_relative_eq!(airstate.beta,0.0);
    }

    #[rstest]
    fn test_updraft(body: Body<f64>) {
        use approx::assert_relative_eq;
        
        let wind = Vector3::new(0.0,0.0,-1.0);
        let wind_model = ConstantWind::new(wind);
        let vehicle = AeroBody::with_wind_model(body,wind_model);
        
        let airstate = vehicle.get_airstate();
        
        assert_relative_eq!(airstate.airspeed,1.0);
        assert_relative_eq!(airstate.q,0.5*ConstantDensity::ISA_STANDARD_DENSITY);
        assert_relative_eq!(airstate.alpha,90.0f64.to_radians());
        assert_relative_eq!(airstate.beta,0.0);
    }

    #[rstest]
    fn test_crosswind(body: Body<f64>) {
        use approx::assert_relative_eq;
        
        let wind = Vector3::new(0.0,-1.0,0.0);
        let wind_model = ConstantWind::new(wind);
        let vehicle = AeroBody::with_wind_model(body,wind_model);
        
        let airstate = vehicle.get_airstate();
        
        assert_relative_eq!(airstate.airspeed,1.0);
        assert_relative_eq!(airstate.q,0.5*ConstantDensity::ISA_STANDARD_DENSITY);
        assert_relative_eq!(airstate.alpha,0.0);
        assert_relative_eq!(airstate.beta,90.0f64.to_radians());
    }

    #[rstest]
    fn test_sideslip(body: Body<f64>) {
        use approx::assert_relative_eq;
        
        let wind = Vector3::new(-1.0,1.0,0.0);
        let wind_model = ConstantWind::new(wind);
        let vehicle = AeroBody::with_wind_model(body,wind_model);
        
        let airstate = vehicle.get_airstate();
        
        assert_relative_eq!(airstate.airspeed,2.0f64.sqrt());
        assert_relative_eq!(airstate.q,0.5*ConstantDensity::ISA_STANDARD_DENSITY*2.0);
        assert_relative_eq!(airstate.alpha,0.0);
        assert_relative_eq!(airstate.beta,-45.0f64.to_radians());
    }

}