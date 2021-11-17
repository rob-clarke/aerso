use crate::{Vector3,Body,Force,Torque,StateView,Frame,UnitQuaternion};
use crate::types::Real;

/// Trait for general wind model
pub trait WindModel<T: Real> {
    
    /// Return the current wind at the specified position in world frame coordinates
    /// Both vectors should be in North-East-Down frame
    fn get_wind(&self, position: &Vector3<T>) -> Vector3<T>;
    
    /// Advance time of the wind model by `delta_t` seconds
    fn step(&mut self, delta_t: T);
    
}

/// Trait for general density model
pub trait DensityModel<T: Real> {

    /// Return the current density at the specified position (kg.m^-3)
    fn get_density(&self, position: &Vector3<T>) -> T;

}

pub struct StandardDensity;
impl StandardDensity {
    const ISA_STANDARD_DENSITY: f64 = 1.225;
}
impl<T: Real> DensityModel<T> for StandardDensity {
    fn get_density(&self, _position: &Vector3<T>) -> T {
        return T::from(Self::ISA_STANDARD_DENSITY);
    }
}

/// Represent generic air state
#[derive(Clone,Copy)]
pub struct AirState<T: Real> {
    /// Angle of attack (radians)
    pub alpha: T,
    /// Angle of sideslip (radians)
    pub beta: T,
    /// Airspeed (m.s^-1)
    pub airspeed: T,
    /// Dynamic pressure (Pa) (kg.m^-1.s^2)
    pub q: T,
}

/// Represent a body with aerodynamics helpers
pub struct AeroBody<T: Real, W: WindModel<T>, D: DensityModel<T>> {
    /// The underlying rigid body
    pub body: Body<T>,
    /// Optional wind model
    wind_model: W,
    /// Optional density model
    density_model: D,
}

use crate::wind_models::ConstantWind;
impl<T: Real> AeroBody<T,ConstantWind<T>,StandardDensity> {
    /// Create an AeroBody with no wind and constant ISA standard sea-level density
    pub fn new(body: Body<T>) -> Self {
        let wind_model = crate::wind_models::ConstantWind::<T>::new(Vector3::new(T::zero(),T::zero(),T::zero()));
        Self::with_wind_model(body,wind_model)
    }
}

impl<T: Real, W: WindModel<T>> AeroBody<T,W,StandardDensity> {
    /// Create an AeroBody with a WindModel and constant ISA standard sea-level density
    pub fn with_wind_model(body: Body<T>, wind_model: W) -> Self {
        let density_model = StandardDensity{};
        Self::with_density_model(body,wind_model,density_model)
    }
}

impl<T: Real, W: WindModel<T>, D: DensityModel<T>> AeroBody<T,W,D> {
    /// Create an AeroBody with a WindModel and a DensityModel
    pub fn with_density_model(body: Body<T>, wind_model: W, density_model: D) -> Self {
        Self {
            body,
            wind_model,
            density_model,
        }
    }
    
    /// Return the current airstate for the rigid body
    /// This includes the angles of attack (`alpha`) and sideslip (`beta`), the `airspeed` and the dynamic pressure, (`q`)
    pub fn get_airstate(&self) -> AirState<T> {
        
        let current_world_wind = self.wind_model.get_wind(&self.body.position());
        
        let current_body_wind = self.body.velocity() - Body::get_dcm(&self.body.statevector) * current_world_wind;
        
        let u = current_body_wind[0];
        let v = current_body_wind[1];
        let w = current_body_wind[2];
        
        let u_sqd = u.powi(2);
        let v_sqd = v.powi(2);
        let w_sqd = w.powi(2);
        
        let airspeed = ( u_sqd + v_sqd + w_sqd ).sqrt();
        
        let alpha = w.atan2(u);
        
        let beta = if airspeed != T::zero() { ( v / airspeed ).asin() } else { T::zero() };
        
        let q = T::from(0.5) * self.density_model.get_density(&self.body.position()) * airspeed.powi(2);
        
        AirState {
            alpha,
            beta,
            airspeed,
            q,
        }
    }
    
    /// Propagate the body state and wind_model by delta_t under the supplied forces and torques
    /// See the documentation for Body::step for further details
    pub fn step(&mut self, forces: &Vec<Force<T>>, torques: &Vec<Torque<T>>, delta_t: T) {
        self.wind_model.step(delta_t);
        self.body.step(forces, torques, delta_t);        
    }
}

use crate::StateVector;
impl<T: Real, W: WindModel<T>, D: DensityModel<T>> StateView<T> for AeroBody<T,W,D> {
    
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
        
        Body::new( 1.0, crate::Matrix3::identity(), initial_position, initial_velocity, initial_attitude, initial_rates)
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
        assert_relative_eq!(airstate.q,0.5*StandardDensity::ISA_STANDARD_DENSITY);
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
        assert_relative_eq!(airstate.q,0.5*StandardDensity::ISA_STANDARD_DENSITY*400.0);
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
        assert_relative_eq!(airstate.q,0.5*StandardDensity::ISA_STANDARD_DENSITY);
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
        assert_relative_eq!(airstate.q,0.5*StandardDensity::ISA_STANDARD_DENSITY);
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
        assert_relative_eq!(airstate.q,0.5*StandardDensity::ISA_STANDARD_DENSITY);
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
        assert_relative_eq!(airstate.q,0.5*StandardDensity::ISA_STANDARD_DENSITY*2.0);
        assert_relative_eq!(airstate.alpha,0.0);
        assert_relative_eq!(airstate.beta,-45.0f64.to_radians());
    }

}