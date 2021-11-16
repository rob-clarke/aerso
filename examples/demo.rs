extern crate aerso;

fn main() {
    
    use aerso::*;
    
    const S: Real = 0.5;
    const AR: Real = 5.0;
    
    struct Lift;
    impl<T: Copy> AeroEffect<T> for Lift {
        fn get_effect(&self, airstate: AirState, _rates: Vector3, _inputstate: T) -> (Force,Torque) {
            const C_L_ALPHA: Real = 2.0*std::f64::consts::PI as Real;
            const C_L0: Real = 0.1;
            
            let c_l = C_L_ALPHA * (AR/(AR+2.0)) * airstate.alpha + C_L0;
            let lift = airstate.q * S * c_l;
            
            (Force::body(0.0,0.0,-lift),Torque::body(0.0,0.0,0.0))
        }
    }
    
    struct Thrust;
    impl AeroEffect<[Real;1]> for Thrust {
        fn get_effect(&self, _airstate: AirState, _rates: Vector3, inputstate: [Real;1]) -> (Force,Torque) {
            let power = inputstate[0];
            let thrust = -0.0000830488*power.powi(2) + 0.0704307060*power + 0.5996810096;
            (Force::body(thrust,0.0,0.0),Torque::body(0.0,0.0,0.0))
        }
    }
    
    struct Drag;
    impl Drag {
        fn get_cl(&self, airstate: AirState) -> Real {
            const C_L_ALPHA: Real = 2.0*std::f64::consts::PI as Real;
            const C_L0: Real = 0.1;
            
            let c_l = C_L_ALPHA * (AR/(AR+2.0)) * airstate.alpha + C_L0;
            return c_l;
        }
    }
    impl<T: Copy> AeroEffect<T> for Drag {
        fn get_effect(&self, airstate: AirState, _rates: Vector3, _inputstate: T) -> (Force,Torque) {
            const C_D_MIN: Real = 0.06;
            let c_l = self.get_cl(airstate);
            
            let c_d_i = c_l.powi(2) / (std::f64::consts::PI as Real * AR);
            let c_d = C_D_MIN + c_d_i;
            
            let drag = airstate.q * S * c_d;
            (Force::body(-drag,0.0,0.0),Torque::body(0.0,0.0,0.0))
        }
    }
    
    let initial_position = Vector3::zeros();
    let initial_velocity = Vector3::zeros();
    let initial_attitude = UnitQuaternion::from_euler_angles(0.0,0.0,0.0);
    let initial_rates = Vector3::zeros();
    
    let k_body = Body::new( 1.0, Matrix3::identity(), initial_position, initial_velocity, initial_attitude, initial_rates);

    let a_body = AeroBody::new(k_body);
    
    let mut vehicle = AffectedBody {
        body: a_body,
        effectors: vec![Box::new(Lift),Box::new(Drag),Box::new(Thrust)],
        };
    
    let delta_t = 0.01;
    let mut time = 0.0;
    while time < 100.0 {
        vehicle.step(delta_t, [200.0]);
        time += delta_t;
        println!("{}",vehicle.position());
        let airstate = vehicle.body.get_airstate();
        println!("A: {}, B: {}, V: {}, Q: {}",airstate.alpha,airstate.beta,airstate.airspeed,airstate.q);
    }
        
}