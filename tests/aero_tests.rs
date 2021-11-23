#![warn(clippy::all)]

use aerso::{Body,AeroBody};
use aerso::types::{Vector3,Matrix3,UnitQuaternion,Force,StateVector,StateView};

use aerso::wind_models::ConstantWind;

use approx::assert_relative_eq;

struct SimResult {
    time: f64,
    statevector: StateVector<f64>,
}

fn run_constant_force(forces: &[Force]) -> SimResult {
    let initial_position = Vector3::zeros();
    let initial_velocity = Vector3::zeros();
    let initial_attitude = UnitQuaternion::from_euler_angles(0.0,0.0,0.0);
    let initial_rates = Vector3::zeros();
    
    let body = Body::new( 1.0, Matrix3::identity(), initial_position, initial_velocity, initial_attitude, initial_rates);

    let constant_wind = ConstantWind::new(Vector3::new(0.0,0.0,0.0));
    
    let mut vehicle = AeroBody::with_wind_model(body,constant_wind);
    
    let a = vehicle.attitude(); 
    println!("[{},\n{},\n{},\n{}]",a[0],a[1],a[2],a[3]);
    
    let torques = vec![];
    
    let delta_t = 0.1;
    let mut time = 0.0;
    while time < 10.0 {
        vehicle.step(forces, &torques, delta_t);
        time += delta_t;
    }
    
    SimResult {
        time,
        statevector: vehicle.body.statevector(),
    }
}

#[test]
fn test_gravity() {
    let result = run_constant_force(&[]);
        
    let suvat_result = 0.5 * 9.80665 * result.time.powi(2);
    assert_relative_eq!(
        result.statevector.position()[2],
        suvat_result,
        max_relative = 0.00001);
}

#[test]
fn test_force() {
    let thrust = Force::body(4.0,0.0,0.0);
    
    let result = run_constant_force(&[thrust]);
    
    let suvat_result = 0.5 * 4.0/1.0 * result.time.powi(2);
    assert_relative_eq!(
        result.statevector.position()[0],
        suvat_result,
        max_relative = 0.00001);
}

#[test]
fn test_alpha() {
    
}

// #[feature(test)]
// mod bench {
//     extern crate test;
//     use test::Bencher;
    
//     use super::*;
    
//     #[bench]
//     fn force_bench(b: &mut Bencher) {
//         let initial_position = Vector3::zeros();
//         let initial_velocity = Vector3::zeros();
//         let initial_attitude = UnitQuaternion::from_euler_angles(0.0,0.0,0.0);
//         let initial_rates = Vector3::zeros();
        
//         let body = Body::new( 1.0, Matrix3::identity(), initial_position, initial_velocity, initial_attitude, initial_rates);
    
//         let constant_wind = ConstantWind::new(Vector3::new(0.0,0.0,0.0));
        
//         let mut vehicle = AeroBody::with_wind_model(body,constant_wind);
        
//         let a = vehicle.attitude(); 
//         println!("[{},\n{},\n{},\n{}]",a[0],a[1],a[2],a[3]);
        
//         let thrust = Force::body(4.0,0.0,0.0);
//         let forces = vec![thrust];
//         let torques = vec![];
    
//         let delta_t = 0.1;
        
//         b.iter(|| {
//             let mut time = 0.0;
//             while time < 10.0 {
//                 vehicle.step(forces, &torques, delta_t);
//                 time += delta_t;
//                 }
//             vehicle.statevector()
//         })
//     }
// }