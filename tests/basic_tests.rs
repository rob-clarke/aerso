#![warn(clippy::all)]

use aerso::{Body};
use aerso::types::{Vector3,Matrix3,UnitQuaternion,Force,Torque,StateVector,StateView};

use approx::assert_relative_eq;

fn run_with_constant_ft(mass: f64, forces: &[Force], torques: &[Torque]) -> (f64,StateVector) {
    let initial_position = Vector3::zeros();
    let initial_velocity = Vector3::zeros();
    let initial_attitude = UnitQuaternion::from_euler_angles(0.0,0.0,0.0);
    let initial_rates = Vector3::zeros();
    
    let mut vehicle = Body::new( mass, Matrix3::identity(), initial_position, initial_velocity, initial_attitude, initial_rates);

    let a = vehicle.attitude(); 
    println!("[{},\n{},\n{},\n{}]",a[0],a[1],a[2],a[3]);
    
    let delta_t = 0.01;
    let mut time = 0.0;
    while time < 10.0 {
        vehicle.step(forces, torques, delta_t);
        time += delta_t;
    }
    
    (time,vehicle.statevector())
}

#[test]
fn test_gravity() {
    let result = run_with_constant_ft(1.0,&[],&[]);
        
    let suvat_result = 0.5 * physical_constants::STANDARD_ACCELERATION_OF_GRAVITY * result.0.powi(2);
    assert_relative_eq!(
        result.1.position().z,
        suvat_result,
        max_relative = 0.00001);
    
    let suvat_result = physical_constants::STANDARD_ACCELERATION_OF_GRAVITY * result.0;
    assert_relative_eq!(
        result.1.velocity().z,
        suvat_result,
        max_relative = 0.00001);
}

#[test]
fn test_force() {
    const FORCE_X: f64 = 4.0;
    const MASS: f64 = 2.0;

    let thrust = Force::body(FORCE_X,0.0,0.0);
    
    let result = run_with_constant_ft(MASS,&[thrust],&[]);
    
    // x
    let suvat_result = 0.5 * FORCE_X/MASS * result.0.powi(2);
    assert_relative_eq!(result.1.position().x,suvat_result, max_relative = 0.00001);
    
    // y
    assert_relative_eq!(result.1.position().y,0.0, max_relative = 0.00001);

    // z
    let suvat_result = 0.5 * physical_constants::STANDARD_ACCELERATION_OF_GRAVITY * result.0.powi(2);
    assert_relative_eq!(result.1.position().z,suvat_result, max_relative = 0.00001);
}

#[test]
fn test_torque() {
    const TORQUE_X: f64 = 1.0;
    const MASS: f64 = 2.0;

    let roll_moment = Torque::body(TORQUE_X,0.0,0.0);
    
    let result = run_with_constant_ft(MASS,&[],&[roll_moment]);
    
    let suvat_result = 0.5 * physical_constants::STANDARD_ACCELERATION_OF_GRAVITY * result.0.powi(2);
    assert_relative_eq!(
        result.1.position()[2],
        suvat_result,
        max_relative = 0.00001);
    
    let suvat_result = TORQUE_X * result.0;
    assert_relative_eq!(
        result.1.rates().x,
        suvat_result,
        max_relative = 0.00001);
    
}