use aerso::{Real,Vector3,Matrix3,UnitQuaternion,Body,Force,StateVector,StateView};

use approx::assert_relative_eq;

struct SimResult {
    time: Real,
    statevector: StateVector,
}

fn run_constant_force(mass: Real, forces: &Vec<Force>) -> SimResult {
    let initial_position = Vector3::zeros();
    let initial_velocity = Vector3::zeros();
    let initial_attitude = UnitQuaternion::from_euler_angles(0.0,0.0,0.0);
    let initial_rates = Vector3::zeros();
    
    let mut vehicle = Body::new( mass, Matrix3::identity(), initial_position, initial_velocity, initial_attitude, initial_rates);

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
        statevector: vehicle.statevector,
    }
}

#[test]
fn test_gravity() {
    let forces = vec![];
    let result = run_constant_force(1.0,&forces);
        
    let suvat_result = 0.5 * 9.80665 * result.time.powi(2);
    assert_relative_eq!(
        result.statevector.position()[2],
        suvat_result,
        max_relative = 0.00001);
}

#[test]
fn test_force() {
    const FORCE_X: Real = 4.0;
    const MASS: Real = 2.0;

    let thrust = Force::body(FORCE_X,0.0,0.0);
    let forces = vec![thrust];
    
    let result = run_constant_force(MASS,&forces);
    

    let suvat_result = 0.5 * FORCE_X/MASS * result.time.powi(2);
    assert_relative_eq!(
        result.statevector.position()[0],
        suvat_result,
        max_relative = 0.00001);
}
