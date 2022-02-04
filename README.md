# `aerso`

`aerso` is a Rust library for kinematics with helpers for aerodynamic forces.

The core kinematics code is in [`src/kinematics.rs`](./blob/master/src/kinematics.rs). The integration is performed using
a 4th order Runge-Kutta process. At present, a fixed timestep is used.

The kinematics code is then wrapped in the `AeroBody` interface which adds options for wind and density models to
enable modelling the atmosphere.

The final layer of helpers comes in the `AffectedBody` interface, which adds the option to attach `AeroEffect`s to the
body. These provide a wrapper for forces and torques that are generated by a combination of the vehicle airstate,
body-axis rates and a generic input state.

## Example

> The full code for the example can be found in [`examples/demo.rs`](./blob/master/examples/demo.rs).

The first step is to create a `Body` which represents the base kinematic properties:

```rs
let initial_position = Vector3::zeros();
let initial_velocity = Vector3::zeros();
let initial_attitude = UnitQuaternion::from_euler_angles(0.0,0.0,0.0);
let initial_rates = Vector3::zeros();

let k_body = Body::new(
        1.0,                 // Mass (kg)
        Matrix3::identity(), // Inertia matrix/tensor
        initial_position,
        initial_velocity,
        initial_attitude,
        initial_rates
        );
```

This is then wrapped in an `AeroBody`, where wind and density models can be attached:

```rs
let wind = Vector3::new(0.0,-1.0,0.0);
let wind_model = ConstantWind::new(wind);

let a_body = AeroBody::with_wind_model(k_body,wind_model);
```

Next any `AeroEffect`s can be defined:

```rs
struct Lift;
impl<I> AeroEffect<I> for Lift {
    fn get_effect(&self, airstate: AirState, _rates: Vector3, _inputstate: &I) -> (Force,Torque) {
        const S: f64 = 0.5;
        const AR: f64 = 5.0;
        
        const C_L_ALPHA: f64 = 2.0*std::f64::consts::PI;
        const C_L0: f64 = 0.1;
        
        let c_l = C_L_ALPHA * (AR/(AR+2.0)) * airstate.alpha + C_L0;
        let lift = airstate.q * S * c_l;
        
        (Force::body(0.0,0.0,-lift),Torque::body(0.0,0.0,0.0))
    }
}

```

Finally, the effects are attached via the `AffectedBody`:

```rs
let mut vehicle = AffectedBody {
    body: a_body,
    effectors: vec![Box::new(Lift)],
    };
```

Then propogating the system is just calling `step`:

```rs
let delta_t = 0.01;
let mut time = 0.0;
while time < 100.0 {
    vehicle.step(delta_t, &[200.0]);
    }
```
