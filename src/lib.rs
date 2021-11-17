mod types;

mod kinematics;
mod aero;

mod effectors;

pub use types::{
    Vector3,Matrix3,UnitQuaternion,
    StateVector,
    Frame,
    Force,Torque,
    StateView
    };
pub use kinematics::Body;
pub use aero::{AeroBody,WindModel,DensityModel,AirState};
pub use effectors::{AeroEffect,AffectedBody};

pub mod wind_models;
