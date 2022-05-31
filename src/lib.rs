#![warn(clippy::all)]

pub mod types;

mod kinematics;
mod aero;

mod effectors;

pub use kinematics::Body;
pub use aero::{AeroBody,WindModel,DensityModel,AirState};
pub use effectors::{AeroEffect,AffectedBody};

pub mod wind_models;
pub mod density_models {
    pub use crate::aero::ConstantDensity;
    pub use crate::aero::StandardDensity;
}
