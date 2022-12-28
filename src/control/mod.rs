//! Utilities for controlling the trajectories of objects in a non-physical way.

pub use self::character_controller::{
    CharacterAutostep, CharacterCollision, CharacterLength, EffectiveCharacterMovement,
    KinematicCharacterController,
};

pub use self::basic_character_controller::BasicKinematicCharacterController;

#[cfg(feature = "dim3")]
pub use self::ray_cast_vehicle_controller::{DynamicRayCastVehicleController, Wheel, WheelTuning};

mod basic_character_controller;
mod character_controller;

#[cfg(feature = "dim3")]
mod ray_cast_vehicle_controller;
