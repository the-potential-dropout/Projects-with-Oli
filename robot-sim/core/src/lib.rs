//! Robot Sim core: the parts of the simulator that run every frame for every
//! robot and benefit from being fast and allocation-free.
//!
//! * `ik`      – 3-joint leg inverse kinematics (coxa / femur / tibia)
//! * `gait`    – tripod walking gait for a hexapod (foot trajectories + IK)
//! * `world`   – projectile physics, collisions, ray casts and sensor scans
//!
//! Units: metres, seconds, radians. Coordinate frame matches three.js:
//! +Y up, robots face +Z when yaw = 0.

pub mod gait;
pub mod ik;
pub mod world;

pub use gait::HexapodGait;
pub use world::World;
