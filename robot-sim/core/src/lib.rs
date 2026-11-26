//! Robot Sim core: the parts of the simulator that run every frame for every
//! robot and benefit from being fast and allocation-free.
//!
//! * `ik`      – 3-joint leg inverse kinematics (coxa / femur / tibia)
//! * `gait`    – tripod / ripple / wave gaits, body pose, terrain adaptation, balance PID
//!               (mirrors firmware/hexapod_controller/kinematics.cpp)
//! * `terrain` – uneven ground and where the body comes to rest on it
//! * `world`   – projectile physics, collisions, ray casts and sensor scans
//!
//! Units: metres, seconds, radians. Coordinate frame matches three.js:
//! +Y up, robots face +Z when yaw = 0.

pub mod gait;
pub mod ik;
pub mod terrain;
pub mod world;

pub use gait::{Balance, HexapodGait};
pub use terrain::Terrain;
pub use world::World;
