// src/shared_data/mod.rs
use nalgebra::{SVector, Vector2, Vector3, Vector6};
use serde::{Deserialize, Serialize};

// =====================
// Custom Types
// =====================
pub type Vector12<T> = SVector<T, 12>;

// =====================
// Topics and Port Mapping
// =====================
#[derive(Debug, Clone)]
pub struct TopicConfig<T> {
    pub port: u16,
    pub datatype: std::marker::PhantomData<T>,
}

// =====================
// Topic Definitions
// =====================
#[allow(non_snake_case)]
pub mod TOPICS {
    use super::*;

    // Data Type [Vector6<f32>]:
    // [Fx, Fy, Fz]
    // [Torque in roll, pitch, yaw]
    pub mod forces_thrusters {
        use super::*;
        use Vector6;
        pub const PORT: u16 = 5550;
        pub type DataType = Vector6<f32>;
    }

    // Data Type [Vector3<f32>]:
    // [velocity [m/s], angle [°], noise [%]]
    pub mod wind_parameters {
        use super::*;
        use Vector3;
        pub const PORT: u16 = 5551;
        pub type DataType = Vector3<f32>;
    }

    // Data Type [Vector3<f32>]:
    // [velocity [m/s], angle [°], noise [%]]
    pub mod current_parameters {
        use super::*;
        use Vector3;
        pub const PORT: u16 = 5552;
        pub type DataType = Vector3<f32>;
    }

    // Data Type [Vector3<f32>]:
    // [vx, vy, vz]
    pub mod wind_speed {
        use super::*;
        use Vector3;
        pub const PORT: u16 = 5560;
        pub type DataType = Vector3<f32>;
    }

    // Data Type [Vector3<f32>]:
    // [vx, vy, vz]
    pub mod current_speed {
        use super::*;
        use Vector3;
        pub const PORT: u16 = 5561;
        pub type DataType = Vector3<f32>;
    }

    // Data Type [Vector12<f32>]:
    // [vx, vy, vz]
    // [angular velocity in roll, pitch, yaw]
    // [x, y, z]
    // [roll, pitch, yaw]
    pub mod x {
        use super::*;
        use Vector12;
        pub const PORT: u16 = 5570;
        pub type DataType = Vector12<f32>;
    }

    // Data Type [Vector12<f32>]:
    // [ax, ay, az]
    // [angular acceleration in roll, pitch, yaw]
    // [vx, vy, vz]
    // [angular velocity in roll, pitch, yaw]
    pub mod dx {
        use super::*;
        use Vector12;
        pub const PORT: u16 = 5571;
        pub type DataType = Vector12<f32>;
    }

    // Data Type [Vector<f32>]:
    // heading speed [m/s]
    // yaw speed [°/s]
    pub mod speed {
        use super::*;
        use Vector2;
        pub const PORT: u16 = 5572;
        pub type DataType = Vector2<f32>;
    }

    // Data Type [Vector3<f32>]:
    // antenna world frame: [x, y]
    pub mod gnss_antenna1 {
        use super::*;
        use Vector2;
        pub const PORT: u16 = 5580;
        pub type DataType = Vector2<f32>;
    }

    // Data Type [Vector3<f32>]:
    // antenna world frame: [x, y]
    pub mod gnss_antenna2 {
        use super::*;
        use Vector2;
        pub const PORT: u16 = 5581;
        pub type DataType = Vector2<f32>;
    }
}

// =====================
// Utilities
// =====================
pub fn encode_json<T: Serialize>(data: &T) -> String {
    serde_json::to_string(data).expect("Failed to serialize")
}

pub fn decode_json<'a, T: Deserialize<'a>>(json: &'a str) -> T {
    serde_json::from_str(json).expect("Failed to deserialize")
}
