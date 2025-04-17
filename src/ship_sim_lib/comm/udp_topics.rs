// src/shared_data/mod.rs
use nalgebra::{SVector, Vector2, Vector3, Vector6};
use serde::{Deserialize, Serialize};

// =====================
// Custom Types
// =====================
pub type Vector7<T> = SVector<T, 7>;
pub type Vector9<T> = SVector<T, 9>;
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

    // Data Type [Vector9<f32>]:
    // antenna 1 position (World Frame): [x, y, z]
    // antenna 2 position (World Frame): [x, y, z]
    // GNSS velocity linear (World Frame): [vx, vy, vz]
    pub mod gnss {
        use super::*;
        use Vector9;
        pub const PORT: u16 = 5580;
        pub type DataType = Vector9<f32>;
    }

    // Data Type [Vector7<f32>]:
    // Linear acceleration (Body Frame): [x, y, z]
    // Angular velocity gyro (Body Frame): [roll, pitch, yaw]
    // Angle magnetic compass (World Frame): [yaw]
    pub mod imu {
        use super::*;
        use Vector7;
        pub const PORT: u16 = 5581;
        pub type DataType = Vector7<f32>;
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
