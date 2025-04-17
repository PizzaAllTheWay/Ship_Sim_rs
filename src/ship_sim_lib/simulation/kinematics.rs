use nalgebra::{Vector3, Matrix3};

/// Rotation around X-axis (roll)
pub fn r_x(roll: f32) -> Matrix3<f32> {
    let (c, s) = (roll.cos(), roll.sin());
    Matrix3::new(
        1.0, 0.0, 0.0,
        0.0, c,   -s,
        0.0, s,    c,
    )
}

/// Rotation around Y-axis (pitch)
pub fn r_y(pitch: f32) -> Matrix3<f32> {
    let (c, s) = (pitch.cos(), pitch.sin());
    Matrix3::new(
        c,   0.0, s,
        0.0, 1.0, 0.0,
        -s,  0.0, c,
    )
}

/// Rotation around Z-axis (yaw)
pub fn r_z(yaw: f32) -> Matrix3<f32> {
    let (c, s) = (yaw.cos(), yaw.sin());
    Matrix3::new(
        c, -s, 0.0,
        s,  c, 0.0,
        0.0, 0.0, 1.0,
    )
}

/// Full rotation matrix from body to world using Euler ZYX (yaw → pitch → roll)
pub fn r_body_to_world(euler_angles: Vector3<f32>) -> Matrix3<f32> {
    let (roll, pitch, yaw) = (euler_angles.x, euler_angles.y, euler_angles.z);
    r_z(yaw) * r_y(pitch) * r_x(roll)
}

/// Rotation matrix from world to body (inverse of ZYX Euler rotation)
pub fn r_world_to_body(euler_angles: Vector3<f32>) -> Matrix3<f32> {
    r_body_to_world(euler_angles).transpose()
}

/// Full rotation matrix from object to body using Euler ZYX (yaw → pitch → roll)
pub fn r_object_to_body(euler_angles: Vector3<f32>) -> Matrix3<f32> {
    let (roll, pitch, yaw) = (euler_angles.x, euler_angles.y, euler_angles.z);
    r_z(yaw) * r_y(pitch) * r_x(roll)
}

/// Rotation matrix from body to object (inverse of ZYX Euler rotation)
pub fn r_body_to_object(euler_angles: Vector3<f32>) -> Matrix3<f32> {
    r_object_to_body(euler_angles).transpose()
}

pub fn jacobian_angular_velocity_zyx(euler_angles: Vector3<f32>) -> Matrix3<f32> {
    let (phi, theta, _) = (euler_angles[0], euler_angles[1], euler_angles[2]);

    let s_phi = phi.sin();
    let c_phi = phi.cos();
    let t_theta = theta.tan();
    let c_theta = theta.cos();

    Matrix3::new(
        1.0, s_phi * t_theta,  c_phi * t_theta,
        0.0, c_phi,           -s_phi,
        0.0, s_phi / c_theta,  c_phi / c_theta,
    )
}

pub fn linear_accel_body_to_world(
    euler_angles: Vector3<f32>,
    a_b: Vector3<f32>,
) -> Vector3<f32> {
    r_body_to_world(euler_angles) * a_b
}

pub fn linear_accel_world_to_body(
    euler_angles: Vector3<f32>,
    a_b: Vector3<f32>,
) -> Vector3<f32> {
    r_world_to_body(euler_angles) * a_b
}

pub fn angular_accel_body_to_world(
    euler_angles: Vector3<f32>,
    alpha_body: Vector3<f32>, 
) -> Vector3<f32> {
    r_body_to_world(euler_angles) * alpha_body // Rotation matrix from body to world
}

pub fn linear_velocity_world_to_body(
    euler_angles: Vector3<f32>,
    v_w: Vector3<f32>,
) -> Vector3<f32> {
    // Try to invert rotation matrix
    #[allow(non_snake_case)]
    let mut r_inv = r_body_to_world(euler_angles);
    if let Some(matrix_inv) = r_inv.try_inverse() {
        r_inv = matrix_inv;
    } else {
        println!("Matrix is not invertible!");
    }

    let v_b = r_inv * v_w;
    
    return v_b;
}

pub fn angular_velocity_world_to_body(
    euler_angles: Vector3<f32>,
    omega_w: Vector3<f32>,
) -> Vector3<f32> {
    // Try to invert rotation matrix
    #[allow(non_snake_case)]
    let mut r_inv = jacobian_angular_velocity_zyx(euler_angles);
    if let Some(matrix_inv) = r_inv.try_inverse() {
        r_inv = matrix_inv;
    } else {
        println!("Matrix is not invertible!");
    }

    let omega_b = r_inv * omega_w;

    return omega_b;
}
