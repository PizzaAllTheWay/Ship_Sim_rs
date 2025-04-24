use nalgebra::{Vector3, Matrix3};



// Euler Angles (START) ==================================================
/// Computes the transformation matrix `T` for ZYX Euler angles (yaw-pitch-roll),
/// which maps angular velocity in the body frame to Euler angle derivatives.
///
/// # Input:
/// - `euler_angles`: A `Vector3<f32>` where:
///     - x = roll (φ)
///     - y = pitch (θ)
///     - z = yaw (ψ)
///
/// # Output:
/// - A `Matrix3<f32>` such that:
/// ```
/// dot_euler_angles = T * omega_body
/// ```
///
/// This matrix is used to integrate Euler angles from body angular velocity
/// (e.g., when simulating orientation using roll, pitch, yaw states).
///
/// # Notes:
/// - The inverse of this matrix (i.e., `T⁻¹`) maps Euler angle rates to body angular velocity:
/// ```
/// omega_body = T⁻¹ * dot_euler_angles
/// ```
/// - Matrix becomes unstable near pitch = ±90° due to gimbal lock (tan(θ) → ∞).
#[allow(non_snake_case, unused_variables)]
pub fn T(
    euler_angles: Vector3<f32>,
) -> Matrix3<f32> {
    let phi = euler_angles.x;
    let theta = euler_angles.y;
    let psi = euler_angles.z;

    let T_inv: Matrix3<f32> = Matrix3::new(
        1.0,        0.0,           theta.sin(),
        0.0,  phi.cos(), theta.cos()*phi.sin(),
        0.0, -phi.sin(), theta.cos()*phi.cos(),
    );

    let T: Matrix3<f32> = T_inv.try_inverse().expect("Rotation matrix is not invertible!");

    return T;
}

pub fn angular_velocity_to_euler_dot_body(
    euler_angles: Vector3<f32>,
    omega: Vector3<f32>,
) -> Vector3<f32> {
    T(euler_angles) * omega
}

#[allow(non_snake_case)]
pub fn euler_dot_to_angular_velocity_body(
    euler_angles: Vector3<f32>,
    euler_dot: Vector3<f32>,
) -> Vector3<f32> {
    let T_inv = T(euler_angles).try_inverse().expect("T matrix is not invertible");
    T_inv * euler_dot
}

/// Converts Euler angle acceleration (second derivative of roll, pitch, yaw)
/// into angular acceleration in the body frame.
///
/// # Inputs:
/// - `euler_angles`: [roll (φ), pitch (θ), yaw (ψ)] in radians
/// - `euler_dot`: First derivative of euler angles [φ̇, θ̇, ψ̇]
/// - `euler_dot_dot`: Second derivative of euler angles [φ̈, θ̈, ψ̈]
///
/// # Output:
/// - Angular acceleration vector in body frame (rad/s²)
///
/// # Note:
/// This is derived from differentiating ω = T⁻¹(θ)·θ̇
#[allow(non_snake_case)]
pub fn euler_dot_dot_to_angular_accel_body(
    euler_angles: Vector3<f32>,
    euler_dot: Vector3<f32>,
    euler_dot_dot: Vector3<f32>,
) -> Vector3<f32> {
    let (phi, theta, _) = (euler_angles.x, euler_angles.y, euler_angles.z);
    let (phidot, thetadot, psidot) = (euler_dot.x, euler_dot.y, euler_dot.z);
    let (phiddot, thetaddot, psiddot) = (
        euler_dot_dot.x,
        euler_dot_dot.y,
        euler_dot_dot.z,
    );

    let sin_phi = phi.sin();
    let cos_phi = phi.cos();
    let sin_theta = theta.sin();
    let cos_theta = theta.cos();

    let alpha_x = phiddot + psiddot * sin_theta + thetadot * psidot * cos_theta;

    let alpha_y =
        thetaddot * cos_phi
        - phidot * thetadot * sin_phi
        + psiddot * cos_theta * sin_phi
        + phidot * psidot * cos_phi * cos_theta
        - thetadot * psidot * sin_phi * sin_theta;

    let alpha_z =
        psiddot * cos_phi * cos_theta
        - thetaddot * sin_phi
        - thetadot * psidot * sin_theta * cos_phi
        - phidot * thetadot * cos_phi
        - phidot * psidot * sin_phi * cos_theta;

    Vector3::new(alpha_x, alpha_y, alpha_z)
}

/// Converts angular acceleration in the body frame to Euler angle acceleration (φ̈, θ̈, ψ̈)
/// assuming current Euler angles and their time derivatives.
///
/// # Inputs:
/// - `euler_angles`: [roll (φ), pitch (θ), yaw (ψ)] in radians
/// - `euler_dot`: First derivative of euler angles [φ̇, θ̇, ψ̇]
/// - `alpha_body`: Angular acceleration in body frame [rad/s²]
///
/// # Output:
/// - Euler angle acceleration vector [φ̈, θ̈, ψ̈]
///
/// # Note:
/// This is approximate and assumes symbolic inversion is valid.
#[allow(non_snake_case)]
pub fn angular_accel_body_to_euler_dot_dot(
    euler_angles: Vector3<f32>,
    euler_dot: Vector3<f32>,
    alpha_body: Vector3<f32>,
) -> Vector3<f32> {
    let T_inv = T(euler_angles).try_inverse().expect("T matrix is not invertible");
    let dT_inv_dt = compute_dT_inv_dt(euler_angles, euler_dot);
    T_inv * (alpha_body - dT_inv_dt * euler_dot)
}

/// Computes time derivative of the inverse T matrix for ZYX Euler angles.
///
/// # Inputs:
/// - `euler_angles`: [roll (φ), pitch (θ), yaw (ψ)] in radians
/// - `euler_dot`: [φ̇, θ̇, ψ̇] in radians/s
///
/// # Output:
/// - d/dt(T⁻¹) as a 3x3 matrix
#[allow(non_snake_case)]
pub fn compute_dT_inv_dt(euler_angles: Vector3<f32>, euler_dot: Vector3<f32>) -> Matrix3<f32> {
    let (phi, theta) = (euler_angles.x, euler_angles.y);
    let (phidot, thetadot, _) = (euler_dot.x, euler_dot.y, euler_dot.z);

    let sin_phi = phi.sin();
    let cos_phi = phi.cos();
    let sin_theta = theta.sin();
    let cos_theta = theta.cos();

    let d11 = 0.0;
    let d12 = 0.0;
    let d13 = thetadot * cos_theta;

    let d21 = 0.0;
    let d22 = -phidot * sin_phi;
    let d23 = -thetadot * sin_theta * phi.sin() + phidot * cos_phi * cos_theta;

    let d31 = 0.0;
    let d32 = -phidot * cos_phi;
    let d33 = -thetadot * sin_theta * phi.cos() - phidot * sin_phi * cos_theta;

    Matrix3::new(
        d11, d12, d13,
        d21, d22, d23,
        d31, d32, d33,
    )
}
// Euler Angles (STOP) ==================================================



// Rotations (START) ==================================================
/// Rotation around X-axis (roll)
pub fn rot_x(roll: f32) -> Matrix3<f32> {
    let (c, s) = (roll.cos(), roll.sin());
    Matrix3::new(
        1.0, 0.0, 0.0,
        0.0, c,   -s,
        0.0, s,    c,
    )
}

/// Rotation around Y-axis (pitch)
pub fn rot_y(pitch: f32) -> Matrix3<f32> {
    let (c, s) = (pitch.cos(), pitch.sin());
    Matrix3::new(
        c,   0.0, s,
        0.0, 1.0, 0.0,
        -s,  0.0, c,
    )
}

/// Rotation around Z-axis (yaw)
pub fn rot_z(yaw: f32) -> Matrix3<f32> {
    let (c, s) = (yaw.cos(), yaw.sin());
    Matrix3::new(
        c, -s, 0.0,
        s,  c, 0.0,
        0.0, 0.0, 1.0,
    )
}

/// Full rotation matrix from body to world using Euler ZYX (yaw → pitch → roll)
pub fn rot_body_to_world(euler_angles: Vector3<f32>) -> Matrix3<f32> {
    let (roll, pitch, yaw) = (euler_angles.x, euler_angles.y, euler_angles.z);
    rot_z(yaw) * rot_y(pitch) * rot_x(roll)
}

/// Rotation matrix from world to body (inverse of ZYX Euler rotation)
pub fn rot_world_to_body(euler_angles: Vector3<f32>) -> Matrix3<f32> {
    rot_body_to_world(euler_angles).transpose()
}

/// Full rotation matrix from object to body using Euler ZYX (yaw → pitch → roll)
pub fn rot_object_to_body(euler_angles: Vector3<f32>) -> Matrix3<f32> {
    let (roll, pitch, yaw) = (euler_angles.x, euler_angles.y, euler_angles.z);
    rot_z(yaw) * rot_y(pitch) * rot_x(roll)
}

/// Rotation matrix from body to object (inverse of ZYX Euler rotation)
pub fn rot_body_to_object(euler_angles: Vector3<f32>) -> Matrix3<f32> {
    rot_object_to_body(euler_angles).transpose()
}
// Rotations (STOP) ==================================================



// Velocity Angular (START) ==================================================
pub fn angular_velocity_body_to_world(
    euler_angles_body: Vector3<f32>,
    omega_body: Vector3<f32>,
) -> Vector3<f32> {
    rot_body_to_world(euler_angles_body) * omega_body
}

pub fn angular_velocity_world_to_body(
    euler_angles_body: Vector3<f32>,
    omega_world: Vector3<f32>,
) -> Vector3<f32> {
    rot_world_to_body(euler_angles_body) * omega_world
}

pub fn angular_velocity_body_to_object(
    euler_angles_object: Vector3<f32>,
    omega_body: Vector3<f32>,
) -> Vector3<f32> {
    rot_body_to_object(euler_angles_object) * omega_body
}

pub fn angular_velocity_object_to_body(
    euler_angles_object: Vector3<f32>,
    omega_object: Vector3<f32>,
) -> Vector3<f32> {
    rot_object_to_body(euler_angles_object) * omega_object
}
// Velocity Angular (STOP) ==================================================



// Velocity Linear (START) ==================================================
pub fn linear_velocity_world_to_body(
    euler_angles_body: Vector3<f32>,
    v_w: Vector3<f32>,
) -> Vector3<f32> {
    rot_world_to_body(euler_angles_body) * v_w
}

pub fn linear_velocity_body_to_world(
    euler_angles_body: Vector3<f32>,
    v_b: Vector3<f32>,
) -> Vector3<f32> {
    rot_body_to_world(euler_angles_body) * v_b
}

/// Computes the linear velocity of an object point in the body frame,
/// accounting for the ship's motion, object's motion relative to the ship,
/// and rotational velocity due to offset from center of mass.
///
/// # Inputs:
/// - `v_body`: Linear velocity of the body frame origin (center of mass) [m/s]
/// - `v_object_in_body`: Linear velocity of the object relative to the body frame [m/s]
/// - `omega_body`: Angular velocity of the body [rad/s]
/// - `r_object_in_body`: Position vector of the object relative to the body origin [m]
///
/// # Output:
/// - `Vector3<f32>`: Total linear velocity of the object point in the body frame [m/s]
///
/// # Formula:
/// ```text
/// v_object =
///     v_body
///   + v_object_in_body
///   + omega × r
/// ```
pub fn linear_velocity_body_to_object(
    v_body: Vector3<f32>,
    v_object_in_body: Vector3<f32>,
    omega_body: Vector3<f32>,
    r_object_in_body: Vector3<f32>,
) -> Vector3<f32> {
    let v_object = v_body + v_object_in_body + omega_body.cross(&r_object_in_body);

    return v_object;
}

pub fn linear_velocity_object_to_body(
    v_object: Vector3<f32>,
    v_object_in_body: Vector3<f32>,
    omega_body: Vector3<f32>,
    r_object_in_body: Vector3<f32>,
) -> Vector3<f32> {
    let v_body = v_object - v_object_in_body - omega_body.cross(&r_object_in_body);

    return v_body;
}
// Velocity Linear (STOP) ==================================================



// Acceleration Angular (START) ==================================================
/// Computes angular acceleration in the world frame from body-frame values,
/// accounting for rotating basis effects (nonlinear dynamics).
///
/// # Inputs:
/// - `euler_angles`: Euler angles (roll, pitch, yaw) representing the orientation of the body frame w.r.t. world frame
/// - `omega_body`: Angular velocity vector in the body frame [rad/s]
/// - `alpha_body`: Angular acceleration vector in the body frame [rad/s²]
///
/// # Output:
/// - Returns angular acceleration vector expressed in the world frame [rad/s²]
///
/// # Note:
/// This computes:
///     α_world = ∑ (α_i * e_i + ω_i * (ω × e_i))
/// where e_i are the body axes (from rotation matrix), and ω is full angular velocity in world frame.
/// This includes gyroscopic terms from frame rotation.
#[allow(non_snake_case)]
pub fn angular_accel_body_to_world(
    euler_angles_body: Vector3<f32>,
    omega_body: Vector3<f32>,
    alpha_body: Vector3<f32>,
) -> Vector3<f32> {
    let R: Matrix3<f32> = rot_body_to_world(euler_angles_body); // Rotation matrix: body → world
    let omega_world: Vector3<f32> = R * omega_body;        // Angular velocity in world frame

    let mut alpha_world: Vector3<f32> = Vector3::zeros();

    for i in 0..3 {
        let e_i: Vector3<f32> = Vector3::new(R[(0, i)], R[(1, i)], R[(2, i)]); // Concrete vector
        alpha_world += alpha_body[i] * e_i + omega_body[i] * omega_world.cross(&e_i);
    }

    alpha_world
}

#[allow(non_snake_case)]
pub fn angular_accel_world_to_body(
    euler_angles_body: Vector3<f32>,
    omega_world: Vector3<f32>,
    alpha_world: Vector3<f32>,
) -> Vector3<f32> {
    let R: Matrix3<f32> = rot_world_to_body(euler_angles_body); // body axes in world frame
    let omega_body: Vector3<f32> = R.transpose() * omega_world;

    let mut alpha_body: Vector3<f32> = Vector3::zeros();

    for i in 0..3 {
        let e_i: Vector3<f32> = Vector3::new(R[(0, i)], R[(1, i)], R[(2, i)]);
        let correction: Vector3<f32> = omega_world.cross(&(omega_body[i] * e_i));
        alpha_body[i] = e_i.dot(&(alpha_world - correction));
    }

    alpha_body
}

#[allow(non_snake_case)]
pub fn angular_accel_body_to_object(
    euler_angles_object: Vector3<f32>,
    omega_body: Vector3<f32>,
    alpha_body: Vector3<f32>,
) -> Vector3<f32> {
    let R: Matrix3<f32> = rot_body_to_object(euler_angles_object); // body → object
    let omega_object: Vector3<f32> = R * omega_body;

    let mut alpha_object: Vector3<f32> = Vector3::zeros();

    for i in 0..3 {
        let e_i: Vector3<f32> = Vector3::new(R[(0, i)], R[(1, i)], R[(2, i)]);
        alpha_object += alpha_body[i] * e_i + omega_body[i] * omega_object.cross(&e_i);
    }

    alpha_object
}

#[allow(non_snake_case)]
pub fn angular_accel_object_to_body(
    euler_angles_object: Vector3<f32>,
    omega_object: Vector3<f32>,
    alpha_object: Vector3<f32>,
) -> Vector3<f32> {
    let R: Matrix3<f32> = rot_object_to_body(euler_angles_object); // body → object
    let omega_body: Vector3<f32> = R.transpose() * omega_object;

    let mut alpha_body: Vector3<f32> = Vector3::zeros();

    for i in 0..3 {
        let e_i: Vector3<f32> = Vector3::new(R[(0, i)], R[(1, i)], R[(2, i)]);
        let correction: Vector3<f32> = omega_object.cross(&(omega_body[i] * e_i));
        alpha_body[i] = e_i.dot(&(alpha_object - correction));
    }

    alpha_body
}
// Acceleration Angular (STOP) ==================================================



// Acceleration Linear (START) ==================================================
pub fn linear_accel_body_to_world(
    euler_angles_body: Vector3<f32>,
    a_b: Vector3<f32>,
) -> Vector3<f32> {
    rot_body_to_world(euler_angles_body) * a_b
}

pub fn linear_accel_world_to_body(
    euler_angles_body: Vector3<f32>,
    a_w: Vector3<f32>,
) -> Vector3<f32> {
    rot_world_to_body(euler_angles_body) * a_w
}

/// Computes the **total acceleration** of a moving object on a rotating body (e.g. ship),
/// **expressed in the body frame**. This includes contributions from:
/// - the ship's center-of-mass acceleration,
/// - the object's own motion relative to the ship,
/// - and all rotating-frame effects (tangential, centripetal, Coriolis).
///
/// # Inputs:
/// - `a_body`: Linear acceleration of the ship's center of mass in the body frame [m/s²]
/// - `a_object_in_body`: Linear acceleration of the object relative to the body frame [m/s²]
///     - (e.g. crate sliding, internal drone motion)
/// - `alpha_body`: Angular acceleration of the body [rad/s²]
/// - `r_obj_in_body`: Position of the object relative to the body origin (CoM), in body frame [m]
/// - `omega_body`: Angular velocity of the body [rad/s]
/// - `v_obj_in_body`: Velocity of the object relative to the body frame [m/s]
///
/// # Output:
/// - `Vector3<f32>`: Total linear acceleration of the object in the body frame [m/s²]
///
/// # Formula:
/// ```text
/// a_total =
///     a_body
///   + a_object_in_body
///   + alpha × r
///   + omega × (omega × r)
///   + 2 * omega × v
/// ```
///
/// This is used when simulating or tracking a moving subsystem or payload on a rotating/moving vehicle.
/// Essential in robotics, ship dynamics, and inertial sensor compensation.
pub fn linear_accel_object(
    a_body: Vector3<f32>,
    a_object_in_body: Vector3<f32>,
    alpha_body: Vector3<f32>,
    r_object_in_body: Vector3<f32>,
    omega_body: Vector3<f32>,
    v_object_in_body: Vector3<f32>,
) -> Vector3<f32> {
    let tangential = alpha_body.cross(&r_object_in_body);
    let centripetal = omega_body.cross(&omega_body.cross(&r_object_in_body));
    let coriolis = 2.0 * omega_body.cross(&v_object_in_body);

    let a_object = a_body + a_object_in_body + tangential + centripetal + coriolis;

    return a_object;
}

pub fn linear_accel_object_to_body(
    a_object: Vector3<f32>,
    a_object_in_body: Vector3<f32>,
    alpha_body: Vector3<f32>,
    r_object_in_body: Vector3<f32>,
    omega_body: Vector3<f32>,
    v_object_in_body: Vector3<f32>,
) -> Vector3<f32> {
    let tangential = alpha_body.cross(&r_object_in_body);
    let centripetal = omega_body.cross(&omega_body.cross(&r_object_in_body));
    let coriolis = 2.0 * omega_body.cross(&v_object_in_body);

    let a_body = a_object - a_object_in_body - tangential - centripetal - coriolis;

    return a_body;
}
// Acceleration Linear (STOP) ==================================================
