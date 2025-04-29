// Custom libraries
use ship_sim_lib::estimators::ship_approx;
use ship_sim_lib::estimators::ekf;
use ship_sim_lib::estimators::ukf;
use ship_sim_lib::simulation::kinematics;
use ship_sim_lib::comm::udp_utils;
use ship_sim_lib::comm::udp_topics::TOPICS;

// Libraries for multithreading
use std::thread;
use std::time::{Duration, Instant};
use std::sync::{Arc, RwLock};

// Library for data formatting
use serde::Deserialize;
use std::fs;
use std::str;

// Libraries for math
use nalgebra::{
    Vector2, Vector3, SVector,
    SMatrix
};
use std::f32::consts::PI;



// System State data structure ----------
pub type Vector7<T> = SVector<T, 7>;
pub type Vector10<T> = SVector<T, 10>;
pub type Vector12<T> = SVector<T, 12>;

pub type Matrix7x7<T> = SMatrix::<T, 7, 7>;
pub type Matrix10x10<T> = SMatrix::<T, 10, 10>;
pub type Matrix12x2<T> = SMatrix::<T, 12, 2>;
pub type Matrix9x12<T> = SMatrix::<T, 9, 12>;
pub type Matrix12x9<T> = SMatrix::<T, 12, 9>;
pub type Matrix12x12<T> = SMatrix::<T, 12, 12>;

// Config data structure ----------
#[derive(Deserialize)]
struct ShipConfig {
    mass: f32,
    dimensions: [f32; 2],
    thruster_placement: [f32; 3],
    velocity_linear_max: f32,
    velocity_angular_max: f32,
    x_0: [f32; 12],
}

#[derive(Deserialize)]
struct SensorsConfig {
    gnss_antenna1_placement: [f32; 3],
    gnss_antenna2_placement: [f32; 3],
    imu_placement: [f32; 3],
    imu_rotation: [f32; 3],
}

#[allow(non_snake_case)]
#[derive(Deserialize)]
struct EstimatorsConfig {
    estimator_pub_frequency: f32,
    P_0: [f32; 12],
    Q: [f32; 12],
    R_gnss: [f32; 10],
    R_imu: [f32; 7],
    imu_drift: [f32; 7],
    kappa: f32,
    alpha: f32,
    beta: f32,
}

#[derive(Deserialize)]
struct Config {
    ship: ShipConfig,
    sensors: SensorsConfig,
    estimators: EstimatorsConfig,
}



// Approximation of ships non linear ODE ----------
// x_dot = f(x, u)
// !NOTE: This is different ODE from ship simulation model!!!
// Pretend like we don't know ship model completely
// This is a estimate of ship model
// Thus it is a simplification/approximation
pub struct ODE {
    pub ship_dynamic: ship_approx::ShipDynamics,
    pub x: Vector12<f32>,
}
impl ODE {
    pub fn new(
        x_0: Vector12<f32>, // Initial states
        ship_mass: f32, // [kg]
        ship_dimensions: [f32; 2], // (r, l) [m]
        thruster_placement: Vector3<f32>, // Placement of thruster on the ship in body frame [x, y, z] [m]
        velocity_linear_max: f32, // [m/s]
        velocity_angular_max: f32, // [m/s]

    ) -> Self {
        // Initialize ship dynamics        
        let ship_dynamic = ship_approx::ShipDynamics::new(
            ship_mass,
            ship_dimensions,
            thruster_placement,
            velocity_linear_max,
            velocity_angular_max,
        );

        // Initialize starting conditions
        let x = x_0;

        // Return the structure
        Self { 
            ship_dynamic,
            x,
        }
    }

    #[allow(unused_variables)]
    fn f(
        &self,
        x: &Vector12<f32>,
        u: &Vector2<f32>,
    ) -> Vector12<f32> {
        // Constant vectors
        let gravity_w = Vector3::new(0.0, 0.0, -9.81);

        // Split up states into manageable subparts
        let v_lin_w: Vector3<f32> = x.fixed_rows::<3>(0).into(); // [vx, vy, vz]
        let euler_dot: Vector3<f32> = x.fixed_rows::<3>(3).into(); // [angular velocity in roll, pitch, yaw]
        let r_lin_w: Vector3<f32> = x.fixed_rows::<3>(6).into(); // [x, y, z]
        let euler: Vector3<f32> = x.fixed_rows::<3>(9).into(); // [roll, pitch, yaw]

        let thruster_rpm: f32 = u[0];
        let thruster_angle: f32 = u[1];

        // Convert to body frame
        let r_ang_b: Vector3<f32> = euler;
        let r_lin_b: Vector3<f32> = kinematics::rot_world_to_body(euler) * r_lin_w;
        let v_ang_b: Vector3<f32> = kinematics::euler_dot_to_angular_velocity_body(euler, euler_dot);
        let v_lin_b: Vector3<f32> = kinematics::linear_velocity_world_to_body(euler, v_lin_w);

        let gravity_b = kinematics::linear_accel_world_to_body(euler, gravity_w);

        // Dynamics
        let (a_lin_b, a_ang_b) = self.ship_dynamic.calc_accel_body(
            thruster_rpm,
            thruster_angle,
            v_lin_b,
            v_ang_b,
            gravity_b,
            r_lin_w,
        );

        // Convert to world frame
        let a_lin_w = kinematics::linear_accel_body_to_world(euler, a_lin_b,);
        let euler_dot_dot = kinematics::angular_accel_body_to_euler_dot_dot(euler, euler_dot, a_ang_b);
        
        // Structure return states properly
        let mut x_dot: Vector12<f32> = Vector12::<f32>::zeros();
        x_dot.fixed_rows_mut::<3>(0).copy_from(&a_lin_w);       // [ax, ay, az]
        x_dot.fixed_rows_mut::<3>(3).copy_from(&euler_dot_dot); // [angular acceleration in roll, pitch, yaw]
        x_dot.fixed_rows_mut::<3>(6).copy_from(&v_lin_w);       // [vx, vy, vz]
        x_dot.fixed_rows_mut::<3>(9).copy_from(&euler_dot);     // [angular velocity in roll, pitch, yaw]
        
        return x_dot;
    }
}



// Measurement Transformation functions for sensors ----------
// estimated measurement: (~z) = h(x)   // transform state -> measurement space
// measured estimate:     (~x) = h⁻¹(z) // transform measurement -> state space

/// Computes the GNSS measurement prediction from the current state estimate.
/// 
/// # Inputs:
/// - `x`: State vector [vx, vy, vz, wx, wy, wz, px, py, pz, roll, pitch, yaw]
/// - `antenna1_placement`: Antenna 1 position in body frame [m]
/// - `antenna2_placement`: Antenna 2 position in body frame [m]
/// 
/// # Output:
/// - GNSS measurement vector [antenna1_x, antenna1_y, antenna1_z, antenna2_x, antenna2_y, antenna2_z, v_avg_x, v_avg_y, v_avg_z] in world frame
pub fn h_gnss(
    x: Vector12<f32>,
    antenna1_placement: Vector3<f32>,
    antenna2_placement: Vector3<f32>,
) -> Vector10<f32> {
    // Split up states into manageable subparts
    let v_lin_w: Vector3<f32> = x.fixed_rows::<3>(0).into(); // [vx, vy, vz]
    let euler_dot: Vector3<f32> = x.fixed_rows::<3>(3).into(); // [angular velocity in roll, pitch, yaw]
    let r_lin_w: Vector3<f32> = x.fixed_rows::<3>(6).into(); // [x, y, z]
    let euler: Vector3<f32> = x.fixed_rows::<3>(9).into(); // [roll, pitch, yaw]

    // Convert Ship to body frame
    let v_ang_ship: Vector3<f32> = kinematics::euler_dot_to_angular_velocity_body(euler, euler_dot);
    let v_lin_ship: Vector3<f32> = kinematics::linear_velocity_world_to_body(euler, v_lin_w);

    // Convert Ship to object frame
    let antenna1_velocity_in_ship = Vector3::<f32>::zeros(); // Antenna sits tight, no linear velocity in the ship
    let antenna2_velocity_in_ship = Vector3::<f32>::zeros(); // Antenna sits tight, no linear velocity in the ship
    let antenna1_placement_in_ship = antenna1_placement;
    let antenna2_placement_in_ship = antenna2_placement;
    let v_lin_antenna1: Vector3<f32> = kinematics::linear_velocity_body_to_object(
        v_lin_ship,
        antenna1_velocity_in_ship,
        v_ang_ship,
        antenna1_placement_in_ship,
    );
    let v_lin_antenna2: Vector3<f32> = kinematics::linear_velocity_body_to_object(
        v_lin_ship,
        antenna2_velocity_in_ship,
        v_ang_ship,
        antenna2_placement_in_ship,
    );

    // Convert Antennas to body frame
    let r_lin_antenna1_b: Vector3<f32> = antenna1_placement_in_ship;
    let r_lin_antenna2_b: Vector3<f32> = antenna2_placement_in_ship;
    let v_lin_antenna1_b: Vector3<f32> = kinematics::linear_velocity_object_to_body(
        v_lin_antenna1,
        antenna1_velocity_in_ship,
        v_ang_ship,
        antenna1_placement_in_ship,
    );
    let v_lin_antenna2_b: Vector3<f32> = kinematics::linear_velocity_object_to_body(
        v_lin_antenna2,
        antenna2_velocity_in_ship,
        v_ang_ship,
        antenna2_placement_in_ship,
    );

    // Convert Antennas to world frame
    let r_lin_antenna1_w: Vector3<f32> = r_lin_antenna1_b + r_lin_w;
    let r_lin_antenna2_w: Vector3<f32> = r_lin_antenna2_b + r_lin_w;
    let v_lin_antenna1_w: Vector3<f32> = kinematics::linear_velocity_body_to_world(euler, v_lin_antenna1_b);
    let v_lin_antenna2_w: Vector3<f32> = kinematics::linear_velocity_body_to_world(euler, v_lin_antenna2_b);

    // Average velocity
    let v_avg = (v_lin_antenna1_w + v_lin_antenna2_w) * 0.5;

    // Yaw angle
    let yaw_angle = euler[2];

    // Save transformed vector
    let mut y = Vector10::<f32>::zeros();
    y.fixed_rows_mut::<3>(0).copy_from(&r_lin_antenna1_w);
    y.fixed_rows_mut::<3>(3).copy_from(&r_lin_antenna2_w);
    y.fixed_rows_mut::<3>(6).copy_from(&v_avg);
    y[9] = yaw_angle;

    y
}

fn h_imu(
    x: Vector12<f32>,            // Estimated full state vector
    imu_placement: Vector3<f32>, // IMU placement: [x, y, z] in body frame
    imu_rotation: Vector3<f32>,  // IMU placement: [roll, pitch, yaw] in IMU frame
) -> Vector7<f32> {
    // Split up states into manageable subparts
    let v_lin_w: Vector3<f32> = x.fixed_rows::<3>(0).into(); // [vx, vy, vz]
    let euler_dot: Vector3<f32> = x.fixed_rows::<3>(3).into(); // [angular velocity in roll, pitch, yaw]
    let euler: Vector3<f32> = x.fixed_rows::<3>(9).into(); // [roll, pitch, yaw]

    // Convert Ship to body frame
    let v_lin_ship: Vector3<f32> = kinematics::linear_velocity_world_to_body(euler, v_lin_w);
    let v_ang_ship: Vector3<f32> = kinematics::euler_dot_to_angular_velocity_body(euler, euler_dot);

    // Convert Ship to object frame
    let imu_velocity_in_ship = Vector3::<f32>::zeros(); // IMU sits tight, no linear velocity in the ship
    let imu_placement_in_ship = imu_placement; // IMU placement relative to ship
    let mut v_lin_imu = kinematics::linear_velocity_body_to_object(
        v_lin_ship,
        imu_velocity_in_ship,
        v_ang_ship,
        imu_placement_in_ship,
    );
    v_lin_imu = kinematics::rot_body_to_object(imu_rotation) * v_lin_imu; // Rotate into IMU frame

    let imu_v_ang_in_ship = Vector3::<f32>::zeros(); // IMU sits tight, no angular velocity in the ship
    let mut v_ang_imu: Vector3<f32> = kinematics::angular_velocity_body_to_object(imu_v_ang_in_ship, v_ang_ship);
    v_ang_imu *= -1.0; // ? Need to invert the velocities for some reason (I have no idea why because its literally same transform as in simulation)

    let mag_imu = euler[2]; // Just assume its 1:1 yaw angle no rotation transforms

    // Pack it
    let mut y = Vector7::<f32>::zeros();
    y.fixed_rows_mut::<3>(0).copy_from(&v_lin_imu); // velocity
    y.fixed_rows_mut::<3>(3).copy_from(&v_ang_imu); // gyro
    y[6] = mag_imu; // yaw angle

    y
}



// Handy functions ----------
/// Limit the estimated ship states to ensure stable EKF operation
///
/// Removes roll/pitch, and wraps yaw angle to [-π, π]
fn limit_states(x: Vector12<f32>) -> Vector12<f32> {
    let mut x_limited = x;

    // Remove roll and pitch position and velocity
    // This is because we estimate ship states, roll and pitch states are irrelevant for estimating ships position and velocity
    // If roll and pitch were anything than 0 the ship would be rolled over game over
    // Also because of euler angles if these come close to +- pi/2 it will get a singularity
    x_limited[3] = 0.0;  // Roll velocity
    x_limited[4] = 0.0;  // Pitch velocity
    x_limited[9] = 0.0;  // Roll
    x_limited[10] = 0.0; // Pitch

    // Limit z velocity as it can cause unstably in approximated ship model
    x_limited[2] = x[2].clamp(-1.0, 1.0); // Z velocity

    // Clamp physically impossible velocities
    x_limited[0] = x[0].clamp(-20.0, 20.0); // X velocity
    x_limited[1] = x[1].clamp(-20.0, 20.0); // Y velocity
    x_limited[5] = x[5].clamp(-2.0, 2.0); // Yaw velocity

    x_limited
}

/// Unwrap angle to preserve continuous angle rotation over time
///
/// Takes previous unwrapped angle and a new wrapped angle (from IMU or measurement),
/// calculates the shortest angular difference, and adds it to the previous angle.
/// This avoids jumps at ±π and allows full angle accumulation (e.g., 0 → 2π → 4π → ...).
///
/// # Inputs:
/// - `prev_unwrapped`: Previous angle in unwrapped continuous radians
/// - `new_wrapped`: New angle in wrapped form (range -π to π or 0 to 2π)
///
/// # Output:
/// - New angle, unwrapped and continuous (can grow beyond ±2π)
fn unwrap_angle(prev_unwrapped: f32, new_wrapped: f32) -> f32 {
    let mut delta = new_wrapped - (prev_unwrapped % (2.0 * PI));
    
    if delta > PI {
        delta -= 2.0 * PI;
    } else if delta < -PI {
        delta += 2.0 * PI;
    }

    prev_unwrapped + delta
}

fn unwrap_angle_inv(prev_unwrapped: f32, new_wrapped: f32) -> f32 {
    let mut delta = (-new_wrapped) - (prev_unwrapped % (2.0 * PI));
    
    if delta > PI {
        delta -= 2.0 * PI;
    } else if delta < -PI {
        delta += 2.0 * PI;
    }

    prev_unwrapped + delta
}

fn angle_between_points(front: Vector3<f32>, back: Vector3<f32>) -> f32 {
    let dx = back.x - front.x;
    let dy = back.y - front.y;
    let mut angle = dy.atan2(dx);

    // wrap to [-π, π]
    if angle > PI {
        angle -= 2.0 * PI;
    } else if angle < -PI {
        angle += 2.0 * PI;
    }

    angle
}

/// Set all negative entries in a matrix to zero.
fn clip_negative_to_zero<const N: usize>(mut matrix: SMatrix<f32, N, N>) -> SMatrix<f32, N, N> {
    matrix.iter_mut().for_each(|x| {
        if *x < 0.0 {
            *x = 0.0;
        }
    });
    matrix
}

/// Print any matrix (SMatrix) nicely with fixed formatting
///
/// # Parameters:
/// - `name`: Matrix label to print before the data
/// - `matrix`: The matrix to print, supports any dimensions R×C
///
/// # Output:
/// Example for 3x2:
/// A = [
///   [   1.0000000,   2.0000000, ],
///   [   3.0000000,   4.0000000, ],
///   [   5.0000000,   6.0000000, ],
/// ]
fn print_matrix<T: std::fmt::Display, const R: usize, const C: usize>(
    name: &str,
    matrix: &nalgebra::SMatrix<T, R, C>
) {
    println!("{} = [", name);
    for r in 0..R {
        print!("  [");
        for c in 0..C {
            // Print each value with consistent spacing
            print!("{:>12.8}, ", matrix[(r, c)]);
        }
        println!("],");
    }
    println!("]");
}



fn main() {
    // Setup (START) ==================================================
    // Get config file
    let config_str = fs::read_to_string("config.toml").expect("Failed to read config file");
    let config: Config = toml::from_str(&config_str).expect("Failed to parse TOML config");

    // Create shared resources for estimators
    let thruster_control= Arc::new(RwLock::new(TOPICS::thruster_control::DataType::zeros()));
    // Setup (STOP) ==================================================



    // GET - Control Forces (START) ==================================================
    let thruster_control_clone = thruster_control.clone();
    thread::spawn(move || {
        loop {
            // Wait for thruster forces data to arrive from gui
            // Once received format to correct datatype
            let msg = udp_utils::subscribe(TOPICS::thruster_control::PORT).expect("Failed to get forces data");
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let control: TOPICS::thruster_control::DataType = udp_utils::decode_json(json_str);

            // Save thruster forces in shared resource for simulator
            let mut thruster_control = thruster_control_clone.write().unwrap();
            *thruster_control = control;
        }
    });
    // GET - Control Forces (STOP) ==================================================


    
    // Extended Kalman Filter (START) ==================================================
    {
        // Initialize EKF shared states ----------
        let x_0: Vector12<f32> = Vector12::<f32>::from_row_slice(&config.ship.x_0);
        #[allow(non_snake_case)]
        let P_0_vector: Vector12<f32> = Vector12::<f32>::from_row_slice(&config.estimators.P_0);
        #[allow(non_snake_case)]
        let P_0: Matrix12x12<f32> = Matrix12x12::<f32>::from_diagonal(&P_0_vector);
        let ekf_data: ekf::SharedState<12> = ekf::SharedState {
            x_est_post: Arc::new(RwLock::new(x_0)),
            P_post: Arc::new(RwLock::new(P_0)),
            x_est_pri: Arc::new(RwLock::new(x_0)),
            P_pri: Arc::new(RwLock::new(P_0)),
        };

        // IMU drift setup
        #[allow(non_snake_case)]
        let R_imu_vector: Vector7<f32> = Vector7::<f32>::from_row_slice(&config.estimators.R_imu);
        #[allow(non_snake_case)]
        let R_imu_0: Matrix7x7<f32> = Matrix7x7::<f32>::from_diagonal(&R_imu_vector);
        #[allow(non_snake_case)]
        let R_imu = Arc::new(RwLock::new(R_imu_0));
        let v_lin_imu = Arc::new(RwLock::new(Vector3::<f32>::zeros()));
        
        // Predict using approximated ship model ----------
        let ekf_data_clone = ekf_data.clone();
        let thruster_control_clone = thruster_control.clone();
        #[allow(non_snake_case)]
        thread::spawn(move || {
            // Initialize system ----------
            // Calculate interval for EKF Estimate publishing rate
            let dt = 1.0/config.estimators.estimator_pub_frequency; // [s]
            let interval = Duration::from_millis((dt * 1000.0) as u64); // [ms]

            // Get initial states
            let x_0: Vector12<f32> = Vector12::<f32>::from_row_slice(&config.ship.x_0);

            // Setup functions to linearize system later for any given work point
            let ode: ODE = ODE::new(
                x_0, 
                config.ship.mass,
                config.ship.dimensions,
                Vector3::<f32>::from_row_slice(&config.ship.thruster_placement),
                config.ship.velocity_linear_max,
                config.ship.velocity_angular_max,
            );

            // Get confidence matrix for our model
            let Q_vector: Vector12<f32> = Vector12::<f32>::from_row_slice(&config.estimators.Q);
            let Q: Matrix12x12<f32> = Matrix12x12::<f32>::from_diagonal(&Q_vector);

            loop {
                let start_t = Instant::now();

                // Get control input states
                let u_prev = *thruster_control_clone.read().unwrap();

                // Get EKF states
                let x_est_post_prev = *ekf_data_clone.x_est_post.read().unwrap();
                let P_post_prev = *ekf_data_clone.P_post.read().unwrap();
                
                // Predict
                let (
                    mut x_est_priori,
                    P_priori
                ) = ekf::predict(
                    |x, u| ode.f(x, u),
                    dt, 
                    x_est_post_prev, 
                    u_prev, 
                    P_post_prev,
                    Q,
                );

                x_est_priori = limit_states(x_est_priori);

                // Update EKF states
                {
                    let mut x_est_pri = ekf_data_clone.x_est_pri.write().unwrap();
                    *x_est_pri = x_est_priori;
                }
                {
                    let mut P_pri = ekf_data_clone.P_pri.write().unwrap();
                    *P_pri = P_priori;
                }

                // Publish EKF data
                let ekf_data: TOPICS::ekf::DataType = x_est_priori;
                let ekf_data_json = udp_utils::encode_json(&ekf_data);
                udp_utils::publish(TOPICS::ekf::PORT, ekf_data_json.as_bytes()).expect("Failed to publish EKF estimate data");

                // Precise way to calculate interval
                // This way we estimate at exactly the desired frequency
                let elapsed_t = start_t.elapsed();
                if elapsed_t < interval {
                    thread::sleep(interval - elapsed_t);
                }
            }
        });
        
        // Correct using GNSS ----------
        let ekf_data_clone = ekf_data.clone();
        #[allow(non_snake_case)]
        let R_imu_clone = R_imu.clone();
        let v_lin_imu_clone = v_lin_imu.clone();
        #[allow(non_snake_case)]
        thread::spawn(move || {
            // Initialize system ----------
            // Get confidence matrix for our measurements
            let R_vector: Vector10<f32> = Vector10::<f32>::from_row_slice(&config.estimators.R_gnss);
            let R: Matrix10x10<f32> = Matrix10x10::<f32>::from_diagonal(&R_vector);

            // Unwrapped yaw follower
            let mut yaw_gnss = 0.0;

            loop {
                // Wait for GNSS data
                let msg = udp_utils::subscribe(TOPICS::gnss::PORT).unwrap();
                let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
                let gnss: TOPICS::gnss::DataType = udp_utils::decode_json(json_str);

                // EKF States
                let x_est_pri = *ekf_data_clone.x_est_pri.read().unwrap();
                let P_pri = *ekf_data_clone.P_pri.read().unwrap();

                // Calculate angle between GNSS Antennas and unwrap them
                let antenna1_pos: Vector3<f32> = gnss.fixed_rows::<3>(0).into(); // [x, y, z]
                let antenna2_pos: Vector3<f32> = gnss.fixed_rows::<3>(3).into(); // [x, y, z]
                let antenna_angle = angle_between_points(antenna1_pos, antenna2_pos);
                yaw_gnss = unwrap_angle(yaw_gnss, antenna_angle);

                // For GNSS measurements we don't need to rotate or adjust, the values are already given properly and absolute
                let yaw_gnss_adj = yaw_gnss;

                // Build measurement vector 
                let mut z = Vector10::<f32>::zeros();
                z.fixed_rows_mut::<9>(0).copy_from(&gnss); // Antenna 1 and 2 positions + Ships Velocity
                z[9] = yaw_gnss_adj;

                // Correction
                let (
                    mut x_est_posterior,
                    P_posterior,
                ) = ekf::correct(
                    |x| h_gnss(
                        x.clone(),
                        Vector3::from(config.sensors.gnss_antenna1_placement),
                        Vector3::from(config.sensors.gnss_antenna2_placement),
                    ),
                    z,
                    x_est_pri,
                    P_pri,
                    R,
                );

                x_est_posterior = limit_states(x_est_posterior);

                // In addition we must update IMU data to absolute certainty values from GNSS where it applies
                // Mainly to linear velocity IMU integral and drift to be reset
                {
                    // Convert measured GNSS velocity to IMU velocity integral
                    let z_imu = h_imu(
                        x_est_posterior.clone(),
                        Vector3::from_column_slice(&config.sensors.imu_placement),
                        Vector3::from_column_slice(&config.sensors.imu_rotation),
                    );

                    // Invert linear velocity because opposite direction 
                    let z_imu_v = z_imu.fixed_rows::<3>(0).into_owned();

                    let mut v_lin_imu_write = v_lin_imu_clone.write().unwrap();
                    *v_lin_imu_write = z_imu_v;
                }
                {
                    let mut R_imu_write = R_imu_clone.write().unwrap();
                    *R_imu_write = R_imu_0;
                }

                // Update EKF states
                {
                    let mut x_est_post = ekf_data_clone.x_est_post.write().unwrap();
                    *x_est_post = x_est_posterior;
                }
                {
                    let mut P_post = ekf_data_clone.P_post.write().unwrap();
                    *P_post = P_posterior;
                }
            }
        });
        
        // Correction using IMU ----------
        let ekf_data_clone = ekf_data.clone();
        #[allow(non_snake_case)]
        let R_imu_clone = R_imu.clone();
        let v_lin_imu_clone = v_lin_imu.clone();
        #[allow(non_snake_case)]
        thread::spawn(move || {
            // Initialize system ----------
            // Integration states
            let mut last_time = Instant::now();

            // Unwrapped yaw follower
            let mut mag_imu = 0.0;

            // Get the IMU drift
            let mut imu_drift_factor: Vector7::<f32> = Vector7::<f32>::from_row_slice(&config.estimators.imu_drift);
            imu_drift_factor += Vector7::<f32>::from_element(1.0);
            let imu_drift_matrix: Matrix7x7<f32> = Matrix7x7::from_diagonal(&imu_drift_factor);
            
            loop {
                // Wait for IMU data
                let msg = udp_utils::subscribe(TOPICS::imu::PORT).unwrap();
                let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
                let imu: TOPICS::imu::DataType = udp_utils::decode_json(json_str);
                
                // EKF States
                let x_est_pri = *ekf_data_clone.x_est_pri.read().unwrap();
                let P_pri = *ekf_data_clone.P_pri.read().unwrap();

                // Integrate acceleration to velocity
                let dt = last_time.elapsed().as_secs_f32();
                last_time = Instant::now();
                
                let mut accel = imu.fixed_rows::<3>(0).into_owned();
                accel[2] -= 9.81; // Subtract the constant acceleration from earths gravity

                let mut v_lin_imu = *v_lin_imu_clone.read().unwrap();
                v_lin_imu += dt * accel;

                // unwrap IMU magnetometer angle to be continuous instead of -pi to pi
                mag_imu = unwrap_angle_inv(mag_imu, imu[6]);

                // For measurements we need to rotate magnetometer values to proper position
                let mag_imu_adj = mag_imu + config.sensors.imu_rotation[2] * 2.0 - PI/2.0;

                // Add drift to the measurement noise matrix
                let mut R: Matrix7x7<f32> = *R_imu_clone.read().unwrap();
                R = R.component_mul(&imu_drift_matrix);

                // Update IMU velocity and drift matrix
                {
                    let mut v_lin_imu_write = v_lin_imu_clone.write().unwrap();
                    *v_lin_imu_write = v_lin_imu;
                }
                {
                    let mut R_imu_write = R_imu_clone.write().unwrap();
                    *R_imu_write = R;
                }
                
                // Build measurement vector 
                let mut z = Vector7::<f32>::zeros();
                z.fixed_rows_mut::<3>(0).copy_from(&v_lin_imu); // velocity
                z.fixed_rows_mut::<3>(3).copy_from(&imu.fixed_rows::<3>(3)); // gyro
                z[6] = mag_imu_adj;

                // Correction
                let (
                    mut x_est_posterior,
                    P_posterior,
                ) = ekf::correct(
                    |x| h_imu(
                        x.clone(),
                        Vector3::from_column_slice(&config.sensors.imu_placement),
                        Vector3::from_column_slice(&config.sensors.imu_rotation),
                    ),
                    z,
                    x_est_pri,
                    P_pri,
                    R,
                );

                x_est_posterior = limit_states(x_est_posterior);

                // Update EKF states
                {
                    let mut x_est_post = ekf_data_clone.x_est_post.write().unwrap();
                    *x_est_post = x_est_posterior;
                }
                {
                    let mut P_post = ekf_data_clone.P_post.write().unwrap();
                    *P_post = P_posterior;
                }
            }
        });
    }
    // Extended Kalman Filter (STOP) ==================================================



    // Unscented Kalman Filter (START) ==================================================
    {
        // Initialize UKF shared states ----------
        // Initial states
        let x_0: Vector12<f32> = Vector12::<f32>::from_row_slice(&config.ship.x_0);
        #[allow(non_snake_case)]
        let P_0_vector: Vector12<f32> = Vector12::<f32>::from_row_slice(&config.estimators.P_0);
        #[allow(non_snake_case)]
        let P_0: Matrix12x12<f32> = Matrix12x12::<f32>::from_diagonal(&P_0_vector);

        #[allow(non_snake_case)]
        const N: usize = 12;
        let lambda: f32 = ukf::calc_lambda::<N>(
            config.estimators.kappa,
            config.estimators.alpha,
        );
        let weights: ukf::Weights<N, {2*N}> = ukf::calc_weights(
            lambda,
            config.estimators.alpha,
            config.estimators.beta,
        );
        let sigma_points: ukf::SigmaPoints<N, {2*N}> = ukf::calc_sigma_points(
            x_0,
            P_0,
            lambda
        );
    
        // Build data structure
        let ukf_data: ukf::SharedState<N, {2*N}> = ukf::SharedState {
            x_est_post: Arc::new(RwLock::new(x_0)),
            P_post: Arc::new(RwLock::new(P_0)),
            x_est_pri: Arc::new(RwLock::new(x_0)),
            P_pri: Arc::new(RwLock::new(P_0)),

            lambda: Arc::new(RwLock::new(lambda)),
            weights: Arc::new(RwLock::new(weights)),
            sigma_points: Arc::new(RwLock::new(sigma_points)),
        };

        // IMU drift setup
        #[allow(non_snake_case)]
        let R_imu_vector: Vector7<f32> = Vector7::<f32>::from_row_slice(&config.estimators.R_imu);
        #[allow(non_snake_case)]
        let R_imu_0: Matrix7x7<f32> = Matrix7x7::<f32>::from_diagonal(&R_imu_vector);
        #[allow(non_snake_case)]
        let R_imu = Arc::new(RwLock::new(R_imu_0));
        let v_lin_imu = Arc::new(RwLock::new(Vector3::<f32>::zeros()));
        
        // Predict using approximated ship model ----------
        let ukf_data_clone = ukf_data.clone();
        let thruster_control_clone = thruster_control.clone();
        #[allow(non_snake_case)]
        thread::spawn(move || {
            // Initialize system ----------
            // Calculate interval for EKF Estimate publishing rate
            let dt = 1.0/config.estimators.estimator_pub_frequency; // [s]
            let interval = Duration::from_millis((dt * 1000.0) as u64); // [ms]

            // Get initial states
            let x_0: Vector12<f32> = Vector12::<f32>::from_row_slice(&config.ship.x_0);

            // Setup functions to linearize system later for any given work point
            let ode: ODE = ODE::new(
                x_0, 
                config.ship.mass,
                config.ship.dimensions,
                Vector3::<f32>::from_row_slice(&config.ship.thruster_placement),
                config.ship.velocity_linear_max,
                config.ship.velocity_angular_max,
            );

            // Get confidence matrix for our model
            let Q_vector: Vector12<f32> = Vector12::<f32>::from_row_slice(&config.estimators.Q);
            let Q: Matrix12x12<f32> = Matrix12x12::<f32>::from_diagonal(&Q_vector);

            loop {
                let start_t = Instant::now();

                // Get control input states
                let u_prev = *thruster_control_clone.read().unwrap();

                // Get UKF states
                let x_est_post_prev = *ukf_data_clone.x_est_post.read().unwrap();
                let P_post_prev = *ukf_data_clone.P_post.read().unwrap();
                let lambda = *ukf_data_clone.lambda.read().unwrap();
                let weights = *ukf_data_clone.weights.read().unwrap();
                
                // Predict
                let (
                    mut x_est_priori,
                    P_priori
                ) = ukf::predict(
                    |x, u| ode.f(x, u),
                    dt, 
                    x_est_post_prev, 
                    u_prev, 
                    P_post_prev,
                    Q,
                    lambda,
                    weights,
                );

                x_est_priori = limit_states(x_est_priori);

                // Update UKF states
                {
                    let mut x_est_pri = ukf_data_clone.x_est_pri.write().unwrap();
                    *x_est_pri = x_est_priori;
                }
                {
                    let mut P_pri = ukf_data_clone.P_pri.write().unwrap();
                    *P_pri = P_priori;
                }

                // Publish UKF data
                let ukf_data: TOPICS::ukf::DataType = x_est_priori;
                let ukf_data_json = udp_utils::encode_json(&ukf_data);
                udp_utils::publish(TOPICS::ukf::PORT, ukf_data_json.as_bytes()).expect("Failed to publish UKF estimate data");

                // Precise way to calculate interval
                // This way we estimate at exactly the desired frequency
                let elapsed_t = start_t.elapsed();
                if elapsed_t < interval {
                    thread::sleep(interval - elapsed_t);
                }
            }
        });
        
        /*
        // Correct using GNSS ----------
        let ukf_data_clone = ukf_data.clone();
        #[allow(non_snake_case)]
        let R_imu_clone = R_imu.clone();
        let v_lin_imu_clone = v_lin_imu.clone();
        #[allow(non_snake_case)]
        thread::spawn(move || {
            // Initialize system ----------
            // Get confidence matrix for our measurements
            let R_vector: Vector10<f32> = Vector10::<f32>::from_row_slice(&config.estimators.R_gnss);
            let R: Matrix10x10<f32> = Matrix10x10::<f32>::from_diagonal(&R_vector);

            // Unwrapped yaw follower
            let mut yaw_gnss = 0.0;

            loop {
                // Wait for GNSS data
                let msg = udp_utils::subscribe(TOPICS::gnss::PORT).unwrap();
                let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
                let gnss: TOPICS::gnss::DataType = udp_utils::decode_json(json_str);

                // UKF States
                let x_est_pri = *ukf_data_clone.x_est_pri.read().unwrap();
                let P_pri = *ukf_data_clone.P_pri.read().unwrap();
                let lambda = *ukf_data_clone.lambda.read().unwrap();
                let weights = *ukf_data_clone.weights.read().unwrap();

                // Calculate angle between GNSS Antennas and unwrap them
                let antenna1_pos: Vector3<f32> = gnss.fixed_rows::<3>(0).into(); // [x, y, z]
                let antenna2_pos: Vector3<f32> = gnss.fixed_rows::<3>(3).into(); // [x, y, z]
                let antenna_angle = angle_between_points(antenna1_pos, antenna2_pos);
                yaw_gnss = unwrap_angle(yaw_gnss, antenna_angle);

                // For GNSS measurements we don't need to rotate or adjust, the values are already given properly and absolute
                let yaw_gnss_adj = yaw_gnss;

                // Build measurement vector 
                let mut z = Vector10::<f32>::zeros();
                z.fixed_rows_mut::<9>(0).copy_from(&gnss); // Antenna 1 and 2 positions + Ships Velocity
                z[9] = yaw_gnss_adj;

                // Correction
                let (
                    mut x_est_posterior,
                    P_posterior,
                ) = ukf::correct(
                    |x| h_gnss(
                        x.clone(),
                        Vector3::from(config.sensors.gnss_antenna1_placement),
                        Vector3::from(config.sensors.gnss_antenna2_placement),
                    ),
                    z,
                    x_est_pri,
                    P_pri,
                    R,
                    lambda,
                    weights,
                );

                x_est_posterior = limit_states(x_est_posterior);

                // ! DEBUGGING
                print_matrix("P_posterior positive", &P_posterior);

                // In addition we must update IMU data to absolute certainty values from GNSS where it applies
                // Mainly to linear velocity IMU integral and drift to be reset
                {
                    // Convert measured GNSS velocity to IMU velocity integral
                    let z_imu = h_imu(
                        x_est_posterior.clone(),
                        Vector3::from_column_slice(&config.sensors.imu_placement),
                        Vector3::from_column_slice(&config.sensors.imu_rotation),
                    );

                    // Invert linear velocity because opposite direction 
                    let z_imu_v = z_imu.fixed_rows::<3>(0).into_owned();

                    let mut v_lin_imu_write = v_lin_imu_clone.write().unwrap();
                    *v_lin_imu_write = z_imu_v;
                }
                {
                    let mut R_imu_write = R_imu_clone.write().unwrap();
                    *R_imu_write = R_imu_0;
                }

                // Update UKF states
                {
                    let mut x_est_post = ukf_data_clone.x_est_post.write().unwrap();
                    *x_est_post = x_est_posterior;
                }
                {
                    let mut P_post = ukf_data_clone.P_post.write().unwrap();
                    *P_post = P_posterior;
                }
            }
        });
        */

        // Correction using IMU ----------
        let ukf_data_clone = ukf_data.clone();
        #[allow(non_snake_case)]
        let R_imu_clone = R_imu.clone();
        let v_lin_imu_clone = v_lin_imu.clone();
        #[allow(non_snake_case)]
        thread::spawn(move || {
            // Initialize system ----------
            // Integration states
            let mut last_time = Instant::now();

            // Unwrapped yaw follower
            let mut mag_imu = 0.0;

            // Get the IMU drift
            let mut imu_drift_factor: Vector7::<f32> = Vector7::<f32>::from_row_slice(&config.estimators.imu_drift);
            imu_drift_factor += Vector7::<f32>::from_element(1.0);
            let imu_drift_matrix: Matrix7x7<f32> = Matrix7x7::from_diagonal(&imu_drift_factor);
            
            loop {
                // Wait for IMU data
                let msg = udp_utils::subscribe(TOPICS::imu::PORT).unwrap();
                let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
                let imu: TOPICS::imu::DataType = udp_utils::decode_json(json_str);
                
                // UKF States
                let x_est_pri = *ukf_data_clone.x_est_pri.read().unwrap();
                let P_pri = *ukf_data_clone.P_pri.read().unwrap();
                let lambda = *ukf_data_clone.lambda.read().unwrap();
                let weights = *ukf_data_clone.weights.read().unwrap();

                // Integrate acceleration to velocity
                let dt = last_time.elapsed().as_secs_f32();
                last_time = Instant::now();
                
                let mut accel = imu.fixed_rows::<3>(0).into_owned();
                accel[2] -= 9.81; // Subtract the constant acceleration from earths gravity

                let mut v_lin_imu = *v_lin_imu_clone.read().unwrap();
                v_lin_imu += dt * accel;

                // unwrap IMU magnetometer angle to be continuous instead of -pi to pi
                mag_imu = unwrap_angle_inv(mag_imu, imu[6]);

                // For measurements we need to rotate magnetometer values to proper position
                let mag_imu_adj = mag_imu + config.sensors.imu_rotation[2] * 2.0 - PI/2.0;

                // Add drift to the measurement noise matrix
                let mut R: Matrix7x7<f32> = *R_imu_clone.read().unwrap();
                R = R.component_mul(&imu_drift_matrix);

                // Update IMU velocity and drift matrix
                {
                    let mut v_lin_imu_write = v_lin_imu_clone.write().unwrap();
                    *v_lin_imu_write = v_lin_imu;
                }
                {
                    let mut R_imu_write = R_imu_clone.write().unwrap();
                    *R_imu_write = R;
                }
                
                // Build measurement vector 
                let mut z = Vector7::<f32>::zeros();
                z.fixed_rows_mut::<3>(0).copy_from(&v_lin_imu); // velocity
                z.fixed_rows_mut::<3>(3).copy_from(&imu.fixed_rows::<3>(3)); // gyro
                z[6] = mag_imu_adj;

                // Correction
                let (
                    mut x_est_posterior,
                    P_posterior,
                ) = ukf::correct(
                    |x| h_imu(
                        x.clone(),
                        Vector3::from_column_slice(&config.sensors.imu_placement),
                        Vector3::from_column_slice(&config.sensors.imu_rotation),
                    ),
                    z,
                    x_est_pri,
                    P_pri,
                    R,
                    lambda,
                    weights,
                );

                x_est_posterior = limit_states(x_est_posterior);

                // Update UKF states
                {
                    let mut x_est_post = ukf_data_clone.x_est_post.write().unwrap();
                    *x_est_post = x_est_posterior;
                }
                {
                    let mut P_post = ukf_data_clone.P_post.write().unwrap();
                    *P_post = P_posterior;
                }
            }
        });
    }
    // Unscented Kalman Filter (STOP) ==================================================



    // Idle (START) ==================================================
    // Ensures we continue multithreading
    loop {
        thread::sleep(Duration::from_secs(1));
    }
    // Idle (STOP) ==================================================
}