// Custom libraries
use ship_sim_lib::estimators::ship_approx;
use ship_sim_lib::estimators::ekf;
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

// Library for linear algebra
use nalgebra::{
    Vector2, Vector3, Vector6, SVector,
    SMatrix
};



// System State data structure ----------
pub type Vector7<T> = SVector<T, 7>;
pub type Vector9<T> = SVector<T, 9>;
pub type Vector12<T> = SVector<T, 12>;

pub type Matrix7x7<T> = SMatrix::<T, 7, 7>;
pub type Matrix9x9<T> = SMatrix::<T, 9, 9>;
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
    imu_placement: [f32; 6],
}

#[allow(non_snake_case)]
#[derive(Deserialize)]
struct EstimatorsConfig {
    kf_pub_frequency: f32,
    P_0: [f32; 12],
    Q: [f32; 12],
    R_gnss: [f32; 9],
    R_imu: [f32; 7],
    imu_drift: [f32; 7],
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
        let v_ang_w: Vector3<f32> = x.fixed_rows::<3>(3).into(); // [angular velocity in roll, pitch, yaw]
        let r_lin_w: Vector3<f32> = x.fixed_rows::<3>(6).into(); // [x, y, z]
        let r_ang_w: Vector3<f32> = x.fixed_rows::<3>(9).into(); // [roll, pitch, yaw]

        let thruster_rpm: f32 = u[0];
        let thruster_angle: f32 = u[1];

        // Inverse kinematics
        let v_lin_b = kinematics::linear_velocity_world_to_body(r_ang_w, v_lin_w);
        let v_ang_b = kinematics::angular_velocity_world_to_body(r_ang_w, v_ang_w);
        
        let gravity_b = kinematics::linear_accel_world_to_body(r_ang_w, gravity_w);

        // Dynamics
        let (a_lin_b, a_ang_b) = self.ship_dynamic.calc_accel_body(
            thruster_rpm,
            thruster_angle,
            v_lin_b,
            v_ang_b,
            gravity_b,
            r_lin_w,
        );

        // Kinematics
        let a_lin_w = kinematics::linear_accel_body_to_world(r_ang_w, a_lin_b,);
        let a_ang_w = kinematics::angular_accel_body_to_world(r_ang_w, a_ang_b);

        // Structure return states properly
        let mut x_dot: Vector12<f32> = Vector12::<f32>::zeros();
        x_dot.fixed_rows_mut::<3>(0).copy_from(&a_lin_w); // [ax, ay, az]
        x_dot.fixed_rows_mut::<3>(3).copy_from(&a_ang_w); // [angular acceleration in roll, pitch, yaw]
        x_dot.fixed_rows_mut::<3>(6).copy_from(&v_lin_w); // [vx, vy, vz]
        x_dot.fixed_rows_mut::<3>(9).copy_from(&v_ang_w); // [angular velocity in roll, pitch, yaw]
        
        return x_dot;
    }
}



// Measurement Transformation functions for sensors ----------
// estimated measurement: (~z) = h(x)   // transform state -> measurement space
// measured estimate:     (~x) = h⁻¹(z) // transform measurement -> state space
fn h_gnss(
    x: Vector12<f32>,                 // Estimated states
    antenna1_placement: Vector3<f32>, // Antenna placement in body frame
    antenna2_placement: Vector3<f32>, // Antenna placement in body frame
) -> Vector9<f32> {
    // Extract states
    let v_lin_w: Vector3<f32> = x.fixed_rows::<3>(0).into(); // velocity in world
    let r_lin_w: Vector3<f32> = x.fixed_rows::<3>(6).into(); // position in world
    let r_ang_w: Vector3<f32> = x.fixed_rows::<3>(9).into(); // orientation in world

    // Inverse Kinematics
    let r_lin_b: Vector3<f32> = kinematics::r_world_to_body(r_ang_w) * r_lin_w;

    // Kinematics
    let r_antenna1_b: Vector3<f32> = r_lin_b + antenna1_placement;
    let r_antenna2_b: Vector3<f32> = r_lin_b + antenna2_placement;
    let r_antenna1_w = kinematics::r_body_to_world(r_ang_w) * r_antenna1_b;
    let r_antenna2_w = kinematics::r_body_to_world(r_ang_w) * r_antenna2_b;

    // Save transformed vector
    let mut y = Vector9::<f32>::zeros();
    y.fixed_rows_mut::<3>(0).copy_from(&r_antenna1_w);
    y.fixed_rows_mut::<3>(3).copy_from(&r_antenna2_w);
    y.fixed_rows_mut::<3>(6).copy_from(&v_lin_w);

    y
}

fn h_imu(
    x: Vector12<f32>,            // Estimated full state vector
    imu_placement: Vector6<f32>, // IMU placement: [x, y, z, roll, pitch, yaw] in body frame
) -> Vector7<f32> {
    // Extract states
    let v_lin_w: Vector3<f32> = x.fixed_rows::<3>(0).into();   // linear velocity in world frame
    let v_ang_w: Vector3<f32> = x.fixed_rows::<3>(3).into();   // angular velocity in world frame
    let r_ang_w: Vector3<f32> = x.fixed_rows::<3>(9).into();   // orientation (roll, pitch, yaw)

    // IMU placement
    let imu_pos_b = Vector3::new(imu_placement[0], imu_placement[1], imu_placement[2]);
    let imu_rot_b = Vector3::new(imu_placement[3], imu_placement[4], imu_placement[5]);

    // Rotation matrices
    let r_world_to_body = kinematics::r_world_to_body(r_ang_w);
    let r_body_to_imu = kinematics::r_body_to_object(imu_rot_b);

    // Linear velocity in body frame
    let v_lin_b = r_world_to_body * v_lin_w;

    // Linear velocity in IMU frame
    let v_lin_imu = r_body_to_imu * (v_lin_b + v_ang_w.cross(&imu_pos_b));

    // Angular velocity in IMU frame
    let gyro = r_body_to_imu * kinematics::angular_velocity_world_to_body(r_ang_w, v_ang_w);

    // Yaw (magnetometer proxy)
    let mag = r_ang_w[2];

    // Pack it
    let mut y = Vector7::<f32>::zeros();
    y.fixed_rows_mut::<3>(0).copy_from(&v_lin_imu); // velocity
    y.fixed_rows_mut::<3>(3).copy_from(&gyro);      // gyro
    y[6] = mag;                                                      // yaw angle

    y
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
        let dt = 1.0/config.estimators.kf_pub_frequency; // [s]
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
            let (mut x_est_priori, P_priori) = ekf::predict(
                |x, u| ode.f(x, u),
                dt, 
                x_est_post_prev, 
                u_prev, 
                P_post_prev,
                Q,
            );

            // Remove roll and pitch position and velocity
            // This is because we estimate ship states, roll and pitch states are irrelevant for estimating ships position and velocity
            // If roll and pitch were anything than 0 the ship would be rolled over game over
            // Also because of euler angles if these come close to +- pi/2 it will get a singularity
            x_est_priori[3] = 0.0; // Roll velocity
            x_est_priori[4] = 0.0; // Pitch velocity
            x_est_priori[9] = 0.0; // Roll
            x_est_priori[10] = 0.0; // Pitch

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
            let kf_data: TOPICS::ekf::DataType = x_est_priori;
            let kf_data_json = udp_utils::encode_json(&kf_data);
            udp_utils::publish(TOPICS::ekf::PORT, kf_data_json.as_bytes()).expect("Failed to publish x estimate data");

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
        let R_vector: Vector9<f32> = Vector9::<f32>::from_row_slice(&config.estimators.R_gnss);
        let R: Matrix9x9<f32> = Matrix9x9::<f32>::from_diagonal(&R_vector);

        loop {
            // Wait for GNSS data
            let msg = udp_utils::subscribe(TOPICS::gnss::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let gnss: TOPICS::gnss::DataType = udp_utils::decode_json(json_str);

            // EKF States
            let z = gnss;
            let x_est_pri = *ekf_data_clone.x_est_pri.read().unwrap();
            let P_pri = *ekf_data_clone.P_pri.read().unwrap();

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

            // Remove roll and pitch position and velocity
            // This is because we estimate ship states, roll and pitch states are irrelevant for estimating ships position and velocity
            // If roll and pitch were anything than 0 the ship would be rolled over game over
            // Also because of euler angles if these come close to +- pi/2 it will get a singularity
            x_est_posterior[3] = 0.0; // Roll velocity
            x_est_posterior[4] = 0.0; // Pitch velocity
            x_est_posterior[9] = 0.0; // Roll
            x_est_posterior[10] = 0.0; // Pitch

            // In addition we must update IMU data to absolute certainty values from GNSS where it applies
            // Mainly to linear velocity IMU integral and drift can be reset
            {
                // Convert measured GNSS velocity to IMU velocity integral
                let z_imu = h_imu(
                    x_est_posterior.clone(),
                    Vector6::from_column_slice(&config.sensors.imu_placement),
                );

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

            // Add drift to the measurement noise matrix
            let mut R: Matrix7x7<f32> = *R_imu_clone.read().unwrap();
            R = R.component_mul(&imu_drift_matrix);

            // Update IMU drift matrix and velocity immediately 
            {
                let mut v_lin_imu_write = v_lin_imu_clone.write().unwrap();
                *v_lin_imu_write = v_lin_imu;
            } 
            {
                let mut R_imu_write = R_imu_clone.write().unwrap();
                *R_imu_write = R;
            }
            
            // Build vector measurement with integrated acceleration
            let mut z = Vector7::<f32>::zeros();
            z.fixed_rows_mut::<3>(0).copy_from(&v_lin_imu); // velocity
            z.fixed_rows_mut::<3>(3).copy_from(&imu.fixed_rows::<3>(3)); // gyro
            z[6] = imu[6]; // yaw

            // Correction
            let (
                mut x_est_posterior,
                P_posterior,
            ) = ekf::correct(
                |x| h_imu(
                    x.clone(),
                    Vector6::from_column_slice(&config.sensors.imu_placement),
                ),
                z,
                x_est_pri,
                P_pri,
                R,
            );

            // Remove roll and pitch position and velocity
            // This is because we estimate ship states, roll and pitch states are irrelevant for estimating ships position and velocity
            // If roll and pitch were anything than 0 the ship would be rolled over game over
            // Also because of euler angles if these come close to +- pi/2 it will get a singularity
            x_est_posterior[3] = 0.0; // Roll velocity
            x_est_posterior[4] = 0.0; // Pitch velocity
            x_est_posterior[9] = 0.0; // Roll
            x_est_posterior[10] = 0.0; // Pitch

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
    // Extended Kalman Filter (STOP) ==================================================



    // Idle (START) ==================================================
    // Ensures we continue multithreading
    loop {
        thread::sleep(Duration::from_secs(1));
    }
    // Idle (STOP) ==================================================
}