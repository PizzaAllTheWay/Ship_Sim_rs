// Custom libraries
use ship_sim_lib::estimators::ship_approx;
use ship_sim_lib::estimators::estimators_utils::{
    numerical_jacobian,
    print_matrix,
    clamp_matrix,
};
use ship_sim_lib::estimators::kf::{self, Vector9, Vector12, Matrix9x9, Matrix12x12};
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
use nalgebra::{Vector3, Vector6};



// Config data structure ----------
#[derive(Deserialize)]
struct ShipConfig {
    mass: f32,
    dimensions: [f32; 2],
    x_0: [f32; 12],
    u_0: [f32; 6],
}

#[derive(Deserialize)]
struct SensorsConfig {
    gnss_antenna1_placement: [f32; 3],
    gnss_antenna2_placement: [f32; 3],
}

#[allow(non_snake_case)]
#[derive(Deserialize)]
struct EstimatorsConfig {
    kf_pub_frequency: f32,
    P_0: [f32; 12],
    Q: [f32; 12],
    R_gnss: [f32; 9],
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

    ) -> Self {
        // Initialize ship dynamics        
        let ship_dynamic = ship_approx::ShipDynamics::new(
            ship_mass,
            ship_dimensions,
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
        u: &Vector6<f32>,
    ) -> Vector12<f32> {
        // Constant vectors
        let gravity_w = Vector3::new(0.0, 0.0, -9.81);

        // Split up states into manageable subparts
        let v_lin_w: Vector3<f32> = x.fixed_rows::<3>(0).into(); // [vx, vy, vz]
        let v_ang_w: Vector3<f32> = x.fixed_rows::<3>(3).into(); // [angular velocity in roll, pitch, yaw]
        let r_lin_w: Vector3<f32> = x.fixed_rows::<3>(6).into(); // [x, y, z]
        let r_ang_w: Vector3<f32> = x.fixed_rows::<3>(9).into(); // [roll, pitch, yaw]

        let force_b: Vector3<f32> = u.fixed_rows::<3>(0).into();  // [Fx, Fy, Fz]
        let torque_b: Vector3<f32> = u.fixed_rows::<3>(3).into(); // [Torque in roll, pitch, yaw]

        // Inverse kinematics
        let v_lin_b = kinematics::linear_velocity_world_to_body(r_ang_w, v_lin_w);
        let v_ang_b = kinematics::angular_velocity_world_to_body(r_ang_w, v_ang_w);
        
        let gravity_b = kinematics::linear_accel_world_to_body(r_ang_w, gravity_w);

        // Dynamics
        let (a_lin_b, a_ang_b) = self.ship_dynamic.calc_accel_body(
            force_b,
            torque_b,
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



// Transformation functions for sensors ----------
// measurements from estimate (~z) = h(x)*x
// estimate from measurements (~x) = h(x)⁽⁻¹⁾*z
fn h_gnss(
    x: Vector12<f32>, // Estimated states
    antenna1_placement: Vector3<f32>, // Antenna placement in body frame
    antenna2_placement: Vector3<f32>, // Antenna placement in body frame
) -> Vector9<f32> {
    // Extract states
    let v_lin_w: Vector3<f32> = x.fixed_rows::<3>(0).into(); // velocity in world
    let r_lin_w: Vector3<f32> = x.fixed_rows::<3>(6).into(); // position in world
    let r_ang_w: Vector3<f32> = x.fixed_rows::<3>(9).into(); // orientation in world

    // Kinematics
    let r_antenna1_w: Vector3<f32> = r_lin_w + kinematics::r_body_to_world(r_ang_w) * antenna1_placement;
    let r_antenna2_w: Vector3<f32> = r_lin_w + kinematics::r_body_to_world(r_ang_w) * antenna2_placement;

    // Save transformed vector
    let mut y = Vector9::<f32>::zeros();
    y.fixed_rows_mut::<3>(0).copy_from(&r_antenna1_w);
    y.fixed_rows_mut::<3>(3).copy_from(&r_antenna2_w);
    y.fixed_rows_mut::<3>(6).copy_from(&v_lin_w);

    // Return transformed vector
    return y;
}



fn main() {
    // Setup (START) ==================================================
    // Get config file
    let config_str = fs::read_to_string("config.toml").expect("Failed to read config file");
    let config: Config = toml::from_str(&config_str).expect("Failed to parse TOML config");

    // Create shared resources for estimators
    let forces_thruster= Arc::new(RwLock::new(TOPICS::forces_thrusters::DataType::zeros()));
    // Setup (STOP) ==================================================



    // GET - Control Forces (START) ==================================================
    let forces_thruster_clone = forces_thruster.clone();
    thread::spawn(move || {
        loop {
            // Wait for thruster forces data to arrive from gui
            // Once received format to correct datatype
            let msg = udp_utils::subscribe(TOPICS::forces_thrusters::PORT).expect("Failed to get forces data");
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let forces: TOPICS::forces_thrusters::DataType = udp_utils::decode_json(json_str);

            // Save thruster forces in shared resource for simulator
            let mut forces_thruster = forces_thruster_clone.write().unwrap();
            *forces_thruster = forces;
        }
    });
    // GET - Control Forces (STOP) ==================================================



    // Kalman Filter (START) ==================================================
    // Shared resources for KF ----------
    let kf = kf::SharedState::default();

    // Estimate using linearized state space model ----------
    let kf_clone = kf.clone();
    let forces_thruster_clone = forces_thruster.clone();
    #[allow(non_snake_case)]
    thread::spawn(move || {
        // Initialize system ----------
        // Calculate interval for KF Estimate publishing rate
        let dt = 1.0/config.estimators.kf_pub_frequency; // [s]
        let interval = Duration::from_millis((dt * 1000.0) as u64); // [ms]

        // Get initial states
        let x_0: Vector12<f32> = Vector12::<f32>::from_row_slice(&config.ship.x_0);
        let u_0: Vector6<f32> = Vector6::<f32>::from_row_slice(&config.ship.u_0);

        // Get linearized system matrixes
        // System matrixes are just system change rate:
        // A: How state affects state change
        // B: How input affects state change
        // These can be found by calculating jacobian of the system:
        // A = df/dx
        // B = df/du
        // Since we have a Normal KF, A and B stay stationary around one work point
        // Linearized around x0 and u0 
        let ode: ODE = ODE::new(
            x_0, 
            config.ship.mass,
            config.ship.dimensions,
        );

        let x_ref = x_0.clone();
        let u_ref = u_0.clone();
        let f_x = |x: &Vector12<f32>| ode.f(x, &u_ref);
        let f_u = |u: &Vector6<f32>| ode.f(&x_ref, u);
        let A = numerical_jacobian(&f_x, &x_0, 1e-4);
        let B = numerical_jacobian(&f_u, &u_0, 1e-4);

        println!("#====================================================================================================#");
        println!("Kalman Filter:");
        println!("Linearized state space matrices of approximated non linear ship model f(x, u)");
        print_matrix("A", &A);
        print_matrix("B", &B);
        println!("#====================================================================================================#");
        println!();

        // Get initial uncertainty matrix of the estimate
        let P_0_vector: Vector12<f32> = Vector12::<f32>::from_row_slice(&config.estimators.P_0);
        let P_0: Matrix12x12<f32> = Matrix12x12::<f32>::from_diagonal(&P_0_vector);

        // Get confidence matrix for our model
        let Q_vector: Vector12<f32> = Vector12::<f32>::from_row_slice(&config.estimators.Q);
        let Q: Matrix12x12<f32> = Matrix12x12::<f32>::from_diagonal(&Q_vector);

        // Update KF states ----------
        {
            let mut x_est = kf_clone.x_est.write().unwrap();
            *x_est = x_0;
        }
        {
            let mut P = kf_clone.P.write().unwrap();
            *P = P_0;
        }

        loop {
            let start_t = Instant::now();

            // Get control input states
            let u_prev = *forces_thruster_clone.read().unwrap();

            // Get KF states
            let x_est_prev = *kf_clone.x_est.read().unwrap();
            let P_prev = *kf_clone.P.read().unwrap();
            
            // Estimate
            let (x_est_priori, P_priori) = kf::estimate(
                dt, 
                x_est_prev, 
                u_prev, 
                P_prev, 
                A, 
                B, 
                Q
            );

            // Clam P because it has a tendency to blow up in certain fields
            let P_priori = clamp_matrix(P_priori, 1e6);

            // Update KF states
            {
                let mut x_est = kf_clone.x_est.write().unwrap();
                *x_est = x_est_priori;
            }
            {
                let mut P = kf_clone.P.write().unwrap();
                *P = P_priori;
            }

            // Publish KF data
            // let kf_data = TOPICS::kf::DataType {
            //     x_est: x_est_priori,
            //     // P: Some(P_priori),
            //     // y_gnss: None,
            //     // S_gnss: None,
            //     // K_gnss: None,
            // };

            let kf_data: TOPICS::kf::DataType = x_est_priori;
            let kf_data_json = udp_utils::encode_json(&kf_data);
            udp_utils::publish(TOPICS::kf::PORT, kf_data_json.as_bytes()).expect("Failed to publish x estimate data");

            // Precise way to calculate interval
            // This way we estimate at exactly the desired frequency
            let elapsed_t = start_t.elapsed();
            if elapsed_t < interval {
                thread::sleep(interval - elapsed_t);
            }
        }
    });

    // Correction using GNSS ----------
    /*
    let kf_clone = kf.clone();
    #[allow(non_snake_case)]
    thread::spawn(move || {
        // Initialize system ----------
        // Get initial states
        let x_0: Vector12<f32> = Vector12::<f32>::from_row_slice(&config.ship.x_0);

        // Linearize measurement matrix
        // In order to compare measurements with estimates, we must first transform estimates to reflect measurement space
        // We do that by running h(x) to get estimates in measurement space
        // However h(x) is highly non linear because of all the transforms
        // For normal KF we must get H matrix
        // We do that by linearizing h(x) with respect to x
        // H = dh/dx = Jacobian(h(x), x)
        let h_fn = |x: &Vector12<f32>| {
            h_gnss(x.clone(), // pass ownership
                   Vector3::from(config.sensors.gnss_antenna1_placement),
                   Vector3::from(config.sensors.gnss_antenna2_placement))
        };
        let H = numerical_jacobian(&h_fn, &x_0, 1e-4);

        println!("#====================================================================================================#");
        println!("Kalman Filter:");
        println!("Linearized measurement state matrix of approximated non linear measurement transform h(x)");
        print_matrix("H", &H);
        println!("#====================================================================================================#");
        println!();

        // Get confidence matrix for our measurements
        let R_vector: Vector9<f32> = Vector9::<f32>::from_row_slice(&config.estimators.R_gnss);
        let R: Matrix9x9<f32> = Matrix9x9::<f32>::from_diagonal(&R_vector);

        loop {
            // Wait for GNSS data
            let msg = udp_utils::subscribe(TOPICS::gnss::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let gnss: TOPICS::gnss::DataType = udp_utils::decode_json(json_str);

            // KF States
            let z = gnss;
            let x_est_priori = *kf_clone.x_est.read().unwrap();
            let P_priori = *kf_clone.P.read().unwrap();

            // Correction
            let (
                x_est,
                P,
                y,
                S,
                K,
            ) = kf::correct(
                z,
                x_est_priori,
                P_priori,
                H,
                R,
            );

            // Clam P because it has a tendency to blow up in certain fields
            let P = clamp_matrix(P, 1e6);
            let K = clamp_matrix(K, 1e6);

            // !println!("x_est: {:?}", x_est);

            // Update KF states
            {
                let mut x_est_priori = kf_clone.x_est.write().unwrap();
                *x_est_priori = x_est;
            }
            {
                let mut P_priori = kf_clone.P.write().unwrap();
                *P_priori = P;
            }

            // Publish KF data
            // let kf_data = TOPICS::kf::DataType {
            //     x_est: x_est,
            //     // P: Some(P),
            //     // y_gnss: Some(y),
            //     // S_gnss: Some(S),
            //     // K_gnss: Some(K),
            // };
            let kf_data: TOPICS::kf::DataType = x_est;
            let kf_data_json = udp_utils::encode_json(&kf_data);
            udp_utils::publish(TOPICS::kf::PORT, kf_data_json.as_bytes()).expect("Failed to publish x estimate data");
        }
    });
    */

    // Correction using IMU ----------
    thread::spawn(move || {
        loop {
            // Wait for IMU data
            let msg = udp_utils::subscribe(TOPICS::imu::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let imu: TOPICS::imu::DataType = udp_utils::decode_json(json_str);

            // Correction
        }
    });
    // Kalman Filter (STOP) ==================================================



    // Idle (START) ==================================================
    // Ensures we continue multithreading
    loop {
        thread::sleep(Duration::from_secs(1));
    }
    // Idle (STOP) ==================================================
}