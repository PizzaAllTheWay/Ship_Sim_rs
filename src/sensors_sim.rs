// Custom libraries
use ship_sim_lib::comm::udp_utils;
use ship_sim_lib::comm::udp_topics::TOPICS;
use ship_sim_lib::simulation::kinematics;
use ship_sim_lib::models::gnss;
use ship_sim_lib::models::imu;

// Library for data formatting
use serde::Deserialize;
use std::fs;
use std::str;

// Libraries for multithreading
use std::thread;
use std::time::Duration;
use std::sync::{Arc, RwLock};

// Library for maths
use std::f32::consts::PI;
use nalgebra::Vector3;

// Library for randomness
use rand::Rng;



// Config data structure ----------
#[derive(Deserialize)]
struct SensorsConfig {
    gnss_pub_frequency: f32,
    gnss_pub_variance: f32,
    gnss_antenna1_placement: [f32; 3],
    gnss_antenna2_placement: [f32; 3],
    gnss_position_noise_horizontal: f32,
    gnss_position_accuracy_horizontal: f32,
    gnss_position_noise_vertical: f32,
    gnss_position_accuracy_vertical: f32,
    gnss_velocity_noise_horizontal: f32,
    gnss_velocity_accuracy_horizontal: f32,
    gnss_velocity_noise_vertical: f32,
    gnss_velocity_accuracy_vertical: f32,

    imu_placement: [f32; 6],
    imu_noise: f32,
    imu_accel_noise: f32,
    imu_gyro_noise: f32,
    imu_mag_noise: f32,
    imu_resolution: u32,
    imu_accel_fsr: f32,
    imu_gyro_fsr: f32,
    imu_mag_fsr: f32,
}

#[derive(Deserialize)]
struct Config {
    sensors: SensorsConfig,
}



fn main() {
    // Setup (START) ==================================================
    // Get config file
    let config_str = fs::read_to_string("config.toml").expect("Failed to read config file");
    let config: Config = toml::from_str(&config_str).expect("Failed to parse TOML config");

    // Create shared resources for GUI
    let x = Arc::new(RwLock::new(TOPICS::x::DataType::zeros()));
    let dx = Arc::new(RwLock::new(TOPICS::dx::DataType::zeros()));
    // Setup (STOP) ==================================================

    // GET - X States (START) ==================================================
    let x_clone = x.clone();
    thread::spawn(move || {
        loop {
            // Wait for state data from GUI
            let msg = udp_utils::subscribe(TOPICS::x::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let states: TOPICS::x::DataType = udp_utils::decode_json(json_str);

            // Save state data
            let mut x = x_clone.write().unwrap();
            *x = states;
        }
    });
    // GET - X States (STOP) ==================================================

    // GET - DX States (START) ==================================================
    let dx_clone = dx.clone();
    thread::spawn(move || {
        loop {
            // Wait for state data from GUI
            let msg = udp_utils::subscribe(TOPICS::dx::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let states: TOPICS::dx::DataType = udp_utils::decode_json(json_str);

            // Save state data
            let mut dx = dx_clone.write().unwrap();
            *dx = states;
        }
    });
    // GET - DX States (STOP) ==================================================



    // SEND - GNSS Data (START) ==================================================
    let x_clone = x.clone();
    thread::spawn(move || {
        let dt = 1.0/config.sensors.gnss_pub_frequency; // [s]
        
        loop {
            // Split up states into manageable subparts
            let x_w = *x_clone.read().unwrap();
            let v_lin_w: Vector3<f32> = x_w.fixed_rows::<3>(0).into(); // [vx, vy, vz]
            let r_lin_w: Vector3<f32> = x_w.fixed_rows::<3>(6).into(); // [x, y, z]
            let mut r_ang_w: Vector3<f32> = x_w.fixed_rows::<3>(9).into(); // [roll, pitch, yaw]

            // Reorient state to NED frame
            r_ang_w[0] += PI;

            // Inverse Kinematics
            let r_lin_b: Vector3<f32> = kinematics::r_world_to_body(r_ang_w) * r_lin_w;

            // Simulate gnss
            let (gnss_antenna1_b, gnss_antenna2_b, gnss_speed_w) = gnss::simulate(
                r_lin_b, 
                v_lin_w,

                Vector3::from(config.sensors.gnss_antenna1_placement),
                Vector3::from(config.sensors.gnss_antenna2_placement),

                config.sensors.gnss_position_noise_horizontal, 
                config.sensors.gnss_position_accuracy_horizontal,
                config.sensors.gnss_position_noise_vertical, 
                config.sensors.gnss_position_accuracy_vertical,

                config.sensors.gnss_velocity_noise_horizontal, 
                config.sensors.gnss_velocity_accuracy_horizontal,
                config.sensors.gnss_velocity_noise_vertical, 
                config.sensors.gnss_velocity_accuracy_vertical,
            );

            // Kinematics
            let gnss_antenna1_w = kinematics::r_body_to_world(r_ang_w) * gnss_antenna1_b;
            let gnss_antenna2_w = kinematics::r_body_to_world(r_ang_w) * gnss_antenna2_b;

            // Publish data
            let mut gnss: TOPICS::gnss::DataType = TOPICS::gnss::DataType::zeros();
            gnss[0] = gnss_antenna1_w[0];
            gnss[1] = gnss_antenna1_w[1];
            gnss[2] = gnss_antenna1_w[2];
            gnss[3] = gnss_antenna2_w[0];
            gnss[4] = gnss_antenna2_w[1];
            gnss[5] = gnss_antenna2_w[2];
            gnss[6] = gnss_speed_w[0];
            gnss[7] = gnss_speed_w[1];
            gnss[8] = gnss_speed_w[2];

            let gnss_json = udp_utils::encode_json(&gnss);

            udp_utils::publish(TOPICS::gnss::PORT, gnss_json.as_bytes()).expect("Failed to publish antenna1 data");
            
            // Wait a bit before publishing with a bit of a random time delay
            // This simulates the random process of sending and receiving GNSS data with a time lag to make it more realistic
            let variance = config.sensors.gnss_pub_variance;
            let mut rng = rand::thread_rng();
            let jitter_factor: f32 = rng.gen_range(1.0 - variance..=1.0 + variance);
            let jittered_dt = dt * jitter_factor;
            let interval = Duration::from_millis((jittered_dt * 1000.0) as u64);
            thread::sleep(interval);
        }
    });
    // SEND - GNSS Data (STOP) ==================================================

    // SEND - IMU Data (START) ==================================================
    let x_clone = x.clone();
    let dx_clone = dx.clone();
    thread::spawn(move || {        
        loop {
            // Wait for simulation step to end and get that simulations step size
            // Only then proceed with simulating IMU for accurate acceleration
            let msg = udp_utils::subscribe(TOPICS::sim_dt::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let sim_dt: TOPICS::sim_dt::DataType = udp_utils::decode_json(json_str);

            // Split up states into manageable subparts
            let x_w = *x_clone.read().unwrap();
            let dx_w = *dx_clone.read().unwrap();
            let mut r_ang_w: Vector3<f32> = x_w.fixed_rows::<3>(9).into(); // [roll, pitch, yaw]
            let a_lin_w: Vector3<f32> = dx_w.fixed_rows::<3>(0).into(); // [ax, ay, az]
            let v_ang_w: Vector3<f32> = dx_w.fixed_rows::<3>(9).into(); // [angular velocity in roll, pitch, yaw]
            
            // Reorient state to NED frame
            r_ang_w[0] += PI;

            // Inverse Kinematics
            let a_lin_b: Vector3<f32> = kinematics::r_world_to_body(r_ang_w) * a_lin_w;
            let v_ang_b: Vector3<f32> = kinematics::angular_velocity_world_to_body(r_ang_w, v_ang_w);

            let imu_pos_b = Vector3::from_column_slice(&config.sensors.imu_placement[0..3]);
            let imu_rot_b = Vector3::from_column_slice(&config.sensors.imu_placement[3..6]);

            let r_body_to_imu = kinematics::r_body_to_object(imu_rot_b);
            let r_world_to_body = kinematics::r_world_to_body(r_ang_w); // ship's current rotation

            // Simulate gnss
            let (imu_accel, imu_gyro, imu_mag) = imu::simulate(
                a_lin_b,
                v_ang_b,
                r_ang_w[2],
                imu_pos_b,
                r_body_to_imu,
                r_world_to_body,

                config.sensors.imu_noise,
                config.sensors.imu_accel_noise,
                config.sensors.imu_gyro_noise,
                config.sensors.imu_mag_noise,
                sim_dt,

                config.sensors.imu_resolution,
                config.sensors.imu_accel_fsr,
                config.sensors.imu_gyro_fsr,
                config.sensors.imu_mag_fsr,
            );

            // Publish data
            let mut imu: TOPICS::imu::DataType = TOPICS::imu::DataType::zeros();
            imu.fixed_rows_mut::<3>(0).copy_from(&imu_accel); // [ax, ay, az]
            imu.fixed_rows_mut::<3>(3).copy_from(&imu_gyro);  // [gx, gy, gz]
            imu[6] = imu_mag; // Magnetic yaw (ψ)

            let imu_json = udp_utils::encode_json(&imu);

            udp_utils::publish(TOPICS::imu::PORT, imu_json.as_bytes()).expect("Failed to publish imu data");
            
            // Wait a bit before publishing
            // IMU has a consistent publishing rate so no need for variation in delay
            thread::sleep(Duration::from_millis((sim_dt * 1000.0) as u64));
        }
    });
    // SEND - IMU Data (STOP) ==================================================

    // Idle (START) ==================================================
    // Ensures we continue multithreading
    loop {
        thread::sleep(Duration::from_secs(1));
    }
    // Idle (STOP) ==================================================
}