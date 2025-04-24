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

    imu_placement: [f32; 3],
    imu_rotation: [f32; 3],
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
            // Get current world state
            let x = *x_clone.read().unwrap();

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
            let antenna1_placement_in_ship = Vector3::from(config.sensors.gnss_antenna1_placement);
            let antenna2_placement_in_ship = Vector3::from(config.sensors.gnss_antenna2_placement);
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
            let r_lin_antenna1_w: Vector3<f32> = kinematics::rot_body_to_world(euler) * r_lin_antenna1_b + r_lin_w;
            let r_lin_antenna2_w: Vector3<f32> = kinematics::rot_body_to_world(euler) * r_lin_antenna2_b + r_lin_w;
            let v_lin_antenna1_w: Vector3<f32> = kinematics::linear_velocity_body_to_world(euler, v_lin_antenna1_b);
            let v_lin_antenna2_w: Vector3<f32> = kinematics::linear_velocity_body_to_world(euler, v_lin_antenna2_b);

            // Simulate GNSS
            let (
                gnss_antenna1,
                gnss_antenna2,
                gnss_speed,
            ) = gnss::simulate(
                r_lin_antenna1_w,
                r_lin_antenna2_w,
                config.sensors.gnss_position_noise_horizontal, 
                config.sensors.gnss_position_accuracy_horizontal,
                config.sensors.gnss_position_noise_vertical, 
                config.sensors.gnss_position_accuracy_vertical,

                v_lin_antenna1_w,
                v_lin_antenna2_w,
                config.sensors.gnss_velocity_noise_horizontal, 
                config.sensors.gnss_velocity_accuracy_horizontal,
                config.sensors.gnss_velocity_noise_vertical, 
                config.sensors.gnss_velocity_accuracy_vertical,
            );

            // Publish data
            let mut gnss: TOPICS::gnss::DataType = TOPICS::gnss::DataType::zeros();
            gnss[0] = gnss_antenna1[0];
            gnss[1] = gnss_antenna1[1];
            gnss[2] = gnss_antenna1[2];
            gnss[3] = gnss_antenna2[0];
            gnss[4] = gnss_antenna2[1];
            gnss[5] = gnss_antenna2[2];
            gnss[6] = gnss_speed[0];
            gnss[7] = gnss_speed[1];
            gnss[8] = gnss_speed[2];

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

            // Get current world state
            let x = *x_clone.read().unwrap();
            let dx = *dx_clone.read().unwrap();

            // Get constants
            let g: Vector3<f32> = Vector3::<f32>::new(0.0, 0.0, 9.81);
            let mag_north_w: Vector3<f32> = Vector3::new(0.0, 1.0, 0.0);
            
            // Split up states into manageable subparts
            let mut a_lin_w: Vector3<f32> = dx.fixed_rows::<3>(0).into(); // [ax, ay, az]
            a_lin_w += g; // Add gravity
            let euler_dot_dot: Vector3<f32> = dx.fixed_rows::<3>(3).into(); // [angular acceleration in roll, pitch, yaw]
            let euler_dot: Vector3<f32> = x.fixed_rows::<3>(3).into(); // [angular velocity in roll, pitch, yaw]
            let euler: Vector3<f32> = x.fixed_rows::<3>(9).into(); // [roll, pitch, yaw]

            // Convert Ship to body frame
            let v_ang_ship: Vector3<f32> = kinematics::euler_dot_to_angular_velocity_body(euler, euler_dot);
            let a_ang_ship: Vector3<f32> = kinematics::euler_dot_dot_to_angular_accel_body(euler, euler_dot, euler_dot_dot);
            let a_lin_ship: Vector3<f32> = kinematics::linear_accel_world_to_body(euler, a_lin_w);
            let mag_north_ship: Vector3<f32> = kinematics::rot_world_to_body(euler) * mag_north_w;

            // Convert Ship to object frame
            let imu_acceleration_in_ship = Vector3::<f32>::zeros(); // IMU sits tight, no linear acceleration in the ship
            let imu_placement_in_ship = Vector3::from(config.sensors.imu_placement); // IMU placement relative to ship
            let imu_velocity_in_ship = Vector3::<f32>::zeros(); // IMU sits tight, no linear velocity in the ship
            let mut a_lin_imu: Vector3<f32> = kinematics::linear_accel_object(
                a_lin_ship, 
                imu_acceleration_in_ship, 
                a_ang_ship, 
                imu_placement_in_ship, 
                v_ang_ship, 
                imu_velocity_in_ship,
            );
            let imu_rotation = Vector3::from(config.sensors.imu_rotation); // IMU Rotation relative to ist own internal frame
            a_lin_imu = kinematics::rot_body_to_object(imu_rotation) * a_lin_imu; // Need to rotate to internal frame else we get wrong acceleration frame

            let imu_v_ang_in_ship = Vector3::<f32>::zeros(); // IMU sits tight, no angular velocity in the ship
            let v_ang_imu: Vector3<f32> = kinematics::angular_velocity_body_to_object(imu_v_ang_in_ship, v_ang_ship);

            let mag_imu_vec = kinematics::rot_object_to_body(imu_rotation) * mag_north_ship;
            let mag_imu = mag_imu_vec.y.atan2(mag_imu_vec.x); // extract heading in world frame

            // Simulate gnss
            let (imu_accel, imu_gyro, imu_mag) = imu::simulate(
                a_lin_imu,
                v_ang_imu,
                mag_imu,

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