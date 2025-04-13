// Custom libraries
use ship_sim_lib::comm::udp_utils;
use ship_sim_lib::comm::udp_topics::{self, TOPICS};
use ship_sim_lib::simulation::kinematics;
use ship_sim_lib::models::gnss;

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
struct SensorConfig {
    gnss_pub_frequency: f32,
    gnss_pub_variance: f32,
    antenna1_placement: [f32; 3],
    antenna2_placement: [f32; 3],
    gnss_noise: f32,
    gnss_accuracy: f32,
}

#[derive(Deserialize)]
struct Config {
    sensor: SensorConfig,
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
            let states: TOPICS::x::DataType = udp_topics::decode_json(json_str);

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
            let states: TOPICS::dx::DataType = udp_topics::decode_json(json_str);

            // Save state data
            let mut dx = dx_clone.write().unwrap();
            *dx = states;
        }
    });
    // GET - DX States (STOP) ==================================================

    // SEND - GNSS Data (START) ==================================================
    let x_clone = x.clone();
    thread::spawn(move || {
        let antenna1_placement = Vector3::from(config.sensor.antenna1_placement);
        let antenna2_placement = Vector3::from(config.sensor.antenna2_placement);

        let dt = 1.0/config.sensor.gnss_pub_frequency; // [s]
        
        loop {
            // Read the ground truth and wait a bit before publishing with a bit of a random time delay
            // This simulates the random process of sending and receiving GNSS data with a time lag to make it more realistic
            let x_w = *x_clone.read().unwrap();

            let variance = config.sensor.gnss_pub_variance;
            let mut rng = rand::thread_rng();
            let jitter_factor: f32 = rng.gen_range(1.0 - variance..=1.0 + variance);
            let jittered_dt = dt * jitter_factor;
            let interval = Duration::from_millis((jittered_dt * 1000.0) as u64);
            thread::sleep(interval);

            // Split up states into manageable subparts
            let r_lin_w: Vector3<f32> = x_w.fixed_rows::<3>(6).into(); // [x, y, z]
            let r_ang_w: Vector3<f32> = x_w.fixed_rows::<3>(9).into(); // [roll, pitch, yaw]

            // Inverse Kinematics
            let r_lin_b: Vector3<f32> = kinematics::r_world_to_body(r_ang_w) * r_lin_w;

            // Simulate gnss
            let (gnss_antenna1_b, gnss_antenna2_b) = gnss::simulate_gnss(
                r_lin_b, 
                antenna1_placement, 
                antenna2_placement, 
                config.sensor.gnss_noise, 
                config.sensor.gnss_accuracy,
            );

            // Kinematics
            let gnss_antenna1_w = kinematics::r_body_to_world(r_ang_w) * gnss_antenna1_b;
            let gnss_antenna2_w = kinematics::r_body_to_world(r_ang_w) * gnss_antenna2_b;

            // Publish data
            let gnss_antenna1: TOPICS::gnss_antenna1::DataType = gnss_antenna1_w.fixed_rows::<2>(0).into();
            let gnss_antenna2: TOPICS::gnss_antenna2::DataType = gnss_antenna2_w.fixed_rows::<2>(0).into();

            let gnss_antenna1_json = udp_topics::encode_json(&gnss_antenna1);
            let gnss_antenna2_json = udp_topics::encode_json(&gnss_antenna2);

            udp_utils::publish(TOPICS::gnss_antenna1::PORT, gnss_antenna1_json.as_bytes()).expect("Failed to publish antenna1 data");
            udp_utils::publish(TOPICS::gnss_antenna2::PORT, gnss_antenna2_json.as_bytes()).expect("Failed to publish antenna2 data");
        }
    });
    // SEND - GNSS Data (STOP) ==================================================

    // Idle (START) ==================================================
    // Ensures we continue multithreading
    loop {
        thread::sleep(Duration::from_secs(1));
    }
    // Idle (STOP) ==================================================
}