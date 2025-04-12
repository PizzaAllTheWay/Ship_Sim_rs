// Custom libraries
use ship_sim_lib::comm::udp_utils;
use ship_sim_lib::comm::udp_topics::{self, TOPICS};
use ship_sim_lib::models::wind;
use ship_sim_lib::models::current;

// Library for data formatting
use serde::Deserialize;
use std::fs;
use std::str;

// Libraries for multithreading
use std::thread;
use std::time::{Duration, Instant};
use std::sync::{Arc, RwLock};



// Config data structure ----------
#[derive(Deserialize)]
struct ExternalForceConfig {
    simulation_frequency: f32,
}

#[derive(Deserialize)]
struct InterfaceConfig {
    wind_speed_max: f32,
    current_speed_max: f32,
}

#[derive(Deserialize)]
struct Config {
    external_force: ExternalForceConfig,
    interface: InterfaceConfig,
}



fn main() {
    // Setup (START) ==================================================
    // Get config file
    let config_str = fs::read_to_string("config.toml").expect("Failed to read config file");
    let config: Config = toml::from_str(&config_str).expect("Failed to parse TOML config");

    // Create shared resources for GUI
    let wind_parameters = Arc::new(RwLock::new(TOPICS::wind_parameters::DataType::zeros()));
    let current_parameters = Arc::new(RwLock::new(TOPICS::current_parameters::DataType::zeros()));
    // Setup (STOP) ==================================================

    // GET - Wind Parameters (START) ==================================================
    let wind_parameters_clone = wind_parameters.clone();
    thread::spawn(move || {
        loop {
            // Wait for control data from GUI
            let msg = udp_utils::subscribe(TOPICS::wind_parameters::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let wind: TOPICS::wind_parameters::DataType = udp_topics::decode_json(json_str);

            // Save control data
            let mut wind_parameters = wind_parameters_clone.write().unwrap();
            *wind_parameters = wind;
        }
    });
    // GET - Wind Parameters (STOP) ==================================================

    // GET - Current Parameters (START) ==================================================
    let current_parameters_clone = current_parameters.clone();
    thread::spawn(move || {
        loop {
            // Wait for control data from GUI
            let msg = udp_utils::subscribe(TOPICS::current_parameters::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let current: TOPICS::current_parameters::DataType = udp_topics::decode_json(json_str);

            // Save control data
            let mut current_parameters = current_parameters_clone.write().unwrap();
            *current_parameters = current;
        }
    });
    // GET - Current Parameters (STOP) ==================================================

    // SEND - External Wind Forces (START) ==================================================
    let wind_parameters_clone = wind_parameters.clone();
    thread::spawn(move || {
        let mut wind_speed = TOPICS::wind_speed::DataType::zeros();

        let mut wind_state = wind::WindState {
            speed_noise: 0.0,
            angle_noise: 0.0,
        };     

        let dt = 1.0/config.external_force.simulation_frequency; // [s]
        let interval = Duration::from_millis((dt * 1000.0) as u64);

        loop {
            let start_t = Instant::now();

            // Get wind control parameters
            let wind_parameters = *wind_parameters_clone.read().unwrap();

            // Simulate wind
            let mut speed = wind_parameters[0];
            let mut angle_deg = wind_parameters[1];
            let noise = wind_parameters[2];
            (speed, angle_deg) = wind::simulate(
                speed,
                angle_deg,
                noise,
                config.interface.wind_speed_max,
                &mut wind_state,
            );

            // Convert simulated data to 3D vector
            let angle_rad = angle_deg.to_radians();
            wind_speed.x = speed * angle_rad.cos();
            wind_speed.y = -speed * angle_rad.sin(); // -1 because mirrored orientation in GUI
            wind_speed.z = 0.0;

            // Packet to JSON
            let wind_speed_json = udp_topics::encode_json(&wind_speed);

            // Publish data
            udp_utils::publish(TOPICS::wind_speed::PORT, wind_speed_json.as_bytes()).expect("Failed to publish wind data");

            // Precise way to calculate interval
            // This way simulation is at the exact same FPS as GUI 
            let elapsed_t = start_t.elapsed();
            if elapsed_t < interval {
                thread::sleep(interval - elapsed_t);
            }
        }
    });
    // SEND - External Wind Forces (STOP) ==================================================

    // SEND - External Current Forces (START) ==================================================
    let current_parameters_clone = current_parameters.clone();
    thread::spawn(move || {
        let mut current_speed = TOPICS::current_speed::DataType::zeros();

        let mut current_state = current::CurrentState {
            speed_noise: 0.0,
            angle_noise: 0.0,
        };

        let dt = 1.0/config.external_force.simulation_frequency; // [s]
        let interval = Duration::from_millis((dt * 1000.0) as u64);

        loop {
            let start_t = Instant::now();

            // Get wind control parameters
            let current_parameters = *current_parameters_clone.read().unwrap();

            // Simulate wind
            let mut speed = current_parameters[0];
            let mut angle_deg = current_parameters[1];
            let noise = current_parameters[2];
            (speed, angle_deg) = current::simulate(
                speed,
                angle_deg,
                noise,
                config.interface.current_speed_max,
                &mut current_state,
            );

            // Convert simulated data to 3D vector
            let angle_rad = angle_deg.to_radians();
            current_speed.x = speed * angle_rad.cos();
            current_speed.y = -speed * angle_rad.sin(); // -1 because mirrored orientation in GUI
            current_speed.z = 0.0;

            // Packet to JSON
            let current_speed_json = udp_topics::encode_json(&current_speed);

            // Publish data
            udp_utils::publish(TOPICS::current_speed::PORT, current_speed_json.as_bytes()).expect("Failed to publish wind data");

            // Precise way to calculate interval
            // This way simulation is at the exact same FPS as GUI 
            let elapsed_t = start_t.elapsed();
            if elapsed_t < interval {
                thread::sleep(interval - elapsed_t);
            }
        }
    });
    // SEND - External Current Forces (STOP) ==================================================

    // Idle (START) ==================================================
    // Ensures we continue multithreading
    loop {
        thread::sleep(Duration::from_secs(1));
    }
    // Idle (STOP) ==================================================
}