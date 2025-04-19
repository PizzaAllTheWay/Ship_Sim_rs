// Custom libraries
use ship_sim_lib::ui::gui;
use ship_sim_lib::comm::udp_utils;
use ship_sim_lib::comm::udp_topics::TOPICS;

// Library for data formatting
use serde::Deserialize;
use std::fs;
use std::str;

// Libraries for multithreading
use std::thread;
use std::time::{Duration, Instant};

// Libraries for math
use std::f32::consts::PI;



// Config data structure ----------
#[derive(Deserialize)]
struct ShipConfig {
    thruster_rpm_max: f32,
}

#[derive(Deserialize)]
struct InterfaceConfig {
    fps: f32,
}

#[derive(Deserialize)]
struct ExternalForcesConfig {
    wind_speed_max: f32,
    current_speed_max: f32,
}

#[derive(Deserialize)]
struct Config {
    ship : ShipConfig,
    interface: InterfaceConfig,
    external_forces: ExternalForcesConfig,
}



fn main() {
    // Setup (START) ==================================================
    // Get config file
    let config_str = fs::read_to_string("config.toml").expect("Failed to read config file");
    let config: Config = toml::from_str(&config_str).expect("Failed to parse TOML config");

    // Create shared resources for GUI
    let gui_state = gui::SharedState::default();
    // Setup (STOP) ==================================================

    // GET - States (START) ==================================================
    let gui_state_clone = gui_state.clone();
    thread::spawn(move || {
        loop {
            // Wait for states
            let msg = udp_utils::subscribe(TOPICS::x::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let x: TOPICS::x::DataType = udp_utils::decode_json(json_str);

            // Update GUI with new states
            {
                let mut ship_pos = gui_state_clone.ship_pos.write().unwrap();
                *ship_pos = x.fixed_rows::<6>(6).into();
            }            
        }
    });
    // GET - States (STOP) ==================================================

    // GET - Speed (START) ==================================================
    let gui_state_clone = gui_state.clone();
    thread::spawn(move || {
        loop {
            // Wait for states
            let msg = udp_utils::subscribe(TOPICS::speed::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let speed: TOPICS::speed::DataType = udp_utils::decode_json(json_str);

            // Update GUI with new speed telemetry data
            if let Ok(mut speed_gui) = gui_state_clone.ship_speed.write() {                
                speed_gui[0] = speed[0]; // heading speed [m/s]
                speed_gui[1] = speed[1]; // yaw speed [°/s]
            }
        }
    });
    // GET - Speed (STOP) ==================================================

    // GET - GNSS (START) ==================================================
    let gui_state_clone = gui_state.clone();
    thread::spawn(move || {
        loop {
            // Wait for sensor data
            let msg = udp_utils::subscribe(TOPICS::gnss::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let gnss: TOPICS::gnss::DataType = udp_utils::decode_json(json_str);

            // Parse data
            let antenna1 = gnss.fixed_rows::<3>(0).into();
            let antenna2 = gnss.fixed_rows::<3>(3).into();
            let velocity = gnss.fixed_rows::<3>(6).into();

            // Append to sensor position history
            {
                let mut history = gui_state_clone.gnss_antenna1_history.write().unwrap();
                history.push(antenna1);
            }
            {
                let mut history = gui_state_clone.gnss_antenna2_history.write().unwrap();
                history.push(antenna2);
            }

            // Update GNSS Velocity
            {
                let mut gnss_velocity = gui_state_clone.gnss_velocity.write().unwrap();
                *gnss_velocity = velocity;
            }
        }
    });
    // GET - GNSS (STOP) ==================================================

    // GET - IMU (START) ==================================================
    let gui_state_clone = gui_state.clone();
    thread::spawn(move || {
        loop {
            // Wait for sensor data
            let msg = udp_utils::subscribe(TOPICS::imu::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let imu: TOPICS::imu::DataType = udp_utils::decode_json(json_str);

            // Split the Vector7 into individual parts
            let accel = imu.fixed_rows::<3>(0).into(); // [ax, ay, az]
            let gyro  = imu.fixed_rows::<3>(3).into(); // [angular velocity roll, pitch, yaw]
            let mag = imu[6];

            // Append to shared state
            {
                let mut accel_vec = gui_state_clone.imu_accel.write().unwrap();
                accel_vec.push(accel);
            }
            {
                let mut gyro_vec = gui_state_clone.imu_gyro.write().unwrap();
                gyro_vec.push(gyro);
            }
            {
                let mut mag_vec = gui_state_clone.imu_mag.write().unwrap();
                mag_vec.push(mag);
            }
        }
    });
    // GET - IMU (STOP) ==================================================

    // GET - Kalman Filter Estimate (START) ==================================================
    let gui_state_clone = gui_state.clone();
    thread::spawn(move || {
        loop {
            // Wait for states
            let msg = udp_utils::subscribe(TOPICS::kf::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let kf: TOPICS::kf::DataType = udp_utils::decode_json(json_str);
            let x_est = kf;

            // Update GUI with new states
            {
                let mut kf_estimate = gui_state_clone.kf_estimate.write().unwrap();
                *kf_estimate = x_est.fixed_rows::<6>(6).into();
            } 
        }
    });
    // GET - Kalman Filter Estimate (STOP) ==================================================



    // SEND - Control Forces (START) ==================================================
    let gui_state_clone = gui_state.clone();
    thread::spawn(move || {
        let mut thruster_control= TOPICS::thruster_control::DataType::zeros();

        let dt = 1.0/config.interface.fps; // [s]
        let interval = Duration::from_millis((dt * 1000.0) as u64);

        loop {
            let start_t = Instant::now();

            // Keyboard controls logic ----------
            // Read current WASD keys
            if *gui_state_clone.key_state_w.read().unwrap() {
                // W = Forward [rpm]
                thruster_control[0] = config.ship.thruster_rpm_max;
                thruster_control[1] = PI;
            }
            if *gui_state_clone.key_state_a.read().unwrap() {
                // A = Rotate left [rad]
                thruster_control[0] = config.ship.thruster_rpm_max;
                thruster_control[1] = -PI/2.0;
            }
            if *gui_state_clone.key_state_s.read().unwrap() {
                // S = Reverse [rpm]
                thruster_control[0] = config.ship.thruster_rpm_max;
                thruster_control[1] = 0.0;
            }
            if *gui_state_clone.key_state_d.read().unwrap() {
                // D = Rotate right [rad]
                thruster_control[0] = config.ship.thruster_rpm_max;
                thruster_control[1] = PI/2.0;
            }
            
            // Reset thruster if no keys pressed
            if !(
                *gui_state_clone.key_state_w.read().unwrap() ||
                *gui_state_clone.key_state_a.read().unwrap() ||
                *gui_state_clone.key_state_s.read().unwrap() ||
                *gui_state_clone.key_state_d.read().unwrap()
            ) 
            {
                thruster_control[0] = 0.0;
                thruster_control[1] = 0.0;
            }

            // Packet to JSON
            let thruster_control_json = udp_utils::encode_json(&thruster_control);

            // Publish data
            udp_utils::publish(TOPICS::thruster_control::PORT, thruster_control_json.as_bytes()).expect("Failed to publish forces data");

            // Precise way to calculate interval
            // This way simulation is at the exact same FPS as GUI 
            let elapsed_t = start_t.elapsed();
            if elapsed_t < interval {
                thread::sleep(interval - elapsed_t);
            }
        }
    });
    // SEND - Control Forces (STOP) ==================================================

    // SEND - External Wind Parameters (START) ==================================================
    let gui_state_clone = gui_state.clone();
    thread::spawn(move || {
        let dt = 1.0/config.interface.fps; // [s]
        let interval = Duration::from_millis((dt * 1000.0) as u64);

        loop {
            let start_t = Instant::now();

            // Send Wind Control to distributed network
            let wind_speed = *gui_state_clone.wind_speed.read().unwrap();
            let wind_angle_deg = *gui_state_clone.wind_angle.read().unwrap();
            let wind_noise = *gui_state_clone.wind_noise.read().unwrap();
            
            let wind = TOPICS::wind_parameters::DataType::new(
                wind_speed,
                wind_angle_deg,
                wind_noise,
            );

            // Packet to JSON
            let wind_json = udp_utils::encode_json(&wind);

            // Publish data
            udp_utils::publish(TOPICS::wind_parameters::PORT, wind_json.as_bytes()).expect("Failed to publish wind parameters");

            // Precise way to calculate interval
            // This way simulation is at the exact same FPS as GUI 
            let elapsed_t = start_t.elapsed();
            if elapsed_t < interval {
                thread::sleep(interval - elapsed_t);
            }
        }
    });
    // SEND - External Wind Parameters (STOP) ==================================================

    // SEND - External Current Parameters (START) ==================================================
    let gui_state_clone = gui_state.clone();
    thread::spawn(move || {
        let dt = 1.0/config.interface.fps; // [s]
        let interval = Duration::from_millis((dt * 1000.0) as u64);

        loop {
            let start_t = Instant::now();

            // Send Current Control to distributed network
            let current_speed = *gui_state_clone.current_speed.read().unwrap();
            let current_angle_deg = *gui_state_clone.current_angle.read().unwrap();
            let current_noise = *gui_state_clone.current_noise.read().unwrap();

            let current = TOPICS::current_parameters::DataType::new(
                current_speed,
                current_angle_deg,
                current_noise,
            );

            // Packet to JSON
            let current_json = udp_utils::encode_json(&current);

            // Publish data
            udp_utils::publish(TOPICS::current_parameters::PORT, current_json.as_bytes()).expect("Failed to publish water current parameters");

            // Precise way to calculate interval
            // This way simulation is at the exact same FPS as GUI 
            let elapsed_t = start_t.elapsed();
            if elapsed_t < interval {
                thread::sleep(interval - elapsed_t);
            }
        }
    });
    // SEND - External Current Parameters (STOP) ==================================================



    // GUI (START) ==================================================
    // Run GUI in main
    // NOTE: It must run in main else you might get event loops O_O
    // Specify limits for the GUI
    let gui_state_clone = gui_state.clone();
    *gui_state_clone.frame_interval_ms.write().unwrap() = (1000.0/config.interface.fps) as u64; // FPS to ms
    *gui_state_clone.show_external_forces.write().unwrap() = false; // Start GUI with external forces velocity vectors invisible
    *gui_state_clone.wind_speed_max.write().unwrap() = config.external_forces.wind_speed_max;
    *gui_state_clone.current_speed_max.write().unwrap() = config.external_forces.current_speed_max;
    *gui_state_clone.show_gnss_data.write().unwrap() = false; // Start GUI with gnss data invisible
    *gui_state_clone.show_imu_graphs.write().unwrap() = false; // Start GUI with imu data invisible
    *gui_state_clone.imu_graphs_period.write().unwrap() = 6000; // Start by showing only the latest specified amount of datapoint of the IMU sensor

    // Run GUI
    gui::window(gui_state_clone);
    // GUI (STOP) ==================================================
}