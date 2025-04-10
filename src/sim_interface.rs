// Custom libraries
use ship_sim_lib::ui::gui;
use ship_sim_lib::comm::udp_utils;
use ship_sim_lib::comm::udp_topics::{self, TOPICS};

// Library for data formatting
use serde::Deserialize;
use std::fs;
use std::str;

// Library for maths
use nalgebra::Vector6;

// Libraries for multithreading
use std::thread;
use std::time::{Duration, Instant};



// Config data structure ----------
#[derive(Deserialize)]
struct InterfaceConfig {
    fps: f32,
}

#[derive(Deserialize)]
struct Config {
    interface: InterfaceConfig,
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
            let x: TOPICS::x::DataType = udp_topics::decode_json(json_str);

            // Update GUI with new states
            if let Ok(mut pos) = gui_state_clone.ship_pos.write() {
                pos[0] = x[6]; // x
                pos[1] = x[7]; // y
            }
            if let Ok(mut ang) = gui_state_clone.ship_angle.write() {
                *ang = x[11]; // yaw
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
            let speed: TOPICS::speed::DataType = udp_topics::decode_json(json_str);

            // Update GUI with new speed telemetry data
            if let Ok(mut speed_gui) = gui_state_clone.ship_speed.write() {                
                speed_gui[0] = speed[0]; // heading speed [m/s]
                speed_gui[1] = speed[1]; // yaw speed [°/s]
            }
        }
    });
    // GET - Speed (STOP) ==================================================

    // SEND - Control Forces (START) ==================================================
    let gui_state_clone = gui_state.clone();
    thread::spawn(move || {
        let mut forces: TOPICS::forces::DataType = Vector6::<f32>::zeros();

        let dt = 1.0/config.interface.fps; // [s]
        let interval = Duration::from_millis((dt * 1000.0) as u64);

        loop {
            let start_t = Instant::now();

            // Keyboard controls logic ----------
            // Read current WASD keys
            if *gui_state_clone.key_state_w.read().unwrap() {
                forces[0] -= 100.0; // W = Forward thrust
            }
            if *gui_state_clone.key_state_a.read().unwrap() {
                forces[5] -= 100.0; // A = Rotate left
            }
            if *gui_state_clone.key_state_s.read().unwrap() {
                forces[0] += 100.0; // S = Reverse thrust
            }
            if *gui_state_clone.key_state_d.read().unwrap() {
                forces[5] += 100.0; // D = Rotate right
            }
            
            // Reset force if none pressed
            if !(*gui_state_clone.key_state_w.read().unwrap() || *gui_state_clone.key_state_s.read().unwrap()) 
            {
                forces[0] = 0.0;
            }
            if !(*gui_state_clone.key_state_a.read().unwrap() || *gui_state_clone.key_state_d.read().unwrap()) 
            {
                forces[5] = 0.0;
            }

            // Packet to JSON
            let forces_json = udp_topics::encode_json(&forces);

            // Publish data
            udp_utils::publish(TOPICS::forces::PORT, forces_json.as_bytes()).expect("Failed to publish forces data");

            // Precise way to calculate interval
            // This way simulation is at the exact same FPS as GUI 
            let elapsed_t = start_t.elapsed();
            if elapsed_t < interval {
                thread::sleep(interval - elapsed_t);
            }
        }
    });
    // SEND - Control Forces (STOP) ==================================================

    // GUI (START) ==================================================
    // Run GUI in main
    // NOTE: It must run in main else you might get event loops O_O
    let gui_state_clone = gui_state.clone();
    *gui_state_clone.frame_interval_ms.write().unwrap() = (1000.0/config.interface.fps) as u64; // FPS to ms
    gui::window(gui_state_clone);
    // GUI (STOP) ==================================================
}