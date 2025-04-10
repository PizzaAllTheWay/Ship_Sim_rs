// Custom libraries
use ship_sim_lib::ship_simulator::gui;
use ship_sim_lib::process_communication::udp_utils;
use ship_sim_lib::process_communication::udp_topics::{self, TOPICS};

// Library for data formatting
use std::str;

// Library for maths
use nalgebra::Vector6;

// Libraries for multithreading
use std::thread;
use std::time::{Duration, Instant};

// Environmental variables
const FPS: f32 = 60.0;

fn main() {
    // Setup (START) ==================================================
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

        let dt = 1.0/FPS; // [s]
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
    *gui_state_clone.frame_interval_ms.write().unwrap() = (1000.0/FPS) as u64; // FPS to ms
    gui::window(gui_state_clone);
    // GUI (STOP) ==================================================
}


/*
// Update simulator ----------
// Set new position
if let Ok(mut pos) = state_clone.ship_pos.write() {
    pos[0] = x[6]; // x
    pos[1] = x[7]; // y
}
if let Ok(mut ang) = state_clone.ship_angle.write() {
    *ang = x[11]; // yaw
}
#[allow(unused_variables)]
if let Ok(mut speed) = state_clone.ship_speed.write() {
    let v_lin_w: Vector3<f32> = x.fixed_rows::<3>(0).into(); // [vx, vy, vz]
    let v_ang_w: Vector3<f32> = x.fixed_rows::<3>(3).into(); // [angular velocity in roll, pitch, yaw]
    let r_lin_w: Vector3<f32> = x.fixed_rows::<3>(6).into(); // [x, y, z]
    let r_ang_w: Vector3<f32> = x.fixed_rows::<3>(9).into(); // [roll, pitch, yaw]
    
    let v_lin_b = kinematics::linear_velocity_world_to_body(r_ang_w, v_lin_w);
    let v_ang_b = kinematics::angular_velocity_world_to_body(r_ang_w, v_ang_w);
    
    speed[0] = (-1.0) * v_lin_b[0]; // heading speed [m/s]
    speed[1] = v_ang_b[2] * (180.0/PI); // yaw speed [°/s]
}

*/