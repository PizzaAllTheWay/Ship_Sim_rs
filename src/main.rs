// Custom libraries
use ship_sim_lib::ship_simulator::dynamics;
use ship_sim_lib::ship_simulator::kinematics;
use ship_sim_lib::ship_simulator::solver;
use ship_sim_lib::ship_simulator::gui;

// Library for linear algebra
use nalgebra::Vector3;

// Libraries for multithreading
use std::thread;
use std::time::{Duration, Instant};

const FPS: f32 = 60.0;

fn main() {
    // Setup (START) ==================================================
    // Create shared state to access from GUI and this main loop
    let state = gui::SharedState::default();
    // Setup (STOP) ==================================================

    // Simulate (START) ==================================================
    // Simulate and react to WASD in one loop
    let state_clone = state.clone();
    thread::spawn(move || {
        // Initialize system
        let mut force_b: Vector3<f32> = Vector3::zeros();  // [Fx, Fy, Fz]
        let mut torque_b: Vector3<f32> = Vector3::zeros(); // [Torque in roll, pitch, yaw]

        let mut a_lin_b: Vector3<f32> = Vector3::zeros(); // [ax, ay, az]
        let mut a_ang_b: Vector3<f32> = Vector3::zeros(); // [angular acceleration in roll, pitch, yaw]
        let mut a_lin_w: Vector3<f32> = Vector3::zeros(); // [ax, ay, az]
        let mut a_ang_w: Vector3<f32> = Vector3::zeros(); // [angular acceleration in roll, pitch, yaw]

        let mut v_lin_b: Vector3<f32> = Vector3::zeros(); // [vx, vy, vz]
        let mut v_ang_b: Vector3<f32> = Vector3::zeros(); // [angular velocity in roll, pitch, yaw]
        let mut v_lin_w: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0); // [vx, vy, vz]
        let mut v_ang_w: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0); // [angular velocity in roll, pitch, yaw]

        let mut r_lin_w: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0); // [x, y, z]
        let mut r_ang_w: Vector3<f32> = Vector3::new(0.0, 0.0, 0.0); // [roll, pitch, yaw]

        let ship_mass = 1000.0; // [kg]
        let ship_dimensions: [f32; 2] = [10.0, 30.0]; // (r, l) [m]
        let ship_dynamic = dynamics::ShipDynamics::new(
            ship_mass,
            ship_dimensions,
        );

        let dt = 1.0/FPS; // [s]
        let interval = Duration::from_millis((dt * 1000.0) as u64);

        // Simulation loop ----------
        loop {
            let start_t = Instant::now();

            // Keyboard controls logic ----------
            // Read current WASD keys
            let keyboard_state = *state_clone.keyboard_state.read().unwrap();

            // Only give force as input if any WASD key is pressed
            if keyboard_state.iter().any(|&key| key) {
                if keyboard_state[0] { force_b[0] -= 1000.0; }  // W
                if keyboard_state[1] { torque_b[2] += 1000.0; } // A
                if keyboard_state[2] { force_b[0] += 1000.0; }  // S
                if keyboard_state[3] { torque_b[2] -= 1000.0; } // D
            }
            else {
                force_b[0] = 0.0;
                torque_b[2] = 0.0;
            }

            // Rigid body kinematics ----------
            // Kinematics
            a_lin_w = kinematics::linear_accel_body_to_world(r_ang_w, a_lin_b,);
            a_ang_w = kinematics::angular_accel_body_to_world(r_ang_w, a_ang_b);

            // Inverse kinematics
            v_lin_b = kinematics::linear_velocity_world_to_body(r_ang_w, v_lin_w);
            v_ang_b = kinematics::angular_velocity_world_to_body(r_ang_w, v_ang_w);

            // Rigid body dynamics ----------
            (a_lin_b, a_ang_b) = ship_dynamic.calc_accel_body(force_b, torque_b, v_lin_b, v_ang_b);

            // Solver for ODEs ----------
            // Solve velocity numerically
            v_lin_w = solver::rk1_step(&v_lin_w, &a_lin_w, dt);
            v_ang_w = solver::rk1_step(&v_ang_w, &a_ang_w, dt);

            // Solve position numerically
            r_lin_w = solver::rk1_step(&r_lin_w, &v_lin_w, dt);
            r_ang_w = solver::rk1_step(&r_ang_w, &v_ang_w, dt);

            // Update simulator ----------
            // Set new position
            if let Ok(mut pos) = state_clone.ship_pos.write() {
                pos[0] = r_lin_w[0]; // x
                pos[1] = r_lin_w[1]; // y
            }
            if let Ok(mut ang) = state_clone.ship_angle.write() {
                *ang = r_ang_w[2]; // yaw
            }            

            // Precise way to calculate interval
            // This way simulation is at the exact same FPS as GUI 
            let elapsed_t = start_t.elapsed();
            if elapsed_t < interval {
                thread::sleep(interval - elapsed_t);
            }
        }
    });
    // Simulate (STOP) ==================================================

    // GUI (START) ==================================================
    // Run GUI in main
    // NOTE: It must run in main else you might get event loops O_O
    let state_clone = state.clone();
    *state_clone.frame_interval_ms.write().unwrap() = (1000.0/FPS) as u64; // FPS to ms
    gui::window(state_clone);
    // GUI (STOP) ==================================================
}