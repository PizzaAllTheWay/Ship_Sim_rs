
// Custom libraries
use ship_sim_lib::ship_simulator::dynamics;
use ship_sim_lib::ship_simulator::kinematics;
use ship_sim_lib::ship_simulator::solver;
use ship_sim_lib::ship_simulator::gui;

// Library for maths
use nalgebra::{Vector3, Vector6, SVector};
use std::f32::consts::PI;

// Libraries for multithreading
use std::thread;
use std::time::{Duration, Instant};

// Environmental variables
const FPS: f32 = 60.0;
const SHIP_V_LIN_MAX: f32 = 10.00; // Maximum linear speed the ship can reach [m/s]
const SHIP_V_ANG_MAX: f32 = 0.15;   // Maximum angular speed the ship can reach [rad/s]

// Construct our own datatype
type Vector12<T> = SVector<T, 12>;

// Our non linear ODEx
// x_dot = f(x, u)
pub struct ODE {
    pub ship_dynamic: dynamics::ShipDynamics,
    pub x: Vector12<f32>,
}

impl ODE {
    pub fn new(
        x_0: Vector12<f32>, // Initial states
    ) -> Self {
        // Initialize ship dynamics
        let ship_mass = 10000.0; // [kg]
        let ship_dimensions: [f32; 2] = [10.0, 30.0]; // (r, l) [m]
        
        let ship_dynamic = dynamics::ShipDynamics::new(
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
        u: &Vector6<f32>
    ) -> Vector12<f32> {
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
        
        // Dynamics
        let (a_lin_b, a_ang_b) = self.ship_dynamic.calc_accel_body(force_b, torque_b, v_lin_b, v_ang_b);

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

fn main() {
    // Setup (START) ==================================================
    // Create shared state to access from GUI and this main loop
    let state = gui::SharedState::default();
    // Setup (STOP) ==================================================

    // Simulate (START) ==================================================
    let state_clone = state.clone();
    thread::spawn(move || {
        // Initialize system ----------
        let mut x = Vector12::<f32>::from_row_slice(&[
            0.0, 0.0, 0.0, // [vx, vy, vz]
            0.0, 0.0, 0.0, // [angular velocity in roll, pitch, yaw]
            0.0, 0.0, 0.0, // [x, y, z]
            0.0, 0.0, 0.0, // [roll, pitch, yaw]
        ]);
        let ode: ODE = ODE::new(x);   
        let mut u = Vector6::<f32>::zeros();
        
        // Tolerance bounds for adaptive timestep control
        // - tol_min: if error is smaller, we increase dt to speed up simulation
        // - tol_max: if error is larger, we decrease dt for stability
        // Values chosen to balance speed and accuracy in typical marine dynamics
        let tolerances: (f32, f32) = (1e-5, 1e-3); // (tol_min, tol_max)
        let dt_limits: (f32, f32) = (0.0001, 0.1); // (min, max) [s]
        let mut dt = 1.0/FPS; // [s]

        let interval = Duration::from_millis((dt * 1000.0) as u64);

        // Simulation loop ----------
        loop {
            let start_t = Instant::now();

            // Keyboard controls logic ----------
            // Read current WASD keys
            if *state_clone.key_state_w.read().unwrap() {
                u[0] -= 100.0; // W = Forward thrust
            }
            if *state_clone.key_state_a.read().unwrap() {
                u[5] -= 100.0; // A = Rotate left
            }
            if *state_clone.key_state_s.read().unwrap() {
                u[0] += 100.0; // S = Reverse thrust
            }
            if *state_clone.key_state_d.read().unwrap() {
                u[5] += 100.0; // D = Rotate right
            }
            
            // Reset force if none pressed
            if !(*state_clone.key_state_w.read().unwrap() || *state_clone.key_state_s.read().unwrap()) 
            {
                u[0] = 0.0;
            }
            if !(*state_clone.key_state_a.read().unwrap() || *state_clone.key_state_d.read().unwrap()) 
            {
                u[5] = 0.0;
            }

            // Solve ODEs ----------
            (x, dt) = solver::rkf45_step(
                &x, 
                &u, 
                dt, 
                |x, u| ode.f(x, u),
                tolerances,
                dt_limits,
            );

            // Limit speed ----------
            // Because of many non linearity's, we must clamp the speed to make sure we don't speed up exponentially
            let v_lin_w: Vector3<f32> = x.fixed_rows::<3>(0).into(); // [vx, vy, vz]
            let v_ang_w: Vector3<f32> = x.fixed_rows::<3>(3).into(); // [angular velocity in roll, pitch, yaw]

            let v_lin_w = if v_lin_w.norm() > SHIP_V_LIN_MAX {
                v_lin_w.normalize() * SHIP_V_LIN_MAX
            } else if v_lin_w.norm() < 0.01 {
                Vector3::zeros()
            } else {
                v_lin_w
            };

            let v_ang_w = if v_ang_w.norm() > SHIP_V_ANG_MAX {
                v_ang_w.normalize() * SHIP_V_ANG_MAX
            } else if v_ang_w.norm() < 0.0001 {
                Vector3::zeros()
            } else {
                v_ang_w
            };

            x.fixed_rows_mut::<3>(0).copy_from(&v_lin_w); // [vx, vy, vz]
            x.fixed_rows_mut::<3>(3).copy_from(&v_ang_w); // [angular velocity in roll, pitch, yaw]

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