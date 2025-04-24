// Custom libraries
use ship_sim_lib::models::ship;
use ship_sim_lib::simulation::kinematics;
use ship_sim_lib::simulation::solver;
use ship_sim_lib::comm::udp_utils;
use ship_sim_lib::comm::udp_topics::{TOPICS, Vector12};

// Library for data formatting
use serde::Deserialize;
use std::fs;
use std::str;

// Library for maths
use nalgebra::{Vector2, Vector3};
use std::f32::consts::PI;

// Libraries for multithreading
use std::thread;
use std::time::Duration;
use std::sync::{Arc, RwLock};



// Config data structure ----------
#[derive(Deserialize)]
struct ShipConfig {
    simulation_frequency: f32,
    mass: f32,
    dimensions: [f32; 2],
    thruster_placement: [f32; 3],
    velocity_linear_max: f32,
    velocity_angular_max: f32,
    x_0: [f32; 12],
}

#[derive(Deserialize)]
struct Config {
    ship: ShipConfig,
}



// Our non linear ODE ----------
// x_dot = f(x, u, w)
pub struct ODE {
    pub ship_dynamic: ship::ShipDynamics,
    pub x: Vector12<f32>,
}

pub struct Disturbance {
    pub wind: Vector3<f32>, // [vx, vy, vz]
    pub current: Vector3<f32>, // [vx, vy, vz]
}

impl ODE {
    pub fn new(
        x_0: Vector12<f32>, // Initial states
        ship_mass: f32, // [kg]
        ship_dimensions: [f32; 2], // (r, l) [m]
        thruster_placement: Vector3<f32>, // Placement of thruster on the ship in body frame [x, y, z] [m]
        velocity_linear_max: f32, // [m/s]
        velocity_angular_max: f32, // [rad/s]

    ) -> Self {
        // Initialize ship dynamics        
        let ship_dynamic = ship::ShipDynamics::new(
            ship_mass,
            ship_dimensions,
            thruster_placement,
            velocity_linear_max,
            velocity_angular_max,
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
        u: &Vector2<f32>,
        w: &Disturbance,
    ) -> Vector12<f32> {
        // Constant vectors
        let gravity_w = Vector3::new(0.0, 0.0, -9.81);

        // Split up states into manageable subparts
        let v_lin_w: Vector3<f32> = x.fixed_rows::<3>(0).into(); // [vx, vy, vz]
        let euler_dot: Vector3<f32> = x.fixed_rows::<3>(3).into(); // [angular velocity in roll, pitch, yaw]
        let r_lin_w: Vector3<f32> = x.fixed_rows::<3>(6).into(); // [x, y, z]
        let euler: Vector3<f32> = x.fixed_rows::<3>(9).into(); // [roll, pitch, yaw]

        let thruster_rpm: f32 = u[0];
        let thruster_angle: f32 = u[1];

        let w_wind_w: Vector3<f32> = w.wind; // [vx, vy, vz]
        let w_current_w: Vector3<f32> = w.current; // [vx, vy, vz]

        // Convert to body frame
        let r_ang_b: Vector3<f32> = euler;
        let r_lin_b: Vector3<f32> = kinematics::rot_world_to_body(euler) * r_lin_w;
        let v_ang_b: Vector3<f32> = kinematics::euler_dot_to_angular_velocity_body(euler, euler_dot);
        let v_lin_b: Vector3<f32> = kinematics::linear_velocity_world_to_body(euler, v_lin_w);
 
        let w_wind_b = kinematics::linear_velocity_world_to_body(euler, w_wind_w);
        let w_current_b = kinematics::linear_velocity_world_to_body(euler, w_current_w);
        
        let gravity_b = kinematics::linear_accel_world_to_body(euler, gravity_w);

        // Dynamics
        let (a_lin_b, a_ang_b) = self.ship_dynamic.calc_accel_body(
            thruster_rpm,
            thruster_angle,
            v_lin_b,
            v_ang_b,
            w_wind_b,
            w_current_b,
            gravity_b,
            r_lin_w,
        );

        // Convert to world frame
        let a_lin_w = kinematics::linear_accel_body_to_world(euler, a_lin_b,);
        let euler_dot_dot = kinematics::angular_accel_body_to_euler_dot_dot(euler, euler_dot, a_ang_b);
        
        // Structure return states properly
        let mut x_dot: Vector12<f32> = Vector12::<f32>::zeros();
        x_dot.fixed_rows_mut::<3>(0).copy_from(&a_lin_w);       // [ax, ay, az]
        x_dot.fixed_rows_mut::<3>(3).copy_from(&euler_dot_dot); // [angular acceleration in roll, pitch, yaw]
        x_dot.fixed_rows_mut::<3>(6).copy_from(&v_lin_w);       // [vx, vy, vz]
        x_dot.fixed_rows_mut::<3>(9).copy_from(&euler_dot);     // [angular velocity in roll, pitch, yaw]
        
        return x_dot;
    }
}



fn main() {
    // Setup (START) ==================================================
    // Get config file
    let config_str = fs::read_to_string("config.toml").expect("Failed to read config file");
    let config: Config = toml::from_str(&config_str).expect("Failed to parse TOML config");

    // Create shared resource to access GUI and states
    let thruster_control= Arc::new(RwLock::new(TOPICS::thruster_control::DataType::zeros()));
    let wind_speed = Arc::new(RwLock::new(TOPICS::wind_speed::DataType::zeros()));
    let current_speed = Arc::new(RwLock::new(TOPICS::current_speed::DataType::zeros()));
    // Setup (STOP) ==================================================

    // GET - Control Forces (START) ==================================================
    let thruster_control_clone = thruster_control.clone();
    thread::spawn(move || {
        loop {
            // Wait for thruster forces data to arrive from gui
            // Once received format to correct datatype
            let msg = udp_utils::subscribe(TOPICS::thruster_control::PORT).expect("Failed to get forces data");
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let control: TOPICS::thruster_control::DataType = udp_utils::decode_json(json_str);

            // Save thruster forces in shared resource for simulator
            let mut thruster_control = thruster_control_clone.write().unwrap();
            *thruster_control = control;
        }
    });
    // GET - Control Forces (STOP) ==================================================

    // GET - External Wind Forces (START) ==================================================
    let wind_speed_clone = wind_speed.clone();
    thread::spawn(move || {
        loop {
            // Wait for thruster forces data to arrive from gui
            // Once received format to correct datatype
            let msg = udp_utils::subscribe(TOPICS::wind_speed::PORT).expect("Failed to get wind data");
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let wind: TOPICS::wind_speed::DataType = udp_utils::decode_json(json_str);

            // Save thruster forces in shared resource for simulator
            let mut wind_speed = wind_speed_clone.write().unwrap();
            *wind_speed = wind;
        }
    });
    // GET - External Wind Forces (STOP) ==================================================

    // GET - Control Forces (START) ==================================================
    let current_speed_clone = current_speed.clone();
    thread::spawn(move || {
        loop {
            // Wait for thruster forces data to arrive from gui
            // Once received format to correct datatype
            let msg = udp_utils::subscribe(TOPICS::current_speed::PORT).expect("Failed to get water current data");
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let current: TOPICS::current_speed::DataType = udp_utils::decode_json(json_str);

            // Save thruster forces in shared resource for simulator
            let mut current_speed = current_speed_clone.write().unwrap();
            *current_speed = current;
        }
    });
    // GET - Control Forces (STOP) ==================================================

    // Simulate (START) ==================================================
    let thruster_control_clone = thruster_control.clone();
    let wind_speed_clone = wind_speed.clone();
    let current_speed_clone = current_speed.clone();
    thread::spawn(move || {
        // Initialize system ----------
        let mut x: TOPICS::x::DataType = TOPICS::x::DataType::from_row_slice(&config.ship.x_0);
        let ode: ODE = ODE::new(
            x, 
            config.ship.mass,
            config.ship.dimensions,
            Vector3::<f32>::from_row_slice(&config.ship.thruster_placement),
            config.ship.velocity_linear_max,
            config.ship.velocity_angular_max,
        );   
        let mut u: Vector2<f32>;

        let mut dx: TOPICS::dx::DataType;
        let mut speed: TOPICS::speed::DataType = Vector2::<f32>::zeros();
        
        // Tolerance bounds for adaptive timestep control
        // - tol_min: if error is smaller, we increase dt to speed up simulation
        // - tol_max: if error is larger, we decrease dt for stability
        // Values chosen to balance speed and accuracy in typical marine dynamics
        let tolerances: (f32, f32) = (1e-5, 1e-3); // (tol_min, tol_max)
        let mut dt = 1.0/config.ship.simulation_frequency; // [s]
        let dt_limits: (f32, f32) = (dt*0.01, dt); // (min, max) [s]
        

        // Simulation loop ----------
        loop {
            // Thruster forces ----------
            u = {
                *thruster_control_clone.read().unwrap()
            };

            // Disturbance ----------
            let w = Disturbance {
                wind: *wind_speed_clone.read().unwrap(),
                current: *current_speed_clone.read().unwrap(),
            };

            // Solve ODEs ----------
            (x, dx, dt) = solver::rkf45_step(
                &x, 
                &u,
                &w, 
                dt, 
                |x, u, w| ode.f(x, u, w),
                tolerances,
                dt_limits,
            );

            // Publish simulated data ----------
            // x
            let x_json = udp_utils::encode_json(&x);
            udp_utils::publish(TOPICS::x::PORT, x_json.as_bytes()).expect("Failed to send x data");

            // dx
            let dx_json = udp_utils::encode_json(&dx);
            udp_utils::publish(TOPICS::dx::PORT, dx_json.as_bytes()).expect("Failed to send dx data");

            // Speed
            let r_ang_w: Vector3<f32> = x.fixed_rows::<3>(9).into(); // [roll, pitch, yaw]
            let v_lin_w: Vector3<f32> = x.fixed_rows::<3>(0).into(); // [vx, vy, vz]
            let v_ang_w: Vector3<f32> = x.fixed_rows::<3>(3).into(); // [angular velocity in roll, pitch, yaw]

            let v_lin_b = kinematics::linear_velocity_world_to_body(r_ang_w, v_lin_w);
            let v_ang_b = kinematics::angular_velocity_world_to_body(r_ang_w, v_ang_w);
            
            speed[0] = (-1.0) * v_lin_b[0]; // heading speed [m/s]
            speed[1] = v_ang_b[2] * (180.0/PI); // yaw speed [°/s]

            let speed_json = udp_utils::encode_json(&speed);
            udp_utils::publish(TOPICS::speed::PORT, speed_json.as_bytes()).expect("Failed to send speed data");

            // Simulation time step
            let sim_dt_json = udp_utils::encode_json(&dt);
            udp_utils::publish(TOPICS::sim_dt::PORT, sim_dt_json.as_bytes()).expect("Failed to send simulation time step data");
            
            // Pause a bit
            thread::sleep(Duration::from_millis((dt * 1000.0) as u64));
        }
    });
    // Simulate (STOP) ==================================================

    // Idle (START) ==================================================
    // Ensures we continue multithreading
    loop {
        thread::sleep(Duration::from_secs(1));
    }
    // Idle (STOP) ==================================================
}