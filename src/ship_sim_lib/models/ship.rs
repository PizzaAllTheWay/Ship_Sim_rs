// Library for linear algebra
use nalgebra::{Vector3, Matrix3};

// Library for constants
use std::f32::consts::PI;

#[allow(non_snake_case)]
pub struct ShipDynamics {
    pub m: f32,                // mass of the boat [kg]
    pub dimensions: [f32; 2],  // dimensions of the boat (r, l) [m]
    pub I_inv: Matrix3<f32>,   // inverse moment of inertia for faster computation [1/kg*m²]
}

impl ShipDynamics {
    // Initializes the dynamics model
    #[allow(non_snake_case)]
    pub fn new(
        m: f32,
        dimensions: [f32; 2],
    ) -> Self {
        // Assume ship is a cylinder
        // Moment of inertia for a solid cylinder
        // I_x = (1/2) * m * r²
        // I_y = I_z = (1/4) * m * r² + (1/12) * m * l²
        let r = dimensions[0];
        let l = dimensions[1];
        let I_x = (1.0/2.0) * m * r.powi(2);
        let I_y = (1.0/4.0) * m * r.powi(2) + (1.0/12.0) * m * l.powi(2);
        let I_z = I_y;

        // Diagonal inertia matrix
        let I = Matrix3::new(
            I_x, 0.0,       0.0,
            0.0,       I_y, 0.0,
            0.0,       0.0,       I_z,
        );

        // Try to invert inertia matrix
        let mut I_inv = Matrix3::zeros();
        if let Some(matrix_inv) = I.try_inverse() {
            I_inv = matrix_inv;
        } else {
            println!("Matrix is not invertible!");
        }
    
        Self {
            m,
            dimensions,
            I_inv,
        }
    }

    // Calculates linear & angular acceleration in body frame
    // Inputs:
    // - force_thrusters: applied force [N]
    // - torque_thrusters: applied torque [Nm]
    // - v_lin: body linear velocity [m/s]
    // - v_ang: body angular velocity [rad/s]
    // Drag: Fd = -0.5 * rho * Cd * A * v * |v|
    // Returns: (linear accel [m/s²], angular accel [rad/s²])
    pub fn calc_accel_body(
        &self,
        force_thrusters: Vector3<f32>,
        torque_thrusters: Vector3<f32>,
        v_lin: Vector3<f32>,
        v_ang: Vector3<f32>,
        wind: Vector3<f32>,
        current: Vector3<f32>,
    ) -> (Vector3<f32>, Vector3<f32>) {
        // Dampening ----------
        // Add a small dampening, helps get rid of oscitation and enhances numerical stability 
        let d_lin: f32 = 0.080;
        let d_ang: f32 = 200000.0;
        let mut force_dampening = (-d_lin) * v_lin;
        let torque_dampening = (-d_ang) * v_ang;

        // Apply extra dampening for the sides
        force_dampening[1] += (-d_lin) * v_lin[1];

        // Calculate water drag forces ----------
        // Constants
        let rho_water: f32 = 1000.0; // water density [kg/m³]
        let c_d_lin: f32 = 0.0025; // linear drag coefficient (Must be < 1.0)
        let r = self.dimensions[0];
        let l = self.dimensions[1];

        // Calculate linear area drag for each direction
        let a_x = (PI / 4.0) * r.powi(2); // A_x [m²]: half circle
        let a_y = 0.5 * l * r;            // A_y [m²]: half rectangular side
        let a_z = l * r;                  // A_z [m²]: rectangular top
        let projected_area_lin = Matrix3::new(
            a_x, 0.0, 0.0,
            0.0, a_y, 0.0,
            0.0, 0.0, a_z,
        );

        // Formula used (per axis): 
        // F_drag = -0.5 * ρ * C_d * A * v * |v|
        // where:
        //   ρ    = water density [kg/m³]
        //   C_d  = drag coefficient (dimensionless, ~0.01–1.0)
        //   A    = projected area in flow direction [m²]
        //   v    = linear velocity in body frame [m/s]
        //   |v|  = absolute velocity (magnitude per component)
        // The result is a force that always opposes the velocity direction.

        // Linear water drag
        let v_lin_abs = v_lin.map(|v| v.abs());
        let force_drag = (-0.5) * rho_water * c_d_lin * projected_area_lin * v_lin.component_mul(&v_lin_abs);

        // Angular water drag
        // !NOTE: To complex, so just added extra dampening to angular momentum to simulate a simple linear relation instead
        let c_d_ang: f32 = 10.0;
        let torque_drag = (-c_d_ang) * v_ang;

        // Calculate air drag forces ----------
        // !NOTE: Don't need it as its forces are negligible 

        // Calculate wind forces ----------
        // Very similar to drag model just positive and fine tuned for wind
        let rho_air: f32 = 1.225; // air density [kg/m³]
        let c_d_wind: f32 = 0.055;  // drag coefficient for wind
        let k_tx = 0.0025; // scaling for Tx
        let k_ty = 0.0005; // scaling for Ty
        let k_tz = 0.0155; // scaling for Tz

        // Wind force
        let wind_abs = wind.map(|v| v.abs());
        let force_wind = 0.5 * rho_air * c_d_wind * projected_area_lin * wind.component_mul(&wind_abs);
        
        // Wind torque
        let mut torque_wind = Vector3::zeros();
        torque_wind[0] = -k_tx * force_wind[1] * force_wind[2]; // Fy * Fz → roll
        torque_wind[1] = -k_ty * force_wind[0] * force_wind[2]; // Fx * Fz → pitch
        torque_wind[2] = -k_tz * force_wind[0] * force_wind[1]; // Fx * Fy → yaw

        // Calculate current forces ----------
        // Very similar to drag model just positive and fine tuned for current
        let c_d_current: f32 = 0.035;  // drag coefficient for current
        let k_tx = 0.0025; // scaling for Tx
        let k_ty = 0.0005; // scaling for Ty
        let k_tz = 0.0155; // scaling for Tz

        // Water current is also affected by wind, around 2%
        let current = current + 0.02 * wind.component_mul(&current);

        // Water Current force
        let current_abs = current.map(|v| v.abs());
        let force_current = 0.5 * rho_water * c_d_current * projected_area_lin * current.component_mul(&current_abs);
        
        // Water Current torque
        let mut torque_current = Vector3::zeros();
        torque_current[0] = k_tx * force_current[1] * force_current[2]; // Fy * Fz → roll
        torque_current[1] = k_ty * force_current[0] * force_current[2]; // Fx * Fz → pitch
        torque_current[2] = -k_tz * force_current[0] * force_current[1]; // Fx * Fy → yaw

        // Calculate subsystem forces ----------
        // x
        let force_x = force_dampening + force_drag;
        let torque_x = torque_dampening + torque_drag;

        // u
        let force_u = force_thrusters;
        let torque_u = torque_thrusters;

        // w
        let force_w = force_wind + force_current;
        let torque_w = torque_wind + torque_current;

        // Calculate total forces ----------
        let force_tot = force_x + force_u + force_w;
        let torque_tot = torque_x + torque_u + torque_w;

        // Calculate acceleration of the body ----------
        let a = (1.0/self.m) * force_tot;
        let alpha = self.I_inv * torque_tot;

        return (a, alpha);
    }
}
