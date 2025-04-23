// Library for linear algebra
use nalgebra::{Vector3, Matrix3};

// Library for constants
use std::f32::consts::PI;



#[allow(non_snake_case)]
pub struct ShipDynamics {
    pub m: f32,                // mass of the boat [kg]
    pub dimensions: [f32; 2],  // dimensions of the boat (r, l) [m]
    pub thruster_placement: Vector3<f32>, // Placement of thruster on the ship in body frame [x, y, z] [m]
    pub I: Matrix3<f32>, // Moment of inertia for faster computation [kg*m²]
    pub I_inv: Matrix3<f32>, // inverse moment of inertia for faster computation [1/kg*m²]
    pub v_lin_max: f32, // [m/s]
    pub v_ang_max: f32, // [rad/s]
}

impl ShipDynamics {
    // Initializes the dynamics model
    #[allow(non_snake_case)]
    pub fn new(
        m: f32,
        dimensions: [f32; 2],
        thruster_placement: Vector3<f32>,
        velocity_lin_max: f32,
        velocity_ang_max: f32,
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
            thruster_placement,
            I,
            I_inv,
            v_lin_max: velocity_lin_max,
            v_ang_max: velocity_ang_max,
        }
    }


    // Applies directional exponential decay to a vector `input`
    // when its direction matches the corresponding `reference` vector
    // and the reference magnitude approaches a `limit`.
    pub fn apply_directional_decay(
        &self,
        input: &mut Vector3<f32>,
        reference: Vector3<f32>,
        limit: f32,
        decay_rate: f32,
    ) {
        for i in 0..3 {
            let ref_val = reference[i];
            let ref_abs = ref_val.abs();
            let sign_match = ref_val.signum() == input[i].signum();

            if sign_match {
                let decay = (-(decay_rate * ref_abs / limit).clamp(0.0, decay_rate)).exp();
                input[i] *= decay;
            }
        }
    }

    // Calculates linear & angular acceleration in body frame
    pub fn calc_accel_body(
        &self,
        thruster_rpm: f32,
        thruster_angle: f32,
        v_lin: Vector3<f32>,
        v_ang: Vector3<f32>,
        wind: Vector3<f32>,
        current: Vector3<f32>,
        gravity_b: Vector3<f32>,
        pos_cg_w: Vector3<f32>,
    ) -> (Vector3<f32>, Vector3<f32>) {
        // calculate Thruster Forces ----------
        // Tuning parameter for how powerful the thruster is
        let k_force = 9.5; // [N/rmp]

        // Compute thrust force
        let dir_prop = Vector3::new(thruster_angle.cos(), thruster_angle.sin(), 0.0);
        let force_thruster = dir_prop * (k_force * thruster_rpm);

        // Compute thruster torque
        // extract only Z (yaw) component
        // This is because roll and pitch angles have no dampening so they will oscillate
        // Moreover because of euler angles if Z axis flips 90* in XY plane we get close to singularity
        // This means if roll or pitch angle over 90* we get explosion in values
        // To mitigate this we ignore roll and pitch contributions because no dampening on those angles
        let torque_thruster_z = self.thruster_placement.cross(&force_thruster)[2];
        let mut torque_thruster = Vector3::zeros();
        torque_thruster[2] = -torque_thruster_z; // Flip sign to match simulation's yaw convention (right-hand rule mismatch)

        // Dampening ----------
        // Add a small dampening, helps get rid of oscitation and enhances numerical stability
        let d_lin_matrix = Matrix3::new(
            0.085,  0.0,   0.0,
              0.0,  10000.7,   0.0,
              0.0,  0.0,  0.95,
        );
        let d_ang: f32 = 1_000_000.0;
        
        let force_dampening = (-d_lin_matrix) * v_lin;
        let torque_dampening = (-d_ang) * v_ang;

        // Calculate water drag forces ----------
        // Constants
        let rho_water: f32 = 1000.0; // water density [kg/m³]
        let c_d_lin: f32 = 0.0005; // linear drag coefficient (Must be < 1.0)
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
        let c_d_ang: f32 = 10000.0;
        let torque_drag = (-c_d_ang) * v_ang;

        // Calculate hydrodynamic cross coupling torque ----------
        // Normalize vector
        let v_lin_dir = if v_lin.norm() > 1e-3 {
            v_lin.normalize()
        } else {
            Vector3::zeros()
        };

        // extract only Z (yaw) component
        // This is because roll and pitch angles have no dampening so they will oscillate
        // Moreover because of euler angles if Z axis flips 90* in XY plane we get close to singularity
        // This means if roll or pitch angle over 90* we get explosion in values
        // To mitigate this we ignore roll and pitch contributions because no dampening on those angles
        let torque_skid_z = v_lin_dir.cross(&force_drag)[2]; 
        let mut torque_skid = Vector3::zeros();
        torque_skid[2] = torque_skid_z;

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

        // Calculate gravity forces ----------
        let mut force_gravity = self.m * gravity_b;

        // Easiest way to dampen oscillations, limit how fast a force can act when reaching its maximum velocity
        // In theory it could be higher but lets be hones, no one is doping a ship from space to earth, in order to reach the absolute maximum terminal velocity
        // For movement up and down on water, this max velocity constraint is more than enough
        self.apply_directional_decay(&mut force_gravity, v_lin, self.v_lin_max, 1.0);

        // Calculate buoyancy forces ----------
        // Constants
        let rho_water = 1000.0; // kg/m³
        let g = 9.81; // m/s²
        let r = self.dimensions[0];
        let l = self.dimensions[1];

        // Calculate boats distance from the surface of the water
        let pos_water_surface_w = pos_cg_w - Vector3::new(0.0, 0.0, r/2.0);
        let dh = pos_water_surface_w[2];
        
        // Calculate volume
        // Assume boat is a half a cylinder
        let area_submerged: f32;
        if dh > 0.0 {
            // No area submerged
            area_submerged = 0.0;
        }
        else if dh < (-r) {
            // The whole body is submerged under water
            area_submerged = (1.0/2.0) * PI * r.powi(2);
        }
        else {
            // Partially submerged, bit more tricky calculation
            let s = r - dh.abs();
            let x = (r.powi(2) - s.powi(2)).sqrt();
            let area_triangle = x * s;
            let theta = ((2.0 * area_triangle)/r.powi(2)).asin();
            let area_sector = (theta * r.powi(2))/2.0;

            area_submerged = area_sector - area_triangle;
        }
        let volume_submerged = area_submerged * l;

        // Calculate buoyancy force
        let force_buoyancy_z = rho_water * g * volume_submerged;
        let mut force_buoyancy = Vector3::new(0.0, 0.0, force_buoyancy_z);

        // Easiest way to dampen oscillations, limit how fast a force can act when reaching its maximum velocity
        // Same reason as for gravity but now the opposite direction
        // For movement up and down on water, this max velocity constraint is more than enough
        self.apply_directional_decay(&mut force_buoyancy, v_lin, self.v_lin_max, 5.0);

        // Calculate subsystem forces ----------
        // x
        let mut force_x = force_dampening + force_drag;
        let mut torque_x = torque_dampening + torque_drag + torque_skid;
        self.apply_directional_decay(&mut force_x, v_lin, self.v_lin_max, 1.0);
        self.apply_directional_decay(&mut torque_x, v_ang, self.v_ang_max, 1.0);
        force_x += force_gravity + force_buoyancy;

        // u
        let mut force_u = force_thruster;
        let mut torque_u = torque_thruster;
        self.apply_directional_decay(&mut force_u, v_lin, self.v_lin_max, 1.0);
        self.apply_directional_decay(&mut torque_u, v_ang, self.v_ang_max, 1.0);

        // w
        let force_w = force_wind + force_current;
        let torque_w = torque_wind + torque_current;

        // Calculate total forces ----------
        let force_tot = force_x + force_u + force_w;
        let torque_tot = torque_x + torque_u + torque_w;

        // Calculate acceleration of the body ----------
        let a = (1.0/self.m) * force_tot;
        let alpha = self.I_inv * (torque_tot - v_ang.cross(&(self.I * v_ang)));

        return (a, alpha);
    }
}