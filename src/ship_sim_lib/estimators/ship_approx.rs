// Library for linear algebra
use nalgebra::{Vector3, Matrix3};



// !NOTE!
// Pretend like we don't know ship model completely
// This is a estimate of ship model
// Thus it is a simplification/approximation
#[allow(non_snake_case)]
pub struct ShipDynamics {
    pub m: f32,                // mass of the boat [kg]
    pub dimensions: [f32; 2],  // dimensions of the boat (r, l) [m]
    pub thruster_placement: Vector3<f32>, // Placement of thruster on the ship in body frame [x, y, z] [m]
    pub I: Matrix3<f32>, // Moment of inertia for faster computation [kg*m²]
    pub I_inv: Matrix3<f32>,   // inverse moment of inertia for faster computation [1/kg*m²]
    pub velocity_linear_max: f32, // [m/s]
    pub velocity_angular_max: f32, // [m/s]
}

impl ShipDynamics {
    // Initializes the dynamics model
    #[allow(non_snake_case)]
    pub fn new(
        m: f32,
        dimensions: [f32; 2],
        thruster_placement: Vector3<f32>,
        velocity_linear_max: f32,
        velocity_angular_max: f32,
    ) -> Self {
        // Simplify ship to a cuboid shape
        // Moment of inertia for a solid cuboid
        // I_x = (1/12) * m * (W² + H²)
        // I_y = (1/12) * m * (L² + H²)
        // I_z = (1/12) * m * (L² + W²)
        // L = l
        // W = 2r
        // H = 2r
        let r = dimensions[0];
        let l = dimensions[1];
        let w = 2.0 * r;
        let h = 2.0 * r;
        let I_x = (1.0/12.0) * m * (w.powi(2) + h.powi(2));
        let I_y = (1.0/12.0) * m * (l.powi(2) + h.powi(2));
        let I_z = (1.0/12.0) * m * (l.powi(2) + w.powi(2));

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
            velocity_linear_max,
            velocity_angular_max,
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
            let mut damping = f32::max((limit - reference[i].abs())/decay_rate, 0.0);
            if damping == 0.0 {
                damping = (limit + reference[i].abs())/decay_rate;
            }
            input[i] *= damping;
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
        thruster_rpm: f32,
        thruster_angle: f32,
        v_lin: Vector3<f32>,
        v_ang: Vector3<f32>,
        gravity_b: Vector3<f32>,
        pos_cg_w: Vector3<f32>,
    ) -> (Vector3<f32>, Vector3<f32>) {
        // calculate Thruster Forces ----------
        // Tuning parameter for how powerful the thruster is
        let k_force = 4.0; // [N/rmp]

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
            200.0,  0.0,  0.0,
            0.0,  200_000.0,  0.0,
            0.0,  0.0,  100_000.0,
        );
        let d_ang: f32 = 50_000.0;
        
        let force_dampening = (-d_lin_matrix) * v_lin;
        let torque_dampening = (-d_ang) * v_ang;

        // Calculate water drag forces ----------
        // Constants
        let rho_water: f32 = 1000.0; // water density [kg/m³]
        let c_d_lin: f32 = 0.001; // linear drag coefficient (Must be < 1.0)
        let r = self.dimensions[0];
        let l = self.dimensions[1];

        // Calculate linear area drag for each direction
        let a_x = (2.0 * r).powi(2); // A_x [m²]: square
        let a_y = (2.0 * r) * l;     // A_y [m²]: rectangle
        let a_z = (2.0 * r) * l;     // A_z [m²]: rectangle
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
        let c_d_ang: f32 = 100.0;
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

        // Calculate gravity forces ----------
        let mut force_gravity = self.m * gravity_b;
        self.apply_directional_decay(&mut force_gravity, v_lin, self.velocity_linear_max * 2.0, 0.5); // Limit ship fall speed because there is no way the ship can fall faster than this

        // Calculate buoyancy forces ----------
        // Constants
        let rho_water = 1000.0; // kg/m³
        let g = 9.81; // m/s²
        let r = self.dimensions[0];
        let l = self.dimensions[1];
        let w = 2.0 * r;
        let h: f32 = 2.0 * r;

        // Calculate boats distance from the surface of the water
        let pos_water_surface_w = pos_cg_w - Vector3::new(0.0, 0.0, r);
        let dh = pos_water_surface_w[2];
        
        // Calculate volume
        // Assume boat is a cuboid
        let area_submerged: f32;
        if dh > 0.0 {
            // No area submerged
            area_submerged = 0.0;
        }
        else if dh < (-h) {
            // The whole body is submerged under water
            area_submerged = w * h;
        }
        else {
            // Partially submerged
            area_submerged = w * dh.abs();
        }
        let volume_submerged = area_submerged * l;

        // Calculate buoyancy force
        let force_buoyancy_z = rho_water * g * volume_submerged;
        let mut force_buoyancy = Vector3::new(0.0, 0.0, force_buoyancy_z);
        
        // Calculate subsystem forces ----------
        // x
        let force_x = force_dampening + force_drag + force_gravity + force_buoyancy;
        let torque_x = torque_dampening + torque_drag + torque_skid;

        // u
        // ?NOTE: Forces need to be prescaled as the original force acting on the body is to small compared to the real model, by scaling up the forces we can approximate our model to the real model a lot better and easier that fine tuning all the parameters and physics individually
        let mut force_u = force_thruster;
        let mut torque_u = torque_thruster;
        self.apply_directional_decay(&mut force_u, v_lin, self.velocity_linear_max, 0.5);
        self.apply_directional_decay(&mut torque_u, v_ang, self.velocity_angular_max, 0.5);      

        // Calculate total forces ----------
        let force_tot = force_x + force_u;
        let torque_tot = torque_x + torque_u;

        // Calculate acceleration of the body ----------
        let a = (1.0/self.m) * force_tot;
        let alpha = self.I_inv * (torque_tot - v_ang.cross(&(self.I * v_ang)));

        return (a, alpha);
    }
}