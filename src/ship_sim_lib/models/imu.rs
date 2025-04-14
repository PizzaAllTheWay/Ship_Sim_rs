use nalgebra::{Vector3, Matrix3};
use rand_distr::{Normal, Distribution};

pub fn simulate(
    a_lin: Vector3<f32>,           // linear acceleration in body frame [m/s²]
    v_ang: Vector3<f32>,           // angular velocity in body frame [rad/s]
    r_ang_z: f32,                  // yaw angle from full r_ang
    imu_pos_b: Vector3<f32>,       // IMU position in body frame
    r_body_to_imu: Matrix3<f32>,   // precomputed rotation: body → imu
    r_world_to_body: Matrix3<f32>, // precomputed rotation: world → body

    noise: f32,         // [%]
    accel_noise: f32,   // [m/s² / h]
    gyro_noise: f32,    // [rad/s / h]
    mag_noise: f32,     // [rad / h]
    pub_frequency: f32, // [Hz]

    resolution: u32, // [bit]
    accel_fsr: f32,  // [m/s²]
    gyro_fsr: f32,   // [rad/s]
    mag_fsr: f32,    // [rad]
) -> (Vector3<f32>, Vector3<f32>, f32) {
    // Calculate Ground Truth (START) ==================================================
    // Constants
    let g_w = Vector3::new(0.0, 0.0, 9.81);

    // Acceleration ----------
    let g_b = r_world_to_body * g_w;
    let mut accel_b = a_lin + g_b;

    // Add centripetal acceleration: a_c = ω × (ω × r)
    let omega_cross_r = v_ang.cross(&imu_pos_b);
    let omega_cross_omega_cross_r = v_ang.cross(&omega_cross_r);
    accel_b += omega_cross_omega_cross_r;

    // Transform to IMU frame
    let accel = r_body_to_imu * accel_b;

    // Angular Velocity ----------
    let gyro = r_body_to_imu * v_ang;

    // Yaw Angle ----------
    let mag = r_ang_z;
    // Calculate Ground Truth (STOP) ==================================================



    // Simulate Noise (START) ==================================================
    // Generate pure randomness
    let mut rng = rand::thread_rng();
    let gauss = Normal::new(0.0, noise).unwrap();

    // Calculate standard deviations over time
    let s_in_h = 3600.0; // seconds in an hour
    let accel_std = accel_noise/s_in_h * pub_frequency.sqrt();
    let gyro_std = gyro_noise/(s_in_h * pub_frequency).sqrt();
    let mag_std = mag_noise/(s_in_h * pub_frequency).sqrt();

    // Non-linear noise for accel
    let accel_noise = {
        let x = gauss.sample(&mut rng) as f32 * accel_std;
        let y = gauss.sample(&mut rng) as f32 * accel_std;
        let z = gauss.sample(&mut rng) as f32 * accel_std;

        Vector3::new(
            (x * x.abs().sqrt() + 0.1 * (x * 8.0).sin())/2.0,
            (y + (y.abs() * 1.5 + 1e-5).ln_1p() * y.signum())/2.0,
            (z * (z * 2.5).cos() + 0.05 * z.powi(3))/2.0,
        ) * accel_fsr
    };

    // Non-linear noise for gyro
    let gyro_noise = {
        let x = gauss.sample(&mut rng) as f32 * gyro_std;
        let y = gauss.sample(&mut rng) as f32 * gyro_std;
        let z = gauss.sample(&mut rng) as f32 * gyro_std;

        Vector3::new(
            x - 0.2 * ((z - 1.0).abs()).ln(),
            ((((x * y * z).abs()).sqrt() + y.sin()).copysign(y)).clamp(-5.0, 5.0),
            z - 0.1 * (z * z + 1.0).ln(),
        ) * gyro_fsr
    };

    // Non-linear noise for magnetometer
    let mag_noise = {
        let n = gauss.sample(&mut rng) as f32 * mag_std;
        // Prevent undefined tan spikes with modulus and clamp
        let safe_tan = ((n * 10.0) % std::f32::consts::PI).tan().clamp(-5.0, 5.0);
        (n + 0.05 * safe_tan).clamp(-1.0, 1.0) * mag_fsr
    };
    // Simulate Noise (STOP) ==================================================



    // Simulate Resolution (START) ==================================================
    fn quantize(v: f32, step: f32) -> f32 {
        (v / step).round() * step
    }
    
    // Define full-scale range per sensor type (change if needed)
    let levels = 2_u32.pow(resolution);
    let accel_step = accel_fsr / levels as f32;
    let gyro_step  = gyro_fsr  / levels as f32;
    let mag_step   = mag_fsr   / levels as f32;
    
    let accel = Vector3::new(
        quantize((accel + accel_noise).x, accel_step),
        quantize((accel + accel_noise).y, accel_step),
        quantize((accel + accel_noise).z, accel_step),
    );
    
    let gyro = Vector3::new(
        quantize((gyro + gyro_noise).x, gyro_step),
        quantize((gyro + gyro_noise).y, gyro_step),
        quantize((gyro + gyro_noise).z, gyro_step),
    );
    
    let mag = quantize(mag + mag_noise, mag_step);
    // Simulate Resolution (START) ==================================================



    return (accel, gyro, mag);
}
