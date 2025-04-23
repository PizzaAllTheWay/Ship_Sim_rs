use nalgebra::Vector3;
use rand_distr::{Normal, Distribution};

pub fn simulate(
    accel_gt: Vector3<f32>, // linear acceleration in IMU frame [m/s²]
    gyro_gt: Vector3<f32>,  // angular velocity in IMU frame [rad/s]
    mag_gt: f32,            // yaw angle in IMU frame

    noise: f32,         // [%]
    accel_noise: f32,   // [m/s² / h]
    gyro_noise: f32,    // [rad/s / h]
    mag_noise: f32,     // [rad / h]
    dt: f32,            // [Hz]

    resolution: u32, // [bit]
    accel_fsr: f32,  // [m/s²]
    gyro_fsr: f32,   // [rad/s]
    mag_fsr: f32,    // [rad]
) -> (Vector3<f32>, Vector3<f32>, f32) {
    // Simulate Noise (START) ==================================================
    // Generate pure randomness
    let mut rng = rand::thread_rng();
    let gauss = Normal::new(0.0, noise).unwrap();

    // Calculate standard deviations over time
    let s_in_h = 3600.0; // seconds in an hour
    let accel_std = accel_noise/s_in_h * dt.sqrt();
    let gyro_std = gyro_noise/(s_in_h * dt).sqrt();
    let mag_std = mag_noise/(s_in_h * dt).sqrt();

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
        quantize((accel_gt + accel_noise).x, accel_step),
        quantize((accel_gt + accel_noise).y, accel_step),
        quantize((accel_gt + accel_noise).z, accel_step),
    );
    
    let gyro = Vector3::new(
        quantize((gyro_gt + gyro_noise).x, gyro_step),
        quantize((gyro_gt + gyro_noise).y, gyro_step),
        -quantize((gyro_gt + gyro_noise).z, gyro_step), // Invert yaw angular velocity because its NED frame
    );    

    let mag = quantize(mag_gt + mag_noise, mag_step);
    // Simulate Resolution (START) ==================================================



    return (accel, gyro, mag);
}
