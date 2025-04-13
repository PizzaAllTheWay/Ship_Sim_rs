use nalgebra::Vector3;
use rand_distr::{Normal, Distribution}; // Gaussian distribution

/// Simulates noisy GNSS antenna measurements in 3D space with nonlinear distortions
///
/// # Arguments
/// * `pos_gt` - Ground truth position of the ship in body/world frame
/// * `antenna1_placement` - Relative position of antenna 1 from ship center (e.g. +10m fwd)
/// * `antenna2_placement` - Relative position of antenna 2 from ship center (e.g. -10m aft)
/// * `noise` - Multiplier for randomness (e.g. 1.0 = 100%)
/// * `accuracy` - Standard deviation of GNSS error in meters (e.g. 0.5 = ±0.5m)
///
/// # Returns
/// A tuple with two 3D vectors: simulated positions of antenna1 and antenna2
pub fn simulate_gnss(
    pos_gt: Vector3<f32>,
    antenna1_placement: Vector3<f32>,
    antenna2_placement: Vector3<f32>,
    noise: f32,
    accuracy: f32,
) -> (Vector3<f32>, Vector3<f32>) {
    // Random number generator
    let mut rng = rand::thread_rng();

    // Create normal (Gaussian) distribution with 0 mean and scaled std deviation
    let gauss = Normal::new(0.0, noise * accuracy).unwrap();

    // Function to generate nonlinear distorted Gaussian noise
    let mut nonlinear_noise = || {
        // Sample 3D noise from Gaussian
        let sample = Vector3::new(
            gauss.sample(&mut rng),
            gauss.sample(&mut rng),
            gauss.sample(&mut rng),
        );

        // Extremely nonlinear and different functions
        let f = |x: f32| {
            // Trig-based chaos with tan protection
            let tan_val = (x + 1.0).tan();
            let val = (x.sin() * (x * 0.3).cos()) - tan_val;
        
            if val.is_finite() {
                val.tanh()
            } else {
                0.0
            }
        };
        
        let g = |x: f32| {
            // Smooth exponential + sqrt with safety
            let safe_x = x.abs().max(1e-4); // Avoid ln(0)
            let val = ((safe_x.sqrt() + safe_x.exp()).ln_1p() * (safe_x / 3.0).exp()).sin();
            if val.is_finite() { val } else { 0.0 }
        };
        
        let h = |x: f32| {
            // Chaotic but avoid NaN
            let safe_x = x.abs().max(1e-4);
            let raw = (safe_x.powf(1.618) * (safe_x * 0.7).sin() + safe_x.powi(5) - (1.0 + safe_x).ln()).tanh();
            if raw.is_finite() {
                (raw * 10.0).fract() * 2.0 - 1.0
            } else {
                0.0
            }
        };
        

        // Return distorted 3D noise vector
        Vector3::new(f(sample.x), g(sample.y), h(sample.z))
    };

    // Final simulated antenna positions = true pos + offset + noisy distortion
    let antenna1 = pos_gt + antenna1_placement + nonlinear_noise();
    let antenna2 = pos_gt + antenna2_placement + nonlinear_noise();

    // Return simulated results
    (antenna1, antenna2)
}
