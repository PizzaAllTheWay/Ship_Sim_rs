use nalgebra::Vector3;
use rand_distr::{Normal, Distribution}; // Gaussian distribution
use rand::Rng;

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
pub fn simulate(
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
        // Sample 3D Gaussian noise
        let sample = Vector3::new(
            gauss.sample(&mut rng),
            gauss.sample(&mut rng),
            gauss.sample(&mut rng),
        );
        let (x, y, z) = (sample.x, sample.y, sample.z);

        // Hyper-chaotic noise functions with inter-axis coupling and spiral effects
        let f = |x: f32, y: f32, z: f32| {
            let r = (x.powi(2) + y.powi(2)).sqrt().max(1e-3); // radial distance
            let theta = y.atan2(x); // angular component
            let spiral = theta * 0.5; // scaling factor for spiral
            let hyper_decay = 1.0 / (1.0 + r.powi(3)); // 3rd-order hyperbolic falloff
            let z_mod = (z * 0.2).cos(); // vertical modulation
            let value = spiral * hyper_decay * z_mod;
            if value.is_finite() { value } else { 0.0 }
        };        

        let g = |x: f32, y: f32, z: f32| {
            let r = (x.powi(2) + y.powi(2)).sqrt().max(1e-3); // avoid division by zero
            let theta = y.atan2(x); // full-range angle
            let spiral = r * theta.sin() + (theta * 4.0).cos();
            let warp = (z + 1.0).ln_1p().powf(1.2) + (x * z * 0.3).cos();
            let distortion = (z * 0.5).sin() + spiral - warp;
            if distortion.is_finite() { distortion - f(x, y, z) } else { 0.0 }
        };

        let h = |x: f32, y: f32, z: f32| {
            let a = 1.0;
            let theta = (y / x.max(1e-5)).atan();
            let r = a * theta;
            let spiral_xy = r * (theta * 5.0).cos();
            let spiral_z = z * (z * 0.3).sin() + (z * 0.7).cos();
            let chaos = spiral_xy + spiral_z + (x * y * z * 0.1).sin();

            let mut rng = rand::thread_rng();
            let choice = rng.gen_range(0..3); // 0 = f, 1 = g, 2 = chaos

            let result = match choice {
                0 => f(x, y, z),
                1 => g(x, y, z),
                _ => chaos,
            };

            if result.is_finite() {
                result
            } else {
                0.0
            }
        };

        // Return distorted 3D noise vector
        Vector3::new(h(x, y, z), h(x, y, z), h(x, y, z))
    };

    // Final simulated antenna positions = true pos + offset + noisy distortion
    let antenna1 = pos_gt + antenna1_placement + nonlinear_noise();
    let antenna2 = pos_gt + antenna2_placement + nonlinear_noise();

    // Return simulated results
    (antenna1, antenna2)
}
