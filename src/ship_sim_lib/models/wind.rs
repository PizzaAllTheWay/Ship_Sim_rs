use rand_distr::{Distribution, Normal};

pub struct WindState {
    pub speed_noise: f32,
    pub angle_noise: f32,
}

pub fn simulate(
    speed: f32,
    angle_deg: f32,
    noise: f32,
    max_speed: f32,
    state: &mut WindState,
) -> (f32, f32) {
    let mut noisy_speed = speed;
    let mut noisy_angle = angle_deg;

    if speed > 0.0 {
        // Constants (do not change)
        let noise_abs = 0.05 * noise * max_speed;
        let bias = 0.4 * noise_abs;
        let angle_noise_range = 0.01 * noise * 360.0;

        // Slow noise update (low-pass random walk)
        let smoothing_speed = 0.90; // how slow the noise changes (close to 1 = slow)
        let smoothing_angle = 0.95; // how slow the noise changes (close to 1 = slow)
        let speed_dist = Normal::new(bias, noise_abs).unwrap();
        let angle_dist = Normal::new(0.0, angle_noise_range).unwrap();

        state.speed_noise = smoothing_speed * state.speed_noise + (1.0 - smoothing_speed) * speed_dist.sample(&mut rand::thread_rng());
        state.angle_noise = smoothing_angle * state.angle_noise + (1.0 - smoothing_angle) * angle_dist.sample(&mut rand::thread_rng());

        noisy_speed = (speed + state.speed_noise).clamp(0.0, max_speed);
        noisy_angle = angle_deg + state.angle_noise;
    }

    (noisy_speed, noisy_angle)
}