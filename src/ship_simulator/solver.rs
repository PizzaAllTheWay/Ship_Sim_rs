// Library for linear algebra
use nalgebra::Vector3;

// Euler forward method
pub fn rk1_step(x: &Vector3<f32>, dx: &Vector3<f32>, dt: f32) -> Vector3<f32>
{
    let b: [f32; 1] = [1.0];
    let k1 = dx;
    let x = x + dt * (b[0] * k1);

    return x;
}

/*
fn rk1_step<F>(x: &Vector3<f32>, dt: f32, ode: F) -> Vector3<f32>
where
    F: Fn(&Vector3<f32>) -> Vector3<f32>,
{
    let dx = ode(x);
    x + dx * dt
}
*/

/// RK2 (midpoint) using arrays for Butcher tableau constants
pub fn rk2_step<F>(x: &Vector3<f32>, dt: f32, ode: F) -> Vector3<f32>
where
    F: Fn(&Vector3<f32>) -> Vector3<f32>,
{
    // Butcher tableau for RK2 midpoint
    let a: [[f32; 2]; 2] = [
        [0.0, 0.0],
        [0.5, 0.0],
    ];
    let b: [f32; 2] = [0.0, 1.0];
    let c: [f32; 2] = [0.0, 0.5];

    // RK2 steps
    let k1 = ode(x);
    let k2 = ode(&(x + dt * a[1][0] * k1));

    x + dt * (b[0] * k1 + b[1] * k2)
}

/// RK4 integrator (3/8-rule) for solving dx/dt = f(x)
/// Inputs:
/// - `x`: current state vector
/// - `dx`: derivative (e.g. acceleration or velocity)
/// - `dt`: time step
/// Output:
/// - new state vector after one step
pub fn rk4_step(x: &Vector3<f32>, dx: &Vector3<f32>, dt: f32) -> Vector3<f32> {
    // Coefficients
    let c: [f64; 4] = [      0.0, (1.0/3.0), (2.0/3.0),     (1.0)];
    let b = [(1.0/8.0), (3.0/8.0), (3.0/8.0), (1.0/8.0)];
    let a = [
        [       0.0,    0.0, 0.0, 0.0],
        [ (1.0/3.0),    0.0, 0.0, 0.0],
        [(-1.0/3.0),    1.0, 0.0, 0.0],
        [       1.0, (-1.0), 1.0, 0.0],
    ];

    // k vectors
    let k1 = dx.clone();
    let k2 = dx + k1 * a[1][0] * dt;
    let k3 = dx + k1 * a[2][0] * dt + k2 * a[2][1] * dt;
    let k4 = dx + k1 * a[3][0] * dt + k2 * a[3][1] * dt + k3 * a[3][2] * dt;

    // Final step
    x + (b[0] * k1 + b[1] * k2 + b[2] * k3 + b[3] * k4) * dt
}

