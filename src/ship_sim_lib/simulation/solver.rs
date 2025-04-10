// Library for linear algebra
use nalgebra::SVector;

// General RK1 solver for arbitrary size state and input vectors
pub fn rk1_step<const N: usize, const M: usize, F>(
    x: &SVector<f32, N>,
    u: &SVector<f32, M>,
    dt: f32,
    f: F,
) -> SVector<f32, N>
where
    F: Fn(&SVector<f32, N>, &SVector<f32, M>) -> SVector<f32, N>,
{
    let dx = f(x, u);
    x + dx * dt
}

// General RK4 solver using the 3/8-rule
#[allow(unused_variables)]
pub fn rk4_step<const N: usize, const M: usize, F>(
    x: &SVector<f32, N>,
    u: &SVector<f32, M>,
    dt: f32,
    f: F,
) -> SVector<f32, N>
where
    F: Fn(&SVector<f32, N>, &SVector<f32, M>) -> SVector<f32, N>,
{
    let a: [[f32; 4]; 4] = [
        [       0.0,    0.0, 0.0, 0.0],
        [ (1.0/3.0),    0.0, 0.0, 0.0],
        [(-1.0/3.0),    1.0, 0.0, 0.0],
        [       1.0, (-1.0), 1.0, 0.0],
    ];
    let b: [f32; 4] = [(1.0/8.0), (3.0/8.0), (3.0/8.0), (1.0/8.0)];
    let c: [f32; 4] = [      0.0, (1.0/3.0), (2.0/3.0),       1.0];

    let k1 = f(x, u);
    let k2 = f(&(x + dt * a[1][0] * k1), u);
    let k3 = f(&(x + dt * (a[2][0] * k1 + a[2][1] * k2)), u);
    let k4 = f(&(x + dt * (a[3][0] * k1 + a[3][1] * k2 + a[3][2] * k3)), u);

    x + dt * (b[0] * k1 + b[1] * k2 + b[2] * k3 + b[3] * k4)
}

// Runge Kutta Fehlberg method
// Adaptive integral RK of order 4(5)
#[allow(unused_variables)]
pub fn rkf45_step<const N: usize, const M: usize, F>(
    x: &SVector<f32, N>,
    u: &SVector<f32, M>,
    dt: f32,
    f: F,
    tolerances: (f32, f32),
    dt_limits: (f32, f32),
) -> (SVector<f32, N>, SVector<f32, N>, f32)
where
    F: Fn(&SVector<f32, N>, &SVector<f32, M>) -> SVector<f32, N>,
{
    let a: [[f32; 6]; 6] = [
        [            0.0,              0.0,              0.0,             0.0,          0.0, 0.0],
        [      (1.0/4.0),              0.0,              0.0,             0.0,          0.0, 0.0],
        [     (3.0/32.0),       (9.0/32.0),              0.0,             0.0,          0.0, 0.0],
        [(1932.0/2197.0), (-7200.0/2197.0),  (7296.0/2197.0),             0.0,          0.0, 0.0],
        [  (439.0/216.0),           (-8.0),   (3680.0/513.0), (-845.0/4104.0),          0.0, 0.0],
        [    (-8.0/27.0),              2.0, (-3544.0/2565.0), (1859.0/4104.0), (-11.0/40.0), 0.0],
    ];

    let c: [f64; 6] = [0.0, (1.0/4.0), (3.0/8.0), (12.0/13.0), 1.0, (1.0/2.0)];

    // 5th order weights
    let b5 = [
             (16.0/135.0),
                      0.0,
         (6656.0/12825.0),
        (28561.0/56430.0),
              (-9.0/50.0),
               (2.0/55.0),
    ];

    // 4th order weights
    let b4 = [
           (25.0/216.0),
                    0.0,
        (1408.0/2565.0),
        (2197.0/4104.0),
             (-1.0/5.0),
                    0.0,
    ];

    // RK stages
    let mut k: [SVector<f32, N>; 6] = [SVector::zeros(); 6];
    k[0] = f(x, u);
    k[1] = f(&(x + dt * (a[1][0] * k[0])), u);
    k[2] = f(&(x + dt * (a[2][0] * k[0] + a[2][1] * k[1])), u);
    k[3] = f(&(x + dt * (a[3][0] * k[0] + a[3][1] * k[1] + a[3][2] * k[2])), u);
    k[4] = f(&(x + dt * (a[4][0] * k[0] + a[4][1] * k[1] + a[4][2] * k[2] + a[4][3] * k[3])), u);
    k[5] = f(&(x + dt * (a[5][0] * k[0] + a[5][1] * k[1] + a[5][2] * k[2] + a[5][3] * k[3] + a[5][4] * k[4])), u);

    // High-order estimate
    let mut x5 = x.clone();
    for i in 0..6 {
        x5 += dt * b5[i] * k[i];
    }

    // Low-order estimate
    let mut x4 = x.clone();
    for i in 0..6 {
        x4 += dt * b4[i] * k[i];
    }

    // Error estimate
    let err = (x5 - x4).abs().max();

    // Adaptive time step
    // --- Adaptive dt logic ---
    let (tol_min, tol_max) = tolerances;
    let (dt_min, dt_max) = dt_limits;
    let mut new_dt = dt;

    // If error too large, reduce dt
    if err > tol_max {
        new_dt *= 0.9;
        new_dt = new_dt.clamp(dt_min, dt_max);
    }
    // If error small and dt is above min threshold, keep dt
    else if err < tol_min && dt > dt_min {
        new_dt *= 1.1;
        new_dt = new_dt.clamp(dt_min, dt_max);
    }

    // Return data of interest
    let dx = k[0];

    return (x5, dx, new_dt);
}
