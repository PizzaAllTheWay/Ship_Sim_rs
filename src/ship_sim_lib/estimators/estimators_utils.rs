// Library for maths
use nalgebra::{SVector, SMatrix, DMatrix};

// Jacobian ----------
// Numerically compute the Jacobian matrix of a function `f` at a point `x`
//
// # Parameters:
// - `f`: A function or closure that maps an N-dimensional input vector to an M-dimensional output vector, i.e., `f: ℝⁿ → ℝᵐ`
// - `x`: The point at which to evaluate the Jacobian (∂f/∂x), given as a vector of size N
// - `epsilon`: A small number used for symmetric finite difference approximation
//
// # Returns:
// - Jacobian matrix of size M×N representing partial derivatives of each output dimension wrt each input dimension:
//     J[i, j] ≈ ∂f_i / ∂x_j
//
// # Method:
// Uses central difference formula for each variable:
//     ∂f/∂x ≈ (f(x+ε) - f(x-ε)) / (2ε)
//
// This is general-purpose, works on any differentiable function that takes in a static `SVector`
// and returns another `SVector`.
pub fn numerical_jacobian<const N: usize, const M: usize, F>(
    f: &F,
    x: &SVector<f32, N>,
    epsilon: f32,
) -> SMatrix<f32, M, N>
where
    F: Fn(&SVector<f32, N>) -> SVector<f32, M>,
{
    // Initialize MxN zero matrix to hold the result
    #[allow(non_snake_case)]
    let mut J = SMatrix::<f32, M, N>::zeros();

    // Loop through each input variable x_i
    for i in 0..N {
        // Copy current state, perturb one component
        let mut x_plus = *x;
        let mut x_minus = *x;

        // Apply symmetric perturbation to i-th component
        x_plus[i] += epsilon;
        x_minus[i] -= epsilon;

        // Evaluate function at both perturbed points
        let f_plus = f(&x_plus);
        let f_minus = f(&x_minus);

        // Finite difference approximation of partial derivatives for column i
        let diff = (f_plus - f_minus) / (2.0 * epsilon);

        // Set the i-th column of Jacobian to the computed gradient vector
        J.set_column(i, &diff);
    }

    // Return full Jacobian matrix
    J
}



// Linearized system ----------
/// Linearize a nonlinear system around the given operating point (x, u)
/// Returns Jacobians A = ∂f/∂x and B = ∂f/∂u in the form required for Kalman Filters.
///
/// Why we linearize:
/// Kalman Filters (especially EKF) require a linear approximation of the nonlinear dynamics f(x, u).
/// We do this by computing Jacobians:
/// - A = ∂f/∂x → describes how small changes in state affect the rate of change of the state.
/// - B = ∂f/∂u → describes how small changes in input affect the rate of change of the state.
///
/// These matrices allow linear propagation of uncertainty and prediction of future state distributions.
///
/// Assumes system is structured as:
///     x = [v_lin; v_ang; pos_lin; pos_ang] with dimension NX = 12
///     u = [rpm, angle] with dimension NU = 2
#[allow(non_snake_case)]
pub fn linearize_system<F, const NX: usize, const NU: usize>(
    f: &F,                                 // full nonlinear dynamics function f(x, u)
    x: &SVector<f32, NX>,                // state vector (linearization point)
    u: &SVector<f32, NU>,                // input vector (linearization point)
    dx: f32,                             // small perturbation for numerical derivative w.r.t. x
    du: f32,                             // small perturbation for numerical derivative w.r.t. u
) -> (SMatrix<f32, NX, NX>, SMatrix<f32, NX, NU>)
where
    F: Fn(&SVector<f32, NX>, &SVector<f32, NU>) -> SVector<f32, NX>,
{
    // Create closures for fixed u and x
    let f_x = |x_: &SVector<f32, NX>| f(x_, u);        // f(x) with u fixed
    let f_u = |u_: &SVector<f32, NU>| f(x, u_);        // f(u) with x fixed

    // Compute Jacobian A: how state affects its own rate of change (∂f/∂x)
    let df_dx: SMatrix<f32, NX, NX> = numerical_jacobian(&f_x, x, dx);

    // Compute Jacobian B: how control inputs affect state rate of change (∂f/∂u)
    let df_du: SMatrix<f32, NX, NU> = numerical_jacobian(&f_u, u, du);

    (df_dx, df_du)
}

// Exponential matrix ----------
// Approximates the matrix exponential using Padé approximation with scaling and squaring
// Used for discretizing continuous-time systems
#[allow(non_snake_case)]
pub fn expm(A: DMatrix<f32>) -> DMatrix<f32> {
    // Compute matrix norm to determine if scaling is needed
    let norm = A.norm();
    let maxnorm = 5.4; // empirically chosen threshold for scaling stability

    // Apply scaling if norm too large to avoid overflow/instability in polynomial approximation
    let (s, A_scaled) = if norm > maxnorm {
        let s = (norm / maxnorm).log2().ceil() as u32; // compute scaling factor
        let scale = 1.0 / (2.0f32).powi(s as i32);      // 2^-s
        (s, A * scale)
    } else {
        (0, A)
    };

    // Compute powers of A_scaled needed for Padé approximation
    let A2 = &A_scaled * &A_scaled;
    let A4 = &A2 * &A2;
    let A6 = &A2 * &A4;

    // Identity matrix for dimension matching
    let I = DMatrix::identity(A_scaled.nrows(), A_scaled.ncols());

    // Compute numerator and denominator of [Pade(6,6)] rational approximation
    let u = &A_scaled * (&A6 * 0.000000025 + &A4 * 0.000001 + &A2 * 0.0002 + &I);
    let v = &A6 * 0.000000025 + &A4 * 0.000001 + &A2 * 0.0002 - &I;

    // Form (U + V)(U - V)^(-1)
    let numer = &u + &v;
    let denom = &u - &v;

    // Final matrix exponential for scaled A
    let mut expA = denom.try_inverse().unwrap() * numer;

    // Apply scaling back by squaring result s times
    for _ in 0..s {
        expA = &expA * &expA;
    }

    expA
}

// Discretize system in state space ---------- 
// Discretizes continuous-time system matrices A and B using Zero-Order Hold (ZOH) method
// This is done by computing the matrix exponential of the augmented system [A B; 0 0]
// Returns the discrete-time equivalents (Ad, Bd)
#[allow(non_snake_case)]
pub fn discretize_ab_zoh<const N: usize, const M: usize>(
    A: &SMatrix<f32, N, N>, // Continuous-time state transition matrix
    B: &SMatrix<f32, N, M>, // Continuous-time control input matrix
    dt: f32,                // Discretization timestep (Δt)
) -> (SMatrix<f32, N, N>, SMatrix<f32, N, M>) {
    // Create (N+M)x(N+M) augmented matrix to capture both A and B
    // Layout:
    // [ A  B ]
    // [ 0  0 ]
    let mut AB_aug = DMatrix::<f32>::zeros(N + M, N + M);

    // Copy A and B into the top part of the matrix
    let A_d = DMatrix::from_row_slice(N, N, A.as_slice());
    let B_d = DMatrix::from_row_slice(N, M, B.as_slice());

    AB_aug.view_mut((0, 0), (N, N)).copy_from(&A_d);     // Top-left: A
    AB_aug.view_mut((0, N), (N, M)).copy_from(&B_d);     // Top-right: B

    // Compute matrix exponential of augmented system: exp([A B; 0 0] * dt)
    let AB_exp = expm(AB_aug * dt);

    // Extract Ad (top-left NxN) and Bd (top-right NxM) from result
    let mut Ad = SMatrix::<f32, N, N>::zeros();
    for i in 0..N {
        for j in 0..N {
            Ad[(i, j)] = AB_exp[(i, j)];
        }
    }

    let mut Bd = SMatrix::<f32, N, M>::zeros();
    for i in 0..N {
        for j in 0..M {
            Bd[(i, j)] = AB_exp[(i, j + N)];
        }
    }

    // Return discretized system matrices
    (Ad, Bd)
}

/// Print any matrix (SMatrix) nicely with fixed formatting
///
/// # Parameters:
/// - `name`: Matrix label to print before the data
/// - `matrix`: The matrix to print, supports any dimensions R×C
///
/// # Output:
/// Example for 3x2:
/// A = [
///   [   1.0000000,   2.0000000, ],
///   [   3.0000000,   4.0000000, ],
///   [   5.0000000,   6.0000000, ],
/// ]
pub fn print_matrix<T: std::fmt::Display, const R: usize, const C: usize>(
    name: &str,
    matrix: &nalgebra::SMatrix<T, R, C>
) {
    println!("{} = [", name);
    for r in 0..R {
        print!("  [");
        for c in 0..C {
            // Print each value with consistent spacing
            print!("{:>12.8}, ", matrix[(r, c)]);
        }
        println!("],");
    }
    println!("]");
}

pub fn wrap_angle(angle: f32) -> f32 {
    (angle + std::f32::consts::PI).rem_euclid(2.0 * std::f32::consts::PI) - std::f32::consts::PI
}


