use nalgebra::{SVector, SMatrix};

/// Numerically compute the Jacobian matrix of a function `f` at a point `x`
///
/// # Parameters:
/// - `f`: A function or closure that maps an N-dimensional input vector to an M-dimensional output vector, i.e., `f: ℝⁿ → ℝᵐ`
/// - `x`: The point at which to evaluate the Jacobian (∂f/∂x), given as a vector of size N
/// - `epsilon`: A small number used for symmetric finite difference approximation
///
/// # Returns:
/// - Jacobian matrix of size M×N representing partial derivatives of each output dimension wrt each input dimension:
///     J[i, j] ≈ ∂f_i / ∂x_j
///
/// # Method:
/// Uses central difference formula for each variable:
///     ∂f/∂x ≈ (f(x+ε) - f(x-ε)) / (2ε)
///
/// This is general-purpose, works on any differentiable function that takes in a static `SVector`
/// and returns another `SVector`.
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

pub fn clamp_matrix<const N: usize, const M: usize>(mat: SMatrix<f32, N, M>, max_val: f32) -> SMatrix<f32, N, M> {
    mat.map(|x| {
        if x.is_finite() {
            x.clamp(-max_val, max_val)
        } else {
            0.0
        }
    })
}

