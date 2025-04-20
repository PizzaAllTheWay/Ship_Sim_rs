// Library for maths
use nalgebra::{SVector, SMatrix, DMatrix};

// Library for multithreading
use std::sync::{Arc, RwLock};




// Custom data types ----------
pub type Vector9<T> = SVector<T, 9>;
pub type Vector12<T> = SVector<T, 12>;
pub type Matrix9x9<T> = SMatrix::<T, 9, 9>;
pub type Matrix12x2<T> = SMatrix::<T, 12, 2>;
pub type Matrix9x12<T> = SMatrix::<T, 9, 12>;
pub type Matrix12x9<T> = SMatrix::<T, 12, 9>;
pub type Matrix12x12<T> = SMatrix::<T, 12, 12>;

#[allow(non_snake_case)]
#[derive(Clone, Default)]
pub struct SharedState {
    pub x_est_post: Arc<RwLock<Vector12<f32>>>,
    pub P_post: Arc<RwLock<Matrix12x12<f32>>>,

    pub x_est_pri: Arc<RwLock<Vector12<f32>>>,
    pub P_pri: Arc<RwLock<Matrix12x12<f32>>>,
}











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
fn expm(A: DMatrix<f32>) -> DMatrix<f32> {
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
fn discretize_ab_zoh<const N: usize, const M: usize>(
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








#[allow(non_snake_case)]
pub fn predict<F, const NX: usize, const NU: usize>(
    f: F,                                     // Nonlinear function f(x, u)
    dt: f32,                                  // Time step [s]
    x_est_post: SVector<f32, NX>,             // Last corrected state estimate
    u: SVector<f32, NU>,                      // Last control input
    P_post: SMatrix<f32, NX, NX>,             // Last corrected state covariance
    Q: SMatrix<f32, NX, NX>,                  // Process noise covariance
) -> (
    SVector<f32, NX>,                         // Predicted state
    SMatrix<f32, NX, NX>,                     // Predicted covariance
)
where
    F: Fn(&SVector<f32, NX>, &SVector<f32, NU>) -> SVector<f32, NX>,
{   
    // Linearize our system around the given work point
    // We linearize the nonlinear function f(x, u) at the current operating point.
    // This gives us the Jacobians:
    //   A = ∂f/∂x : how state affects change of state
    //   B = ∂f/∂u : how input affects change of state
    //
    // These are used to propagate uncertainty and discretize the system model.
    let (A, B) = linearize_system(&f, &x_est_post, &u, 1e-4, 1e-4);

    // Get discretized state matrixes
    // In order to do proper estimation we use euler forward method
    // This yields us state estimate using only linearized model
    // x_est_priori[k] = x_est[k-1] + dt*x_dot(t[k-1])
    //      x_dot(t[k-1]) = A*x_est[k-1] + B*u[k-1]
    // x_est_priori[k] = x_est[k-1] + dt*(A*x_est[k-1] + B*u[k-1])
    // x_est_priori[k] = x_est[k-1] + dt*A*x_est[k-1] + dt*B*u[k-1]
    // x_est_priori[k] = (I + dt*A)*x_est[k-1] + (dt*B)*u[k-1]
    // x_est_priori[k] = F_d*x_est[k-1] + B_d*u[k-1]
    //      F_d = I + dt*A
    //      B_d = dt*B
    //
    // x_est_priori[k]: Current estimate using ONLY model
    // x_est[k-1]: Corrected previous estimate using model AND measurements
    // u[k-1]: Previous control input
    // F_d: Discrete state transition matrix
    // B_d: Discrete control input matrix
    //
    // However for EKF the prediction itself does not use the linearized state matrixes
    // Instead it uses the non linear ODE f(x, u) straight
    // In addition it uses explicit solver to predict the next states
    // More specifically euler forward method to predict our next state based on f(x, u) work point
    // x[k] = x[k-1] + dt*f(x[k-1], u[k-1])
    // x_est_priori[k] = x_est[k-1] + dt*f(x_est[k-1], u[k-1])
    let (F_d, B_d) = discretize_ab_zoh::<NX, NU>(&A, &B, dt);
    let x_est_pri: SVector<f32, NX> = x_est_post + dt * f(&x_est_post, &u);

    // Calculate state uncertainty
    // We must calculate how uncertain we are with the estimate using only model to estimate
    // Over time if no correction is made we will get higher and higher uncertainty
    // P_priori[k] = F_d*P[k-1]*F_d.T + Q
    // 
    // P_priori[k]: Current state uncertainty BEFORE measurements (state estimated using only model)
    // P[k-1]: Previous corrected uncertainty AFTER measurements
    // Q: Trust matrix for our model, each diagonal value represents how much we trust that model is correct on that particular state
    //      Q << 1 => Trust the model A LOT
    //      Q >> 1 => DON'T trust the model that much
    let P_pri: SMatrix<f32, NX, NX> = F_d*P_post*F_d.transpose() + Q;

    // Return the estimate and the uncertainty
    return (x_est_pri, P_pri);
}



#[allow(non_snake_case)]
pub fn correct<F, const NX: usize, const NY: usize>(
    h: F,                                        // Nonlinear measurement function h(x)
    z: SVector<f32, NY>,                         // Current measurement
    x_est_pri: SVector<f32, NX>,                 // Prior state estimate (predicted)
    P_pri: SMatrix<f32, NX, NX>,                 // Prior covariance estimate (predicted)
    R: SMatrix<f32, NY, NY>,                     // Measurement noise covariance matrix
) -> (
    SVector<f32, NX>,                            // Corrected state estimate
    SMatrix<f32, NX, NX>,                        // Corrected covariance matrix
)
where
    F: Fn(&SVector<f32, NX>) -> SVector<f32, NY>,
{
    // Linearize measurement matrix
    // In order to compare measurements with estimates, we must first transform estimates to reflect measurement space
    // We do that by running h(x) to get estimates in measurement space
    // However h(x) is highly non linear because of all the transforms
    // For normal KF we must get H matrix
    // We do that by linearizing h(x) with respect to x
    // H = dh/dx = Jacobian(h(x), x)
    let H = numerical_jacobian(&h, &x_est_pri, 1e-4);

    // Calculate Innovation Residual
    // Fist check if what model estimated and what was measured is the same
    // 99.999% of the time they are not the same
    // This leads to a residual error between measured and model estimated value
    // In order to compare estimate with measured value that are in different state spaces we must preform a transform matrix
    // More exactly, apply measurement matrix (H) to model estimate to get it to measurement space
    // Now measurement and model estimate transformed to measurement can be compared to get the residual
    // y[k] = z[k] - H*x_est_priori[k]
    //
    // y[k]: Current Innovation Residual that tells how far off measurement and estimate are
    // z[k]: Current Measurement
    // H: measurement transform matrix, transforms estimate to measurement space
    // x_est_priori[k]: Current estimate using ONLY model to predict next states
    //
    // However since we are using EKF we don't use H linearization for comparing predicted measurement and measured value
    // We use H only for uncertainty calculation transform
    // For the Innovation Residual itself we can use the non linear measurement transform, ie:
    // predicted measurement = non-linear measurement transform of predicted states = h(x_est_pri[k])
    // y[k] = z[k] - h(x_est_pri[k])
    let y: SVector<f32, NY> = z - h(&x_est_pri);

    // Calculate Innovation Covariance
    // Just like with estimate uncertainty prior and post measurement, so does measurement have some uncertainty attached to them
    // This time however, uncertainty represents model estimate measurement (the model estimate we just transformed H*x_est_priori[k])
    // It is a product of model ONLY estimate uncertainty (P_priori) and measurement uncertainty
    // S[k] = H*P_priori[k]*H.T + R
    //
    // S[k]: Current Innovation Covariance, it represents the uncertainty of the estimate measurement (ie the x_est_priori[k] we just transformed using H matrix)
    // R: Uncertainty from sensor (Found by taking measurements of the sensor and getting variance of the different measurement states)
    //      R << 1 => Trust the measurements A LOT
    //      R >> 1 => DON'T trust the measurements that much
    let S: SMatrix<f32, NY, NY> = H*P_pri*H.transpose() + R;

    // Calculate Kalman Gain
    // Now we know from before hand, estimate uncertainty BEFORE measurement (ie only estimate using model x_est_priori[k])
    // We also just calculated estimate measurement uncertainty (ie estimate of the measurement S[k])
    // If we now take proportions between:
    // estimate uncertainty BEFORE measurement
    // vs 
    // estimate uncertainty AFTER measurement
    // Then we get kalman gain for each measurement error state we must compensate for to get optimal state between estimate and corrected estimate
    // Of course we must first transform estimate uncertainty BEFORE measurement into measurement space using measurement transform matrix (H)
    // Only then can we take proportions to get kalman gains 
    // K[k] = P_priori[k]*H.T*(S[k])⁽⁻¹⁾
    //
    // K[k]: Current Kalman Gain, tells how much we must reduce/increase Innovation Residual to get perfect blend between Estimate and Measurement
    let S_inv = S.try_inverse().unwrap();
    let K: SMatrix<f32, NX, NY> = P_pri*H.transpose()*S_inv;

    // Correct estimate using model AND measurement
    // To know how much to subtract/add from estimate priori, we must utilize kalman gain on Innovation Residual
    // This will give optimal balance of how much to add to each estimate prior to get a good balance between estimate and measurement
    // x_est[k] = x_est_priori[k] + K[k]*y[k]
    let x_est: SVector<f32, NX> = x_est_pri + K*y;

    // Correct Estimate Uncertainty
    // Before we finish, we just corrected estimate
    // However if we don't correct the uncertainty the estimator function will still believe that there is a large uncertainty in estimate
    // This would make model estimate less and less reliable over time even after correcting, when we know that model estimate is better after correction, not worse
    // To solve this we simply correct the estimate uncertainty as well
    // Of course the amount we must correct is proportional to the kalman gain
    // However we must perform a state transition in order for Correction to be applied properly in estimate states space 
    // P[k] = (I - K[k]*H)*P_priori[k]
    //
    // P[k]: Current state uncertainty AFTER measurements/correction
    let I = SMatrix::<f32, NX, NX>::identity();
    let P: SMatrix<f32, NX, NX> = (I - K*H)*P_pri;

    // Return results
    return (x_est, P);
}