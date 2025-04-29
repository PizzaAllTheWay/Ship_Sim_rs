// Library for maths
use nalgebra::{SVector, SMatrix};

// Library for multithreading
use std::sync::{Arc, RwLock};



// Data Structure (START) ==================================================
#[derive(Clone)]
pub struct SigmaPoints<const N: usize, const TWO_N: usize> {
    pub x_0: SVector<f32, N>, // Central Sampling point
    pub x_i: [SVector<f32, N>; TWO_N], // Points surrounding the central point and their mirrored samples
}

#[derive(Clone, Copy)]
pub struct Weights<const N: usize, const TWO_N: usize> {
    pub w_m_0: f32, // Central sigma point median weight
    pub w_m_i: [f32; TWO_N], // Sigma points surrounding the central sigma point and their median weights

    pub w_c_0: f32, // Central sigma point covariance weight
    pub w_c_i: [f32; TWO_N], // Sigma points surrounding the central sigma point and their covariance weights
}

#[allow(non_snake_case)]
#[derive(Clone)]
pub struct SharedState<const N: usize, const TWO_N: usize> {
    pub x_est_post: Arc<RwLock<SVector<f32, N>>>, // Estimate after measurement/correction
    pub P_post: Arc<RwLock<SMatrix<f32, N, N>>>,  // Estimate Uncertainty after measurement/correction
    pub x_est_pri: Arc<RwLock<SVector<f32, N>>>,  // Estimate BEFORE measurement/correction (ie only using model)
    pub P_pri: Arc<RwLock<SMatrix<f32, N, N>>>,   // Estimate Uncertainty BEFORE measurement/correction (ie only using model)

    pub lambda: Arc<RwLock<f32>>,                         // controls spread of sigma points around mean
    pub weights: Arc<RwLock<Weights<N, TWO_N>>>,          // Unscented Form wights
    pub sigma_points: Arc<RwLock<SigmaPoints<N, TWO_N>>>, // Unscented Form samples
}
// Data Structure (STOP) ==================================================



// Utility Functions (START) ==================================================
/// Calculate the "square root" of a positive definite matrix using Cholesky decomposition.
/// 
/// # Arguments
/// * `matrix` - The matrix you want the square root of. Must be positive definite.
///
/// # Returns
/// * Lower triangular matrix L such that:  
///     L * Lᵀ = matrix
///
/// # Explanation
/// - In Kalman filters (and Unscented Kalman Filter), 
///   when we generate sigma points, we need a "square root" of the covariance matrix.
/// - The "square root" here is not element-wise, but matrix-wise:  
///   Find a matrix **L** such that:  
///   P = L * L^T
/// - We use **Cholesky decomposition** to efficiently and stably get this.
///   (Only works for symmetric positive definite matrices.)
/// - Why Cholesky?
///   - Very fast (O(n³) but 2x faster than eigenvalue methods).
///   - Numerically stable for good matrices.
///   - Unique lower-triangular form.
/// - If Cholesky fails, your matrix P is not positive definite -> serious bug upstream
pub fn sqrt_matrix<const N: usize>(matrix: &SMatrix<f32, N, N>) -> SMatrix<f32, N, N> {
    matrix.clone()
        .cholesky()
        .expect("Cholesky decomposition failed! The matrix must be positive definite.")
        .l()
}

/// Calculate sigma points for Unscented Kalman Filter
/// 
/// # Description
/// In normal Kalman filters (linear), prediction of mean/covariance is easy.
/// In nonlinear cases, EKF approximates using first-order linearization (bad if system is highly nonlinear).
///
/// UKF improves:
/// - It represents the distribution by **sampling** special "sigma points" around the mean.
/// - These points are propagated through the **nonlinear** function.
/// - Then you recover the new mean/covariance by weighted averaging.
///
/// This process captures mean and covariance **up to third-order accuracy** for Gaussian inputs.
///
/// 
/// # Algorithm
/// - Given prior mean `x_est` and covariance `P`.
/// - Define number of states `L = n_states`.
/// - Choose scaling parameter `lambda = alpha^2 * (L + kappa) - L`.
/// - Compute **matrix square root** of `(L + lambda) * P`.
/// - Sigma points:
///   - Central point:  
///     x₀ = x_est
///   - First L points (positive side):  
///     xᵢ = x_est + (i-th column of sqrt[(L+lambda)P])
///   - Next L points (negative side):  
///     xᵢ₊ₗ = x_est - (i-th column of sqrt[(L+lambda)P])
///
/// 
/// # Arguments
/// * `x_est` - State estimate (posterior mean).
/// * `P` - State covariance matrix (posterior uncertainty).
/// * `L` - State dimension (number of states).
/// * `lambda` - Scaling parameter.
///
/// # Returns
/// * `(x_0, x_i)`
///   * `x_0` - Central sigma point (SVector).
///   * `x_i` - Array of 2*L sigma points (positive and negative deviations).
///
/// # Panics
/// Panics if matrix square root (Cholesky) fails.
/// 
/// # Notes
/// - Accurate to 3rd order for Gaussian distributions.
/// - Needs only 2L + 1 points, **no random sampling**.
/// - Much faster than particle filters.
/// - Only valid if P is symmetric positive definite.
#[allow(non_snake_case)]
pub fn calc_sigma_points<const N: usize, const TWO_N: usize>(
    x_est: SVector<f32, N>,
    P: SMatrix<f32, N, N>,
    lambda: f32,
) -> SigmaPoints<N, TWO_N> {
    // Calculate central sigma point
    // Literally just the estimate lol
    let sigma_point_center: SVector<f32, N> = x_est;

    // Scale the covariance matrix and find its square root
    let L = N as f32;
    let P_scaled: SMatrix<f32, N, N> = (L + lambda) * P;
    let P_sqrt: SMatrix<f32, N, N> = sqrt_matrix(&P_scaled);

    // Initialize array of sigma points
    let mut sigma_points = [SVector::<f32, N>::zeros(); TWO_N];

    // Generate sigma points
    for i in 0..N {
        let offset = P_sqrt.column(i);
        sigma_points[i] = x_est + offset;
        sigma_points[i + N] = x_est - offset;
    }

    // Return central sigma point and array of sigma points
    SigmaPoints {
        x_0: sigma_point_center,
        x_i: sigma_points,
    }
}

/// Perform simple Euler forward integration
/// 
/// # Arguments
/// * `x` - Current state (SVector)
/// * `u` - Current control input (SVector)
/// * `f` - Nonlinear system dynamics function f(x, u)
/// * `dt` - Time step [s]
/// 
/// # Returns
/// * `x_next` - Predicted next state after dt using Euler Forward method
pub fn euler_forward<F, const NX: usize, const NU: usize>(
    x: &SVector<f32, NX>,
    u: &SVector<f32, NU>,
    f: &F,
    dt: f32,
) -> SVector<f32, NX>
where
    F: Fn(&SVector<f32, NX>, &SVector<f32, NU>) -> SVector<f32, NX>,
{
    x + dt * f(x, u)
}

/// Calculate the cross-covariance matrix between two sets of propagated sigma points.
/// 
/// # Arguments
/// * `mean_a` - Mean of set A (e.g., predicted state or predicted measurement)
/// * `mean_b` - Mean of set B (e.g., predicted state or predicted measurement)
/// * `sigma_a_0` - Central sigma point of set A
/// * `sigma_b_0` - Central sigma point of set B
/// * `sigma_a` - Other sigma points of set A
/// * `sigma_b` - Other sigma points of set B
/// * `w_c_0` - Covariance weight for the central sigma point
/// * `w_c_i` - Covariance weights for the other sigma points
///
/// # Returns
/// * Cross-covariance matrix between A and B
///
/// # Note
/// This function can compute:
/// - State covariance (P) when A = B
/// - Innovation covariance (S) when A = predicted measurements, B = predicted measurements
/// - Cross covariance (Pxz) when A = predicted states, B = predicted measurements
#[allow(non_snake_case)]
pub fn calc_unscented_covariance<const NA: usize, const NB: usize, const TWO_N: usize>(
    mean_a: &SVector<f32, NA>,
    mean_b: &SVector<f32, NB>,
    sigma_a_0: &SVector<f32, NA>,
    sigma_b_0: &SVector<f32, NB>,
    sigma_a: &[SVector<f32, NA>; TWO_N],
    sigma_b: &[SVector<f32, NB>; TWO_N],
    w_c_0: f32,
    w_c_i: &[f32; TWO_N],
) -> SMatrix<f32, NA, NB> {
    let mut P = SMatrix::<f32, NA, NB>::zeros();

    // Central point contribution
    let da0 = sigma_a_0 - mean_a;
    let db0 = sigma_b_0 - mean_b;
    P += w_c_0 * da0 * db0.transpose();

    // Other sigma points contribution
    for i in 0..TWO_N {
        let da = sigma_a[i] - mean_a;
        let db = sigma_b[i] - mean_b;
        P += w_c_i[i] * da * db.transpose();
    }

    P
}

#[allow(non_snake_case)]
pub fn calc_lambda<const N: usize>(
    kappa: f32,
    alpha: f32,
) -> f32 {
    let L = N as f32;

    alpha.powi(2) * (L + kappa) - L
}

#[allow(non_snake_case)]
pub fn calc_weights<const N: usize, const TWO_N: usize>(
    lambda: f32, // Scaling parameter
    alpha: f32,  // Spread of the sigma points
    beta: f32,   // Prior knowledge about distribution
) -> Weights<N, TWO_N> {
    let L = N as f32;

    let w_m_0 = lambda / (L + lambda);
    let w_c_0 = w_m_0 + (1.0 - alpha.powi(2) + beta);

    let mut w_m_i = [0.0; TWO_N];
    let mut w_c_i = [0.0; TWO_N];
    for i in 0..TWO_N {
        w_m_i[i] = 1.0 / (2.0 * (L + lambda));
        w_c_i[i] = w_m_i[i];
    }

    Weights {
        w_m_0,
        w_m_i,
        w_c_0,
        w_c_i,
    }
}
// Utility Functions (STOP) ==================================================



// Unscented Kalman Filter (START) ==================================================
#[allow(non_snake_case)]
pub fn predict<F, const NX: usize, const TWO_NX: usize, const NU: usize>(
    f: F,                                     // Nonlinear function f(x, u)
    dt: f32,                                  // Time step [s]
    x_est_post: SVector<f32, NX>,             // Last corrected state estimate
    u: SVector<f32, NU>,                      // Last control input
    P_post: SMatrix<f32, NX, NX>,             // Last corrected state covariance
    Q: SMatrix<f32, NX, NX>,                  // Process noise covariance
    lambda: f32,                              // controls spread of sigma points around mean
    weights: Weights<NX, TWO_NX>,             // Weights for sigma points
) -> (
    SVector<f32, NX>,                         // Predicted state
    SMatrix<f32, NX, NX>,                     // Predicted covariance
)
where
    F: Fn(&SVector<f32, NX>, &SVector<f32, NU>) -> SVector<f32, NX>,
{   
    // Calculate sigma points
    // These are unscented transform points
    // These sigma points will be used later to propagate into the nonlinear model to get propagated priori estimate points
    // Then these propagated priori estimate points will be used to calculate final priori estimate
    // The whole UKF hinges on this Unscented transform 
    let sigma_points: SigmaPoints<NX, TWO_NX> = calc_sigma_points(x_est_post, P_post, lambda);

    // Propagate sigma points into priori estimate points
    // Then these points will be later merged to create final priori estimate 
    let x_est_pri_0: SVector<f32, NX> = euler_forward(&sigma_points.x_0, &u, &f, dt);
    let mut x_est_pri_i = [SVector::<f32, NX>::zeros(); TWO_NX];
    for i in 0..NX {
        x_est_pri_i[i] = euler_forward(&sigma_points.x_i[i], &u, &f, dt);
        x_est_pri_i[i + NX] = euler_forward(&sigma_points.x_i[i + NX], &u, &f, dt);
    }

    // Calculate priori estimate
    // Take all the priori estimate points and weight them
    // Then combine all of them into a single priori estimate
    let mut x_est_pri: SVector<f32, NX> = weights.w_m_0 * x_est_pri_0;
    for i in 0..TWO_NX {
        x_est_pri += weights.w_m_i[i] * x_est_pri_i[i];
    }

    // Calculate state uncertainty
    // We must calculate how uncertain we are with the estimate using only model to estimate
    // Over time if no correction is made we will get higher and higher uncertainty
    // This time instead of normal EKF method we will use unscented transform to calculate uncertainty
    // In addition we must not forget to add Q matrix
    // This ensures we propagate more accurate amount of uncertainty
    //
    // Q: Trust matrix for our model, each diagonal value represents how much we trust that model is correct on that particular state
    //      Q << 1 => Trust the model A LOT
    //      Q >> 1 => DON'T trust the model that much
    let mut P_pri: SMatrix<f32, NX, NX> = calc_unscented_covariance(
        &x_est_pri,
        &x_est_pri,
        &x_est_pri_0,
        &x_est_pri_0,
        &x_est_pri_i,
        &x_est_pri_i,
        weights.w_c_0,
        &weights.w_c_i,
    );
    P_pri += Q;

    // Return the estimate and the uncertainty
    return (x_est_pri, P_pri);
}

#[allow(non_snake_case)]
pub fn correct<F, const NX: usize, const TWO_NX: usize, const NY: usize>(
    h: F,                                        // Nonlinear measurement function h(x)
    y: SVector<f32, NY>,                         // Current measurement
    x_est_pri: SVector<f32, NX>,                 // Prior state estimate (predicted)
    P_pri: SMatrix<f32, NX, NX>,                 // Prior covariance estimate (predicted)
    R: SMatrix<f32, NY, NY>,                     // Measurement noise covariance matrix
    lambda: f32,                                 // controls spread of sigma points around mean
    weights: Weights<NX, TWO_NX>,                // Weights for sigma points
) -> (
    SVector<f32, NX>,                            // Corrected state estimate
    SMatrix<f32, NX, NX>,                        // Corrected covariance matrix
)
where
    F: Fn(&SVector<f32, NX>) -> SVector<f32, NY>,
{
    // Calculate sigma points
    // These are unscented transform points
    // These sigma points will be used later to propagate into the nonlinear model to get propagated measurement estimate points
    // Then these propagated measurement estimate points will be used to calculate final measurement estimate point
    // The whole UKF hinges on this Unscented transform 
    let sigma_points: SigmaPoints<NX, TWO_NX> = calc_sigma_points(x_est_pri, P_pri, lambda);
    
    // Propagate sigma points into measurement estimate points
    // Then these points will be later merged to create final measurement estimate point
    let y_est_0: SVector<f32, NY> = h(&sigma_points.x_0);
    let mut y_est_i = [SVector::<f32, NY>::zeros(); TWO_NX];
    for i in 0..NX {
        y_est_i[i] = h(&sigma_points.x_i[i]);
        y_est_i[i + NX] = h(&sigma_points.x_i[i + NX]);
    }

    // Calculate median measurement estimate
    // Take all the measurement estimate points and weight them
    // Then combine all of them into a single measurement estimate point
    let mut y_est: SVector<f32, NY> = weights.w_m_0 * y_est_0;
    for i in 0..TWO_NX {
        y_est += weights.w_m_i[i] * y_est_i[i];
    }

    // Calculate Innovation Covariance
    // Just like in EKF we must calculate the uncertainty between the measurement and the estimate measurement
    // However now we use again the unscented transform covariance to figure this out
    // Since this is a innovation covariance matrix we must also add the added measurement noise matrix R
    //
    // S[k]: Current Innovation Covariance, it represents the uncertainty of the estimate measurement (ie the x_est_priori[k] we just transformed using H matrix)
    // R: Uncertainty from sensor (Found by taking measurements of the sensor and getting variance of the different measurement states)
    //      R << 1 => Trust the measurements A LOT
    //      R >> 1 => DON'T trust the measurements that much
    let mut S: SMatrix<f32, NY, NY> = calc_unscented_covariance(
        &y_est,
        &y_est,
        &y_est_0,
        &y_est_0,
        &y_est_i,
        &y_est_i,
        weights.w_c_0,
        &weights.w_c_i,
    );
    S += R;

    // Calculate Covariance between state estimate and measurement estimate
    // Purpose of Cross-Covariance (P_xy)
    // ---------------------------------------------------------
    // In the Unscented Kalman Filter (UKF) after prediction step,
    // we get two things:
    //   - Predicted state (x_est) and its sigma points
    //   - Predicted measurement (y_est) and its sigma points
    //
    // However, to CORRECT the state with new incoming measurement (y), we need to know HOW the uncertainty in the state affects the uncertainty in the measurement.
    //
    // This "relationship" between state uncertainty and measurement uncertainty is captured through the **cross-covariance matrix P_xy**.
    //
    // It answers the question:
    //   => If there is a small error/change in x, how much would that impact z?
    //
    // We need P_xy to calculate the Kalman Gain (K):
    //    K = P_xy * S⁻¹
    //
    // Where:
    //   - P_xy: Cross covariance between state and measurement
    //   - S: Innovation covariance (uncertainty in predicted measurement)
    //
    // Without P_xy, we would have no idea how "trustworthy" the measurement is relative to our current estimate.
    //
    // A high P_xy means state and measurement are strongly linked → big correction.
    // A low P_xy means state and measurement are weakly linked → small correction.
    //
    // In short:
    //   - P_xy makes the correction step smart and precise.
    //   - It tells the filter how sensitive the measurement is to changes in state.
    //
    // TL;DR:
    // -> P_xy bridges the uncertainty from x_est to z_est
    // -> Needed to compute Kalman Gain (K)
    // -> Used to correct the predicted state with measurement information
    // ---------------------------------------------------------
    let P_xy: SMatrix<f32, NX, NY> = calc_unscented_covariance(
        &x_est_pri,
        &y_est,
        &sigma_points.x_0,
        &y_est_0,
        &sigma_points.x_i,
        &y_est_i,
        weights.w_c_0,
        &weights.w_c_i,
    );

    // Calculate Kalman Gain
    // ---------------------------------------------------------
    // Why K = P_xy * S⁻¹ ?
    //
    // In EKF:
    //    - K balances how much you trust model vs measurement
    //    - Done through P * H.T * (H * P * H.T + R)⁻¹
    //
    // In UKF:
    //    - No Jacobians, no H matrix.
    //    - Instead, we use sigma points to naturally find:
    //         - P_xy → how state uncertainty pushes into measurement uncertainty.
    //         - S → total measurement uncertainty (model + sensor noise).
    //
    // Idea:
    //  - P_xy tells how errors in state prediction relate to errors in measurement prediction. It measures how much a mistake in x affects a mistake in y.
    //  - S tells **how wide** the measurement uncertainty is.
    //  - Dividing (multiplying by S⁻¹) gives a *proportion*:
    //      -> How much correction should come from the measurement vs from the prediction.
    //
    // Meaning:
    // - If P_xy is big and S is small → model badly matches sensor → trust measurement a lot (big K).
    // - If P_xy is small or S is large → model and sensor agree → trust model more (small K).
    //
    // Mathematically:
    // - K aligns the two uncertainty clouds (state and measurement) optimally.
    //
    // TL;DR:
    //   - P_xy = "how much x affects y"
    //   - S = "how uncertain y is"
    //   - K = "how to mix x and y best to correct x"
    // ---------------------------------------------------------
    let S_inv = S.try_inverse().expect("Innovation covariance S is not invertible");
    let K: SMatrix<f32, NX, NY> = P_xy * S_inv;

    // Correct estimate using model AND measurement
    let x_est: SVector<f32, NX> = x_est_pri + K * (y - y_est);

    // Correct Estimate Uncertainty
    let P: SMatrix<f32, NX, NX> = P_pri - K*S*K.transpose();

    // Return results
    return (x_est, P);
}
// Unscented Kalman Filter (STOP) ==================================================