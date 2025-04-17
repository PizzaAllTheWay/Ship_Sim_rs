// Library for maths
use nalgebra::{Vector6, SMatrix, SVector};

// Library for multithreading
use std::sync::{Arc, RwLock};



// Custom data types ----------
pub type Vector9<T> = SVector<T, 9>;
pub type Vector12<T> = SVector<T, 12>;
pub type Matrix9x9<T> = SMatrix::<T, 9, 9>;
pub type Matrix12x6<T> = SMatrix::<T, 12, 6>;
pub type Matrix9x12<T> = SMatrix::<T, 9, 12>;
pub type Matrix12x9<T> = SMatrix::<T, 12, 9>;
pub type Matrix12x12<T> = SMatrix::<T, 12, 12>;

#[allow(non_snake_case)]
#[derive(Clone, Default)]
pub struct SharedState {
    pub x_est: Arc<RwLock<Vector12<f32>>>,
    pub P: Arc<RwLock<Matrix12x12<f32>>>,
}



#[allow(non_snake_case)]
pub fn estimate(
    dt: f32,
    x_est_prev: Vector12<f32>,
    u_prev: Vector6<f32>,
    P_prev: Matrix12x12<f32>,
    A: Matrix12x12<f32>,
    B: Matrix12x6<f32>,
    Q: Matrix12x12<f32>,
) -> (Vector12<f32>, Matrix12x12<f32>) {
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
    let F_d: Matrix12x12<f32> = Matrix12x12::identity() + dt*A;
    let B_d: Matrix12x6<f32> = dt*B;

    // Calculate estimate based ONLY on state
    let x_est_priori: Vector12<f32> = F_d*x_est_prev + B_d*u_prev;

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
    let P_priori: Matrix12x12<f32> = F_d*P_prev*F_d.transpose() + Q;

    // Return the estimate and the uncertainty
    return (x_est_priori, P_priori);
}



#[allow(non_snake_case)]
pub fn correct(
    z: Vector9<f32>,
    x_est_priori: Vector12<f32>,
    P_priori: Matrix12x12<f32>,
    H: Matrix9x12<f32>,
    R: Matrix9x9<f32>,
) -> (
    Vector12<f32>,
    Matrix12x12<f32>,
    Vector9<f32>,
    Matrix9x9<f32>,
    Matrix12x9<f32>,
){
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
    let y: Vector9<f32> = z - H*x_est_priori;

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
    let S: Matrix9x9<f32> = H*P_priori*H.transpose() + R;

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
    //let S_inv: Matrix9x9<f32> = S.try_inverse().expect("Matrix S is not invertible");
    let S_inv: Matrix9x9<f32> = S.pseudo_inverse(1e-6).unwrap();
    let K: Matrix12x9<f32> = P_priori*H.transpose()*S_inv;

    // Correct estimate using model AND measurement
    // To know how much to subtract/add from estimate priori, we must utilize kalman gain on Innovation Residual
    // This will give optimal balance of how much to add to each estimate prior to get a good balance between estimate and measurement
    // x_est[k] = x_est_priori[k] - K[k]*y[k]
    let x_est: Vector12<f32> = x_est_priori - K*y;

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
    let P: Matrix12x12<f32> = (Matrix12x12::identity() - K*H)*P_priori;

    // Return results
    return (x_est, P, y, S, K);
}