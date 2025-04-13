use nalgebra::Vector3;
use rand_distr::{Normal, Distribution}; // Gaussian distribution
use rand::Rng;

pub fn simulate(
    a_lin: Vector3<f32>,
    v_ang: Vector3<f32>,
    yaw_ang: f32,
) -> (Vector3<f32>, Vector3<f32>, f32) {
    // ?NOTE: Dont forget to trasnform from body to acelerometer
    // ?NOTE; Dont forget g on the aceleration down

    let imu_accel = a_lin;
    let imu_gyro = v_ang;
    let imu_mag = yaw_ang;

    return (imu_accel, imu_gyro, imu_mag);
}