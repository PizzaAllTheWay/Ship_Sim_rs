use std::{thread, time::Duration};
use ship_sim_lib::comm::udp_utils;
use ship_sim_lib::comm::udp_topics::{self, TOPICS, Vector12};
use nalgebra::Vector6;

fn main() -> std::io::Result<()> {
    loop {
        // Forces
        let forces: TOPICS::forces::DataType = Vector6::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let forces_json = udp_topics::encode_json(&forces);
        udp_utils::publish(TOPICS::forces::PORT, forces_json.as_bytes())?;

        // X
        let x: TOPICS::x::DataType = Vector12::<f32>::from_element(7.0);
        let x_json = udp_topics::encode_json(&x);
        udp_utils::publish(TOPICS::x::PORT, x_json.as_bytes())?;

        // DX
        let dx: TOPICS::dx::DataType  = Vector12::<f32>::from_element(30.0);
        let dx_json = udp_topics::encode_json(&dx);
        udp_utils::publish(TOPICS::dx::PORT, dx_json.as_bytes())?;

        println!("Sent: FORCES, X, DX");

        thread::sleep(Duration::from_secs(1));
    }
}
