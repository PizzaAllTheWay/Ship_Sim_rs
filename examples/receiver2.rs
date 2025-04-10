use ship_sim_lib::comm::udp_utils;
use ship_sim_lib::comm::udp_topics::{self, TOPICS};
use std::{str, thread};

fn main() {
    println!("Listening for X on port {}...", TOPICS::x::PORT);
    println!("Listening for DX on port {}...", TOPICS::dx::PORT);

    // X listener
    let x_thread = thread::spawn(|| {
        loop {
            let msg = udp_utils::subscribe(TOPICS::x::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let x: TOPICS::x::DataType = udp_topics::decode_json(json_str);
            println!("[X]  {:?}", x);
        }
    });

    // DX listener
    let dx_thread = thread::spawn(|| {
        loop {
            let msg = udp_utils::subscribe(TOPICS::dx::PORT).unwrap();
            let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
            let dx: TOPICS::dx::DataType = udp_topics::decode_json(json_str);
            println!("[DX] {:?}", dx);
        }
    });

    x_thread.join().unwrap();
    dx_thread.join().unwrap();
}
