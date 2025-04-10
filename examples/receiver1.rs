use ship_sim_lib::process_communication::udp_utils;
use ship_sim_lib::process_communication::udp_topics::{self, TOPICS};
use std::str;

fn main() -> std::io::Result<()> {
    println!("Listening for FORCES on port {}...", TOPICS::forces::PORT);

    loop {
        let msg = udp_utils::subscribe(TOPICS::forces::PORT)?;
        let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
        let forces: TOPICS::forces::DataType = udp_topics::decode_json(json_str);

        println!("Received FORCES: {:?}", forces);
    }
}
