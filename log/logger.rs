use ship_sim_lib::comm::udp_utils;
use ship_sim_lib::comm::udp_topics::{self, TOPICS};

use std::{
    fs::{File, OpenOptions},
    io::{BufWriter, Write},
    str,
    thread,
};

use chrono::Local;
use once_cell::sync::Lazy;

// Timestamped log folder path
static LOG_FOLDER: Lazy<String> = Lazy::new(|| {
    let ts = Local::now().format("%Y-%m-%d_%H-%M-%S").to_string();
    let path = format!("log/data/{}", ts);
    std::fs::create_dir_all(&path).expect("Failed to create log folder");
    path
});

// Helper: create writer to timestamped file
fn make_writer(name: &str) -> BufWriter<File> {
    let path = format!("{}/{}", *LOG_FOLDER, name);
    BufWriter::new(
        OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .expect("Failed to open log file"),
    )
}

fn main() {
    println!("Logging all UDP topics to {}/", *LOG_FOLDER);

    let mut handles = vec![];

    macro_rules! spawn_logger {
        ($topic:ident, $filename:expr) => {
            handles.push(thread::spawn(|| {
                let mut writer = make_writer($filename);
                loop {
                    let msg = udp_utils::subscribe(TOPICS::$topic::PORT).unwrap();
                    let json_str = str::from_utf8(&msg).expect("Invalid UTF-8");
                    let data: TOPICS::$topic::DataType = udp_topics::decode_json(json_str);
    
                    let timestamp = Local::now().format("%Y-%m-%d %H:%M:%S%.3f");
                    writeln!(
                        writer,
                        "{},{}",
                        timestamp,
                        format!("{:?}", data).replace(&[',', '[', ']'][..], "")
                    )
                    .and_then(|_| writer.flush())
                    .expect("Write failed");
                }
            }));
        };
    }    

    spawn_logger!(forces_thrusters,    "forces_thrusters.csv");
    spawn_logger!(wind_parameters,     "wind.csv");
    spawn_logger!(current_parameters,  "current.csv");
    spawn_logger!(x,                   "x.csv");
    spawn_logger!(dx,                  "dx.csv");
    spawn_logger!(speed,               "speed.csv");
    spawn_logger!(gnss,                "gnss.csv");
    spawn_logger!(imu,                 "imu.csv");
    

    for h in handles {
        h.join().unwrap();
    }
}
