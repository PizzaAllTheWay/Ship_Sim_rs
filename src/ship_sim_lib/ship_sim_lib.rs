pub mod comm {
    pub mod udp_topics;
    pub mod udp_utils;
}

pub mod estimators {
    pub mod estimators_utils;
    pub mod ekf;
    pub mod ship_approx;
}

pub mod models {
    pub mod current;
    pub mod gnss;
    pub mod imu;
    pub mod ship;
    pub mod wind;
}

pub mod simulation {
    pub mod kinematics;
    pub mod solver;
}

pub mod ui {
    pub mod gui;
}


