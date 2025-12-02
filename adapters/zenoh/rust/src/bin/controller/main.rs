use kk_robotics::controller::tui_controller::{ControllerEvent, TuiController};
use std::time::Duration;

use zenoh::config::Config;
use zenoh::prelude::r#async::*;
use crossterm::terminal::{enable_raw_mode, disable_raw_mode};


pub struct RawModeGuard;

impl RawModeGuard {
    pub fn new() -> Self {
        enable_raw_mode().unwrap();
        Self
    }
}

impl Drop for RawModeGuard {
    fn drop(&mut self) {
        disable_raw_mode().unwrap();
    }
}

#[tokio::main]
async fn main() {
    enable_raw_mode().unwrap();
    let mut controller = TuiController::new();

    let session = zenoh::open(Config::default()).res().await.unwrap();
    let key = "cmd_vel".to_string();
    let publisher = session.declare_publisher(&key).res().await.unwrap();

    loop {
        let event = controller.update_input();
        if let ControllerEvent::Exit = event {
            break;
        }
        controller.draw_ui();

        let cmd = controller.get_cmd_vel();
        let data = serde_json::to_vec(&cmd).unwrap();
        println!("{}", cmd.linear);
        publisher.put(data).res().await.unwrap();

        tokio::time::sleep(Duration::from_millis(30)).await;
    }
    disable_raw_mode().unwrap();
}
