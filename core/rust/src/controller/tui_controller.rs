use crossterm::{
    cursor,
    terminal::{Clear, ClearType},
    event::{self, Event, KeyCode},
    QueueableCommand,
};
use std::time::Duration;
use std::io::{stdout, Write};

use super::cmd_vel::CmdVel;

pub struct TuiController {
    linear: f32,
    angular: f32,
}

pub enum ControllerEvent {
    None,
    Exit,
}

impl TuiController {
    pub fn new() -> Self {
        Self {
            linear: 0.0,
            angular: 0.0,
        }
    }

    pub fn update_input(&mut self) ->ControllerEvent{
        if event::poll(Duration::from_millis(500)).unwrap() {
            if let Event::Key(key) = event::read().unwrap() {
                match key.code {
                    KeyCode::Char('w') => self.linear += 0.1, 
                    KeyCode::Char('s') => self.linear -= 0.1,
                    KeyCode::Char('a') => self.angular += 0.1,
                    KeyCode::Char('d') => self.angular -= 0.1,
                    KeyCode::Char(' ') => {
                        self.linear = 0.0;
                        self.angular = 0.0;
                    }
                    KeyCode::Esc => {
                        return ControllerEvent::Exit;
                    }
                    _ => {}
                }
            }
        }
        ControllerEvent::None
    }

    pub fn get_cmd_vel(&self) -> CmdVel {
        CmdVel {
            linear: self.linear,
            angular: self.angular,
        }
    }

    pub fn draw_ui(&self) {
        let mut out = stdout();

        out.queue(cursor::MoveTo(0, 0)).unwrap();
        out.queue(Clear(ClearType::All)).unwrap();

        let lines = [
            "===============================",
            "KK ROBOTICS CONTROLLER",
            "===============================",
            "",
            "Controls:",
            " w : Forward",
            " s : Backward",
            " a : Turn Left",
            " d : Turn Right",
            " SPACE : Stop",
            " ESC : Quit",
            "",
            &format!("Current Velocity:"),
            &format!(" Linear  : {:>6.2} m/s", self.linear),
            &format!(" Angular : {:>6.2} rad/s", self.angular),
        ];

        for line in &lines {
            out.queue(crossterm::style::Print(line)).unwrap();
            out.queue(crossterm::cursor::MoveToColumn(0)).unwrap();
            out.queue(crossterm::style::Print("\n")).unwrap();
        }

        out.flush().unwrap();

    }
}

