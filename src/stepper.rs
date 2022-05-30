use hal::prelude::*;

const UNIPOLAR_STEPS: [usize; 8] = [
    0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001,
];

const BIPOLAR_STEPS: [usize; 8] = [
    0b1001, 0b1001, 0b1100, 0b1100, 0b0110, 0b0110, 0b0011, 0b0011,
];

#[derive(PartialEq)]
pub enum Rotation {
    Cw,
    Ccw,
}

#[derive(PartialEq)]
pub enum Mode {
    Bipolar,
    Unipolar,
}

pub struct StepperMotor<A, B, C, D> {
    pins: (A, B, C, D),
    dir: Rotation,
    mode: Mode,
    step: usize,
}

impl<A: OutputPin, B: OutputPin, C: OutputPin, D: OutputPin> StepperMotor<A, B, C, D> {
    pub fn new(a: A, b: B, c: C, d: D) -> Self {
        Self {
            pins: (a, b, c, d),
            step: 0,
            mode: Mode::Unipolar,
            dir: Rotation::Cw,
        }
    }

    pub fn set_mode(&mut self, mode: Mode) {
        self.mode = mode;
    }

    pub fn set_dir(&mut self, dir: Rotation) {
        self.dir = dir;
    }

    pub fn disable(&mut self) {
        Self::drive_pin(&mut self.pins.0, false);
        Self::drive_pin(&mut self.pins.1, false);
        Self::drive_pin(&mut self.pins.2, false);
        Self::drive_pin(&mut self.pins.3, false);
    }

    pub fn turn(&mut self) {
        let steps = if self.mode == Mode::Unipolar {
            UNIPOLAR_STEPS
        } else {
            BIPOLAR_STEPS
        };
        self.step = if self.dir == Rotation::Cw {
            if self.step == steps.len() - 1 {
                0
            } else {
                self.step + 1
            }
        } else if self.step == 0 {
            steps.len() - 1
        } else {
            self.step - 1
        };

        Self::drive_pin(&mut self.pins.0, steps[self.step] & 0b1000 > 0);
        Self::drive_pin(&mut self.pins.1, steps[self.step] & 0b0100 > 0);
        Self::drive_pin(&mut self.pins.2, steps[self.step] & 0b0010 > 0);
        Self::drive_pin(&mut self.pins.3, steps[self.step] & 0b0001 > 0);
    }

    fn drive_pin(pin: &mut impl OutputPin, val: bool) {
        if val {
            pin.set_high().ok();
        } else {
            pin.set_low().ok();
        }
    }
}
