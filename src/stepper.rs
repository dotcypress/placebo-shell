use hal::prelude::*;

#[derive(PartialEq)]
pub enum Rotation {
    Cw,
    Ccw,
}

pub struct Stepper<A, B, C, D> {
    pins: (A, B, C, D),
    dir: Rotation,
    step: usize,
}

impl<A: OutputPin, B: OutputPin, C: OutputPin, D: OutputPin> Stepper<A, B, C, D> {
    pub fn new(a: A, b: B, c: C, d: D) -> Self {
        Stepper {
            pins: (a, b, c, d),
            step: 0,
            dir: Rotation::Cw,
        }
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
        const STEPS: [usize; 8] = [
            0b0001, 0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001,
        ];
        self.step = if self.dir == Rotation::Cw {
            if self.step == STEPS.len() - 1 {
                0
            } else {
                self.step + 1
            }
        } else if self.step == 0 {
            STEPS.len() - 1
        } else {
            self.step - 1
        };

        Self::drive_pin(&mut self.pins.0, STEPS[self.step] & 0b1000 == 0b1000);
        Self::drive_pin(&mut self.pins.1, STEPS[self.step] & 0b0100 == 0b0100);
        Self::drive_pin(&mut self.pins.2, STEPS[self.step] & 0b0010 == 0b0010);
        Self::drive_pin(&mut self.pins.3, STEPS[self.step] & 0b0001 == 0b0001);
    }
    fn drive_pin(pin: &mut impl OutputPin, val: bool) {
        if val {
            pin.set_high().ok();
        } else {
            pin.set_low().ok();
        }
    }
}
