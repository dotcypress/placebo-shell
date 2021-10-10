use core::fmt::Write;
use hal::prelude::*;
use hal::serial;
use hal::stm32;
use rtic::mutex_prelude::*;
use rtic::Mutex;
use stm32g0xx_hal::gpio::SignalEdge;
use ushell::{autocomplete::StaticAutocomplete, control, history::LRUHistory, Environment, UShell};

pub const CMD_MAX_LEN: usize = 32;
pub type Autocomplete = StaticAutocomplete<19>;
pub type History = LRUHistory<{ CMD_MAX_LEN }, 32>;
pub type Serial = serial::Serial<stm32::USART2, serial::BasicConfig>;
pub type Shell = UShell<Serial, Autocomplete, History, { CMD_MAX_LEN }>;
pub type Env<'a> = crate::app::serial_data::SharedResources<'a>;
pub type EnvResult = Result<(), ushell::SpinError<Serial, ()>>;

impl Environment<Serial, Autocomplete, History, (), { CMD_MAX_LEN }> for Env<'_> {
    fn command(&mut self, shell: &mut Shell, cmd: &str, args: &str) -> EnvResult {
        match cmd {
            "adc" => {
                self.adc_cmd(shell, args)?;
            }
            "pwm" => {
                self.pwm_cmd(shell, args)?;
            }
            "duty" => {
                self.duty_cmd(shell, args)?;
            }
            "spin" => {
                self.spin_cmd(shell, args)?;
            }
            "pulse" => {
                self.opm_cmd(shell, args)?;
            }
            "trigger" => {
                self.trigger_cmd(shell, args)?;
            }
            "servo" => {
                self.servo_cmd(shell, args)?;
            }
            "pin" => {
                self.pin_cmd(shell, args)?;
            }
            "clear" => {
                shell.clear().ok();
            }
            "help" => match args {
                "pinout" => {
                    shell.write_str(PINOUT).ok();
                }
                "usage" => {
                    shell.write_str(USAGE).ok();
                }
                _ => {
                    shell.write_str(HELP).ok();
                }
            },
            "" => {
                shell.write_str(CR).ok();
            }
            _ => {
                write!(shell, "{0:}unsupported command{0:}", CR).ok();
            }
        }
        shell.write_str(SHELL_PROMPT).ok();
        Ok(())
    }

    fn control(&mut self, _shell: &mut Shell, code: u8) -> EnvResult {
        match code {
            control::CTRL_X => {
                self.pin_a.lock(|pin| pin.set_high().ok());
            }
            control::CTRL_C => {
                self.pin_a.lock(|pin| pin.set_low().ok());
            }
            control::CTRL_S => {
                self.pin_b.lock(|pin| pin.set_high().ok());
            }
            control::CTRL_D => {
                self.pin_b.lock(|pin| pin.set_low().ok());
            }
            control::CTRL_Z => {
                self.opm.lock(|opm| opm.generate());
            }
            _ => {}
        }
        Ok(())
    }
}

impl Env<'_> {
    fn adc_cmd(&mut self, shell: &mut Shell, _args: &str) -> EnvResult {
        let (vdd, vpin) =
            (&mut self.adc, &mut self.adc_pin, &mut self.vbat).lock(|adc, pin, vbat| {
                let vdd = adc.read_voltage(vbat).expect("adc read failed");
                let vpin = adc.read_voltage(pin).expect("adc read failed");
                (vdd * 3, vpin)
            });
        write!(shell, "{0:}Vbat: {1:}mV{0:}Vpin: {2:}mV{0:}", CR, vdd, vpin).ok();
        Ok(())
    }

    fn pwm_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        match btoi::btoi::<u32>(args.as_bytes()) {
            Ok(freq) if freq <= 1_000_000 => {
                self.pwm.lock(|pwm| {
                    pwm.set_freq(freq.hz());
                });
                shell.write_str(CR).ok();
            }
            _ => {
                write!(shell, "{0:}unsupported PWM frequency{0:}", CR).ok();
            }
        }
        Ok(())
    }

    fn duty_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        match args.split_once(" ") {
            Some((ch, duty)) => match btoi::btoi::<u32>(duty.as_bytes()) {
                Ok(duty) if duty <= 100 => {
                    let duty = match duty {
                        0 => 0,
                        duty => {
                            let max_duty = self.pwm_ch1.lock(|ch| ch.get_max_duty());
                            max_duty * duty / 100
                        }
                    };
                    match ch {
                        "1" => {
                            self.pwm_ch1.lock(|ch| ch.set_duty(duty));
                        }
                        "2" => {
                            self.pwm_ch2.lock(|ch| ch.set_duty(duty));
                        }
                        "3" => {
                            self.pwm_ch3.lock(|ch| ch.set_duty(duty));
                        }
                        _ => {
                            write!(shell, "{0:}unsupported PWM channel{0:}", CR).ok();
                            return Ok(());
                        }
                    }
                    shell.write_str(CR).ok();
                }
                _ => {
                    write!(shell, "{0:}unsupported duty cycle{0:}", CR).ok();
                }
            },
            None => {
                write!(shell, "{0:}invalid duty arguments{0:}", CR).ok();
            }
        }
        Ok(())
    }

    fn pin_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        match args {
            "a high" => {
                self.pin_a.lock(|pin| pin.set_high().ok());
            }
            "a low" => {
                self.pin_a.lock(|pin| pin.set_low().ok());
            }
            "b high" => {
                self.pin_b.lock(|pin| pin.set_high().ok());
            }
            "b low" => {
                self.pin_b.lock(|pin| pin.set_low().ok());
            }
            _ => {
                write!(shell, "{0:}unsupported pin arguments{0:}", CR).ok();
                return Ok(());
            }
        }
        shell.write_str(CR).ok();

        Ok(())
    }

    fn opm_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        match args
            .strip_suffix("us")
            .map(|us| (us, 1))
            .or_else(|| args.strip_suffix("ms").map(|ms| (ms, 1_000)))
        {
            Some((width, mul)) => match btoi::btoi::<u32>(width.as_bytes()) {
                Ok(pulse) => {
                    let pulse = (pulse * mul).us();
                    self.opm.lock(|opm| {
                        opm.set_pulse(pulse);
                        opm.generate();
                    });
                    shell.write_str(CR).ok();
                }
                _ => {
                    write!(shell, "{0:}unsupported pulse width{0:}", CR).ok();
                }
            },
            None => {
                write!(shell, "{0:}unsupported pulse unit{0:}", CR).ok();
            }
        }
        Ok(())
    }

    fn trigger_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        let trigger = match args {
            "rise" => Some(SignalEdge::Rising),
            "fall" => Some(SignalEdge::Falling),
            "off" => None,
            _ => {
                write!(shell, "{0:}unsupported trigger{0:}", CR).ok();
                return Ok(());
            }
        };
        self.trigger.lock(|t| *t = trigger);
        shell.write_str(CR).ok();
        Ok(())
    }

    fn servo_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        match btoi::btoi::<u32>(args.as_bytes()) {
            Ok(angle) if angle <= 180 => {
                let max_duty = self.servo.lock(|servo| servo.get_max_duty()) as u32;
                let duty = (max_duty * 5 + (max_duty * 20 * angle) / 180) / 100;
                self.servo.lock(|servo| {
                    servo.set_duty(duty as _);
                });
                shell.write_str(CR).ok();
            }
            _ => {
                write!(shell, "{0:}unsupported servo angle{0:}", CR).ok();
            }
        }
        Ok(())
    }

    fn spin_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        use crate::stepper::Rotation;

        match args.split_once(" ") {
            Some((dir, speed)) => match btoi::btoi::<u32>(speed.as_bytes()) {
                Ok(speed) => {
                    match dir {
                        "cw" => {
                            self.stepper.lock(|stepper| stepper.set_dir(Rotation::Cw));
                        }
                        "ccw" => {
                            self.stepper.lock(|stepper| stepper.set_dir(Rotation::Ccw));
                        }
                        _ => {
                            write!(shell, "{0:}unsupported direction{0:}", CR).ok();
                            return Ok(());
                        }
                    }
                    if speed > 0 {
                        self.stepper_timer.lock(|tim| tim.start(speed.hz()));
                    } else {
                        (&mut self.stepper, &mut self.stepper_timer).lock(|stepper, tim| {
                            tim.pause();
                            tim.clear_irq();
                            stepper.disable();
                        });
                    }
                    shell.write_str(CR).ok();
                }
                _ => {
                    write!(shell, "{0:}unsupported speed{0:}", CR).ok();
                }
            },
            None => {
                write!(shell, "{0:}invalid spin arguments{0:}", CR).ok();
            }
        }
        Ok(())
    }
}

pub const AUTOCOMPLETE: Autocomplete = StaticAutocomplete([
    "adc",
    "clear",
    "duty ",
    "help ",
    "help pinout",
    "help usage",
    "pin ",
    "pin a high",
    "pin a low",
    "pin b high",
    "pin b low",
    "pulse ",
    "pwm ",
    "servo ",
    "spin ",
    "trigger ",
    "trigger fall",
    "trigger off",
    "trigger rise",
]);

const CR: &str = "\r\n";
const SHELL_PROMPT: &str = "\x1b[35m» \x1b[0m";
const HELP: &str = "\r\n\
\x1b[33mPlacebo Shell \x1b[32mv0.0.1\x1b[0m\r\n\r\n\
COMMANDS:\r\n\
\x20 adc                  Measure voltage\r\n\
\x20 duty <1|2|3> <duty>  Set PWM channel duty (0-100)\r\n\
\x20 pin <a|b> <level>    Set pin level (low, high)\r\n\
\x20 pwm <freq>           Set PWM frequency (1-1000000)\r\n\
\x20 pulse <width>        Generate pulse (1us-1000ms)\r\n\
\x20 trigger <edge>       Set external trigger edge (rise, fall, off)\r\n\
\x20 servo <angle>        Turn servo to angle (0-180)\r\n\
\x20 spin <cw|ccw> <pps>  Set stepper rotation direction and speed\r\n\
\x20 help [pinout|usage]  Print help message\r\n\
\x20 clear                Clear screen\r\n\r\n\
CONTROL KEYS:\r\n\
\x20 Ctrl+Z               Emulate pulse trigger\r\n\
\x20 Ctrl+X / Ctrl+C      Assert/De-assert pin A\r\n\
\x20 Ctrl+S / Ctrl+D      Assert/De-assert pin B\r\n\r\n\
LINKS:\r\n\
\x20 * \x1b[34mhttps://github.com/dotcypress/placebo-shell \x1b[0m\r\n\
\x20 * \x1b[34mhttps://github.com/dotcypress/placebo \x1b[0m\r\n\r\n\
";

const USAGE: &str = "\r\n\
USAGE EXAMPLES:\r\n\
\x20 help pinout     Print pinout\r\n\
\x20 duty 1 50       Set duty cycle to 50%\r\n\
\x20 pin a high      Assert pin A\r\n\
\x20 pin b low       De-assert pin B\r\n\
\x20 pwm 1000        Set PWM freqency to 1KHz\r\n\
\x20 pulse 120us     Generate 120 microseconds pulse\r\n\
\x20 servo 45        Turn servo to 45 degrees\r\n\
\x20 trigger fall    Set external trigger to falling edge\r\n\
\x20 trigger off     Disable pulse external trigger\r\n\
\x20 spin cw 200     Rotate stepper clock-wise 200 pulses per second\r\n\r\n\
";

const PINOUT: &str = "\r\n\
\x20                STM32G0xxFx  \r\n\
\x20               ╔═══════════╗ \r\n\
\x20 (A)   PB7|PB8 ╣1 ¤      20╠ PB3|PB4|PB5|PB6  (PWM1)\r\n\
\x20 (B)  PC9|PC14 ╣2        19╠ PA14|PA15       (SWDIO)\r\n\
\x20 (TRIG)   PC15 ╣3        18╠ PA13           (SWDCLK)\r\n\
\x20           Vdd ╣4        17╠ PA12[PA10]  (STEPPER_A)\r\n\
\x20           Vss ╣5        16╠ PA11[PA9]   (STEPPER_B)\r\n\
\x20          nRst ╣6        15╠ PA8|PB0|PB1|PB2  (PWM3)\r\n\
\x20 (PULSE)   PA0 ╣7        14╠ PA7              (PWM2)\r\n\
\x20 (ADC)     PA1 ╣8        13╠ PA6         (STEPPER_C)\r\n\
\x20 (TX)      PA2 ╣9        12╠ PA5         (STEPPER_D)\r\n\
\x20 (RX)      PA3 ╣10       11╠ PA4             (SERVO)\r\n\
\x20               ╚═══════════╝ \r\n\r\n\
";
