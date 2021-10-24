use btoi::btoi;
use core::fmt::Write;
use hal::prelude::*;
use hal::{gpio::SignalEdge, serial, stm32};
use infrared::protocols::nec::NecCommand;
use rtic::Mutex;
use ushell::*;

pub const CMD_MAX_LEN: usize = 32;

pub type Autocomplete = autocomplete::StaticAutocomplete<19>;
pub type History = history::LRUHistory<{ CMD_MAX_LEN }, 32>;
pub type Uart = serial::Serial<stm32::USART2, serial::BasicConfig>;
pub type Shell = UShell<Uart, Autocomplete, History, { CMD_MAX_LEN }>;

pub enum EnvSignal {
    SpinShell,
    LogAdc(u16),
    LogIRCmd(NecCommand),
}

pub type Env<'a> = crate::app::env::SharedResources<'a>;
pub type EnvResult = SpinResult<Uart, ()>;

impl Env<'_> {
    pub fn on_signal(&mut self, shell: &mut Shell, sig: EnvSignal) -> EnvResult {
        match sig {
            EnvSignal::SpinShell => shell.spin(self),
            EnvSignal::LogAdc(voltage) => {
                write!(shell, "\r\n{}mV", voltage)?;
                Ok(())
            }
            EnvSignal::LogIRCmd(cmd) => {
                write!(
                    shell,
                    "\r\nIR command: addr: 0x{:x}, cmd: 0x{:x}, repeat: {}",
                    cmd.addr, cmd.cmd, cmd.repeat
                )?;
                Ok(())
            }
        }
    }

    fn adc_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        if args.len() == 0 {
            let (vdd, vpin) = self.adc.lock(|adc| {
                let vdd = adc.adc.read_voltage(&mut adc.vbat).unwrap();
                let vpin = adc.adc.read_voltage(&mut adc.pin).unwrap();
                (vdd * 3, vpin)
            });
            write!(shell, "{0:}Vbat: {1:}mV{0:}Vpin: {2:}mV{0:}", CR, vdd, vpin)?;
        } else {
            match btoi::<u32>(args.as_bytes()) {
                Ok(freq) if freq <= 1_000_000 => {
                    self.adc.lock(|adc| adc.timer.start(freq.hz()));
                    shell.write_str(CR)?;
                }
                _ => write!(shell, "{0:}invalid adc arguments{0:}", CR)?,
            }
        }
        Ok(())
    }

    fn pwm_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        match btoi::<u32>(args.as_bytes()) {
            Ok(freq) if freq <= 1_000_000 => {
                self.pwm.lock(|pwm| {
                    pwm.pwm.set_freq(freq.hz());

                    let duty = pwm.ch1.get_max_duty() / 2;
                    pwm.ch1.set_duty(duty);
                    pwm.ch2.set_duty(duty);
                    pwm.ch3.set_duty(duty);
                });
                shell.write_str(CR)?;
            }
            _ => write!(shell, "{0:}unsupported PWM frequency{0:}", CR)?,
        }
        Ok(())
    }

    fn duty_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        match args.split_once(" ") {
            Some((ch, duty)) => match btoi::<u32>(duty.as_bytes()) {
                Ok(duty) if duty <= 100 => {
                    let duty = match duty {
                        0 => 0,
                        duty => {
                            let max_duty = self.pwm.lock(|pwm| pwm.ch1.get_max_duty());
                            max_duty * duty / 100
                        }
                    };
                    match ch {
                        "1" => self.pwm.lock(|pwm| pwm.ch1.set_duty(duty)),
                        "2" => self.pwm.lock(|pwm| pwm.ch2.set_duty(duty)),
                        "3" => self.pwm.lock(|pwm| pwm.ch3.set_duty(duty)),
                        _ => {
                            write!(shell, "{}unsupported PWM channel: \"{}\"", CR, ch)?;
                        }
                    }
                    shell.write_str(CR)?;
                }
                _ => write!(shell, "{0:}unsupported duty cycle{0:}", CR)?,
            },
            None => write!(shell, "{0:}invalid duty arguments{0:}", CR)?,
        }
        Ok(())
    }

    fn pin_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        match args {
            "a high" => self.gpio.lock(|gpio| gpio.pin_a.set_high()),
            "a low" => self.gpio.lock(|gpio| gpio.pin_a.set_low()),
            "b high" => self.gpio.lock(|gpio| gpio.pin_b.set_high()),
            "b low" => self.gpio.lock(|gpio| gpio.pin_b.set_low()),
            _ => {
                write!(shell, "{}unsupported pin arguments: \"{}\"", CR, args)?;
                Ok(())
            }
        }
        .ok();
        shell.write_str(CR)?;
        Ok(())
    }

    fn opm_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        match args
            .strip_suffix("us")
            .map(|us| (us, 1))
            .or_else(|| args.strip_suffix("ms").map(|ms| (ms, 1_000)))
        {
            Some((width, mul)) => match btoi::<u32>(width.as_bytes()) {
                Ok(pulse) => {
                    let pulse = (pulse * mul).us();
                    self.opm.lock(|opm| {
                        opm.set_pulse(pulse);
                        opm.generate();
                    });
                    shell.write_str(CR)?;
                }
                _ => write!(shell, "{0:}unsupported pulse width{0:}", CR)?,
            },
            None => write!(shell, "{0:}unsupported pulse unit{0:}", CR)?,
        }
        Ok(())
    }

    fn trigger_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        let trigger = match args {
            "rise" => Some(SignalEdge::Rising),
            "fall" => Some(SignalEdge::Falling),
            "off" => None,
            trig => {
                write!(
                    shell,
                    "{0:}unsupported trigger type: \"{1:}\"{0:}",
                    CR, trig
                )?;
                return Ok(());
            }
        };
        self.gpio.lock(|gpio| gpio.trigger = trigger);
        shell.write_str(CR)?;
        Ok(())
    }

    fn servo_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        match btoi::<u32>(args.as_bytes()) {
            Ok(angle) if angle <= 180 => {
                let max_duty = self.servo.lock(|servo| servo.get_max_duty()) as u32;
                let duty = (max_duty * 5 + (max_duty * 20 * angle) / 180) / 100;
                self.servo.lock(|servo| {
                    servo.set_duty(duty as _);
                });
                shell.write_str(CR)?;
            }
            _ => write!(shell, "{0:}unsupported servo angle{0:}", CR)?,
        }
        Ok(())
    }

    fn spin_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        use crate::stepper::Rotation::*;

        match args.split_once(" ") {
            Some((dir, speed)) => match btoi::<u32>(speed.as_bytes()) {
                Ok(speed) => {
                    match dir {
                        "cw" => self.stepper.lock(|st| st.motor.set_dir(Cw)),
                        "ccw" => self.stepper.lock(|st| st.motor.set_dir(Ccw)),
                        _ => {
                            write!(shell, "{0:}unsupported direction{0:}", CR)?;
                            return Ok(());
                        }
                    }

                    self.stepper.lock(|st| {
                        if speed > 0 {
                            st.timer.start(speed.hz());
                        } else {
                            st.motor.disable();
                            st.timer.pause();
                            st.timer.clear_irq();
                        }
                    });
                    shell.write_str(CR)?;
                }
                _ => write!(shell, "{0:}unsupported speed{0:}", CR)?,
            },
            None => write!(shell, "{0:}invalid spin arguments{0:}", CR)?,
        }

        Ok(())
    }

    fn help_cmd(&mut self, shell: &mut Shell, args: &str) -> EnvResult {
        match args {
            "pinout" => shell.write_str(PINOUT)?,
            "usage" => shell.write_str(USAGE)?,
            _ => shell.write_str(HELP)?,
        }
        Ok(())
    }
}

impl Environment<Uart, Autocomplete, History, (), { CMD_MAX_LEN }> for Env<'_> {
    fn command(&mut self, shell: &mut Shell, cmd: &str, args: &str) -> EnvResult {
        match cmd {
            "clear" => shell.clear()?,
            "adc" => self.adc_cmd(shell, args)?,
            "duty" => self.duty_cmd(shell, args)?,
            "help" => self.help_cmd(shell, args)?,
            "pin" => self.pin_cmd(shell, args)?,
            "pulse" => self.opm_cmd(shell, args)?,
            "pwm" => self.pwm_cmd(shell, args)?,
            "servo" => self.servo_cmd(shell, args)?,
            "spin" => self.spin_cmd(shell, args)?,
            "trigger" => self.trigger_cmd(shell, args)?,
            "" => shell.write_str(CR)?,
            _ => write!(shell, "{0:}unsupported command: \"{1:}\"{0:}", CR, cmd)?,
        }
        shell.write_str(SHELL_PROMPT)?;
        Ok(())
    }

    fn control(&mut self, shell: &mut Shell, code: u8) -> EnvResult {
        match code {
            control::CTRL_C => {
                self.adc.lock(|adc| adc.timer.pause());
                shell.write_str(CR)?;
                shell.write_str(SHELL_PROMPT)?;
            }
            control::CTRL_W => self.gpio.lock(|gpio| gpio.pin_a.set_high().unwrap()),
            control::CTRL_E => self.gpio.lock(|gpio| gpio.pin_a.set_low().unwrap()),
            control::CTRL_S => self.gpio.lock(|gpio| gpio.pin_b.set_high().unwrap()),
            control::CTRL_D => self.gpio.lock(|gpio| gpio.pin_b.set_low().unwrap()),
            control::CTRL_Z => self.opm.lock(|opm| opm.generate()),
            _ => {}
        }
        Ok(())
    }
}

pub const AUTOCOMPLETE: Autocomplete = autocomplete::StaticAutocomplete([
    "adc ",
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
\x20 adc [freq]           Measure voltage\r\n\
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
\x20 Ctrl+C               Stop background ADC process\r\n\
\x20 Ctrl+Z               Emulate pulse trigger\r\n\
\x20 Ctrl+W / Ctrl+E      Assert/De-assert pin A(Push-Pull)\r\n\
\x20 Ctrl+S / Ctrl+D      Assert/De-assert pin B(Open-Drain)\r\n\r\n\
LINKS:\r\n\
\x20 * \x1b[34mhttps://github.com/dotcypress/placebo-shell \x1b[0m\r\n\
\x20 * \x1b[34mhttps://github.com/dotcypress/placebo \x1b[0m\r\n\r\n\
";

const USAGE: &str = "\r\n\
USAGE EXAMPLES:\r\n\
\x20 help pinout     Print pinout\r\n\
\x20 adc             Measure Vdd and ADC pin voltage\r\n\
\x20 adc 5           Continuously measure ADC pin voltage\r\n\
\x20 duty 1 50       Set PWM channel 1 duty cycle to 50%\r\n\
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
\x20                 STM32G0xxFx  \r\n\
\x20                ╔═══════════╗ \r\n\
\x20 (A)    PB7|PB8 ╣1 ¤      20╠ PB3|PB4|PB5|PB6  (PWM1)\r\n\
\x20 (B)   PC9|PC14 ╣2        19╠ PA14|PA15       (SWDIO)\r\n\
\x20 (TRG|IR)  PC15 ╣3        18╠ PA13           (SWDCLK)\r\n\
\x20            Vdd ╣4        17╠ PA12[PA10]  (STEPPER_A)\r\n\
\x20            Vss ╣5        16╠ PA11[PA9]   (STEPPER_B)\r\n\
\x20           nRst ╣6        15╠ PA8|PB0|PB1|PB2  (PWM3)\r\n\
\x20 (PULSE)    PA0 ╣7        14╠ PA7              (PWM2)\r\n\
\x20 (ADC)      PA1 ╣8        13╠ PA6         (STEPPER_C)\r\n\
\x20 (TX)       PA2 ╣9        12╠ PA5         (STEPPER_D)\r\n\
\x20 (RX)       PA3 ╣10       11╠ PA4             (SERVO)\r\n\
\x20                ╚═══════════╝ \r\n\r\n\
";
