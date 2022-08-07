#![no_std]
#![no_main]
#![deny(warnings)]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_halt;
extern crate rtic;
extern crate stm32g0xx_hal as hal;

mod shell;
mod stepper;

use hal::gpio::*;
use hal::prelude::*;
use hal::serial::*;
use hal::time::Hertz;
use hal::timer::*;
use hal::{analog::adc, exti, rcc, stm32};
use infrared::{protocols::Nec, PeriodicReceiver};
use shell::*;
use ushell::{history::LRUHistory, UShell};

pub const IR_POLL_FREQ: Hertz = Hertz::from_raw(20_000);

pub struct Gpio {
    pub trigger: Option<SignalEdge>,
    pub pin_a: gpiob::PB7<Output<PushPull>>,
    pub pin_b: gpioc::PC14<Output<OpenDrain>>,
}

pub struct IR {
    pub receiver: PeriodicReceiver<Nec, gpioc::PC15<Input<Floating>>>,
    pub timer: Timer<stm32::TIM1>,
}

pub struct Adc {
    pub adc: adc::Adc,
    pub vbat: adc::VBat,
    pub pin: gpioa::PA1<Analog>,
    pub timer: Timer<stm32::TIM17>,
}

pub struct Pwm {
    pub pwm: pwm::Pwm<stm32::TIM3>,
    pub ch1: pwm::PwmPin<stm32::TIM3, Channel1>,
    pub ch2: pwm::PwmPin<stm32::TIM3, Channel2>,
    pub ch3: pwm::PwmPin<stm32::TIM3, Channel3>,
}

pub struct Stepper {
    pub timer: Timer<stm32::TIM16>,
    pub motor: stepper::StepperMotor<
        gpioa::PA12<Output<PushPull>>,
        gpioa::PA11<Output<PushPull>>,
        gpioa::PA6<Output<PushPull>>,
        gpioa::PA5<Output<PushPull>>,
    >,
}

#[rtic::app(device = hal::stm32, peripherals = true, dispatchers = [CEC])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        adc: Adc,
        pwm: Pwm,
        gpio: Gpio,
        opm: opm::Opm<stm32::TIM2>,
        servo: pwm::PwmPin<stm32::TIM14, Channel1>,
        stepper: Stepper,
    }

    #[local]
    struct Local {
        exti: stm32::EXTI,
        shell: Shell,
        ir: IR,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut rcc = ctx.device.RCC.freeze(rcc::Config::pll());

        let port_a = ctx.device.GPIOA.split(&mut rcc);
        let port_b = ctx.device.GPIOB.split(&mut rcc);
        let port_c = ctx.device.GPIOC.split(&mut rcc);

        let uart_cfg = BasicConfig::default().baudrate(115_200.bps());
        let mut uart = ctx
            .device
            .USART2
            .usart((port_a.pa2, port_a.pa3), uart_cfg, &mut rcc)
            .expect("Failed to init serial port");
        uart.listen(Event::Rxne);
        let shell = UShell::new(uart, AUTOCOMPLETE, LRUHistory::default());

        let opm = ctx.device.TIM2.opm(10.millis(), &mut rcc);
        opm.bind_pin(port_a.pa0).enable();
        let trigger = None;

        let mut exti = ctx.device.EXTI;
        let trigger_pin = port_c
            .pc15
            .into_pull_down_input()
            .listen(SignalEdge::All, &mut exti);

        let mut ir_timer = ctx.device.TIM1.timer(&mut rcc);
        ir_timer.start(IR_POLL_FREQ.into_duration());
        ir_timer.listen();

        let ir = IR {
            receiver: PeriodicReceiver::new(trigger_pin, IR_POLL_FREQ.raw()),
            timer: ir_timer,
        };

        let pwm = ctx.device.TIM3.pwm(10.kHz(), &mut rcc);
        let mut ch1 = pwm.bind_pin(port_b.pb4);
        let mut ch2 = pwm.bind_pin(port_a.pa7);
        let mut ch3 = pwm.bind_pin(port_b.pb0);

        let servo_pwm = ctx.device.TIM14.pwm(100.Hz(), &mut rcc);
        let mut servo = servo_pwm.bind_pin(port_a.pa4);
        servo.set_duty(0);
        servo.enable();

        let duty = ch1.get_max_duty() / 2;
        ch1.set_duty(duty);
        ch2.set_duty(duty);
        ch3.set_duty(duty);

        ch1.enable();
        ch2.enable();
        ch3.enable();

        let pwm = Pwm { pwm, ch1, ch2, ch3 };

        let mut adc_timer = ctx.device.TIM17.timer(&mut rcc);
        adc_timer.listen();

        let motor = stepper::StepperMotor::new(
            port_a.pa12.into(),
            port_a.pa11.into(),
            port_a.pa6.into(),
            port_a.pa5.into(),
        );

        let mut stepper_timer = ctx.device.TIM16.timer(&mut rcc);
        stepper_timer.listen();

        let stepper = Stepper {
            motor,
            timer: stepper_timer,
        };

        let mut delay = ctx.core.SYST.delay(&mut rcc);
        let mut adc = ctx.device.ADC.constrain(&mut rcc);
        adc.set_sample_time(adc::SampleTime::T_80);
        adc.set_precision(adc::Precision::B_12);
        adc.set_oversampling_ratio(adc::OversamplingRatio::X_16);
        adc.set_oversampling_shift(16);
        adc.oversampling_enable(true);

        delay.delay(100.micros());
        adc.calibrate();

        let mut vbat = adc::VBat::new();
        vbat.enable(&mut adc);

        let adc = Adc {
            adc,
            vbat,
            pin: port_a.pa1,
            timer: adc_timer,
        };

        let pin_a = port_b.pb7.into();
        let pin_b = port_c.pc14.into();

        let gpio = Gpio {
            pin_a,
            pin_b,
            trigger,
        };

        let local = Local { ir, exti, shell };

        let shared = Shared {
            adc,
            gpio,
            opm,
            pwm,
            servo,
            stepper,
        };

        (shared, local, init::Monotonics())
    }

    #[task(binds = EXTI4_15, priority = 4, local = [exti], shared = [opm, gpio])]
    fn trigger(ctx: trigger::Context) {
        let exti = ctx.local.exti;
        let trigger::SharedResources { mut gpio, mut opm } = ctx.shared;

        use SignalEdge::*;
        let generate = gpio.lock(|gpio| match gpio.trigger {
            Some(Falling) => exti.is_pending(exti::Event::GPIO15, Falling),
            Some(Rising) => exti.is_pending(exti::Event::GPIO15, Rising),
            _ => false,
        });
        if generate {
            opm.lock(|opm| opm.generate());
        }
        exti.unpend(exti::Event::GPIO15);
    }

    #[task(binds = TIM16, priority = 3, shared = [stepper])]
    fn stepper_timer(ctx: stepper_timer::Context) {
        let mut stepper = ctx.shared.stepper;
        stepper.lock(|stepper| {
            stepper.motor.turn();
            stepper.timer.clear_irq();
        });
    }

    #[task(binds = TIM17, priority = 3, shared = [adc])]
    fn adc_timer(ctx: adc_timer::Context) {
        let mut adc = ctx.shared.adc;
        let voltage = adc.lock(|adc| {
            adc.timer.clear_irq();
            adc.adc.read_voltage(&mut adc.pin).unwrap()
        });

        env::spawn(EnvSignal::LogAdc(voltage)).ok();
    }

    #[task(binds = TIM1_BRK_UP_TRG_COM, priority = 3, local = [ir])]
    fn ir_timer(ctx: ir_timer::Context) {
        ctx.local.ir.timer.clear_irq();
        if let Ok(Some(cmd)) = ctx.local.ir.receiver.poll() {
            env::spawn(EnvSignal::LogIRCmd(cmd)).ok();
        }
    }

    #[task(binds = USART2, priority = 1)]
    fn uart_rx(_: uart_rx::Context) {
        env::spawn(EnvSignal::SpinShell).ok();
    }

    #[task(priority = 2, capacity = 8, local = [shell], shared = [adc, gpio, opm, pwm, servo, stepper])]
    fn env(ctx: env::Context, sig: EnvSignal) {
        let mut env = ctx.shared;
        env.on_signal(ctx.local.shell, sig).ok();
    }
}
