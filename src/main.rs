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

use hal::analog::adc;
use hal::gpio::*;
use hal::prelude::*;
use hal::serial::*;
use hal::{rcc, stm32, exti};
use hal::timer::*;
use shell::*;
use ushell::{history::LRUHistory, UShell};

#[rtic::app(device = hal::stm32, peripherals = true)]
mod app {
    use super::*;

    type Stepper = stepper::Stepper<
        gpioa::PA12<Output<PushPull>>,
        gpioa::PA11<Output<PushPull>>,
        gpioa::PA6<Output<PushPull>>,
        gpioa::PA5<Output<PushPull>>,
    >;

    #[shared]
    struct Shared {
        adc: adc::Adc,
        adc_pin: gpioa::PA1<Analog>,
        vbat: adc::VBat,
        opm: opm::Opm<stm32::TIM2>,
        trigger: Option<SignalEdge>,
        pin_a: gpiob::PB7<Output<PushPull>>,
        pin_b: gpioc::PC14<Output<PushPull>>,
        stepper: Stepper,
        stepper_timer: Timer<stm32::TIM16>,
        servo: pwm::PwmPin<stm32::TIM14, Channel1>,
        pwm: pwm::Pwm<stm32::TIM3>,
        pwm_ch1: pwm::PwmPin<stm32::TIM3, Channel1>,
        pwm_ch2: pwm::PwmPin<stm32::TIM3, Channel2>,
        pwm_ch3: pwm::PwmPin<stm32::TIM3, Channel3>,
    }

    #[local]
    struct Local {
        exti: stm32::EXTI,
        shell: Shell,
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
            .usart(port_a.pa2, port_a.pa3, uart_cfg, &mut rcc)
            .expect("Failed to init serial port");
        uart.listen(Event::Rxne);
        let shell = UShell::new(uart, AUTOCOMPLETE, LRUHistory::default());

        let opm = ctx.device.TIM2.opm(10.ms(), &mut rcc);
        opm.bind_pin(port_a.pa0).enable();
        let trigger = None;

        let mut exti = ctx.device.EXTI;
        port_c.pc15
            .into_pull_down_input()
            .listen(SignalEdge::All, &mut exti);

        let pwm = ctx.device.TIM3.pwm(10.khz(), &mut rcc);
        let mut pwm_ch1 = pwm.bind_pin(port_b.pb4);
        let mut pwm_ch2 = pwm.bind_pin(port_a.pa7);
        let mut pwm_ch3 = pwm.bind_pin(port_b.pb0);

        let servo_pwm = ctx.device.TIM14.pwm(100.hz(), &mut rcc);
        let mut servo = servo_pwm.bind_pin(port_a.pa4);
        servo.set_duty(0);
        servo.enable();

        let duty = pwm_ch1.get_max_duty() / 2;
        pwm_ch1.set_duty(duty);
        pwm_ch2.set_duty(duty);
        pwm_ch3.set_duty(duty);

        pwm_ch1.enable();
        pwm_ch2.enable();
        pwm_ch3.enable();

        let mut stepper_timer = ctx.device.TIM16.timer(&mut rcc);
        stepper_timer.start(1.hz());
        stepper_timer.listen();

        let stepper = stepper::Stepper::new(
            port_a.pa12.into(),
            port_a.pa11.into(),
            port_a.pa6.into(),
            port_a.pa5.into(),
        );

        let mut delay = ctx.core.SYST.delay(&mut rcc);
        let mut adc = ctx.device.ADC.constrain(&mut rcc);
        adc.set_sample_time(adc::SampleTime::T_80);
        adc.set_precision(adc::Precision::B_12);
        adc.set_oversampling_ratio(adc::OversamplingRatio::X_16);
        adc.set_oversampling_shift(16);
        adc.oversampling_enable(true);

        delay.delay(20.us());
        adc.calibrate();

        let mut vbat = adc::VBat::new();
        vbat.enable(&mut adc);

        let adc_pin = port_a.pa1;

        let pin_a = port_b.pb7.into();
        let pin_b = port_c.pc14.into();

        (
            Shared {
                adc,
                adc_pin,
                pin_a,
                pin_b,
                vbat,
                pwm,
                opm,
                trigger,
                pwm_ch1,
                pwm_ch2,
                pwm_ch3,
                servo,
                stepper,
                stepper_timer,
            },
            Local { exti, shell },
            init::Monotonics(),
        )
    }

    #[task(binds = TIM16, priority = 2, shared = [stepper, stepper_timer])]
    fn stepper_timer_tick(mut ctx: stepper_timer_tick::Context) {
        ctx.shared.stepper.lock(|stepper| stepper.turn());
        ctx.shared.stepper_timer.lock(|tim| tim.clear_irq());
    }

    #[task(binds = EXTI4_15, priority = 2, local = [exti], shared = [opm, trigger])]
    fn ext_trigger(ctx: ext_trigger::Context) {
        let mut trigger = ctx.shared.trigger;
        let mut opm = ctx.shared.opm;
        let exti = ctx.local.exti;
        let generate = trigger.lock(|t| match t {
            Some(SignalEdge::Falling) if exti.is_pending(exti::Event::GPIO15, SignalEdge::Falling) => true,
            Some(SignalEdge::Rising) if exti.is_pending(exti::Event::GPIO15, SignalEdge::Rising)  => true,
            _ => false
        });
        if generate {
            opm.lock(|opm| opm.generate());
        }
        exti.unpend(exti::Event::GPIO15);
    }

    #[task(
        binds = USART2, 
        priority = 1, 
        local = [shell], 
        shared = [adc, adc_pin, pin_a, pin_b, vbat, opm, trigger, pwm, pwm_ch1, pwm_ch2, pwm_ch3, servo, stepper, stepper_timer]
    )]
    fn serial_data(mut ctx: serial_data::Context) {
        ctx.local.shell.spin(&mut ctx.shared).ok();
    }
}
