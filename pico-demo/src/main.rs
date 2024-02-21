#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

#[rtic::app(
        device = rp_pico::hal::pac,
        dispatchers = [TIMER_IRQ_1,  SW1_IRQ, SW2_IRQ]
    )]
mod app {
    use defmt::*;
    use defmt_rtt as _;
    use rp_pico::hal::{
        clocks, gpio, gpio::pin::bank0::Gpio25, gpio::pin::PushPullOutput, rom_data, sio::Sio,
        watchdog::Watchdog,
    };
    use rp_pico::XOSC_CRYSTAL_FREQ;

    use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};

    use panic_probe as _;
    use rtic_monotonics::rp2040::*;
    //use rtic_monotonics::systick::Systick;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: gpio::Pin<Gpio25, PushPullOutput>,
        // i2c: &'static mut I2CBus,
    }

    #[init(local=[
            // Task local initialized resources are static
            // Here we use MaybeUninit to allow for initialization in init()
            // This enables its usage in driver initialization
            //i2c_ctx: MaybeUninit<I2CBus> = MaybeUninit::uninit()
        ])]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let rom_version = rom_data::rom_version_number();
        let git_revision = rom_data::git_revision();
        let copyright = rom_data::copyright_string();

        info!(
            "RP2040-B{=u8} (ROM {=u32:x}) {=str}",
            rom_version, git_revision, copyright
        );

        //let core_peripherals: pac::CorePeripherals = ctx.core;
        // let mut peripherals: pac::Peripherals = ctx.device;

        // Initialize the interrupt for the RP2040 timer and obtain the token
        // proving that we have.
        let rp2040_timer_token = rtic_monotonics::create_rp2040_monotonic_token!();
        // Configure the clocks, watchdog - The default is to generate a 125 MHz system clock
        Timer::start(ctx.device.TIMER, &mut ctx.device.RESETS, rp2040_timer_token); // default rp2040 clock-rate is 125MHz
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let _clocks = clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // Init LED pin
        let single_cycle_io = Sio::new(ctx.device.SIO);
        let pins = rp_pico::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            single_cycle_io.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        let mut led = pins.led.into_push_pull_output();
        led.set_low().unwrap();

        // Spawn heartbeat task
        heartbeat::spawn().ok();

        bar::spawn().ok();
        baz::spawn().ok();

        // Return resources and timer
        (Shared {}, Local { led })
    }

    #[idle]
    fn idle(_context: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfe();
        }
    }

    #[task(local = [led], priority = 1)]
    async fn heartbeat(ctx: heartbeat::Context) {
        // Loop forever.
        //
        // It is important to remember that tasks that loop
        // forever should have an `await` somewhere in that loop.
        //
        // Without the await, the task will never yield back to
        // the async executor, which means that no other lower or
        // equal  priority task will be able to run.
        loop {
            info!("LED");
            // Flicker the built-in LED
            _ = ctx.local.led.toggle();

            // Congrats, you can use your i2c and have access to it here,
            // now to do something with it!

            // Delay for 1 second
            Timer::delay(1000.millis()).await;
        }
    }

    #[task(priority = 3)]
    async fn bar(_: bar::Context) {
        loop {
            info!("bar");
            Timer::delay(500.millis()).await;
        }
    }

    #[task(priority = 2)]
    async fn baz(_: baz::Context) {
        loop {
            info!("baz");
            Timer::delay(500.millis()).await;
        }
    }
}
