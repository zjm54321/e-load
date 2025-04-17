#![no_std]
#![no_main]
#![deny(unsafe_code)]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m::{asm, peripheral::syst::SystClkSource};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hprintln;
use stm32f1xx_hal::{self, pac, prelude::*};

#[entry]
fn main() -> ! {
    //init code
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(72.MHz())
        .pclk1(36.MHz())
        .freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain();


    // set clock
    let mut syst = cp.SYST;

    syst.set_clock_source(SystClkSource::Core);
    syst.set_reload(72_000_000);//72MHz
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();

    loop {
        asm::nop();
    }
}

#[exception]
fn SysTick() {
    static mut COUNT: u32 = 0;

    *COUNT += 1;

    hprintln!("{}", *COUNT);
}
