//! Blink the PC13 LED on the board

#![no_std]
#![no_main]
#![deny(unsafe_code)]

use panic_halt as _;

use cortex_m::asm;
use cortex_m_rt::entry;
use stm32f1xx_hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let _clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(72.MHz())
        .pclk1(36.MHz())
        .freeze(&mut flash.acr);

    let mut gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    loop {
        asm::delay(4_000_000);
        led.set_high();
        asm::delay(4_000_000);
        led.set_low();
    }
}
