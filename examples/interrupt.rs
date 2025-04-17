#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger
use core::cell::RefCell;
use cortex_m::{asm, interrupt::Mutex,peripheral::{nvic, NVIC}};
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32f1xx_hal::{
    self,
    device::TIM2,
    pac::{self, interrupt, tim1},
    prelude::*,
    rcc::RccExt,
    timer::{CounterHz, Event,TimerExt},
};

static G_TIM: Mutex<RefCell<Option<CounterHz<TIM2>>>> = Mutex::new(RefCell::new(None));


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

    // set timer
    let mut tim2 = dp.TIM2;
    let mut nvic = cp.NVIC;
    unsafe {
        NVIC::unmask(interrupt::TIM2);
        nvic.set_priority(interrupt::TIM2, 2);
    }
    let mut timer = tim2.counter_hz(&clocks);

    timer.start_raw(100, 2500);

    timer.listen(Event::Update);
    cortex_m::interrupt::free(|cs| G_TIM.borrow(cs).replace(Some(timer)));


    loop {
        asm::nop();
    }
}

#[interrupt]
fn TIM2() {
    static mut COUNT: u32 = 1;
    *COUNT += 1;
    hprintln!("{}", *COUNT);
}
