#![no_std]
#![no_main]
#![deny(unsafe_code)]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;

#[cfg(debug_assertions)]
use cortex_m_semihosting::hprintln;

use stm32f1xx_hal::{
    self, i2c::{BlockingI2c, DutyCycle, Mode}, pac::{self}, prelude::*
};

use bitbang_hal::i2c::I2cBB;
use core::fmt::Write;
use ssd1306::{
    I2CDisplayInterface, Ssd1306, mode::DisplayConfig, prelude::DisplayRotation,
    size::DisplaySize128x64,
};

use mcp4725::{MCP4725, PowerDown};

//use ina226::{AVG, DEFAULT_ADDRESS, INA226, VBUSCT, VSHCT};

#[entry]
fn main() -> ! {
    //初始化代码
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    led.set_low(); //点亮led，代码开始加载

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(72.MHz())
        .pclk1(36.MHz())
        .freeze(&mut flash.acr);

    let mut delay = cp.SYST.delay(&clocks);
    let mut afio = dp.AFIO.constrain();

    led.set_high(); //系统配置加载完毕。
    #[cfg(debug_assertions)]
    hprintln!("sys init finished");

    delay.delay(1.secs()); //等待外设准备就绪

    //设置ssd1306，使用模拟i2c(pa6,pa7)

    let screen_scl = gpioa.pa6.into_open_drain_output(&mut gpioa.crl);
    let screen_sda = gpioa.pa7.into_open_drain_output(&mut gpioa.crl);
    let mut screen_tmr = dp.TIM3.counter_hz(&clocks);
    screen_tmr.start(800.kHz()).ok();
    let screen_i2c = I2cBB::new(screen_scl, screen_sda, screen_tmr);
    led.set_high(); //模拟i2c加载完毕

    let screen_interface = I2CDisplayInterface::new(screen_i2c);
    let mut screen = Ssd1306::new(
        screen_interface,
        DisplaySize128x64,
        DisplayRotation::Rotate0,
    )
    .into_terminal_mode();
    screen.init().ok();
    led.set_low(); //屏幕加载完毕
    screen.clear().ok();
    write!(screen, "Screen Init Finished.\n").ok();
    #[cfg(debug_assertions)]
    hprintln!("ssd1306 init finished");

    // 设置 mcp4725 ，使用硬件i2c(pb6,pb7)
    let dac_scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
    let dac_sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
    let dac_i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (dac_scl, dac_sda),
        &mut afio.mapr,
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );
    write!(screen, "Dac_i2c init finished.\n").ok();

    let mut dac = MCP4725::new(dac_i2c, 0b000);
    write!(screen, "Dac init finished.\n").ok();
    dac.set_dac_and_eeprom(PowerDown::Normal, 0x0000).ok();

    // 设置ina226，使用硬件i2c (pb10,pb11)
    /*
    let ina_scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
    let ina_sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);
    let ina226_i2c = BlockingI2c::i2c2(
        dp.I2C2,
        (ina_scl, ina_sda),
        Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );
    write!(screen, "Ina226_i2c init finished.\n").ok();
    write!(screen, "Ina226_i2c init finished.\n").ok();

    let mut ina226 = INA226::new(ina226_i2c, DEFAULT_ADDRESS);
    ina226
        .set_configuration(&ina226::Config {
            avg: AVG::_1,
            vbusct: VBUSCT::_1100us,
            vshct: VSHCT::_1100us,
            mode: ina226::MODE::BusVoltageContinuous,
        })
        .ok();
    ina226.callibrate(0.01, 2.5).ok();
    write!(screen, "Ina226 init finished.\n").ok();
    */

    //按键启用
    let _row = gpioa.pa11.into_open_drain_output(&mut gpioa.crh);
    let (col1, col2, col3, col4) = (
        gpiob.pb12.into_pull_up_input(&mut gpiob.crh),
        gpiob.pb13.into_pull_up_input(&mut gpiob.crh),
        gpiob.pb14.into_pull_up_input(&mut gpiob.crh),
        gpiob.pb15.into_pull_up_input(&mut gpiob.crh),
    );
    write!(screen, "Key init finished.\n").ok();

    
    //相关代码
    const  VREF: VoltageReference = VoltageReference {
        v15: [
            0x0000, 0x000f, 0x00f0, 0x00ff, 0x0f00, 0x0f0f, 0x0ff0, 0x0fff, 0xf000, 0xf00f,
        ],
        v10: [0; 10],
        v5: [0; 10],
        v1: [0; 10],
    };
    let mut voltage_mode: u8 = 0;
    let mut current_mode = 0;

    screen.clear().ok();

    loop {
        write!(screen,"still loop\n").ok();
        if col1.is_low() {
            delay.delay_ms(20_u16);
            while col1.is_low() {}
            delay.delay_ms(20_u16);
            if voltage_mode < 4 {
                voltage_mode += 1;
            } else {
                voltage_mode = 0;
            }
            write!(screen, "voltage mode is {}\n", voltage_mode).ok();
        }
        if col2.is_low() {
            delay.delay_ms(20_u16);
            while col2.is_low() {}
            delay.delay_ms(20_u16);
            if current_mode < 10 {
                current_mode += 1;
            } else {
                current_mode = 0;
            }
            write!(screen, "current mode is {}\n", current_mode).ok();
        }
        if col3.is_low() {
            delay.delay_ms(20_u16);
            while col3.is_low() {}
            delay.delay_ms(20_u16);
        }
        if col4.is_low() {
            delay.delay_ms(20_u16);
            while col4.is_low() {}
            delay.delay_ms(20_u16);
        }
        
        match (voltage_mode, current_mode) {
            (0, _) => {
                dac.set_dac_fast(PowerDown::Normal, 0x0000).ok();
            }
            (1, i) => {
                #[cfg(feature = "dbg")]
                hprintln!("{}",VREF.v15[i]);
                dac.set_dac_fast(PowerDown::Normal, VREF.v15[i]).ok();
            }
            (2, i) => {
                #[cfg(feature = "dbg")]
                hprintln!("{}",VREF.v10[i]);
                dac.set_dac_fast(PowerDown::Normal, VREF.v10[i]).ok();
            }
            (3, i) => {
                #[cfg(feature = "dbg")]
                hprintln!("{}",VREF.v5[i]);
                dac.set_dac_fast(PowerDown::Normal, VREF.v5[i]).ok();
            }
            (4, i) => {
                #[cfg(feature = "dbg")]
                hprintln!("{}",VREF.v1[i]);
                dac.set_dac_fast(PowerDown::Normal, VREF.v1[i]).ok();
            }
            (_, _) => {}
        }
    
    }
}

struct VoltageReference {
    v15: [u16; 10],
    v10: [u16; 10],
    v5: [u16; 10],
    v1: [u16; 10],
}
