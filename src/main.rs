#![no_std]
#![no_main]
#![deny(unsafe_code)]

// pick a panicking behavior
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;

#[cfg(feature = "semihosting")]
use cortex_m_semihosting::hprintln;

use stm32f1xx_hal::{
    self,
    i2c::{BlockingI2c, DutyCycle, Mode},
    pac::{self},
    prelude::*,
};

use bitbang_hal::i2c::I2cBB;
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10, iso_8859_13::FONT_10X20},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
    text::{Baseline, Text},
};
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
    #[cfg(feature = "semihosting")]
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
    .into_buffered_graphics_mode();
    screen.init().ok();
    led.set_low(); //屏幕加载完毕

    let big_text = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(BinaryColor::On)
        .build();

    let text = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline(".", Point::new(0, 0), big_text, Baseline::Top)
        .draw(&mut screen)
        .ok();
    screen.flush().ok();

    #[cfg(feature = "semihosting")]
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

    let mut dac = MCP4725::new(dac_i2c, 0b000);

    Text::with_baseline(".", Point::new(10, 0), big_text, Baseline::Top)
        .draw(&mut screen)
        .ok();
    screen.flush().ok();

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

    */

    //按键启用
    let _row = gpioa.pa11.into_open_drain_output(&mut gpioa.crh);
    let (col1, col2, col3, col4) = (
        gpiob.pb12.into_pull_up_input(&mut gpiob.crh),
        gpiob.pb13.into_pull_up_input(&mut gpiob.crh),
        gpiob.pb14.into_pull_up_input(&mut gpiob.crh),
        gpiob.pb15.into_pull_up_input(&mut gpiob.crh),
    );

    Text::with_baseline(".", Point::new(12, 0), big_text, Baseline::Top)
        .draw(&mut screen)
        .ok();
    screen.flush().ok();
    screen.clear_buffer();

    //相关代码
    const VREF: VoltageReference = VoltageReference {
        v15: [
            0x01a3, 0x0331, 0x04a7, 0x0636, 0x07b0, 0x0935, 0x0a9e, 0x0c47, 0x0df6, 0x0f49,
        ],
        v10: [
            0x01a3, 0x0331, 0x04a7, 0x0636, 0x07b0, 0x0935, 0x0a9e, 0x0c47, 0x0df6, 0x0f49,
        ],
        v5: [
            0x01a3, 0x0331, 0x04a7, 0x0636, 0x07b0, 0x0935, 0x0a9e, 0x0c47, 0x0df6, 0x0f49,
        ],
        v1: [
            0x01a3, 0x0331, 0x04a7, 0x0636, 0x07b0, 0x0935, 0x0a9e, 0x0c47, 0x0df6, 0x0f49,
        ],
    };
    let mut voltage_mode: u8 = 0;
    let mut current_mode: usize = 0;
    let mut dac_data: Option<u16>;

    Text::with_baseline("Booted!", Point::new(0, 0), big_text, Baseline::Top)
        .draw(&mut screen)
        .ok();
    screen.flush().ok();
    screen.clear_buffer();
    delay.delay(1.secs());

    loop {
        if col1.is_low() {
            delay.delay_ms(20_u16);
            while col1.is_low() {}
            delay.delay_ms(20_u16);
            if voltage_mode < 4 {
                voltage_mode += 1;
            } else {
                voltage_mode = 0;
            }
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
            (0, _) => dac_data = None,
            (_, 0) => dac_data = None,
            (1, i) => dac_data = Some(VREF.v1[i - 1]),
            (2, i) => dac_data = Some(VREF.v5[i - 1]),
            (3, i) => dac_data = Some(VREF.v10[i - 1]),
            (4, i) => dac_data = Some(VREF.v15[i - 1]),
            (_, _) => dac_data = None,
        }

        if dac_data.is_some() {
            dac.set_dac_fast(PowerDown::Normal, dac_data.unwrap()).ok();
        }

        screen_loop(&mut screen, voltage_mode, current_mode, text);
    }
}

struct VoltageReference {
    v15: [u16; 10],
    v10: [u16; 10],
    v5: [u16; 10],
    v1: [u16; 10],
}

fn screen_loop(
    screen: &mut Ssd1306<
        ssd1306::prelude::I2CInterface<
            bitbang_hal::i2c::I2cBB<
                stm32f1xx_hal::gpio::Pin<
                    'A',
                    6,
                    stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>,
                >,
                stm32f1xx_hal::gpio::Pin<
                    'A',
                    7,
                    stm32f1xx_hal::gpio::Output<stm32f1xx_hal::gpio::OpenDrain>,
                >,
                stm32f1xx_hal::timer::CounterHz<pac::TIM3>,
            >,
        >,
        DisplaySize128x64,
        ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
    >,
    v: u8,
    i: usize,
    text: embedded_graphics::mono_font::MonoTextStyle<'_, BinaryColor>,
) {
    screen.clear_buffer();
    Line::new(Point::new(0, 45), Point::new(128, 45))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(screen)
        .ok();
    Text::with_baseline("V=  V", Point::new(0, 50), text, Baseline::Top)
        .draw(screen)
        .ok();
    Text::with_baseline("I=   A", Point::new(92, 50), text, Baseline::Top)
        .draw(screen)
        .ok();
    Text::with_baseline(
        match v {
            0 => "xx",
            1 => "01",
            2 => "05",
            3 => "10",
            4 => "15",
            _ => "??",
        },
        Point::new(12, 50),
        text,
        Baseline::Top,
    )
    .draw(screen)
    .ok();
    Text::with_baseline(
        match i {
            0 => "xxx",
            1 => "0.2",
            2 => "0.4",
            3 => "0.6",
            4 => "0.8",
            5 => "1.0",
            6 => "1.2",
            7 => "1.4",
            8 => "1.6",
            9 => "1.8",
            10 => "2.0",
            _ => "???",
        },
        Point::new(104, 50),
        text,
        Baseline::Top,
    )
    .draw(screen)
    .ok();

    if (v!= 0) && (i != 0) {
        Text::with_baseline("Working", Point::new(46, 25), text, Baseline::Top)
        .draw(screen)
        .ok();
    } else {
        Text::with_baseline("Offline", Point::new(46, 25), text, Baseline::Top)
        .draw(screen)
        .ok();
    }

    screen.flush().ok();
}
