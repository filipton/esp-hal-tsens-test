#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp32c3::{APB_SARADC, RTC_CNTL, SYSTEM};
use esp_backtrace as _;
use esp_hal::prelude::*;
use log::info;

extern crate alloc;

const TEMPERATURE_SENSOR_ATTR_RANGE_NUM: usize = 5;
struct TemperatureSensorAttribute {
    offset: i16,
    reg_val: u8,
    min: i16,
    max: i16,
    error: u8,
}

const TEMPERATURE_SENSOR_ATTRIBUTES: [TemperatureSensorAttribute;
    TEMPERATURE_SENSOR_ATTR_RANGE_NUM] = [
    TemperatureSensorAttribute {
        offset: -2,
        reg_val: 5,
        min: 50,
        max: 125,
        error: 3,
    },
    TemperatureSensorAttribute {
        offset: -1,
        reg_val: 7,
        min: 20,
        max: 100,
        error: 2,
    },
    TemperatureSensorAttribute {
        offset: 0,
        reg_val: 15,
        min: -10,
        max: 80,
        error: 1,
    },
    TemperatureSensorAttribute {
        offset: 1,
        reg_val: 11,
        min: -30,
        max: 50,
        error: 2,
    },
    TemperatureSensorAttribute {
        offset: 2,
        reg_val: 10,
        min: -40,
        max: 20,
        error: 3,
    },
];

#[allow(unused)]
extern "C" {
    pub(crate) fn rom_i2c_writeReg(block: u32, block_hostid: u32, reg_add: u32, indata: u32);

    pub(crate) fn rom_i2c_writeReg_Mask(
        block: u32,
        block_hostid: u32,
        reg_add: u32,
        reg_add_msb: u32,
        reg_add_lsb: u32,
        indata: u32,
    );
}

macro_rules! regi2c_write_mask {
    ( $block: ident, $reg_add: ident, $indata: expr ) => {
        paste::paste! {
            #[allow(unused_unsafe)]
            unsafe {
                rom_i2c_writeReg_Mask(
                    $block as u32,
                    [<$block _HOSTID>] as u32,
                    $reg_add as u32,
                    [<$reg_add _MSB>] as u32,
                    [<$reg_add _LSB>] as u32,
                    $indata as u32
                )
            }
        }
    };
}

const I2C_SAR_ADC: u32 = 0x69;
const I2C_SARADC_TSENS_DAC: u32 = 0x06;
const I2C_SAR_ADC_HOSTID: u32 = 0;
const I2C_SARADC_TSENS_DAC_MSB: u32 = 3;
const I2C_SARADC_TSENS_DAC_LSB: u32 = 0;

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        config.cpu_clock = CpuClock::max();
        config
    });

    esp_alloc::heap_allocator!(72 * 1024);
    esp_println::logger::init_logger_from_env();

    let timg0 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // TRY TO: regi2c_saradc_enable
    let rtc_cntl = unsafe { &*RTC_CNTL::PTR };
    let ana_conf = rtc_cntl.ana_conf();
    ana_conf.modify(|_, w| w.reset_por_force_pd().clear_bit().sar_i2c_pu().bit(true));

    let system = unsafe { &*SYSTEM::PTR };

    // adc_apb_periph_claim
    system
        .perip_clk_en0()
        .modify(|_, w| w.apb_saradc_clk_en().bit(true));
    system
        .perip_rst_en0()
        .modify(|_, w| w.apb_saradc_rst().bit(true));
    system
        .perip_rst_en0()
        .modify(|_, w| w.apb_saradc_rst().bit(false));

    // temperature_sensor_ll_bus_clk_enable
    system
        .perip_clk_en1()
        .modify(|_, w| w.tsens_clk_en().bit(true));

    // temperature_sensor_ll_reset_module
    system
        .perip_rst_en1()
        .modify(|_, w| w.tsens_rst().bit(true));
    system
        .perip_rst_en1()
        .modify(|_, w| w.tsens_rst().bit(false));

    // temperature_sensor_ll_enable
    let apb_saradc = unsafe { &*APB_SARADC::PTR };
    apb_saradc.tsens_ctrl().modify(|_, w| w.pu().bit(true));
    apb_saradc
        .tsens_ctrl2()
        .modify(|_, w| w.clk_sel().bit(true));

    // temperature_sensor_ll_set_range
    regi2c_write_mask!(
        I2C_SAR_ADC,
        I2C_SARADC_TSENS_DAC,
        TEMPERATURE_SENSOR_ATTRIBUTES[3].reg_val
    );

    let mut counter = 0;
    loop {
        let raw = apb_saradc.tsens_ctrl().read().out().bits() as u32;
        let temp_sel = &TEMPERATURE_SENSOR_ATTRIBUTES[3];
        let raw = 0.4386 * raw as f32 - 27.88 * temp_sel.offset as f32 - 20.52;

        info!("Hello world! {counter} {raw}");
        counter += 1;
        Timer::after(Duration::from_millis(100)).await;
    }
}
