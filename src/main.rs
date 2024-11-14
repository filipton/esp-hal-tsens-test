#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp32c3::{
    apb_saradc::{tsens_ctrl, TSENS_CTRL},
    APB_SARADC,
};
use esp_backtrace as _;
use esp_hal::prelude::*;
use log::info;

extern crate alloc;

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
    let rtc_cntl = unsafe { &*esp32c3::RTC_CNTL::PTR };
    let ana_conf = rtc_cntl.ana_conf();
    ana_conf.modify(|_, w| w.reset_por_force_pd().clear_bit().sar_i2c_pu().bit(true));

    let system = unsafe { &*esp32c3::SYSTEM::PTR };

    // temperature_sensor_ll_bus_clk_enable
    system
        .perip_clk_en1()
        .modify(|_, w| w.tsens_clk_en().bit(true));
    system
        .perip_rst_en1()
        .modify(|_, w| w.tsens_rst().bit(true));
    system
        .perip_rst_en1()
        .modify(|_, w| w.tsens_rst().bit(false));

    // temperature_sensor_ll_enable
    let apb_saradc = unsafe { &*esp32c3::APB_SARADC::PTR };
    apb_saradc.tsens_ctrl().modify(|_, w| w.pu().bit(true));
    apb_saradc
        .tsens_ctrl2()
        .modify(|_, w| w.clk_sel().bit(true));


    let mut counter = 0;
    loop {
        let r = apb_saradc.tsens_ctrl().read().out().bits();
        info!("Hello world! {counter} {r}");
        counter += 1;
        Timer::after(Duration::from_millis(100)).await;
    }
}
