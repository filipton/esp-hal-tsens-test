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
    let apb_saradc = unsafe { &*APB_SARADC::PTR };
    apb_saradc.tsens_ctrl().modify(|_, w| w.pu().bit(true));
    apb_saradc.tsens_ctrl2().modify(|_, w| w.clk_sel().bit(false));

    let mut counter = 0;
    loop {
        let r = apb_saradc.tsens_ctrl().read().out().bits();
        info!("Hello world! {counter} {r}");
        counter += 1;
        Timer::after(Duration::from_millis(100)).await;
    }
}
