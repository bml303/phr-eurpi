use defmt::*;
use embassy_rp::adc::{Adc, Async as AdcAsync, Channel as AdcChannel};
use embassy_time::Timer;

use crate::utils::Debouncer;

use super::ChannelInputsType;

#[embassy_executor::task]
pub async fn core0_task(
    mut adc: Adc<'static, AdcAsync>,
    mut p26: AdcChannel<'static>,
    mut p27: AdcChannel<'static>,
    mut p28: AdcChannel<'static>,
    mut btn1: Debouncer<'static>,
    mut btn2: Debouncer<'static>,
    channel_inputs: &'static ChannelInputsType,
) {
    info!("Hello from core 0");
    loop {
        // -- do this every 100 milliseconds
        let ain = adc.read(&mut p26).await.unwrap();
        let kn1 = adc.read(&mut p27).await.unwrap();
        let kn2 = adc.read(&mut p28).await.unwrap();
        let btn1_lvl = btn1.level().await;
        let btn2_lvl = btn2.level().await;
        channel_inputs
            .send((ain, kn1, kn2, btn1_lvl, btn2_lvl))
            .await;
        Timer::after_millis(100).await;
    }
}
