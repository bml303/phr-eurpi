use defmt::*;
use embassy_rp::{flash::Flash, peripherals::FLASH};
use embassy_time::Instant;

pub const FLASH_SIZE: usize = 2 * 1024 * 1024;

pub fn check_flash(
    flash: &mut Flash<'static, FLASH, embassy_rp::flash::Async, FLASH_SIZE>,
) -> (u64, [u8; 8]) {
    // -- get JEDEC id
    let jedec_id = match flash.blocking_jedec_id() {
        Ok(jedec_id) => jedec_id,
        Err(err) => {
            error!("Failed to read jedec id: {}", err);
            Instant::now().as_micros() as u32
        }
    };
    info!("jedec id: 0x{:x}", jedec_id);
    // -- get unique id
    let mut unique_id_bytes = [0; 8];
    if let Err(err) = flash.blocking_unique_id(&mut unique_id_bytes) {
        error!("Failed to read unique id: {}", err);
        let micros = Instant::now().as_micros();
        unique_id_bytes[0] = micros as u8;
        let micros = micros >> 8;
        unique_id_bytes[1] = micros as u8;
        let micros = micros >> 8;
        unique_id_bytes[2] = micros as u8;
        let micros = micros >> 8;
        unique_id_bytes[3] = micros as u8;
        let micros = micros >> 8;
        unique_id_bytes[4] = micros as u8;
        let micros = micros >> 8;
        unique_id_bytes[5] = micros as u8;
        let micros = micros >> 8;
        unique_id_bytes[6] = micros as u8;
        let micros = micros >> 8;
        unique_id_bytes[7] = micros as u8;
    }
    info!("unique id bytes: {:?}", unique_id_bytes);
    // -- get 64 bit value
    let flash_uid = u64::from_ne_bytes(unique_id_bytes);
    info!("flash uid: {:?}", flash_uid);
    (flash_uid, unique_id_bytes)
}
