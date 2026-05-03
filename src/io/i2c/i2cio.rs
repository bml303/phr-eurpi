#[allow(unused_imports)]
use defmt::*;
use embassy_rp::{
    PeripheralType,
    i2c::{Async, Error, I2c, Instance},
};
use embassy_time::Delay;
use embedded_hal_async::delay::DelayNs;

pub async fn read_word<T>(
    i2c: &mut I2c<'static, T, Async>,
    dev_addr: u16,
    register: u8,
) -> Result<u16, Error>
where
    T: PeripheralType + Instance,
{
    let reg = [register];
    let mut data = [0; 2];
    i2c.write_read_async(dev_addr, reg.into_iter(), &mut data)
        .await?;
    let data = ((data[1] as u16) << 8) | (data[0] as u16);
    Ok(data)
}

pub async fn read_byte<T>(
    i2c: &mut I2c<'static, T, Async>,
    dev_addr: u16,
    register: u8,
) -> Result<u8, Error>
where
    T: PeripheralType + Instance,
{
    let reg = [register];
    let mut data = [0];
    i2c.write_read_async(dev_addr, reg.into_iter(), &mut data)
        .await?;
    Ok(data[0])
}

pub async fn read_bytes<T>(
    i2c: &mut I2c<'static, T, Async>,
    dev_addr: u16,
    data: &mut [u8],
) -> Result<(), Error>
where
    T: PeripheralType + Instance,
{
    i2c.read_async(dev_addr, data).await?;
    Ok(())
}

pub async fn read_bytes_reg<T>(
    i2c: &mut I2c<'static, T, Async>,
    dev_addr: u16,
    register: u8,
    data: &mut [u8],
) -> Result<(), Error>
where
    T: PeripheralType + Instance,
{
    let reg = [register];
    i2c.write_read_async(dev_addr, reg.into_iter(), data)
        .await?;
    Ok(())
}

pub async fn write_read_bytes<T, const LEN: usize>(
    i2c: &mut I2c<'static, T, Async>,
    dev_addr: u16,
    write_data: [u8; LEN],
    read_data: &mut [u8],
) -> Result<(), Error>
where
    T: PeripheralType + Instance,
{
    i2c.write_read_async(dev_addr, write_data.into_iter(), read_data)
        .await?;
    Ok(())
}

#[allow(dead_code)]
pub async fn write_byte_single<T>(
    i2c: &mut I2c<'static, T, Async>,
    dev_addr: u16,
    data: u8,
) -> Result<(), Error>
where
    T: PeripheralType + Instance,
{
    let data = [data];
    i2c.write_async(dev_addr, data.into_iter()).await
}

pub async fn write_byte<T>(
    i2c: &mut I2c<'static, T, Async>,
    dev_addr: u16,
    register: u8,
    data: u8,
) -> Result<(), Error>
where
    T: PeripheralType + Instance,
{
    let data = [register, data];
    i2c.write_async(dev_addr, data.into_iter()).await
}

pub async fn write_bytes<T, const LEN: usize>(
    i2c: &mut I2c<'static, T, Async>,
    dev_addr: u16,
    data: [u8; LEN],
) -> Result<(), Error>
where
    T: PeripheralType + Instance,
{
    i2c.write_async(dev_addr, data.into_iter()).await
}

pub async fn write_word<T>(
    i2c: &mut I2c<'static, T, Async>,
    dev_addr: u16,
    register: u8,
    data: u16,
) -> Result<(), Error>
where
    T: PeripheralType + Instance,
{
    let dbt_msb = (data >> 8) as u8;
    let dbt_lsb = data as u8;
    let data = [register, dbt_lsb, dbt_msb];
    i2c.write_async(dev_addr, data.into_iter()).await
}

pub async fn delay(milli_secs: u32) {
    let mut delay = Delay {};
    delay.delay_ms(milli_secs).await;
}
