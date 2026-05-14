use defmt::*;
use embassy_rp::{
    PeripheralType,
    i2c::{Async, Error, I2c, Instance},
};
use embedded_hal_1::i2c::{I2c as I2cHal, Operation};

use crate::io::i2c::i2cio;

const MPC4725_GENERAL_COMMAND: u8 = 0x00;
const MPC4725_GENERAL_COMMAND_RESET: u8 = 0x06;
const MPC4725_GENERAL_COMMAND_WAKEUP: u8 = 0x09;

const MPC4725_WRITE_COMMAND_FAST: u8 = 0x00;
const MPC4725_WRITE_COMMAND_REGULAR: u8 = 0x40;
const MPC4725_WRITE_COMMAND_EEPROM: u8 = 0x50;

#[derive(Clone, Debug, PartialEq)]
pub enum Mpc4725DeviceAddress {
    Default,
    Secondary,
}

impl Default for Mpc4725DeviceAddress {
    fn default() -> Self {
        Mpc4725DeviceAddress::Default
    }
}

impl Mpc4725DeviceAddress {
    const DEVICE_ADDR_DEFAULT: u16 = 0x62;
    const DEVICE_ADDR_SECONDARY: u16 = 0x63;

    fn value(&self) -> u16 {
        match *self {
            Self::Default => Self::DEVICE_ADDR_DEFAULT,
            Self::Secondary => Self::DEVICE_ADDR_SECONDARY,
        }
    }
}

pub struct Mpc4725 {
    // -- device address
    device_addr: Mpc4725DeviceAddress,
}

impl Mpc4725 {
    pub async fn new<T>(
        i2c: &mut I2c<'static, T, Async>,
        device_addr: Mpc4725DeviceAddress,
    ) -> Result<Mpc4725, Error>
    where
        T: PeripheralType + Instance,
    {
        // -- create SHT31 object
        let mut mpc4725 = Mpc4725 { device_addr };
        // -- wakeup & reset
        debug!("Waking up Mpc4725");
        mpc4725.wakeup(i2c).await?;
        debug!("Resetting Mpc4725");
        mpc4725.reset(i2c).await?;
        Ok(mpc4725)
    }

    pub async fn reset<T>(&mut self, i2c: &mut I2c<'static, T, Async>) -> Result<(), Error>
    where
        T: PeripheralType + Instance,
    {
        let cmd: u8 = MPC4725_GENERAL_COMMAND;
        let data: u8 = MPC4725_GENERAL_COMMAND_RESET;
        debug!("Sending MPC4725 command: {:#08b} {:#08b}", cmd, data);
        i2cio::write_byte(i2c, self.device_addr.value(), cmd, data).await
    }

    pub async fn wakeup<T>(&mut self, i2c: &mut I2c<'static, T, Async>) -> Result<(), Error>
    where
        T: PeripheralType + Instance,
    {
        let cmd: u8 = MPC4725_GENERAL_COMMAND;
        let data: u8 = MPC4725_GENERAL_COMMAND_WAKEUP;
        debug!("Sending MPC4725 command: {:#08b} {:#08b}", cmd, data);
        i2cio::write_byte(i2c, self.device_addr.value(), cmd, data).await
    }

    pub async fn write_dac_value_fast<T>(
        &mut self,
        i2c: &mut I2c<'static, T, Async>,
        value: u16,
    ) -> Result<(), Error>
    where
        T: PeripheralType + Instance,
    {
        let data_byte1: u8 = MPC4725_WRITE_COMMAND_FAST | (((value >> 8) as u8) & 0xf);
        let data_byte2: u8 = (value & 0xff) as u8;
        // debug!(
        //     "Fast writing MPC4725 data bytes: {:#08b} {:#08b}",
        //     data_byte1, data_byte2
        // );
        let data_bytes = [data_byte1, data_byte2];
        // i2cio::write_bytes(i2c, self.device_addr.value(), data_bytes).await
        let mut ops = [Operation::Write(&data_bytes)];
        i2c.transaction(self.device_addr.value() as u8, &mut ops)
    }

    pub async fn write_dac_value_regular<T>(
        &mut self,
        i2c: &mut I2c<'static, T, Async>,
        value: u16,
    ) -> Result<(), Error>
    where
        T: PeripheralType + Instance,
    {
        let data_byte1: u8 = MPC4725_WRITE_COMMAND_REGULAR;
        let data_byte2: u8 = (value >> 8) as u8;
        let data_byte3: u8 = ((value & 0xf) as u8) << 4;
        // debug!(
        //     "Regular writing MPC4725 data bytes: {:#08b} {:#08b}, {:#08b}",
        //     data_byte1, data_byte2, data_byte3
        // );
        let data_bytes = [data_byte1, data_byte2, data_byte3];
        //i2cio::write_bytes(i2c, self.device_addr.value(), data_bytes).await
        let mut ops = [Operation::Write(&data_bytes)];
        i2c.transaction(self.device_addr.value() as u8, &mut ops)
    }
}
