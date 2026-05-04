use defmt::*;
use embassy_rp::{
    PeripheralType,
    i2c::{Async, Error, I2c, Instance},
};

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
        let status_reg_val = mpc4725.wakeup(i2c).await?;
        debug!("Resetting Mpc4725");
        let status_reg_val = mpc4725.reset(i2c).await?;
        Ok(mpc4725)
    }

    pub async fn reset<T>(&mut self, i2c: &mut I2c<'static, T, Async>) -> Result<(), Error>
    where
        T: PeripheralType + Instance,
    {
        // -- SHT31 expects most significant byte first
        let cmd: u8 = MPC4725_GENERAL_COMMAND;
        let data: u8 = MPC4725_GENERAL_COMMAND_RESET;
        // -- send MSB as command and LSB as data
        debug!("Sending MPC4725 command: {:#08b} {:#08b}", cmd, data);
        i2cio::write_byte(i2c, self.device_addr.value(), cmd, data).await
    }

    pub async fn wakeup<T>(&mut self, i2c: &mut I2c<'static, T, Async>) -> Result<(), Error>
    where
        T: PeripheralType + Instance,
    {
        // -- MPC4725 expects most significant byte first
        let cmd: u8 = MPC4725_GENERAL_COMMAND;
        let data: u8 = MPC4725_GENERAL_COMMAND_RESET;
        // -- send MSB as command and LSB as data
        debug!("Sending MPC4725 command: {:#08b} {:#08b}", cmd, data);
        i2cio::write_byte(i2c, self.device_addr.value(), cmd, data).await
    }

    pub async fn write_dac_value<T>(
        &mut self,
        i2c: &mut I2c<'static, T, Async>,
        value: u16,
    ) -> Result<(), Error>
    where
        T: PeripheralType + Instance,
    {
        // -- MPC4725 expects most significant byte first
        let cmd: u8 = MPC4725_WRITE_COMMAND_REGULAR;
        let data: u16 = value << 4;
        // -- send MSB as command and LSB as data
        debug!("Sending MPC4725 command: {:#08b} {:#016b}", cmd, data);
        i2cio::write_word(i2c, self.device_addr.value(), cmd, data).await
    }
}
