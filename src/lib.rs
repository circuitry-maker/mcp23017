#![deny(missing_docs)]
#![deny(warnings)]
#![allow(dead_code)]
#![no_std]

//! Manages a new MCP23017, a 16-Bit I2C I/O Expander with Serial Interface module

extern crate cast;
extern crate embedded_hal as ehal;
extern crate generic_array;
extern crate nb;

use ehal::blocking::i2c::{Write, WriteRead};

const ADDRESS_DEFAULT: u8 = 0x20;

/// Struct for MCP23017. It provides 16-bit, general purpose parallel I/O expansion for I2C bus.
/// It consists of multiple 8-bit configuration registers for input, output and polarity selection.
/// The system master can enable the I/Os as either inputs or outputs by writing the I/O configuration
/// bits (IODIRA/B). The data for each input or output is kept in the corresponding input or output
/// register. The polarity of the Input Port register can be inverted with the Polarity Inversion register
pub struct MCP23017<I2C: WriteRead> {
    com: I2C,
    address: u8,
}

//https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library/blob/master/Adafruit_MCP23017.h

/// Defines errors
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// Underlying bus error
    BusError(E),
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::BusError(error)
    }
}

impl<I2C, E> MCP23017<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// Creates an expander with default configuration
    pub fn default(i2c: I2C) -> Result<MCP23017<I2C>, Error<E>>
    where
        I2C: WriteRead<Error = E>,
    {
        MCP23017::new(i2c, ADDRESS_DEFAULT)
    }

    /// Creates an expander with specific configuration
    pub fn new(i2c: I2C, address: u8) -> Result<MCP23017<I2C>, Error<E>>
    where
        I2C: WriteRead<Error = E>,
    {
        let chip = MCP23017 {
            com: i2c,
            address
        };

        Ok(chip)
    }
}

/// Returns bit number associated to a give pin
fn bit_for_pin(pin: u8) -> u8 {
	pin % 8
}

/// Returns register address, port dependent, for a given pin
fn register_for_pin(pin: u8, port_a_addr: u8, port_b_addr: u8) -> u8 {
    if pin < 8 {
        port_a_addr
    } else {
        port_b_addr
    }
}

#[allow(non_camel_case_types)]
enum Register {
    IODIRA = 0x00,
    IPOLA = 0x02,
    GPINTENA = 0x04,
    DEFVALA = 0x06,
    INTCONA = 0x08,
    IOCONA = 0x0A,
    GPPUA = 0x0C,
    INTFA = 0x0E,
    INTCAPA = 0x10,
    GPIOA = 0x12,
    OLATA = 0x14,
    IODIRB = 0x01,
    IPOLB = 0x03,
    GPINTENB = 0x05,
    DEFVALB = 0x07,
    INTCONB = 0x09,
    IOCONB = 0x0B,
    GPPUB = 0x0D,
    INTFB = 0x0F,
    INTCAPB = 0x11,
    GPIOB = 0x13,
    OLATB = 0x15,
    INT_ERR = 255,
}