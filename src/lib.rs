#![no_std]

//! Manages a new MCP23017, a 16-Bit I2C I/O Expander with Serial Interface module

#![deny(
    missing_docs,
    missing_debug_implementations,
    missing_copy_implementations,
    trivial_casts,
    trivial_numeric_casts,
    unstable_features,
    unused_import_braces,
    unused_qualifications,
    warnings
)]
#![allow(
    dead_code,
    non_camel_case_types,
)]

extern crate cast;
extern crate embedded_hal as ehal;
extern crate generic_array;
extern crate nb;

use ::core::mem::MaybeUninit;

use cast::u16;

use ehal::blocking::i2c::{Write, WriteRead};
use generic_array::typenum::consts::*;
use generic_array::{ArrayLength, GenericArray};

const ADDRESS_DEFAULT: u8 = 0x20;
const LOW: u8 = 0;
const HIGH: u8 = 1;

/// Struct for MCP23017. It provides 16-bit, general purpose parallel I/O expansion for I2C bus.
/// It consists of multiple 8-bit configuration registers for input, output and polarity selection.
/// The system master can enable the I/Os as either inputs or outputs by writing the I/O configuration
/// bits (IODIRA/B). The data for each input or output is kept in the corresponding input or output
/// register. The polarity of the Input Port register can be inverted with the Polarity Inversion register
#[derive(Clone, Copy, Debug)]
pub struct MCP23017<I2C: WriteRead> {
    com: I2C,
    /// lol
    pub address: u8,
}

/// Defines errors
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// Underlying bus error
    BusError(E),
    /// Interrupt pin not found
    InterruptPinError,
}

impl<E> From<E> for Error<E> {
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

    fn init_hardware(&mut self) -> Result<(), Error<E>> {
	// set all inputs to defaults on port A and B
        self.write_register(Register::IODIRA, 0xff)?;
	    self.write_register(Register::IODIRB, 0xff)?;

        Ok(())
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, E> {
        let mut data: [u8; 1] = [0];
        self.com.write_read(self.address, &[reg as u8], &mut data)?;
        Ok(data[0])
    }

    fn read_registers<N>(&mut self, reg: Register) -> Result<GenericArray<u8, N>, E>
    where
        N: ArrayLength<u8>,
    {
        let mut buffer: GenericArray<u8, N> = unsafe { MaybeUninit::<GenericArray<u8, N>>::uninit().assume_init() };

        {
            let buffer: &mut [u8] = &mut buffer;
            const I2C_AUTO_INCREMENT: u8 = 0;
            self.com
                .write_read(self.address, &[(reg as u8) | I2C_AUTO_INCREMENT], buffer)?;
        }

        Ok(buffer)
    }

    fn write_byte(&mut self, reg: Register, byte: u8) -> Result<(), E> {
        let mut buffer = [0];
        self.com.write_read(self.address, &[reg as u8, byte], &mut buffer)
    }

    fn read_byte(&mut self, reg: Register) -> Result<u8, E> {
        let mut data: [u8; 1] = [0];
        self.com.write_read(self.address, &[reg as u8], &mut data)?;
        Ok(data[0])
    }

    fn read_16bit(&mut self, reg: Register) -> Result<u16, E> {
        let buffer: GenericArray<u8, U2> = self.read_registers(reg)?;
        Ok((u16(buffer[0]) << 8) + u16(buffer[1]))
    }

    fn write_register(&mut self, reg: Register, byte: u8) -> Result<(), E> {
        let mut buffer = [0];
        self.com.write_read(self.address, &[reg as u8, byte], &mut buffer)
    }

    fn write_16bit(&mut self, reg: Register, word: u16) -> Result<(), E> {
        let mut buffer = [0];
        let msb = (word >> 8) as u8;
        self.com
            .write_read(self.address, &[reg as u8, word as u8, msb], &mut buffer)
    }

    /// Updates a register associated with a pin (whether port A/B) reads its value,
    /// updates the particular bit, and writes its value
    fn update_register_bit(&mut self, pin: u8, pin_value: u8, port_a_reg: Register, port_b_reg: Register) -> Result<(), E>  {  
        let reg = register_for_pin(pin, port_a_reg, port_b_reg);
        let bit = bit_for_pin(pin);
        let reg_value = self.read_register(reg)?;
        let reg_value_mod = write_bit(reg_value, bit, pin_value);
        self.write_register(reg, reg_value_mod)
    }

    /// Sets the single pin mode to either Mode::INPUT or Mode::OUTPUT
    pub fn pin_mode(&mut self, pin: u8, pin_mode: PinMode) -> Result<(), E>  {
        self.update_register_bit(pin, pm(pin_mode), Register::IODIRA, Register::IODIRB)
    }

    /// Sets pin mode to either Mode::INPUT or Mode::OUTPUT
    pub fn all_pin_mode(&mut self, pin_mode: PinMode) -> Result<(), E>  {
        self.write_register(Register::IODIRA, pm(pin_mode))?;
	    self.write_register(Register::IODIRB, pm(pin_mode))
    }

    /// Reads all 16 pins (port A and B) into a single 16 bits variable
    pub fn read_gpioab(&mut self) -> Result<u16, E>  {
        let buffer: GenericArray<u8, U2> = self.read_registers(Register::GPIOA)?;
        Ok((u16(buffer[0]) << 8) + u16(buffer[1]))
    }

    /// Reads a single port, A or B, and return its current 8 bit value
    pub fn read_gpio(&mut self, port: Port) -> Result<u8, E>  {
        let reg = match port {
            Port::GPIOA => Register::GPIOA,
            _ => Register::GPIOB,
        };
        self.read_byte(reg)
    }

    /// Writes all the pins with the value at the same time
    pub fn write_gpioab(&mut self, value: u16) -> Result<(), E>  {
        self.write_16bit(Register::GPIOA, value)
    }

    /// Writes all the pins with the value at the same time for GPIOA
    pub fn write_gpioa(&mut self, value: u8) -> Result<(), E>  {
        self.write_byte(Register::GPIOA, value)
    }
    
    /// Writes all the pins with the value at the same time for GPIOB
    pub fn write_gpiob(&mut self, value: u8) -> Result<(), E>  {
        self.write_byte(Register::GPIOB, value)
    }

    /// Writes digital value
    pub fn digital_write(&mut self, pin: u8, value: u8) -> Result<(), E>  {
        let bit = bit_for_pin(pin);
        // read the current GPIO output latches
        let reg_ol = register_for_pin(pin, Register::OLATA, Register::OLATB);
        let gpio = self.read_register(reg_ol)?;

        // set the pin and direction
	    let gpio_mod = write_bit(gpio, bit, value);

        // write the new GPIO
	    let reg_gp = register_for_pin(pin, Register::GPIOA, Register::GPIOB);
        self.write_register(reg_gp, gpio_mod)
    }

    /// Reads digital pin
    pub fn digital_read(&mut self, pin: u8) -> Result<u8, E>  {
        let bit = bit_for_pin(pin);
        let reg = register_for_pin(pin, Register::GPIOA, Register::GPIOB);
        let value = self.read_register(reg)?;
        Ok((value >> bit) & 0x1)
    }

    /// The GPPU register controls the pull-up resistors for the port pins.
    /// If a bit is set and the corresponding pin is configured as an input,
    /// the corresponding port pin is internally pulled up with a 100 kohm resistor
    pub fn pull_up(&mut self, pin: u8, value: u8) -> Result<(), E>  {
        self.update_register_bit(pin, value, Register::GPPUA, Register::GPPUB)
    }

    /// Configures the interrupt system. both port A and B are assigned the same configuration.
    /// mirroring will OR both INTA and INTB pins.
    /// open_drain will set the INT pin to value or open drain.
    /// polarity will set LOW or HIGH on interrupt.
    /// Default values after Power On Reset are: (false, false, LOW)
    pub fn setup_interrupts(&mut self, mirroring: u8, open_drain: u8, polarity: Polarity) -> Result<(), E>  {
        // configure the port A
        self.setup_interrupt_port(Register::IOCONA, mirroring, open_drain, p(polarity))?;
	    
        // configure the port B
        self.setup_interrupt_port(Register::IOCONB, mirroring, open_drain, p(polarity))
    }

    fn setup_interrupt_port(&mut self, register: Register, mirroring: u8, open_drain: u8, polarity: u8) -> Result<(), E>  {
        let mut io_conf_value = self.read_register(register)?;
        io_conf_value = write_bit(io_conf_value, 6, mirroring);
        io_conf_value = write_bit(io_conf_value, 2, open_drain);
        io_conf_value = write_bit(io_conf_value, 1, polarity);
        self.write_register(register, io_conf_value)
    }

    /// Sets up a pin for interrupt
    /// Note that the interrupt condition finishes when you read the information about
    /// the port / value that caused the interrupt or you read the port itself
    pub fn setup_interrupt_pin(&mut self, pin: u8, int_mode: InterruptMode) -> Result<(), E>  {
        // set the pin interrupt control (0 means change, 1 means compare against given value)
        self.update_register_bit(pin, (im(int_mode) != im(InterruptMode::CHANGE)) as u8, Register::INTCONA, Register::INTCONB)?;

        // in a RISING interrupt the default value is 0, interrupt is triggered when the pin goes to 1
	    // in a FALLING interrupt the default value is 1, interrupt is triggered when pin goes to 0
        self.update_register_bit(pin, (im(int_mode) == im(InterruptMode::FALLING)) as u8, Register::DEFVALA, Register::DEFVALB)?;

        // enable the pin for interrupt
        self.update_register_bit(pin, HIGH, Register::GPINTENA, Register::GPINTENB)
    }

    /// Get last interrupt pin
    pub fn get_last_interrupt_pin(&mut self) -> Result<u8, Error<E>>  {
        // try port A
        let intf_a = self.read_register(Register::INTFA)?;
        for x in 0..8 {
            if read_bit(intf_a, x) > 0 {
                return Ok(x)
            }
        }

        // try port B
        let intf_b = self.read_register(Register::INTFB)?;
        for x in 0..8 {
            if read_bit(intf_b, x) > 0 {
                return Ok(x+8)
            }

        }

        Err(Error::InterruptPinError)
    }

    /// Gets last interrupt value
    pub fn get_last_interrupt_value(&mut self) -> Result<u8, Error<E>>  {
        match self.get_last_interrupt_pin() {
            Ok(pin) => {
                let int_reg = register_for_pin(pin, Register::INTCAPA, Register::INTCAPB);
                let bit = bit_for_pin(pin);
                let val = self.read_register(int_reg)?;
                Ok((val >> bit) & 0x01)
            }
            Err(e) => Err(e),
        } 
    }
}

fn write_bit(reg: u8, bit: u8, val: u8) -> u8 {
    let mut res = reg;
    if val > 1 {
      res |= 1 << bit;
    }
    else {
      res &= !(1 << bit);
    }

    res
}

fn read_bit(reg: u8, bit: u8) -> u8 {
    // (reg & (1 << bit)) ? 1 : 0
    reg & (1 << bit)
}

/// Returns bit number associated to a give pin
fn bit_for_pin(pin: u8) -> u8 {
	pin % 8
}

/// Returns register address, port dependent, for a given pin
fn register_for_pin(pin: u8, port_a_addr: Register, port_b_addr: Register) -> Register {
    if pin < 8 {
        port_a_addr
    } else {
        port_b_addr
    }
}

/// Pin modes
#[derive(Debug, Copy, Clone)]
pub enum PinMode {
    /// Represent input mode
    INPUT = 0,
    /// Represent output mode
    OUTPUT = 1,
}

fn pm(pin_mode: PinMode) -> u8 {
    pin_mode as u8
}

/// Interrupt modes
#[derive(Debug, Copy, Clone)]
pub enum InterruptMode {
    /// Represent change mode
    CHANGE = 0,
    /// Represent falling mode
    FALLING = 1,
    /// Represent rising mode
    RISING = 2,
}

fn im(interrupt_mode: InterruptMode) -> u8 {
    interrupt_mode as u8
}

/// Polarity modes
#[derive(Debug, Copy, Clone)]
pub enum Polarity {
    /// Represent low mode
    LOW = 0,
    /// Represent high mode
    HIGH = 1,
}

fn p(polarity: Polarity) -> u8 {
    polarity as u8
}

/// Generic port definition
#[derive(Debug, Copy, Clone)]
pub enum Port {
    /// Represent GPIOA port generic definition
    GPIOA,
    /// Represent GPIOB port generic definition
    GPIOB,
}

#[derive(Debug, Copy, Clone)]
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
}
