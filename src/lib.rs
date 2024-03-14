#![no_std]

//! Manages an MCP23017, a 16-Bit I2C I/O Expander with Serial Interface module.
//!
//! This operates the chip in `IOCON.BANK=0` mode, i.e. the registers are mapped sequentially.
//! This driver does not set `IOCON.BANK`, but the factory default is `0` and this driver does
//! not change that value.
//!
//! See [the datasheet](http://ww1.microchip.com/downloads/en/DeviceDoc/20001952C.pdf) for more
//! information on the device.

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
#![allow(dead_code, non_camel_case_types)]
#![allow(clippy::uninit_assumed_init, clippy::upper_case_acronyms)]

extern crate embedded_hal as ehal;

use ehal::blocking::i2c::{Write, WriteRead};

/// The default I2C address of the MCP23017.
const DEFAULT_ADDRESS: u8 = 0x20;

/// Binary constants.
const HIGH: bool = true;
const LOW: bool = false;

/// Struct for an MCP23017.
/// See the crate-level documentation for general info on the device and the operation of this
/// driver.
#[derive(Clone, Copy, Debug)]
pub struct MCP23017<I2C: Write + WriteRead> {
    com: I2C,
    /// The I2C slave address of this device.
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
    /// Creates an expander with the default configuration.
    pub fn default(i2c: I2C) -> Result<MCP23017<I2C>, Error<E>>
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        MCP23017::new(i2c, DEFAULT_ADDRESS)
    }

    /// Creates an expander with specific address.
    pub fn new(i2c: I2C, address: u8) -> Result<MCP23017<I2C>, Error<E>>
    where
        I2C: Write<Error = E> + WriteRead<Error = E>,
    {
        let chip = MCP23017 { com: i2c, address };

        Ok(chip)
    }

    /// Initiates hardware with basic setup.
    pub fn init_hardware(&mut self) -> Result<(), Error<E>> {
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

    fn read_double_register(&mut self, reg: Register) -> Result<[u8; 2], E> {
        let mut buffer: [u8; 2] = [0; 2];
        self.com
            .write_read(self.address, &[reg as u8], &mut buffer)?;
        Ok(buffer)
    }

    fn write_register(&mut self, reg: Register, byte: u8) -> Result<(), E> {
        self.com.write(self.address, &[reg as u8, byte])
    }

    fn write_double_register(&mut self, reg: Register, word: u16) -> Result<(), E> {
        let msb = (word >> 8) as u8;
        self.com.write(self.address, &[reg as u8, word as u8, msb])
    }

    /// Updates a single bit in the register associated with the given pin.
    /// This will read the register (`port_a_reg` for pins 0-7, `port_b_reg` for the other eight),
    /// set the bit (as specified by the pin position within the register), and write the register
    /// back to the device.
    fn update_register_bit(
        &mut self,
        pin: u8,
        pin_value: bool,
        port_a_reg: Register,
        port_b_reg: Register,
    ) -> Result<(), E> {
        let reg = register_for_pin(pin, port_a_reg, port_b_reg);
        let bit = bit_for_pin(pin);
        let reg_value = self.read_register(reg)?;
        let reg_value_mod = write_bit(reg_value, bit, pin_value);
        self.write_register(reg, reg_value_mod)
    }

    /// Sets the mode for a single pin to either `Mode::INPUT` or `Mode::OUTPUT`.
    pub fn pin_mode(&mut self, pin: u8, pin_mode: PinMode) -> Result<(), E> {
        self.update_register_bit(
            pin,
            pin_mode.bit_value(),
            Register::IODIRA,
            Register::IODIRB,
        )
    }

    /// Sets all pins' modes to either `Mode::INPUT` or `Mode::OUTPUT`.
    pub fn all_pin_mode(&mut self, pin_mode: PinMode) -> Result<(), E> {
        self.write_register(Register::IODIRA, pin_mode.register_value())?;
        self.write_register(Register::IODIRB, pin_mode.register_value())
    }

    /// Reads all 16 pins (port A and B) into a single 16 bit variable.
    pub fn read_gpioab(&mut self) -> Result<u16, E> {
        let buffer = self.read_double_register(Register::GPIOA)?;
        Ok((buffer[0] as u16) << 8 | (buffer[1] as u16))
    }

    /// Reads a single port, A or B, and returns its current 8 bit value.
    pub fn read_gpio(&mut self, port: Port) -> Result<u8, E> {
        let reg = match port {
            Port::GPIOA => Register::GPIOA,
            Port::GPIOB => Register::GPIOB,
        };
        self.read_register(reg)
    }

    /// Writes all the pins with the value at the same time.
    pub fn write_gpioab(&mut self, value: u16) -> Result<(), E> {
        self.write_double_register(Register::GPIOA, value)
    }

    /// Writes all the pins of one port with the value at the same time.
    pub fn write_gpio(&mut self, port: Port, value: u8) -> Result<(), E> {
        let reg = match port {
            Port::GPIOA => Register::GPIOA,
            Port::GPIOB => Register::GPIOB,
        };
        self.write_register(reg, value)
    }

    /// Writes a single bit to a single pin.
    /// This function internally reads from the output latch register (`OLATA`/`OLATB`) and writes
    /// to the GPIO register.
    pub fn digital_write(&mut self, pin: u8, value: bool) -> Result<(), E> {
        let bit = bit_for_pin(pin);
        // Read the current GPIO output latches.
        let ol_register = register_for_pin(pin, Register::OLATA, Register::OLATB);
        let gpio = self.read_register(ol_register)?;

        // Set the pin.
        let gpio_mod = write_bit(gpio, bit, value);

        // Write the modified register.
        let reg_gp = register_for_pin(pin, Register::GPIOA, Register::GPIOB);
        self.write_register(reg_gp, gpio_mod)
    }

    /// Reads a single pin.
    pub fn digital_read(&mut self, pin: u8) -> Result<bool, E> {
        let bit = bit_for_pin(pin);
        let reg = register_for_pin(pin, Register::GPIOA, Register::GPIOB);
        let value = self.read_register(reg)?;
        Ok(read_bit(value, bit))
    }

    /// Enables or disables the internal pull-up resistor for a single pin.
    pub fn pull_up(&mut self, pin: u8, value: bool) -> Result<(), E> {
        self.update_register_bit(pin, value, Register::GPPUA, Register::GPPUB)
    }

    /// Inverts the input polarity for a single pin.
    /// This uses the `IPOLA` or `IPOLB` registers, see the datasheet for more information.
    pub fn invert_input_polarity(&mut self, pin: u8, value: bool) -> Result<(), E> {
        self.update_register_bit(pin, value, Register::IPOLA, Register::IPOLB)
    }

    /// Configures the interrupt system. both port A and B are assigned the same configuration.
    /// mirroring will OR both INTA and INTB pins.
    /// open_drain will set the INT pin to value or open drain.
    /// polarity will set LOW or HIGH on interrupt.
    /// Default values after Power On Reset are: (false, false, LOW)
    pub fn setup_interrupts(
        &mut self,
        mirroring: bool,
        open_drain: bool,
        polarity: Polarity,
    ) -> Result<(), E> {
        // configure port A
        self.setup_interrupt_port(Register::IOCONA, mirroring, open_drain, polarity)?;

        // configure port B
        self.setup_interrupt_port(Register::IOCONB, mirroring, open_drain, polarity)
    }

    fn setup_interrupt_port(
        &mut self,
        register: Register,
        mirroring: bool,
        open_drain: bool,
        polarity: Polarity,
    ) -> Result<(), E> {
        let mut io_conf_value = self.read_register(register)?;
        io_conf_value = write_bit(io_conf_value, 6, mirroring);
        io_conf_value = write_bit(io_conf_value, 2, open_drain);
        io_conf_value = write_bit(io_conf_value, 1, polarity.bit_value());
        self.write_register(register, io_conf_value)
    }

    /// Sets up a pin for interrupt.
    /// Note that the interrupt condition finishes when you read the information about
    /// the port / value that caused the interrupt or you read the port itself.
    pub fn setup_interrupt_pin(&mut self, pin: u8, int_mode: InterruptMode) -> Result<(), E> {
        // set the pin interrupt control (0 means change, 1 means compare against given value)
        self.update_register_bit(
            pin,
            int_mode != InterruptMode::CHANGE,
            Register::INTCONA,
            Register::INTCONB,
        )?;

        // in a RISING interrupt the default value is 0, interrupt is triggered when the pin goes to 1
        // in a FALLING interrupt the default value is 1, interrupt is triggered when pin goes to 0
        self.update_register_bit(
            pin,
            int_mode == InterruptMode::FALLING,
            Register::DEFVALA,
            Register::DEFVALB,
        )?;

        // enable the pin for interrupt
        self.update_register_bit(pin, HIGH, Register::GPINTENA, Register::GPINTENB)
    }

    /// Get last interrupt pin
    pub fn get_last_interrupt_pin(&mut self) -> Result<u8, Error<E>> {
        // try port A
        let intf_a = self.read_register(Register::INTFA)?;
        for x in 0..8 {
            if read_bit(intf_a, x) {
                return Ok(x);
            }
        }

        // try port B
        let intf_b = self.read_register(Register::INTFB)?;
        for x in 0..8 {
            if read_bit(intf_b, x) {
                return Ok(x + 8);
            }
        }

        Err(Error::InterruptPinError)
    }

    /// Gets last interrupt value
    pub fn get_last_interrupt_value(&mut self) -> Result<u8, Error<E>> {
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

    /// Get the complete value captured at the last interrupt of the specified port
    pub fn get_captured_value(&mut self, port: Port) -> Result<u8, E> {
        let reg = match port {
            Port::GPIOA => Register::INTCAPA,
            Port::GPIOB => Register::INTCAPB,
        };
        self.read_register(reg)
    }
}

/// Changes the bit at position `bit` within `reg` to `val`.
fn write_bit(reg: u8, bit: u8, val: bool) -> u8 {
    let mut res = reg;
    if val {
        res |= 1 << bit;
    } else {
        res &= !(1 << bit);
    }

    res
}

/// Returns whether the bit at position `bit` within `reg` is set.
fn read_bit(reg: u8, bit: u8) -> bool {
    reg & (1 << bit) != 0
}

/// Returns the bit index associated with a given pin.
fn bit_for_pin(pin: u8) -> u8 {
    pin % 8
}

/// Returns the register address, port dependent, for a given pin.
fn register_for_pin(pin: u8, port_a_addr: Register, port_b_addr: Register) -> Register {
    if pin < 8 {
        port_a_addr
    } else {
        port_b_addr
    }
}

/// Pin modes.
#[derive(Debug, Copy, Clone)]
pub enum PinMode {
    /// Represents input mode.
    INPUT = 1,
    /// Represents output mode.
    OUTPUT = 0,
}

impl PinMode {
    /// Returns the binary value of the `PinMode`, as used in IODIR.
    fn bit_value(&self) -> bool {
        match *self {
            PinMode::INPUT => true,
            PinMode::OUTPUT => false,
        }
    }

    /// Returns a whole register full of the binary value of the `PinMode`.
    fn register_value(&self) -> u8 {
        match *self {
            PinMode::INPUT => 0xff,
            PinMode::OUTPUT => 0x00,
        }
    }
}

/// Interrupt modes.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum InterruptMode {
    /// Represents change mode.
    CHANGE = 0,
    /// Represents falling mode.
    FALLING = 1,
    /// Represents rising mode.
    RISING = 2,
}

/// Polarity modes.
#[derive(Debug, Copy, Clone)]
pub enum Polarity {
    /// Represents active-low mode.
    LOW = 0,
    /// Represents active-high mode.
    HIGH = 1,
}

impl Polarity {
    /// Returns the binary value of the Polarity, as used in IOCON.
    fn bit_value(&self) -> bool {
        match self {
            Polarity::LOW => false,
            Polarity::HIGH => true,
        }
    }
}

/// Generic port definitions.
#[derive(Debug, Copy, Clone)]
pub enum Port {
    /// Represent port A.
    GPIOA,
    /// Represent port B.
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
