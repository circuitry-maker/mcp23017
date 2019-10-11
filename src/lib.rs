#![deny(missing_docs)]
#![deny(warnings)]
#![allow(dead_code)]
#![no_std]

//! Constructs a new
//!
//! 
//!

extern crate cast;
extern crate embedded_hal as ehal;
extern crate generic_array;
extern crate nb;

const ADDRESS: u8 = 0x20;

/// dummy
pub struct MCP23017<I2C: ehal::blocking::i2c::WriteRead> {
    com: I2C,
    /// dummy
    pub revision_id: u8,
    io_mode2v8: bool,
    stop_variable: u8,
    measurement_timing_budget_microseconds: u32,
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