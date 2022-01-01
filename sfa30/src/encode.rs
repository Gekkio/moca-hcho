// SPDX-FileCopyrightText: 2021-2022 Joonas Javanainen <joonas.javanainen@gmail.com>
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use core::fmt;
#[cfg(feature = "std")]
use std::io::{self, Write};

#[derive(Copy, Clone, Debug)]
pub enum EncodeError {
    DataLengthOverflow,
}

impl fmt::Display for EncodeError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            EncodeError::DataLengthOverflow => write!(f, "data length overflow"),
        }
    }
}

pub trait ByteSink {
    type Error: From<EncodeError>;
    fn write_byte(&mut self, byte: u8) -> Result<(), Self::Error> {
        self.write_bytes(&[byte])
    }
    fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Self::Error>;
}

#[cfg(feature = "std")]
impl std::error::Error for EncodeError {}

#[cfg(feature = "std")]
impl From<EncodeError> for io::Error {
    fn from(err: EncodeError) -> Self {
        match err {
            EncodeError::DataLengthOverflow => io::Error::new(io::ErrorKind::Other, err),
        }
    }
}

#[cfg(feature = "std")]
impl<T> ByteSink for T
where
    T: Write,
{
    type Error = io::Error;

    fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_all(bytes)?;
        Ok(())
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Encoder<S: ByteSink> {
    sink: S,
}

impl<S: ByteSink> Encoder<S> {
    pub fn new(sink: S) -> Encoder<S> {
        Encoder { sink }
    }
    fn write_byte(&mut self, byte: u8) -> Result<(), S::Error> {
        match byte {
            // Reference: SFA30 Data Sheet - Table 9: Reference table for byte-stuffing
            0x7e => self.sink.write_bytes(&[0x7d, 0x5e]),
            0x7d => self.sink.write_bytes(&[0x7d, 0x5d]),
            0x11 => self.sink.write_bytes(&[0x7d, 0x31]),
            0x13 => self.sink.write_bytes(&[0x7d, 0x33]),
            _ => self.sink.write_byte(byte),
        }
    }
    fn write_frame_byte(&mut self, checksum: &mut u8, byte: u8) -> Result<(), S::Error> {
        self.write_byte(byte)?;
        *checksum = checksum.wrapping_add(byte);
        Ok(())
    }
    pub fn encode_command(&mut self, cmd: u8, data: &[u8]) -> Result<(), S::Error> {
        let len: u8 = data
            .len()
            .try_into()
            .map_err(|_| EncodeError::DataLengthOverflow)?;
        // Reference: SFA30 Data Sheet - 4.2 SHDLC Frame Layer
        const START: u8 = 0x7e;
        const ADR: u8 = 0x00;
        const STOP: u8 = 0x7e;
        self.sink.write_byte(START)?;
        let mut checksum = 0x00;
        self.write_frame_byte(&mut checksum, ADR)?;
        self.write_frame_byte(&mut checksum, cmd)?;
        self.write_frame_byte(&mut checksum, len)?;
        for &byte in data {
            self.write_frame_byte(&mut checksum, byte)?;
        }
        checksum = !checksum;
        self.sink.write_byte(checksum)?;
        self.sink.write_byte(STOP)
    }
    pub fn finish(self) -> S {
        self.sink
    }
}

#[test]
fn test_encode() {
    let mut buffer = Vec::new();
    let mut encoder = Encoder::new(&mut buffer);
    encoder.encode_command(0x00, &[0x01, 0x01]).unwrap();
    assert_eq!(
        &buffer[..],
        &[0x7e, 0x00, 0x00, 0x02, 0x01, 0x01, 0xfb, 0x7e]
    );
}
