// SPDX-FileCopyrightText: 2021-2022 Joonas Javanainen <joonas.javanainen@gmail.com>
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use alloc::vec::Vec;

pub trait ByteSource {
    type Error;
    fn read_byte(&mut self) -> Result<u8, Self::Error>;
}

#[derive(Copy, Clone, Debug)]
pub struct Frame {
    cmd: u8,
    state: u8,
    len: u8,
}

#[derive(Copy, Clone, Debug)]
pub struct Decoder<S: ByteSource> {
    source: S,
}

impl<S: ByteSource> Decoder<S> {
    pub fn new(source: S) -> Decoder<S> {
        Decoder { source }
    }
    fn read_byte(&mut self) -> Result<u8, S::Error> {
        let byte = self.source.read_byte()?;
        match byte {
            // Reference: SFA30 Data Sheet - Table 9: Reference table for byte-stuffing
            0x7d => match self.source.read_byte()? {
                0x5e => Ok(0x7e),
                0x5d => Ok(0x7d),
                0x31 => Ok(0x11),
                0x33 => Ok(0x13),
                _ => panic!("Argh"),
            },
            _ => Ok(byte),
        }
    }
    fn read_frame_byte(&mut self, checksum: &mut u8) -> Result<u8, S::Error> {
        let byte = self.read_byte()?;
        *checksum = checksum.wrapping_add(byte);
        Ok(byte)
    }
    pub fn decode_to(&mut self, buffer: &mut Vec<u8>) -> Result<Frame, S::Error> {
        let start = self.read_byte()?;
        assert_eq!(start, 0x7e);
        let mut checksum = 0;
        let adr = self.read_frame_byte(&mut checksum)?;
        let cmd = self.read_frame_byte(&mut checksum)?;
        let state = self.read_frame_byte(&mut checksum)?;
        let len = self.read_frame_byte(&mut checksum)?;
        for _ in 0..len {
            buffer.push(self.read_frame_byte(&mut checksum)?);
        }
        let chk = self.read_byte()?;
        let stop = self.read_byte()?;
        checksum = !checksum;
        assert_eq!(stop, 0x7e);
        assert_eq!(chk, checksum);
        Ok(Frame { cmd, state, len })
    }
}
