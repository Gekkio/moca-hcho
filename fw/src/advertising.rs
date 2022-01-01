// SPDX-FileCopyrightText: 2021-2022 Joonas Javanainen <joonas.javanainen@gmail.com>
//
// SPDX-License-Identifier: MIT OR Apache-2.0

use core::{borrow::Borrow, fmt};

use nrf_softdevice::ble::TxPower;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct AdvertisingPayloadOverflow;

impl defmt::Format for AdvertisingPayloadOverflow {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "advertising payload overflow");
    }
}

impl fmt::Display for AdvertisingPayloadOverflow {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "advertising payload overflow")
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum AdvertisingDataKind {
    Flags = 0x01,
    ShortenedName = 0x08,
    CompleteName = 0x09,
    TxPower = 0x0a,
    ManufacturerSpecific = 0xff,
}

#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct AdvertisingPayload {
    buffer: [u8; 31],
    length: u8,
}

impl AdvertisingPayload {
    pub fn push_ad_flags(&mut self, flags: u8) -> Result<(), AdvertisingPayloadOverflow> {
        self.begin(AdvertisingDataKind::Flags, 1)?;
        self.push_bytes(&[flags])
    }
    pub fn push_shortened_name(&mut self, name: &str) -> Result<(), AdvertisingPayloadOverflow> {
        self.begin(AdvertisingDataKind::ShortenedName, name.as_bytes().len())?;
        self.push_bytes(name.as_bytes())
    }
    pub fn push_complete_name(&mut self, name: &str) -> Result<(), AdvertisingPayloadOverflow> {
        self.begin(AdvertisingDataKind::CompleteName, name.as_bytes().len())?;
        self.push_bytes(name.as_bytes())
    }
    pub fn push_manufacturer_data(
        &mut self,
        vendor_id: u16,
        data: &[u8],
    ) -> Result<(), AdvertisingPayloadOverflow> {
        let size = data
            .len()
            .checked_add(2)
            .ok_or(AdvertisingPayloadOverflow)?;
        self.begin(AdvertisingDataKind::ManufacturerSpecific, size)?;
        self.push_bytes(&vendor_id.to_be_bytes())?;
        self.push_bytes(data)
    }
    pub fn push_tx_power(&mut self, tx_power: TxPower) -> Result<(), AdvertisingPayloadOverflow> {
        self.begin(AdvertisingDataKind::TxPower, 1)?;
        self.push_bytes(&[tx_power as i8 as u8])
    }
    #[inline]
    fn begin(
        &mut self,
        kind: AdvertisingDataKind,
        size: usize,
    ) -> Result<(), AdvertisingPayloadOverflow> {
        let header = [
            size.checked_add(1).ok_or(AdvertisingPayloadOverflow)? as u8,
            kind as u8,
        ];
        self.push_bytes(&header)
    }
    #[inline]
    fn push_bytes(&mut self, bytes: &[u8]) -> Result<(), AdvertisingPayloadOverflow> {
        let start = self.length as usize;
        let end = start
            .checked_add(bytes.len())
            .ok_or(AdvertisingPayloadOverflow)?;
        if end > self.buffer.len() {
            Err(AdvertisingPayloadOverflow)
        } else {
            self.buffer[start..end].copy_from_slice(bytes);
            self.length = end as u8;
            Ok(())
        }
    }
}

impl Borrow<[u8]> for AdvertisingPayload {
    fn borrow(&self) -> &[u8] {
        &self.buffer[..self.length as usize]
    }
}
