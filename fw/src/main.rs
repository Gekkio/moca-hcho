// SPDX-FileCopyrightText: 2021-2022 Joonas Javanainen <joonas.javanainen@gmail.com>
//
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![feature(alloc_error_handler)]

extern crate alloc;

use alloc::{string::String, vec::Vec};
use alloc_cortex_m::CortexMHeap;
use core::{
    alloc::Layout,
    borrow::Borrow,
    convert::Infallible,
    sync::atomic::{AtomicUsize, Ordering},
};
use defmt::{debug, error, info, unwrap, Format};
use embassy::{
    executor::{Executor, Spawner},
    time::{Duration, Timer},
    util::Forever,
};
use embassy_nrf::{
    self as _,
    config::HfclkSource,
    gpio::{Level, NoPin, Output, OutputDrive},
    interrupt::{self, Priority},
    peripherals::{P0_02, UARTE0},
    uarte::{self, Uarte},
    Peripherals,
};
use embassy_traits::uart::{Read, Write};
use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin};
use futures::{join, pin_mut};
use moca_hcho::advertising::AdvertisingPayload;
use nrf_softdevice::{
    ble::{
        peripheral::{self, AdvertiseError},
        TxPower,
    },
    raw, Softdevice,
};
use nrf_softdevice_defmt_rtt as _;
use panic_probe as _;

const VENDOR_ID: u16 = 0xffff;
const LED_BLINKER_DURATION: Duration = Duration::from_secs(60 * 5);

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();
const ALLOCATOR_SIZE: usize = 4096;

// define what happens in an Out Of Memory (OOM) condition
#[alloc_error_handler]
fn alloc_error(layout: Layout) -> ! {
    panic!("Failed to allocate {} bytes", layout.size());
}

defmt::timestamp! {
    "{=u64}", {
        static COUNT: AtomicUsize = AtomicUsize::new(0);
        // NOTE(no-CAS) `timestamps` runs with interrupts disabled
        let n = COUNT.load(Ordering::Relaxed);
        COUNT.store(n + 1, Ordering::Relaxed);
        n as u64
    }
}

static EXECUTOR: Forever<Executor> = Forever::new();

#[embassy::task]
async fn softdevice_task(sd: &'static Softdevice) {
    debug!("Starting softdevice");
    sd.run().await;
}

type LedPin = P0_02;

struct Sfa30Buffer {
    data: [u8; 0x100],
    len: usize,
}

impl Default for Sfa30Buffer {
    fn default() -> Self {
        Sfa30Buffer {
            data: [0; 0x100],
            len: 0,
        }
    }
}

impl sfa30::ByteSink for Sfa30Buffer {
    type Error = sfa30::EncodeError;

    fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Self::Error> {
        self.data[self.len..(self.len + bytes.len())].copy_from_slice(bytes);
        self.len += bytes.len();
        Ok(())
    }
}

impl sfa30::ByteSource for Sfa30Buffer {
    type Error = Infallible;

    fn read_byte(&mut self) -> Result<u8, Self::Error> {
        let byte = self.data[self.len];
        self.len += 1;
        Ok(byte)
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq, Format)]
struct Sfa30Measurement {
    hcho_concentration: i16,
    relative_humidity: i16,
    temperature: i16,
}

#[derive(Default)]
struct Sfa30 {
    read_buf: Vec<u8>,
}

impl Sfa30 {
    async fn write_command(
        &mut self,
        uart: &mut Uarte<'_, UARTE0>,
        cmd: u8,
        data: &[u8],
    ) -> Result<(), anyhow::Error> {
        let mut encoder = sfa30::Encoder::new(Sfa30Buffer::default());
        encoder
            .encode_command(cmd, data)
            .map_err(anyhow::Error::msg)?;
        let buf = encoder.finish();
        debug!("SFA W: {:02x}", &buf.data[..buf.len]);

        uart.write(&buf.data[..buf.len])
            .await
            .map_err(|_| anyhow::Error::msg("UART write error"))?;
        Ok(())
    }
    async fn read_response(
        &mut self,
        uart: &mut Uarte<'_, UARTE0>,
    ) -> Result<Vec<u8>, anyhow::Error> {
        self.read_buf.clear();
        self.read_buf.push(0x00);
        uart.read(&mut self.read_buf)
            .await
            .map_err(|_| anyhow::Error::msg("UART read error"))?;

        loop {
            let mut byte = [0x00];
            uart.read(&mut byte)
                .await
                .map_err(|_| anyhow::Error::msg("UART read error"))?;
            self.read_buf.push(byte[0]);
            if byte[0] == 0x7e {
                break;
            }
        }

        debug!("SFA R: {:02x}", &self.read_buf[..]);
        let mut buf = Sfa30Buffer::default();
        buf.data[..self.read_buf.len()].copy_from_slice(&self.read_buf);

        let mut decoder = sfa30::Decoder::new(buf);
        let mut output = Vec::new();
        decoder.decode_to(&mut output).map_err(anyhow::Error::msg)?;
        Ok(output)
    }
    pub async fn reset(&mut self, uart: &mut Uarte<'_, UARTE0>) -> Result<(), anyhow::Error> {
        self.write_command(uart, 0xd3, &[]).await?;
        let _ = self.read_response(uart).await?;
        Ok(())
    }
    pub async fn read_device_marking(
        &mut self,
        uart: &mut Uarte<'_, UARTE0>,
    ) -> Result<String, anyhow::Error> {
        self.write_command(uart, 0xd0, &[0x06]).await?;
        let mut output = self.read_response(uart).await?;
        let _ = output.pop();
        Ok(String::from_utf8(output).map_err(anyhow::Error::msg)?)
    }
    pub async fn start_measurement(
        &mut self,
        uart: &mut Uarte<'_, UARTE0>,
    ) -> Result<(), anyhow::Error> {
        self.write_command(uart, 0x00, &[0x00]).await?;
        self.read_response(uart).await?;
        Ok(())
    }
    pub async fn read_measurement(
        &mut self,
        uart: &mut Uarte<'_, UARTE0>,
    ) -> Result<Sfa30Measurement, anyhow::Error> {
        self.write_command(uart, 0x03, &[0x02]).await?;
        let data = self.read_response(uart).await?;
        Ok(Sfa30Measurement {
            hcho_concentration: i16::from_be_bytes([data[0], data[1]]),
            relative_humidity: i16::from_be_bytes([data[2], data[3]]),
            temperature: i16::from_be_bytes([data[4], data[5]]),
        })
    }
}

#[embassy::task]
async fn async_main(_spawner: Spawner, sd: &'static Softdevice, p: Peripherals) {
    let mut led: Output<LedPin> = Output::new(p.P0_02, Level::Low, OutputDrive::Standard);

    led.set_low().unwrap();

    let mut config = uarte::Config::default();
    config.parity = uarte::Parity::EXCLUDED;
    config.baudrate = uarte::Baudrate::BAUD115200;

    let irq = interrupt::take!(UARTE0_UART0);
    let mut uart = uarte::Uarte::new(p.UARTE0, irq, p.P0_27, p.P0_26, NoPin, NoPin, config);

    info!("uarte initialized!");

    Timer::after(Duration::from_secs(1)).await;

    let mut sfa = Sfa30::default();
    sfa.reset(&mut uart).await.unwrap();
    Timer::after(Duration::from_secs(1)).await;

    info!(
        "SFA30 marking: {}",
        sfa.read_device_marking(&mut uart).await.unwrap()
    );

    let led_timeout = Timer::after(LED_BLINKER_DURATION);
    let led_blinker = async move {
        let _ = led_blinker(led).await;
    };

    pin_mut!(led_timeout);
    pin_mut!(led_blinker);

    let measurement = measurement(sd, uart, sfa);
    let led = futures::future::select(led_timeout, led_blinker);

    join!(led, measurement);
}

async fn led_blinker(mut led: Output<'_, LedPin>) -> Result<(), Infallible> {
    led.set_high()?;
    loop {
        if led.is_set_high()? {
            led.set_low()?;
        } else {
            led.set_high()?;
        }
        Timer::after(Duration::from_millis(1000)).await;
    }
}

async fn measurement(sd: &'static Softdevice, mut uart: Uarte<'_, UARTE0>, mut sfa: Sfa30) {
    sfa.start_measurement(&mut uart).await.unwrap();

    loop {
        let tx_power = TxPower::ZerodBm;
        Timer::after(Duration::from_millis(10_000)).await;

        let meas = sfa.read_measurement(&mut uart).await.unwrap();
        debug!("Raw measurement: {:?}", meas);

        let mut packet = AdvertisingPayload::default();

        unwrap!(packet.push_ad_flags(raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8));
        unwrap!(packet.push_complete_name("moca-hcho"));
        unwrap!(packet.push_tx_power(tx_power));

        let mut custom = [0x00; 0x06];
        custom[0..2].copy_from_slice(&meas.hcho_concentration.to_be_bytes());
        custom[2..4].copy_from_slice(&meas.relative_humidity.to_be_bytes());
        custom[4..6].copy_from_slice(&meas.temperature.to_be_bytes());

        unwrap!(packet.push_manufacturer_data(VENDOR_ID, &custom));

        let config = peripheral::Config {
            max_events: Some(1),
            tx_power,
            ..Default::default()
        };
        let data: &[u8] = packet.borrow();
        let adv = peripheral::NonconnectableAdvertisement::NonscannableUndirected {
            adv_data: packet.borrow(),
        };
        match peripheral::advertise(sd, adv, &config).await {
            Ok(_) | Err(AdvertiseError::Timeout) => (),
            Err(err) => error!("{:?}", err),
        }

        debug!("BLE advertisement done");
    }
}

fn init_hal() -> Peripherals {
    let mut config = embassy_nrf::config::Config::default();
    config.hfclk_source = HfclkSource::ExternalXtal;
    config.gpiote_interrupt_priority = Priority::P7;
    config.time_interrupt_priority = Priority::P2;
    embassy_nrf::init(config)
}

fn enable_softdevice() -> &'static Softdevice {
    const CLOCK_SOURCE: u8 = raw::NRF_CLOCK_LF_SRC_RC as u8;
    const CALIBRATION_INTERVAL_4S: u8 = 16; // unit = 1/4 of seconds
    const TEMP_CALIBRATION_INTERVAL: u8 = 2; // unit = calibration intervals
    const CLOCK_ACCURACY: u8 = raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8;

    let config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: CLOCK_SOURCE,
            rc_ctiv: CALIBRATION_INTERVAL_4S,
            rc_temp_ctiv: TEMP_CALIBRATION_INTERVAL,
            accuracy: CLOCK_ACCURACY,
        }),
        ..Default::default()
    };

    Softdevice::enable(&config)
}

#[cortex_m_rt::entry]
fn main() -> ! {
    info!("Initializing global allocator");
    let start = cortex_m_rt::heap_start() as usize;
    unsafe { ALLOCATOR.init(start, ALLOCATOR_SIZE) }

    info!("Initializing HAL");
    let p = init_hal();

    info!("Enabling softdevice");
    let sd = enable_softdevice();

    let executor = EXECUTOR.put(Executor::new());
    info!("Starting system executor");
    executor.run(|spawner| {
        unwrap!(spawner.spawn(softdevice_task(sd)));
        unwrap!(spawner.spawn(async_main(spawner, sd, p)));
    });
}
