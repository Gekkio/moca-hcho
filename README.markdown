<!--
SPDX-FileCopyrightText: 2021-2022 Joonas Javanainen <joonas.javanainen@gmail.com>

SPDX-License-Identifier: CC0-1.0
-->

# moca-hcho: Bluetooth 5 BLE formaldehyde/temperature/humidity sensor beacon

moca-hcho is a simple Bluetooth Low Energy sensor beacon that uses a [Sensirion SFA30 Formaldehyde Sensor Module](https://www.sensirion.com/en/environmental-sensors/formaldehyde-sensor-sfa30/) to sense formaldehyde/temperature/humidity.
This information is measured and sent periodically using Bluetooth BLE advertisements, which can be received by any Bluetooth 4.0+ device within range.

![Wall-mounted moca-hcho with banana for scale](moca-hcho-vs-banana.jpg)

Overview:

* Enclosure: Hammond 1551V4WH vented sensor enclosure (wall-mountable)
* Sensor: Sensirion SFA30 Formaldehyde sensor module
* Microcontroller: Fanstel BT832 Bluetooth module with Nordic nRF52832
* Power supply: external +5V power supply
* Firmware (less than 1 kLOC) is written in [Rust](https://www.rust-lang.org/) using the [Embassy embedded async/await framework](https://github.com/embassy-rs/embassy)
* Data is sent periodically using Bluetooth BLE advertisements that are very simple to send and receive
* **Not FCC/Bluetooth/etc certified/qualified** but a "best effort" -style approach is used by using qualified BT832 module with integrated antenna + qualified Nordic S112 SoftDevice Bluetooth stack.

![moca-hcho v1.0 assembled, without lid](moca-hcho.jpg)

## Potential future improvements

* Tidy up the code...currently the code is very crude and could use a lot of polish and some tests should be written
* Improve environmental coupling of the sensor. The vented enclosure works fine, but SFA30 Design-In Guide suggests even better environmental coupling options
* Add encryption. Unencrypted advertisements can be read or spoofed by anyone
* Re-evaluate the feasibility of using a battery

## License

Hardware is licensed under Creative Commons Attribution 4.0 International.

Firmware is licensed under either of

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
