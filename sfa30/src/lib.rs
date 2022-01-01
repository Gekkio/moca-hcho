// SPDX-FileCopyrightText: 2021-2022 Joonas Javanainen <joonas.javanainen@gmail.com>
//
// SPDX-License-Identifier: MIT OR Apache-2.0

#![cfg_attr(not(feature = "std"), no_std)]
#![cfg(not(feature = "std"))]
extern crate alloc;

pub use decode::*;
pub use encode::*;

mod decode;
mod encode;
