//! # zcssr
//! 
//! Firmware for Zero Crossing Solid State Relay controller board

#![no_std]

// Required for helper_macros::str_to_slice
#![feature(proc_macro_hygiene)]

#![cfg_attr(feature = "bench", feature(test))]

#![deny(warnings, missing_docs)]

/// Constants and utility functions related to the hardware board
pub mod board;

/// Provides a simple UI based on a 4 digit alphanumeric display and a rotary encoder
pub mod ui;