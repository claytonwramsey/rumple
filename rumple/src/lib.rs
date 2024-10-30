//! `rumple` is a motion planning library implemented in Rust.
//! It contains generic implementations of common motion planning algorithms with a flexible,
//! zero-cost API.

#![cfg_attr(not(feature = "std"), no_std)]
#![warn(clippy::pedantic, clippy::nursery)]
#![warn(clippy::allow_attributes, reason = "prefer expect over allow")]
// #![warn(missing_docs)]

#[macro_use]
extern crate alloc;

pub mod geo;
pub mod kino;
pub mod metric;
pub mod nn;
pub mod sample;
pub mod space;
pub mod time;
pub mod valid;
