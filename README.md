# Trinamic TMC5160 Rust Driver

[![crates.io](https://img.shields.io/crates/v/tmc5160.svg)](https://crates.io/crates/tmc5160)
[![Docs](https://docs.rs/tmc5160/badge.svg)](https://docs.rs/tmc5160)
[![Rust](https://github.com/hacknus/tmc5160-rs/actions/workflows/rust.yml/badge.svg)](https://github.com/hacknus/tmc5160-rs/actions/workflows/rust.yml)

This is a rust driver for the [TMC5160](https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC5160A_datasheet_rev1.18.pdf) Trinamic integrated stepper motor controller.

It is designed to work with SPI peripherals described in [linux-embedded-hal](https://crates.io/crates/linux-embedded-hal).

This is a fork from [hacknus/tmc5160-rs](https://github.com/hacknus/tmc5160-rs). It has been modified to use `std::io::{Read, Write}`, instead of the embedded-hal equivalent.

## Example
An example can be found in `examples/startup_check.rs`.  
To implement this driver, consult the example:  
Put this into your `cargo.toml`:
```toml
[dependencies]
tmc5160 = { git = "https://github.com/hacknus/tmc5160-rs" }
# required for the register configs to_u32_le() function
modular-bitfield-to-value = {git = "https://github.com/hacknus/modular-bitfield-to-value"}
```
Add the following imports:
```rust
use tmc5160::registers::*;
use tmc5160::{DataPacket, Tmc5160};

// required for the to_u32_le() function.
use modular_bitfield_to_value::ToValue;
```

Configure the SPI bus in the `main()` function as follows:
```rust
let mut spi = linux_embedded_hal::spidev::Spidev::open(spi)?;
spi.configure(&spidev::SpidevOptions {
    bits_per_word: Some(8),
    max_speed_hz: Some(4_000_000),
    lsb_first: Some(false),
    spi_mode: Some(spidev::SpiModeFlags::SPI_MODE_0),
})?;
```
and to use the driver, implement the driver as shown below:
```rust
// set up spi ...

// set up stepper driver
let mut stepper_driver = Tmc5160::new(spi);

// clear G_STAT register
match stepper_driver.clear_g_stat(){
    Ok(packet) => {
        sprintln!(in_out, "SPI status has been updated: {}", packet.status.to_u32_le().unwrap_or(0));
    }
    Err(error) => {
        sprintln!(in_out, "Error clearing GSTAT is {:?}", error);
    }
}

// read OFFSET
match stepper_driver.read_offset() {
    Ok(offset) => {
        sprintln!(in_out, "Stepper driver offset is {}", offset);
    }
    Err(error) => {
        sprintln!(in_out, "Error for reading offset is {:?}", error);
    }
}

// set G_CONF register
stepper_driver
    .g_conf
    .set_recalibrate(true)
    .set_faststandstill(true)
    .set_en_pwm_mode(true);
match stepper_driver.update_g_conf(){
    Ok(packet) => {
        sprintln!(in_out, "SPI status has been updated: {}", packet.status.to_u32_le().unwrap_or(0));
    }
    Err(error) => {
        sprintln!(in_out, "Error for updating GCONF is {:?}", error);
    }
}

match stepper_driver.read_drv_status() {
    Ok(status) => {
        // either use fields of the register
        sprintln!(in_out, "Stepper driver is in standstill: {}", status.standstill());
        // or extract the u32 value from the register
        sprintln!(in_out, "Stepper driver DRV_STATUS register is {}", status.to_u32_le().unwrap_or(0));
        sprintln!(in_out, "SPI status has been updated: {}", stepper_driver.status.to_u32_le().unwrap_or(0));
    }
    Err(error) => {
        sprintln!(in_out, "Error for reading DRV_STATUS is {:?}", error);
    }
}

match stepper_driver.read_gstat() {
    Ok(status) => {
        sprintln!(in_out, "Stepper GSTAT register is {}", status.to_u32_le().unwrap_or(0));
        sprintln!(in_out, "SPI status has been updated: {}", stepper_driver.status.to_u32_le().unwrap_or(0));
    }
    Err(error) => {
        sprintln!(in_out, "Error for reading GSTAT is {:?}", error);
    }
}
```