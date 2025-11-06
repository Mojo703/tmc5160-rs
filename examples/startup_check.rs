use linux_embedded_hal::spidev;
use modular_bitfield_to_value::ToValue; // required for the to_u32_le() function.
use tmc5160;

fn main() -> std::io::Result<()> {
    let step_count = 256.0;
    let mut spi = spidev::Spidev::open("/dev/spidev0.0")?;
    spi.configure(&spidev::SpidevOptions {
        bits_per_word: Some(8),
        max_speed_hz: Some(500_000),
        lsb_first: Some(false),
        spi_mode: Some(spidev::SpiModeFlags::SPI_MODE_0),
    })?;

    let mut driver = tmc5160::Tmc5160::new(spi).step_count(step_count);

    // clear G_STAT register
    match driver.clear_g_stat() {
        Ok(packet) => {
            println!(
                "SPI status has been updated: {}",
                packet.status.to_u32_le().unwrap_or(0)
            );
        }
        Err(error) => {
            println!("Error clearing GSTAT is {:?}", error);
        }
    }

    // read OFFSET
    match driver.read_offset() {
        Ok(offset) => {
            println!("Stepper driver offset is {}", offset);
        }
        Err(error) => {
            println!("Error for reading offset is {:?}", error);
        }
    }

    // set G_CONF register
    let mut g_conf = driver.g_conf;

    g_conf.set_recalibrate(true);
    g_conf.set_faststandstill(true);
    g_conf.set_en_pwm_mode(true);

    {
        let g_conf = driver.update_g_conf()?;
        println!(
            "SPI status has been updated: {}",
            g_conf.status.to_u32_le().unwrap_or(0)
        );
    }

    {
        let drv_status = driver.read_drv_status()?;
        // either use fields of the register
        println!(
            "Stepper driver is in standstill: {}",
            drv_status.standstill()
        );
        // or extract the u32 value from the register
        println!(
            "Stepper driver DRV_STATUS register is {}",
            drv_status.to_u32_le().unwrap_or(0)
        );
        println!(
            "SPI status has been updated: {}",
            driver.status.to_u32_le().unwrap_or(0)
        );
    }

    {
        let gstat = driver.read_gstat()?;
        println!(
            "Stepper GSTAT register is {}",
            gstat.to_u32_le().unwrap_or(0)
        );
        println!(
            "SPI status has been updated: {}",
            driver.status.to_u32_le().unwrap_or(0)
        );
    }

    Ok(())
}
