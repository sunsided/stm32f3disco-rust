#![allow(dead_code)]

use embedded_hal::spi::{ErrorKind, ErrorType, Operation, SpiDevice};
use l3gd20_ng::{Characteristics, I16x3, L3GD20SPI};
use l3gd20_registers::*;
use stm32f3xx_hal::gpio::{gpioa, gpioe, Alternate, Gpioa, Gpioe, Output, Pin, PushPull, U};
use stm32f3xx_hal::hal::blocking::spi::{Transfer, Write};
use stm32f3xx_hal::pac::SPI1;
use stm32f3xx_hal::prelude::*;
use stm32f3xx_hal::rcc;
use stm32f3xx_hal::spi::{self, Spi};

type SpiType = Spi<
    SPI1,
    (
        Pin<Gpioa, U<5>, Alternate<PushPull, 5>>,
        Pin<Gpioa, U<6>, Alternate<PushPull, 5>>,
        Pin<Gpioa, U<7>, Alternate<PushPull, 5>>,
    ),
    u8,
>;

type CsPin = Pin<Gpioe, U<3>, Output<PushPull>>;

/// A gyroscope error.
pub type GyroError = l3gd20_ng::Error<SpiError>;

/// SPI Driver for the STMicroelectronics L3GD20 Gyroscope.
pub struct Gyroscope {
    l3gd20: L3GD20SPI<GyroSpi>,
}

/// Chip select for the L3GD20.
pub struct GyroscopeChipSelect(CsPin);

impl GyroscopeChipSelect {
    /// Initialize the chip select.
    pub fn new<CsMode>(
        cs: gpioe::PE3<CsMode>,
        moder: &mut gpioe::MODER,
        otyper: &mut gpioe::OTYPER,
    ) -> Self {
        let mut l3gd20_cs = cs.into_push_pull_output(moder, otyper);
        // Disable the line.
        l3gd20_cs.set_high().ok();
        GyroscopeChipSelect(l3gd20_cs)
    }
}

/// An SPI error of the `stm32f3xx-hal` bus.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct SpiError(pub spi::Error);

impl embedded_hal::spi::Error for SpiError {
    fn kind(&self) -> ErrorKind {
        match self.0 {
            spi::Error::Overrun => ErrorKind::Overrun,
            spi::Error::ModeFault => ErrorKind::ModeFault,
            _ => ErrorKind::Other,
        }
    }
}

/// The SPI bus plus the gyroscope's chip select, as an embedded-hal 1.0 [`SpiDevice`].
///
/// `stm32f3xx-hal` 0.10 only implements the embedded-hal 0.2 traits.
pub struct GyroSpi {
    spi: SpiType,
    cs: CsPin,
}

impl GyroSpi {
    fn run(&mut self, operation: &mut Operation<'_, u8>) -> Result<(), spi::Error> {
        match operation {
            Operation::Read(words) => {
                words.fill(0);
                self.spi.transfer(words).map(|_| ())
            }
            Operation::Write(words) => self.spi.write(words),
            Operation::Transfer(read, write) => {
                for i in 0..read.len().max(write.len()) {
                    let mut word = [write.get(i).copied().unwrap_or(0)];
                    self.spi.transfer(&mut word)?;
                    if let Some(out) = read.get_mut(i) {
                        *out = word[0];
                    }
                }
                Ok(())
            }
            Operation::TransferInPlace(words) => self.spi.transfer(words).map(|_| ()),
            // shortcut: no delay source here; l3gd20-ng never requests one. Pass a DelayNs if that changes.
            Operation::DelayNs(_) => Ok(()),
        }
    }
}

impl ErrorType for GyroSpi {
    type Error = SpiError;
}

impl SpiDevice for GyroSpi {
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), SpiError> {
        self.cs.set_low().ok();
        let result = operations.iter_mut().try_for_each(|op| self.run(op));
        self.cs.set_high().ok();
        result.map_err(SpiError)
    }
}

impl Gyroscope {
    /// Initialize the SPI connection.
    #[allow(clippy::too_many_arguments)]
    pub fn new<SckMode, MisoMode, MosiMode>(
        sck: gpioa::PA5<SckMode>,
        miso: gpioa::PA6<MisoMode>,
        mosi: gpioa::PA7<MosiMode>,
        moder: &mut gpioa::MODER,
        otyper: &mut gpioa::OTYPER,
        alternate_function_low: &mut gpioa::AFRL,
        spi: SPI1,
        clocks: rcc::Clocks,
        advanced_periph_bus: &mut rcc::APB2,
        chip_select: GyroscopeChipSelect,
    ) -> Result<Self, GyroError> {
        let sck = sck.into_af_push_pull(moder, otyper, alternate_function_low);
        let miso = miso.into_af_push_pull(moder, otyper, alternate_function_low);
        let mosi = mosi.into_af_push_pull(moder, otyper, alternate_function_low);

        // Set up SPI.
        let spi = Spi::new(spi, (sck, miso, mosi), 3.MHz(), clocks, advanced_periph_bus);
        let mut device = L3GD20SPI::new(GyroSpi {
            spi,
            cs: chip_select.0,
        })?;

        // Requires `PE1` and `EXTI1` to be set up correctly.
        device.enable_data_ready(true)?;
        device.set_odr(OutputDataRate::Hz380)?;
        device.set_bandwidth(Bandwidth::Medium)?;
        device.set_sensitivity(Sensitivity::D500)?;

        Ok(Self { l3gd20: device })
    }

    /// Identifies this chip by querying the `WHO_AM_I` register.
    pub fn identify(&mut self) -> Result<bool, GyroError> {
        self.l3gd20.identify()
    }

    /// Resets the device to reasonable defaults.
    pub fn reset(&mut self) -> Result<(), GyroError> {
        self.l3gd20.reset()
    }

    /// Make the sensor as slow as possible.
    pub fn slowpoke(&mut self) -> Result<(), GyroError> {
        self.l3gd20.set_odr(OutputDataRate::Hz95)
    }

    /// Reads the raw temperature from the `OUT_TEMP` register.
    pub fn temp_raw(&mut self) -> Result<i8, GyroError> {
        self.l3gd20.temp_raw()
    }

    /// Fetches all data off the sensor.
    pub fn xyz_raw(&mut self) -> Result<I16x3, GyroError> {
        self.l3gd20.xyz_raw()
    }

    /// Get the sensor characteristics.
    pub fn characteristics(&mut self) -> Result<Characteristics, GyroError> {
        self.l3gd20.characteristics()
    }
}
