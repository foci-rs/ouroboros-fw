//! Async SPI2 multiplexer for Ouroboros TMC4671 devices.

use embassy_stm32::gpio::Output;
use embedded_hal::delay::DelayNs as BlockingDelayNs;
use embedded_hal::spi::{
    Operation as BlockingOperation, SpiBus as BlockingSpiBus, SpiDevice as BlockingSpiDevice,
};
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::{ErrorType, Operation, SpiBus, SpiDevice};
use embedded_hal_bus::spi::DeviceError;

use crate::app_types::{CycleDelay, TmcSpiBus};

/// Owns the shared SPI2 bus and both TMC chip-select pins.
pub(crate) struct TmcSpiMux {
    bus: TmcSpiBus,
    cs0: Output<'static>,
    cs1: Output<'static>,
}

impl TmcSpiMux {
    /// Create a mux around SPI2 and channel-local chip selects.
    pub(crate) fn new(bus: TmcSpiBus, cs0: Output<'static>, cs1: Output<'static>) -> Self {
        Self { bus, cs0, cs1 }
    }

    /// Borrow a selected SPI device for `channel`.
    pub(crate) fn device(&mut self, channel: u8) -> Option<SelectedTmcDevice<'_>> {
        let cs = match channel {
            0 => &mut self.cs0,
            1 => &mut self.cs1,
            _ => return None,
        };
        Some(SelectedTmcDevice {
            bus: &mut self.bus,
            cs,
            delay: CycleDelay,
        })
    }
}

/// Temporary async SpiDevice for one selected TMC4671.
pub(crate) struct SelectedTmcDevice<'a> {
    bus: &'a mut TmcSpiBus,
    cs: &'a mut Output<'static>,
    delay: CycleDelay,
}

impl ErrorType for SelectedTmcDevice<'_> {
    type Error = DeviceError<embassy_stm32::spi::Error, core::convert::Infallible>;
}

impl SpiDevice<u8> for SelectedTmcDevice<'_> {
    async fn transaction(
        &mut self,
        operations: &mut [Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.cs.set_low();

        let op_result = 'ops: {
            for op in operations {
                let result = match op {
                    Operation::Read(buf) => self.bus.read(buf).await,
                    Operation::Write(buf) => self.bus.write(buf).await,
                    Operation::Transfer(read, write) => self.bus.transfer(read, write).await,
                    Operation::TransferInPlace(buf) => self.bus.transfer_in_place(buf).await,
                    Operation::DelayNs(ns) => {
                        match <TmcSpiBus as SpiBus<u8>>::flush(self.bus).await {
                            Err(e) => Err(e),
                            Ok(()) => {
                                DelayNs::delay_ns(&mut self.delay, *ns).await;
                                Ok(())
                            }
                        }
                    }
                };
                if let Err(e) = result {
                    break 'ops Err(e);
                }
            }
            Ok(())
        };

        let flush_result = <TmcSpiBus as SpiBus<u8>>::flush(self.bus).await;
        self.cs.set_high();

        op_result.map_err(DeviceError::Spi)?;
        flush_result.map_err(DeviceError::Spi)?;
        Ok(())
    }
}

impl BlockingSpiDevice<u8> for SelectedTmcDevice<'_> {
    fn transaction(
        &mut self,
        operations: &mut [BlockingOperation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.cs.set_low();

        let op_result = 'ops: {
            for op in operations {
                let result = match op {
                    BlockingOperation::Read(buf) => BlockingSpiBus::read(self.bus, buf),
                    BlockingOperation::Write(buf) => BlockingSpiBus::write(self.bus, buf),
                    BlockingOperation::Transfer(read, write) => {
                        BlockingSpiBus::transfer(self.bus, read, write)
                    }
                    BlockingOperation::TransferInPlace(buf) => {
                        BlockingSpiBus::transfer_in_place(self.bus, buf)
                    }
                    BlockingOperation::DelayNs(ns) => {
                        match <TmcSpiBus as BlockingSpiBus<u8>>::flush(self.bus) {
                            Err(error) => Err(error),
                            Ok(()) => {
                                BlockingDelayNs::delay_ns(&mut self.delay, *ns);
                                Ok(())
                            }
                        }
                    }
                };
                if let Err(error) = result {
                    break 'ops Err(error);
                }
            }
            Ok(())
        };

        let flush_result = <TmcSpiBus as BlockingSpiBus<u8>>::flush(self.bus);
        self.cs.set_high();

        op_result.map_err(DeviceError::Spi)?;
        flush_result.map_err(DeviceError::Spi)?;
        Ok(())
    }
}
