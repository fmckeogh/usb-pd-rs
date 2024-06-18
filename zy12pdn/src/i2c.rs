use {
    embassy_time::{Duration, Ticker},
    embedded_hal::digital::{InputPin, OutputPin},
    embedded_hal_async::i2c::{ErrorType, I2c, NoAcknowledgeSource, Operation},
};

/// Bit banging I2C device
pub struct I2cBB<SCL, SDA>
where
    SCL: OutputPin,
    SDA: OutputPin + InputPin,
{
    scl: SCL,
    sda: SDA,
    ticker: Ticker,
}

impl<SCL, SDA> I2cBB<SCL, SDA>
where
    SCL: OutputPin,
    SDA: OutputPin + InputPin,
{
    /// Create instance
    pub fn new(scl: SCL, sda: SDA, frequency: u64) -> Self {
        let interval = Duration::from_hz(frequency);
        I2cBB {
            scl,
            sda,
            ticker: Ticker::every(interval),
        }
    }

    fn set_scl_high(&mut self) -> Result<(), Error> {
        self.scl.set_high().map_err(|_| Error::Bus)
    }

    fn set_scl_low(&mut self) -> Result<(), Error> {
        self.scl.set_low().map_err(|_| Error::Bus)
    }

    fn set_sda_high(&mut self) -> Result<(), Error> {
        self.sda.set_high().map_err(|_| Error::Bus)
    }

    fn set_sda_low(&mut self) -> Result<(), Error> {
        self.sda.set_low().map_err(|_| Error::Bus)
    }

    async fn wait_for_clk(&mut self) {
        self.ticker.next().await;
    }

    async fn i2c_start(&mut self) -> Result<(), Error> {
        self.set_scl_high()?;
        self.set_sda_high()?;
        self.wait_for_clk().await;

        self.set_sda_low()?;
        self.wait_for_clk().await;

        self.set_scl_low()?;
        self.wait_for_clk().await;

        Ok(())
    }

    async fn i2c_stop(&mut self) -> Result<(), Error> {
        self.set_scl_high()?;
        self.wait_for_clk().await;

        self.set_sda_high()?;
        self.wait_for_clk().await;

        Ok(())
    }

    async fn i2c_is_ack(&mut self) -> Result<bool, Error> {
        self.set_sda_high()?;
        self.set_scl_high()?;
        self.wait_for_clk().await;

        let ack = self.sda.is_low().map_err(|_| Error::Bus)?;

        self.set_scl_low()?;
        self.set_sda_low()?;
        self.wait_for_clk().await;

        Ok(ack)
    }

    async fn i2c_read_byte(&mut self, should_send_ack: bool) -> Result<u8, Error> {
        let mut byte: u8 = 0;

        self.set_sda_high()?;

        for bit_offset in 0..8 {
            self.set_scl_high()?;
            self.wait_for_clk().await;

            if self.sda.is_high().map_err(|_| Error::Bus)? {
                byte |= 1 << (7 - bit_offset);
            }

            self.set_scl_low()?;
            self.wait_for_clk().await;
        }

        if should_send_ack {
            self.set_sda_low()?;
        } else {
            self.set_sda_high()?;
        }

        self.set_scl_high()?;
        self.wait_for_clk().await;

        self.set_scl_low()?;
        self.set_sda_low()?;
        self.wait_for_clk().await;

        Ok(byte)
    }

    async fn i2c_write_byte(&mut self, byte: u8) -> Result<(), Error> {
        for bit_offset in 0..8 {
            let out_bit = (byte >> (7 - bit_offset)) & 0b1;

            if out_bit == 1 {
                self.set_sda_high()?;
            } else {
                self.set_sda_low()?;
            }

            self.set_scl_high()?;
            self.wait_for_clk().await;

            self.set_scl_low()?;
            self.set_sda_low()?;
            self.wait_for_clk().await;
        }

        Ok(())
    }

    async fn read_from_slave(&mut self, input: &mut [u8], is_last_op: bool) -> Result<(), Error> {
        for i in 0..input.len() {
            let should_send_ack = if is_last_op {
                i != (input.len() - 1)
            } else {
                true
            };
            input[i] = self.i2c_read_byte(should_send_ack).await?;
        }
        Ok(())
    }

    async fn write_to_slave(&mut self, output: &[u8]) -> Result<(), Error> {
        for byte in output {
            self.i2c_write_byte(*byte).await?;
            self.check_ack().await?;
        }
        Ok(())
    }

    async fn check_ack(&mut self) -> Result<(), Error> {
        if !self.i2c_is_ack().await? {
            Err(Error::NoAck)
        } else {
            Ok(())
        }
    }
}

impl<SCL: OutputPin, SDA: OutputPin + InputPin> I2c for I2cBB<SCL, SDA> {
    /// Execute the provided operations on the I2C bus as a single transaction.
    ///
    /// Transaction contract:
    /// - Before executing the first operation an ST is sent automatically. This
    ///   is followed by SAD+R/W as appropriate.
    /// - Data from adjacent operations of the same type are sent after each
    ///   other without an SP or SR.
    /// - Between adjacent operations of a different type an SR and SAD+R/W is
    ///   sent.
    /// - After executing the last operation an SP is sent automatically.
    /// - If the last operation is a `Read` the master does not send an
    ///   acknowledge for the last byte.
    ///
    /// - `ST` = start condition
    /// - `SAD+R/W` = slave address followed by bit 1 to indicate reading or 0
    ///   to indicate writing
    /// - `SR` = repeated start condition
    /// - `SP` = stop condition
    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        #[derive(Clone, Copy, PartialEq, Eq)]
        enum OpKind {
            Read,
            Write,
        }

        // Before executing the first operation an ST is sent automatically. This is
        // followed by SAD+R/W as appropriate. ST
        self.i2c_start().await?;

        let mut last_op = None;

        let last_op_index = operations.len() - 1;
        for (index, op) in operations.iter_mut().enumerate() {
            match op {
                Operation::Read(buf) => {
                    // Data from adjacent operations of the same type are sent after each other
                    // without an SP or SR. Between adjacent operations of a different type an SR
                    // and SAD+R/W is sent.
                    if last_op != Some(OpKind::Read) {
                        // SR
                        self.i2c_start().await?;

                        // SAD + W
                        self.i2c_write_byte((address << 1) | 0x1).await?;
                        self.check_ack().await?;
                    }

                    // If the last operation is a Read the master does not send an acknowledge for
                    // the last byte.
                    self.read_from_slave(buf, index == last_op_index).await?;

                    last_op = Some(OpKind::Read);
                }
                Operation::Write(buf) => {
                    if last_op != Some(OpKind::Read) {
                        // SR
                        self.i2c_start().await?;

                        // SAD + W
                        self.i2c_write_byte((address << 1) | 0x0).await?;
                        self.check_ack().await?;
                    }

                    self.write_to_slave(buf).await?;

                    last_op = Some(OpKind::Write);
                }
            }
        }

        // After executing the last operation an SP is sent automatically.
        // SP
        self.i2c_stop().await
    }
}

impl<SCL: OutputPin, SDA: OutputPin + InputPin> ErrorType for I2cBB<SCL, SDA> {
    type Error = Error;
}

/// I2C error
#[derive(Debug, Eq, PartialEq, defmt::Format)]
pub enum Error {
    Bus,
    NoAck,
}

impl embedded_hal_async::i2c::Error for Error {
    fn kind(&self) -> embedded_hal::i2c::ErrorKind {
        match self {
            Error::Bus => embedded_hal::i2c::ErrorKind::Bus,
            Error::NoAck => {
                embedded_hal::i2c::ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown)
            }
        }
    }
}
