//! vl53l0x --https://www.st.com/resource/en/datasheet/vl53l0x.pdf -- Time of Flight IR distance sensor.

#![deny(missing_docs)]
#![deny(warnings)]
#![allow(dead_code)]
#![no_std]

use cast::u16;
use hal::i2c::I2c;
use nb;

const DEFAULT_ADDRESS: u8 = 0x29;

/// dummy
pub struct VL53L0x<I2C: I2c> {
    com: I2C,
    /// dummy
    pub revision_id: u8,
    io_mode2v8: bool,
    stop_variable: u8,
    measurement_timing_budget_microseconds: u32,
    address: u8,
}

/// MPU Error
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// WHO_AM_I returned invalid value (returned value is argument).
    InvalidDevice(u8),
    /// Underlying bus error.
    BusError(E),
    /// Timeout
    Timeout,
    /// I2C address not valid, needs to be between 0x08 and 0x77.
    /// It is a 7 bit address thus the range is 0x00 - 0x7F but
    /// 0x00 - 0x07 and 0x78 - 0x7F are reserved I2C addresses and cannot be used.
    InvalidAddress(u8),
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::BusError(error)
    }
}

impl<I2C, E> VL53L0x<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Creates new driver with default address.
    pub fn new(i2c: I2C) -> Result<VL53L0x<I2C>, Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        VL53L0x::with_address(i2c, DEFAULT_ADDRESS)
    }

    /// Creates new driver with given address.
    pub fn with_address(i2c: I2C, address: u8) -> Result<VL53L0x<I2C>, Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        let mut chip = VL53L0x {
            com: i2c,
            revision_id: 0x00,
            io_mode2v8: true,
            stop_variable: 0,
            measurement_timing_budget_microseconds: 0,
            address,
        };

        let wai = chip.who_am_i()?;
        if wai == 0xEE {
            chip.init_hardware()?;
            // FIXME: return an error/optional
            /*
            chip.set_high_i2c_voltage(); // TODO: make configurable
            chip.revision_id = chip.read_revision_id();
            chip.reset();
            chip.set_high_i2c_voltage();
            chip.set_standard_i2c_mode(); // TODO: make configurable
             */
            Ok(chip)
        } else {
            Err(Error::InvalidDevice(wai))
        }
    }

    /// Changes the I2C address of the sensor.
    /// Note that the address resets when the device is powered off.
    /// Only allows values between 0x08 and 0x77 as the device uses a 7 bit address and
    /// 0x00 - 0x07 and 0x78 - 0x7F are reserved
    pub fn set_address(&mut self, new_address: u8) -> Result<(), Error<E>> {
        if new_address < 0x08 || new_address > 0x77 {
            return Err(Error::InvalidAddress(new_address));
        }
        self.com.write(
            self.address,
            &[Register::I2C_SLAVE_DEVICE_ADDRESS as u8, new_address],
        )?;
        self.address = new_address;

        Ok(())
    }

    fn power_on(&mut self) -> Result<(), E> {
        // TODO use pin to poweron
        // XXX: not-needed?
        Ok(())
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, E> {
        let mut data: [u8; 1] = [0];
        // FIXME:
        //  * device address is not a const
        //  * register address is u16
        self.com.write_read(self.address, &[reg as u8], &mut data)?;
        Ok(data[0])
    }

    fn read_byte(&mut self, reg: u8) -> Result<u8, E> {
        let mut data: [u8; 1] = [0];
        // FIXME:
        //  * device address is not a const
        //  * register address is u16
        self.com.write_read(self.address, &[reg], &mut data)?;
        Ok(data[0])
    }

    fn read_6bytes(&mut self, reg: Register) -> Result<[u8; 6], E> {
        let mut ret: [u8; 6] = Default::default();
        self.read_registers(reg, &mut ret)?;

        Ok(ret)
    }

    fn read_registers(
        &mut self,
        reg: Register,
        buffer: &mut [u8],
    ) -> Result<(), E> {
        // const I2C_AUTO_INCREMENT: u8 = 1 << 7;
        const I2C_AUTO_INCREMENT: u8 = 0;
        self.com.write_read(
            self.address,
            &[(reg as u8) | I2C_AUTO_INCREMENT],
            buffer,
        )?;

        Ok(())
    }

    fn read_16bit(&mut self, reg: Register) -> Result<u16, E> {
        let mut buffer: [u8; 2] = [0, 0];
        self.read_registers(reg, &mut buffer)?;
        Ok((u16(buffer[0]) << 8) + u16(buffer[1]))
    }

    fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), E> {
        let mut buffer = [0];
        self.com.write_read(self.address, &[reg, byte], &mut buffer)
    }

    fn write_register(&mut self, reg: Register, byte: u8) -> Result<(), E> {
        let mut buffer = [0];
        self.com
            .write_read(self.address, &[reg as u8, byte], &mut buffer)
    }

    fn write_6bytes(&mut self, reg: Register, bytes: [u8; 6]) -> Result<(), E> {
        let mut buf: [u8; 6] = [0, 0, 0, 0, 0, 0];
        self.com.write_read(
            self.address,
            &[
                reg as u8, bytes[0], bytes[1], bytes[2], bytes[3], bytes[4],
                bytes[5],
            ],
            &mut buf,
        )
    }

    fn write_16bit(&mut self, reg: Register, word: u16) -> Result<(), E> {
        let mut buffer = [0];
        let msb = (word >> 8) as u8;
        let lsb = (word & 0xFF) as u8;
        self.com
            .write_read(self.address, &[reg as u8, msb, lsb], &mut buffer)
    }

    fn write_32bit(&mut self, reg: Register, word: u32) -> Result<(), E> {
        let mut buffer = [0];
        let v1 = (word & 0xFF) as u8;
        let v2 = ((word >> 8) & 0xFF) as u8;
        let v3 = ((word >> 16) & 0xFF) as u8;
        let v4 = ((word >> 24) & 0xFF) as u8;
        self.com.write_read(
            self.address,
            &[reg as u8, v1, v2, v3, v4],
            &mut buffer,
        )
    }

    fn set_signal_rate_limit(&mut self, limit: f32) -> Result<bool, E> {
        if limit < 0.0 || limit > 511.99 {
            Ok(false)
        } else {
            // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
            self.write_16bit(
                Register::FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                (limit * ((1 << 7) as f32)) as u16,
            )?;
            Ok(true)
        }
    }

    fn get_spad_info(&mut self) -> Result<(u8, u8), Error<E>> {
        self.write_byte(0x80, 0x01)?;
        self.write_byte(0xFF, 0x01)?;
        self.write_byte(0x00, 0x00)?;

        self.write_byte(0xFF, 0x06)?;
        let mut tmp83 = self.read_byte(0x83)?;
        self.write_byte(0x83, tmp83 | 0x04)?;
        self.write_byte(0xFF, 0x07)?;
        self.write_byte(0x81, 0x01)?;

        self.write_byte(0x80, 0x01)?;

        self.write_byte(0x94, 0x6b)?;
        self.write_byte(0x83, 0x00)?;

        let mut c = 0;
        while self.read_byte(0x83)? == 0x00 {
            c += 1;
            if c == 65535 {
                return Err(Error::Timeout);
            }
        }

        self.write_byte(0x83, 0x01)?;
        let tmp = self.read_byte(0x92)?;

        let count: u8 = tmp & 0x7f;
        let type_is_aperture: u8 = (tmp >> 7) & 0x01;

        self.write_byte(0x81, 0x00)?;
        self.write_byte(0xFF, 0x06)?;
        tmp83 = self.read_byte(0x83)?;
        self.write_byte(0x83, tmp83 & !0x04)?;
        self.write_byte(0xFF, 0x01)?;
        self.write_byte(0x00, 0x01)?;

        self.write_byte(0xFF, 0x00)?;
        self.write_byte(0x80, 0x00)?;

        Ok((count, type_is_aperture))
    }

    /// startContinuous
    pub fn start_continuous(&mut self, period_millis: u32) -> Result<(), E> {
        self.write_byte(0x80, 0x01)?;
        self.write_byte(0xFF, 0x01)?;
        self.write_byte(0x00, 0x00)?;
        let sv = self.stop_variable;
        self.write_byte(0x91, sv)?;
        self.write_byte(0x00, 0x01)?;
        self.write_byte(0xFF, 0x00)?;
        self.write_byte(0x80, 0x00)?;

        let mut period_millis = period_millis;
        if period_millis != 0 {
            // continuous timed mode
            // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
            let osc_calibrate_value =
                self.read_16bit(Register::OSC_CALIBRATE_VAL)?;

            if osc_calibrate_value != 0 {
                period_millis *= osc_calibrate_value as u32;
            }

            self.write_32bit(
                Register::SYSTEM_INTERMEASUREMENT_PERIOD,
                period_millis,
            )?;
            // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end
            // VL53L0X_REG_SYSRANGE_MODE_TIMED
            self.write_register(Register::SYSRANGE_START, 0x04)?;
        } else {
            // continuous back-to-back mode
            // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
            self.write_register(Register::SYSRANGE_START, 0x02)?;
        }

        Ok(())
    }

    /// stopContinuous()
    pub fn stop_continuous(&mut self) -> Result<(), E> {
        // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
        self.write_register(Register::SYSRANGE_START, 0x01)?;
        self.write_byte(0xFF, 0x01)?;
        self.write_byte(0x00, 0x00)?;
        self.write_byte(0x91, 0x00)?;
        self.write_byte(0x00, 0x01)?;
        self.write_byte(0xFF, 0x00)?;

        Ok(())
    }

    /// reads and returns range measurement or nb::Error::WouldBlock if it's not ready yet
    pub fn read_range_mm(&mut self) -> nb::Result<u16, Error<E>> {
        match self.read_register(Register::RESULT_INTERRUPT_STATUS) {
            Ok(r) => {
                if (r & 0x07) == 0 {
                    Err(nb::Error::WouldBlock)
                } else {
                    let range_err =
                        self.read_16bit(Register::RESULT_RANGE_STATUS_plus_10);
                    let write_err = self
                        .write_register(Register::SYSTEM_INTERRUPT_CLEAR, 0x01);
                    match (range_err, write_err) {
                        (Ok(res), Ok(_)) => Ok(res),
                        (Err(e), _) => Err(nb::Error::Other(Error::from(e))),
                        (_, Err(e)) => Err(nb::Error::Other(Error::from(e))),
                    }
                }
            }
            Err(e) => Err(nb::Error::Other(Error::from(e))),
        }
    }

    /// readRangeContinuousMillimeters (blocking)
    pub fn read_range_continuous_millimeters_blocking(
        &mut self,
    ) -> Result<u16, Error<E>> {
        let mut c = 0;
        while (self.read_register(Register::RESULT_INTERRUPT_STATUS)? & 0x07)
            == 0
        {
            c += 1;
            if c == 10000 {
                return Err(Error::Timeout);
            }
        }
        let range_err = self.read_16bit(Register::RESULT_RANGE_STATUS_plus_10);
        // don't use ? to cleanup
        self.write_register(Register::SYSTEM_INTERRUPT_CLEAR, 0x01)?;

        Ok(range_err?)
    }

    /// readRangeSingleMillimeters (blocking)
    pub fn read_range_single_millimeters_blocking(
        &mut self,
    ) -> Result<u16, Error<E>> {
        self.write_byte(0x80, 0x01)?;
        self.write_byte(0xFF, 0x01)?;
        self.write_byte(0x00, 0x00)?;
        let sv = self.stop_variable;
        self.write_byte(0x91, sv)?;
        self.write_byte(0x00, 0x01)?;
        self.write_byte(0xFF, 0x00)?;
        self.write_byte(0x80, 0x00)?;

        self.write_register(Register::SYSRANGE_START, 0x01)?;

        // "Wait until start bit has been cleared"
        let mut c = 0;
        while (self.read_register(Register::SYSRANGE_START)? & 0x01) != 0 {
            c += 1;
            if c == 10000 {
                return Err(Error::Timeout);
            }
        }
        self.read_range_continuous_millimeters_blocking()
    }

    // performSingleRefCalibration(uint8_t vhvInitByte)
    fn perform_single_ref_calibration(
        &mut self,
        vhv_init_byte: u8,
    ) -> Result<(), Error<E>> {
        // VL53L0X_REG_SYSRANGE_MODE_START_STOP
        self.write_register(Register::SYSRANGE_START, 0x01 | vhv_init_byte)?;
        let mut c = 0;
        while (self.read_register(Register::RESULT_INTERRUPT_STATUS)? & 0x07)
            == 0
        {
            c += 1;
            if c == 10000 {
                return Err(Error::Timeout);
            }
        }
        self.write_register(Register::SYSTEM_INTERRUPT_CLEAR, 0x01)?;
        self.write_register(Register::SYSRANGE_START, 0x00)?;

        Ok(())
    }

    fn init_hardware(&mut self) -> Result<(), Error<E>> {
        // Enable the sensor

        self.power_on()?;
        // VL53L0X_DataInit() begin

        // Sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
        if self.io_mode2v8 {
            // set bit 0
            let ext_sup_hv = self
                .read_register(Register::VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV)?;
            self.write_register(
                Register::VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                ext_sup_hv | 0x01,
            )?;
        }

        // "Set I2C standard mode"
        self.write_byte(0x88, 0x00)?;
        self.write_byte(0x80, 0x01)?;
        self.write_byte(0xFF, 0x01)?;
        self.write_byte(0x00, 0x00)?;
        self.stop_variable = self.read_byte(0x91)?;
        self.write_byte(0x00, 0x01)?;
        self.write_byte(0xFF, 0x00)?;
        self.write_byte(0x80, 0x00)?;

        // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
        let config = self.read_register(Register::MSRC_CONFIG_CONTROL)?;
        self.write_register(Register::MSRC_CONFIG_CONTROL, config | 0x12)?;

        // set final range signal rate limit to 0.25 MCPS (million counts per second)
        self.set_signal_rate_limit(0.25)?;

        self.write_register(Register::SYSTEM_SEQUENCE_CONFIG, 0xFF)?;

        // VL53L0X_DataInit() end

        // VL53L0X_StaticInit() begin

        // TODO fail to initialize on timeout of this
        let (spad_count, spad_type_is_aperture) = self.get_spad_info()?;

        // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in the API,
        // but the same data seems to be more easily readable from GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
        let mut ref_spad_map =
            self.read_6bytes(Register::GLOBAL_CONFIG_SPAD_ENABLES_REF_0)?;

        // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

        self.write_byte(0xFF, 0x01)?;
        self.write_register(Register::DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)?;
        self.write_register(
            Register::DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD,
            0x2C,
        )?;
        self.write_byte(0xFF, 0x00)?;
        self.write_register(Register::GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)?;

        // 12 is the first aperture spad
        let first_spad_to_enable =
            if spad_type_is_aperture != 0 { 12 } else { 0 };
        let mut spads_enabled: u8 = 0;

        for i in 0..48 {
            if i < first_spad_to_enable || spads_enabled == spad_count {
                // This bit is lower than the first one that should be enabled, or (reference_spad_count) bits have already been enabled, so zero this bit
                ref_spad_map[i / 8] &= !(1 << (i % 8));
            } else if (ref_spad_map[i / 8] >> (i % 8)) & 0x1 > 0 {
                spads_enabled = spads_enabled + 1;
            }
        }

        self.write_6bytes(
            Register::GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
            ref_spad_map,
        )?;

        // -- VL53L0X_set_reference_spads() end

        // -- VL53L0X_load_tuning_settings() begin
        // DefaultTuningSettings from vl53l0x_tuning.h

        self.write_byte(0xFF, 0x01)?;
        self.write_byte(0x00, 0x00)?;

        self.write_byte(0xFF, 0x00)?;
        self.write_byte(0x09, 0x00)?;
        self.write_byte(0x10, 0x00)?;
        self.write_byte(0x11, 0x00)?;

        self.write_byte(0x24, 0x01)?;
        self.write_byte(0x25, 0xFF)?;
        self.write_byte(0x75, 0x00)?;

        self.write_byte(0xFF, 0x01)?;
        self.write_byte(0x4E, 0x2C)?;
        self.write_byte(0x48, 0x00)?;
        self.write_byte(0x30, 0x20)?;

        self.write_byte(0xFF, 0x00)?;
        self.write_byte(0x30, 0x09)?;
        self.write_byte(0x54, 0x00)?;
        self.write_byte(0x31, 0x04)?;
        self.write_byte(0x32, 0x03)?;
        self.write_byte(0x40, 0x83)?;
        self.write_byte(0x46, 0x25)?;
        self.write_byte(0x60, 0x00)?;
        self.write_byte(0x27, 0x00)?;
        self.write_byte(0x50, 0x06)?;
        self.write_byte(0x51, 0x00)?;
        self.write_byte(0x52, 0x96)?;
        self.write_byte(0x56, 0x08)?;
        self.write_byte(0x57, 0x30)?;
        self.write_byte(0x61, 0x00)?;
        self.write_byte(0x62, 0x00)?;
        self.write_byte(0x64, 0x00)?;
        self.write_byte(0x65, 0x00)?;
        self.write_byte(0x66, 0xA0)?;

        self.write_byte(0xFF, 0x01)?;
        self.write_byte(0x22, 0x32)?;
        self.write_byte(0x47, 0x14)?;
        self.write_byte(0x49, 0xFF)?;
        self.write_byte(0x4A, 0x00)?;

        self.write_byte(0xFF, 0x00)?;
        self.write_byte(0x7A, 0x0A)?;
        self.write_byte(0x7B, 0x00)?;
        self.write_byte(0x78, 0x21)?;

        self.write_byte(0xFF, 0x01)?;
        self.write_byte(0x23, 0x34)?;
        self.write_byte(0x42, 0x00)?;
        self.write_byte(0x44, 0xFF)?;
        self.write_byte(0x45, 0x26)?;
        self.write_byte(0x46, 0x05)?;
        self.write_byte(0x40, 0x40)?;
        self.write_byte(0x0E, 0x06)?;
        self.write_byte(0x20, 0x1A)?;
        self.write_byte(0x43, 0x40)?;

        self.write_byte(0xFF, 0x00)?;
        self.write_byte(0x34, 0x03)?;
        self.write_byte(0x35, 0x44)?;

        self.write_byte(0xFF, 0x01)?;
        self.write_byte(0x31, 0x04)?;
        self.write_byte(0x4B, 0x09)?;
        self.write_byte(0x4C, 0x05)?;
        self.write_byte(0x4D, 0x04)?;

        self.write_byte(0xFF, 0x00)?;
        self.write_byte(0x44, 0x00)?;
        self.write_byte(0x45, 0x20)?;
        self.write_byte(0x47, 0x08)?;
        self.write_byte(0x48, 0x28)?;
        self.write_byte(0x67, 0x00)?;
        self.write_byte(0x70, 0x04)?;
        self.write_byte(0x71, 0x01)?;
        self.write_byte(0x72, 0xFE)?;
        self.write_byte(0x76, 0x00)?;
        self.write_byte(0x77, 0x00)?;

        self.write_byte(0xFF, 0x01)?;
        self.write_byte(0x0D, 0x01)?;

        self.write_byte(0xFF, 0x00)?;
        self.write_byte(0x80, 0x01)?;
        self.write_byte(0x01, 0xF8)?;

        self.write_byte(0xFF, 0x01)?;
        self.write_byte(0x8E, 0x01)?;
        self.write_byte(0x00, 0x01)?;
        self.write_byte(0xFF, 0x00)?;
        self.write_byte(0x80, 0x00)?;

        // -- VL53L0X_load_tuning_settings() end

        // "Set interrupt config to new sample ready"
        // -- VL53L0X_SetGpioConfig() begin

        self.write_register(Register::SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)?;
        // active low
        let high = self.read_register(Register::GPIO_HV_MUX_ACTIVE_HIGH)?;
        self.write_register(Register::GPIO_HV_MUX_ACTIVE_HIGH, high & !0x10)?;
        self.write_register(Register::SYSTEM_INTERRUPT_CLEAR, 0x01)?;

        // -- VL53L0X_SetGpioConfig() end
        // "Disable MSRC and TCC by default"
        // MSRC = Minimum Signal Rate Check
        // TCC = Target CentreCheck
        // -- VL53L0X_SetSequenceStepEnable() begin
        self.measurement_timing_budget_microseconds =
            self.get_measurement_timing_budget()?;
        self.write_register(Register::SYSTEM_SEQUENCE_CONFIG, 0xE8)?;

        // -- VL53L0X_SetSequenceStepEnable() end

        // "Recalculate timing budget"
        let mtbm = self.measurement_timing_budget_microseconds;
        self.set_measurement_timing_budget(mtbm)?;

        // VL53L0X_StaticInit() end

        // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

        // -- VL53L0X_perform_vhv_calibration() begin

        self.write_register(Register::SYSTEM_SEQUENCE_CONFIG, 0x01)?;
        self.perform_single_ref_calibration(0x40)?;
        // -- VL53L0X_perform_vhv_calibration() end
        // -- VL53L0X_perform_phase_calibration() begin

        self.write_register(Register::SYSTEM_SEQUENCE_CONFIG, 0x02)?;
        self.perform_single_ref_calibration(0x00)?;

        // -- VL53L0X_perform_phase_calibration() end

        // "restore the previous Sequence Config"
        self.write_register(Register::SYSTEM_SEQUENCE_CONFIG, 0xE8)?;

        // VL53L0X_PerformRefCalibration() end
        Ok(())
    }

    /// Returns who am i
    pub fn who_am_i(&mut self) -> Result<u8, E> {
        self.read_register(Register::WHO_AM_I)
    }

    fn get_vcsel_pulse_period(&mut self, ty: VcselPeriodType) -> Result<u8, E> {
        match ty {
            VcselPeriodType::VcselPeriodPreRange => Ok(decode_vcsel_period(
                self.read_register(Register::PRE_RANGE_CONFIG_VCSEL_PERIOD)?,
            )),
            VcselPeriodType::VcselPeriodFinalRange => Ok(decode_vcsel_period(
                self.read_register(Register::FINAL_RANGE_CONFIG_VCSEL_PERIOD)?,
            )),
        }
    }

    // getSequenceStepEnables(VL53L0XSequenceStepEnables* enables) {
    fn get_sequence_step_enables(&mut self) -> Result<SeqStepEnables, E> {
        let sequence_config: u8 =
            self.read_register(Register::SYSTEM_SEQUENCE_CONFIG)?;
        Ok(SeqStepEnables {
            tcc: ((sequence_config >> 4) & 0x1) == 1,
            dss: ((sequence_config >> 3) & 0x1) == 1,
            msrc: ((sequence_config >> 2) & 0x1) == 1,
            pre_range: ((sequence_config >> 6) & 0x1) == 1,
            final_range: ((sequence_config >> 7) & 0x1) == 1,
        })
    }

    // getSequenceStepTimeouts(timeouts)
    fn get_sequence_step_timeouts(
        &mut self,
        enables: &SeqStepEnables,
    ) -> Result<SeqStepTimeouts, E> {
        let pre_range_mclks = decode_timeout(
            self.read_16bit(Register::PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI)?,
        );
        let mut final_range_mclks = decode_timeout(
            self.read_16bit(Register::FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI)?,
        );
        if enables.pre_range {
            final_range_mclks -= pre_range_mclks;
        };
        let pre_range_vcselperiod_pclks =
            self.get_vcsel_pulse_period(VcselPeriodType::VcselPeriodPreRange)?;
        let msrc_dss_tcc_mclks =
            self.read_register(Register::MSRC_CONFIG_TIMEOUT_MACROP)? + 1;
        let final_range_vcsel_period_pclks = self
            .get_vcsel_pulse_period(VcselPeriodType::VcselPeriodFinalRange)?;
        Ok(SeqStepTimeouts {
            pre_range_vcselperiod_pclks,
            msrc_dss_tcc_mclks,
            msrc_dss_tcc_microseconds: timeout_mclks_to_microseconds(
                msrc_dss_tcc_mclks as u16,
                pre_range_vcselperiod_pclks,
            ),
            pre_range_mclks: pre_range_mclks,
            pre_range_microseconds: timeout_mclks_to_microseconds(
                pre_range_mclks,
                pre_range_vcselperiod_pclks,
            ),
            final_range_mclks,
            final_range_vcsel_period_pclks,
            final_range_microseconds: timeout_mclks_to_microseconds(
                final_range_mclks,
                final_range_vcsel_period_pclks,
            ),
        })
    }

    // uint32_t VL53L0X::getMeasurementTimingBudget() {
    fn get_measurement_timing_budget(&mut self) -> Result<u32, E> {
        let start_overhead: u32 = 1910;
        let end_overhead: u32 = 960;
        let msrc_overhead: u32 = 660;
        let tcc_overhead: u32 = 590;
        let dss_overhead: u32 = 690;
        let pre_range_overhead: u32 = 660;
        let final_range_overhead: u32 = 550;

        let enables = self.get_sequence_step_enables()?;
        let timeouts = self.get_sequence_step_timeouts(&enables)?;

        // "Start and end overhead times always present"
        let mut budget_microseconds = start_overhead + end_overhead;
        if enables.tcc {
            budget_microseconds +=
                timeouts.msrc_dss_tcc_microseconds + tcc_overhead;
        }
        if enables.dss {
            budget_microseconds +=
                2 * (timeouts.msrc_dss_tcc_microseconds + dss_overhead);
        } else if enables.msrc {
            budget_microseconds +=
                timeouts.msrc_dss_tcc_microseconds + msrc_overhead;
        }
        if enables.pre_range {
            budget_microseconds +=
                timeouts.pre_range_microseconds + pre_range_overhead;
        }
        if enables.final_range {
            budget_microseconds +=
                timeouts.final_range_microseconds + final_range_overhead;
        }

        // store for internal reuse
        Ok(budget_microseconds)
    }

    /// setMeasurementTimingBudget(budget_microseconds)
    pub fn set_measurement_timing_budget(
        &mut self,
        budget_microseconds: u32,
    ) -> Result<bool, E> {
        // note that these are different than values in get_
        let start_overhead: u32 = 1320;
        let end_overhead: u32 = 960;
        let msrc_overhead: u32 = 660;
        let tcc_overhead: u32 = 590;
        let dss_overhead: u32 = 690;
        let pre_range_overhead: u32 = 660;
        let final_range_overhead: u32 = 550;
        let min_timing_budget: u32 = 20000;

        if budget_microseconds < min_timing_budget {
            return Ok(false);
        }

        let enables = self.get_sequence_step_enables()?;
        let timeouts = self.get_sequence_step_timeouts(&enables)?;

        let mut use_budget_microseconds: u32 =
            (start_overhead + end_overhead) as u32;
        if enables.tcc {
            use_budget_microseconds +=
                timeouts.msrc_dss_tcc_microseconds + tcc_overhead;
        }
        if enables.dss {
            use_budget_microseconds +=
                2 * timeouts.msrc_dss_tcc_microseconds + dss_overhead;
        } else if enables.msrc {
            use_budget_microseconds +=
                timeouts.msrc_dss_tcc_microseconds + msrc_overhead;
        }
        if enables.pre_range {
            use_budget_microseconds +=
                timeouts.pre_range_microseconds + pre_range_overhead;
        }
        if enables.final_range {
            use_budget_microseconds += final_range_overhead;
        }

        // "Note that the final range timeout is determined by the timing
        // budget and the sum of all other timeouts within the sequence.
        // If there is no room for the final range timeout, then an error
        // will be set. Otherwise the remaining time will be applied to
        // the final range."

        if use_budget_microseconds > budget_microseconds {
            // "Requested timeout too small."
            return Ok(false);
        }

        let final_range_timeout_microseconds: u32 =
            budget_microseconds - use_budget_microseconds;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)
        // "For the final range timeout, the pre-range timeout
        // must be added. To do this both final and pre-range
        // timeouts must be expressed in macro periods MClks
        // because they have different vcsel periods."
        let mut final_range_timeout_mclks: u16 = timeout_microseconds_to_mclks(
            final_range_timeout_microseconds,
            timeouts.final_range_vcsel_period_pclks,
        ) as u16;

        if enables.pre_range {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        self.write_16bit(
            Register::FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
            encode_timeout(final_range_timeout_mclks),
        )?;

        // set_sequence_step_timeout() end
        // store for internal reuse
        self.measurement_timing_budget_microseconds = budget_microseconds;
        Ok(true)
    }

    /*
        fn write_byte_raw(&mut self, reg: u8, byte: u8) {
            // FIXME:
            //  * remove this function
            //  * device address is not a const
            //  * register address is u16
            let mut buffer = [0];
            let _ = self.com.write_read(ADDRESS, &[reg, byte], &mut buffer);
        }

        fn read_byte_raw(&mut self, reg: u8) -> u8 {
            // FIXME:
            //  * remove this function
            //  * device address is not a const
            //  * register address is u16
            let mut data: [u8; 1] = [0];
            let _ = self.com.write_read(ADDRESS, &[reg], &mut data);
            data[0]
        }

        fn write_byte(&mut self, reg: Register, byte: u8) {
            let mut buffer = [0];
            let _ = self
                .com
                .write_read(ADDRESS, &[reg as u8, byte], &mut buffer);
        }
    */
}

struct SeqStepEnables {
    tcc: bool,
    dss: bool,
    msrc: bool,
    pre_range: bool,
    final_range: bool,
}

struct SeqStepTimeouts {
    pre_range_vcselperiod_pclks: u8,
    final_range_vcsel_period_pclks: u8,
    msrc_dss_tcc_mclks: u8,
    pre_range_mclks: u16,
    final_range_mclks: u16,
    msrc_dss_tcc_microseconds: u32,
    pre_range_microseconds: u32,
    final_range_microseconds: u32,
}

fn decode_timeout(register_value: u16) -> u16 {
    // format: "(LSByte * 2^MSByte) + 1"
    ((register_value & 0x00FF) << (((register_value & 0xFF00) as u16) >> 8))
        as u16
        + 1
}

fn encode_timeout(timeout_mclks: u16) -> u16 {
    if timeout_mclks == 0 {
        return 0;
    }
    let mut ls_byte: u32;
    let mut ms_byte: u16 = 0;

    ls_byte = (timeout_mclks as u32) - 1;

    while (ls_byte & 0xFFFFFF00) > 0 {
        ls_byte >>= 1;
        ms_byte += 1;
    }

    return (ms_byte << 8) | ((ls_byte & 0xFF) as u16);
}

fn calc_macro_period(vcsel_period_pclks: u8) -> u32 {
    ((2304u32 * (vcsel_period_pclks as u32) * 1655u32) + 500u32) / 1000u32
}

fn timeout_mclks_to_microseconds(
    timeout_period_mclks: u16,
    vcsel_period_pclks: u8,
) -> u32 {
    let macro_period_nanoseconds: u32 =
        calc_macro_period(vcsel_period_pclks) as u32;
    (((timeout_period_mclks as u32) * macro_period_nanoseconds)
        + (macro_period_nanoseconds / 2))
        / 1000
}

fn timeout_microseconds_to_mclks(
    timeout_period_microseconds: u32,
    vcsel_period_pclks: u8,
) -> u32 {
    let macro_period_nanoseconds: u32 =
        calc_macro_period(vcsel_period_pclks) as u32;

    ((timeout_period_microseconds * 1000) + (macro_period_nanoseconds / 2))
        / macro_period_nanoseconds
}

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs from register value based on VL53L0X_decode_vcsel_period()
fn decode_vcsel_period(register_value: u8) -> u8 {
    ((register_value) + 1) << 1
}

// Encode VCSEL pulse period register value from period in PCLKs based on VL53L0X_encode_vcsel_period()
fn encode_vcsel_period(period_pclks: u8) -> u8 {
    ((period_pclks) >> 1) - 1
}

#[allow(non_camel_case_types)]
enum Register {
    SYSRANGE_START = 0x00,
    WHO_AM_I = 0xC0,
    VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,
    MSRC_CONFIG_CONTROL = 0x60,
    SYSTEM_SEQUENCE_CONFIG = 0x01,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
    DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
    GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
    SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,
    GPIO_HV_MUX_ACTIVE_HIGH = 0x84,
    SYSTEM_INTERRUPT_CLEAR = 0x0B,
    RESULT_INTERRUPT_STATUS = 0x13,
    RESULT_RANGE_STATUS = 0x14,
    RESULT_RANGE_STATUS_plus_10 = 0x1e,
    OSC_CALIBRATE_VAL = 0xF8,
    SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,
    FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
    PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72,
    CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20,
    MSRC_CONFIG_TIMEOUT_MACROP = 0x46,
    I2C_SLAVE_DEVICE_ADDRESS = 0x8A,
}

#[derive(Debug, Copy, Clone)]
enum VcselPeriodType {
    VcselPeriodPreRange = 0,
    VcselPeriodFinalRange = 1,
}
