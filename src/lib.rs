//! Dummy

#![deny(missing_docs)]
#![deny(warnings)]
#![allow(dead_code)]
#![no_std]

extern crate cast;
extern crate embedded_hal as ehal;
extern crate generic_array;

use core::mem;

use cast::u16;

use ehal::blocking::i2c::{Write, WriteRead};
use generic_array::typenum::consts::*;
use generic_array::{ArrayLength, GenericArray};

const ADDRESS: u8 = 0x29;

/// dummy
pub struct VL53L0x<I2C: ehal::blocking::i2c::WriteRead> {
    com: I2C,
    /// dummy
    pub revision_id: u8,
    io_mode2v8: bool,
    stop_variable: u8,
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
}

impl<E> core::convert::From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::BusError(error)
    }
}

impl<I2C, E> VL53L0x<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// Dummy.
    pub fn new(i2c: I2C) -> Result<VL53L0x<I2C>, Error<E>>
    where
        I2C: ehal::blocking::i2c::WriteRead<Error = E>,
    {
        let mut chip = VL53L0x {
            com: i2c,
            revision_id: 0x00,
            io_mode2v8: true,
            stop_variable: 0,
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
        self.com.write_read(ADDRESS, &[reg as u8], &mut data)?;
        Ok(data[0])
    }

    fn read_byte(&mut self, reg: u8) -> Result<u8, E> {
        let mut data: [u8; 1] = [0];
        // FIXME:
        //  * device address is not a const
        //  * register address is u16
        self.com.write_read(ADDRESS, &[reg], &mut data)?;
        Ok(data[0])
    }

    fn read_6bytes(&mut self, reg: Register) -> Result<[u8; 6], E> {
        let buffer: GenericArray<u8, U6> = self.read_registers(reg)?;
        // XXX: copy!
        let mut ret: [u8; 6] = Default::default();
        ret.copy_from_slice(buffer.as_slice());

        Ok(ret)
    }

    fn read_registers<N>(&mut self, reg: Register) -> Result<GenericArray<u8, N>, E>
    where
        N: ArrayLength<u8>,
    {
        let mut buffer: GenericArray<u8, N> = unsafe { mem::uninitialized() };

        {
            let buffer: &mut [u8] = &mut buffer;
            const I2C_AUTO_INCREMENT: u8 = 1 << 7;
            self.com
                .write_read(ADDRESS, &[(reg as u8) | I2C_AUTO_INCREMENT], buffer)?;
        }

        Ok(buffer)
    }

    fn read_16bit(&mut self, reg: Register) -> Result<u16, E> {
        let buffer: GenericArray<u8, U6> = self.read_registers(reg)?;
        Ok((u16(buffer[0]) << 8) + u16(buffer[1]))
    }

    fn write_byte(&mut self, reg: u8, byte: u8) -> Result<(), E> {
        let mut buffer = [0];
        self.com.write_read(ADDRESS, &[reg, byte], &mut buffer)
    }

    fn write_register(&mut self, reg: Register, byte: u8) -> Result<(), E> {
        let mut buffer = [0];
        self.com
            .write_read(ADDRESS, &[reg as u8, byte], &mut buffer)
    }

    fn write_6bytes(&mut self, reg: Register, bytes: [u8; 6]) -> Result<(), E> {
        let mut buf: [u8; 6] = [0, 0, 0, 0, 0, 0];
        self.com.write_read(
            ADDRESS,
            &[
                reg as u8, bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5],
            ],
            &mut buf,
        )
    }

    fn write_16bit(&mut self, reg: Register, word: u16) -> Result<(), E> {
        let mut buffer = [0];
        let msb = (word >> 8) as u8;
        let lsb = (word & 0xFF) as u8;
        self.com
            .write_read(ADDRESS, &[reg as u8, msb, lsb], &mut buffer)
    }

    fn write_32bit(&mut self, reg: Register, word: u32) -> Result<(), E> {
        let mut buffer = [0];
        let v1 = (word & 0xFF) as u8;
        let v2 = ((word >> 8) & 0xFF) as u8;
        let v3 = ((word >> 16) & 0xFF) as u8;
        let v4 = ((word >> 24) & 0xFF) as u8;
        self.com
            .write_read(ADDRESS, &[reg as u8, v1, v2, v3, v4], &mut buffer)
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
            let osc_calibrate_value = self.read_16bit(Register::OSC_CALIBRATE_VAL)?;

            if osc_calibrate_value != 0 {
                period_millis *= osc_calibrate_value as u32;
            }

            self.write_32bit(Register::SYSTEM_INTERMEASUREMENT_PERIOD, period_millis)?;
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

    /// readRangeContinuousMillimeters
    pub fn read_range_continuous_millimeters(&mut self) -> Result<u16, Error<E>> {
        let mut c = 0;
        while (self.read_register(Register::RESULT_INTERRUPT_STATUS)? & 0x07) == 0 {
            c += 1;
            if c == 65535 {
                return Err(Error::Timeout);
            }
        }
        let range_err = self.read_16bit(Register::RESULT_RANGE_STATUS_plus_10);
        // don't use ? to cleanup
        self.write_register(Register::SYSTEM_INTERRUPT_CLEAR, 0x01)?;

        Ok(range_err?)
    }

    /// readRangeSingleMillimeters
    pub fn read_range_single_millimeters(&mut self) -> Result<u16, Error<E>> {
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
            if c == 65535 {
                return Err(Error::Timeout);
            }
        }

        self.read_range_continuous_millimeters()
    }

    fn init_hardware(&mut self) -> Result<(), Error<E>> {
        // Enable the sensor

        self.power_on()?;
        // VL53L0X_DataInit() begin

        // Sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
        if self.io_mode2v8 {
            // set bit 0
            let ext_sup_hv = self.read_register(Register::VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV)?;
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
        let mut ref_spad_map = self.read_6bytes(Register::GLOBAL_CONFIG_SPAD_ENABLES_REF_0)?;

        // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

        self.write_byte(0xFF, 0x01)?;
        self.write_register(Register::DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)?;
        self.write_register(Register::DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)?;
        self.write_byte(0xFF, 0x00)?;
        self.write_register(Register::GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)?;

        // 12 is the first aperture spad
        let first_spad_to_enable = if spad_type_is_aperture > 0 { 12 } else { 0 };
        let mut spads_enabled: u8 = 0;

        for i in 0..48 {
            if i < first_spad_to_enable || spads_enabled == spad_count {
                // This bit is lower than the first one that should be enabled, or (reference_spad_count) bits have already been enabled, so zero this bit
                ref_spad_map[i / 8] &= !(1 << (i % 8));
            } else if (ref_spad_map[i / 8] >> (i % 8)) & 0x1 > 0 {
                spads_enabled = spads_enabled + 1;
            }
        }

        self.write_6bytes(Register::GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map)?;

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
        /*
        TODO
        self.measurement_timing_budget_microseconds = self.get_measurement_timing_budget();
        */
        // "Disable MSRC and TCC by default"
        // MSRC = Minimum Signal Rate Check
        // TCC = Target CentreCheck
        // -- VL53L0X_SetSequenceStepEnable() begin

        self.write_register(Register::SYSTEM_SEQUENCE_CONFIG, 0xE8)?;

        // -- VL53L0X_SetSequenceStepEnable() end

        // "Recalculate timing budget"
        // TODO
        // self.set_measurement_timing_budget(self.measurement_timing_budget_microseconds);

        // VL53L0X_StaticInit() end

        // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

        // -- VL53L0X_perform_vhv_calibration() begin

        self.write_register(Register::SYSTEM_SEQUENCE_CONFIG, 0x01)?;
        // TODO
        // if (!self.perform_single_ref_calibration(0x40)) {
        //     throw(std::runtime_error("Failed performing ref/vhv calibration!"));
        // }

        // -- VL53L0X_perform_vhv_calibration() end

        // -- VL53L0X_perform_phase_calibration() begin

        self.write_register(Register::SYSTEM_SEQUENCE_CONFIG, 0x02)?;
        // TODO:
        // if (!self.perform_single_ref_calibration(0x00)) {
        //     throw(std::runtime_error("Failed performing ref/phase calibration!"));
        // }

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
}
