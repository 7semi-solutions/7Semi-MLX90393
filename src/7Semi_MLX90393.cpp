#include "7Semi_MLX90393.h"

MLX90393_7Semi::MLX90393_7Semi() {}
/**
 * Base sensitivity x y  (RES = 0)
 * Units: µT/LSB
 */
const float base_xy[8] =
{
    0.751f, 0.601f, 0.451f, 0.376f,
    0.300f, 0.250f, 0.200f, 0.150f
};
/**
 * Base sensitivity z (RES = 0)
 * Units: µT/LSB
 */
const float base_z[8] =
{
    1.210f, 0.968f, 0.726f, 0.605f,
    0.484f, 0.403f, 0.323f, 0.242f
};

const float base_conv_delay[8] =
{
    1.27f, 1.46f, 1.84f, 2.61f,
    4.15f, 7.22f, 13.36f, 25.65f
};
bool MLX90393_7Semi::begin(uint8_t i2cAddress, TwoWire &i2cPort, uint32_t i2cClock, uint8_t sda, uint8_t scl)
{
    i2c = &i2cPort;
    address = i2cAddress;

#ifdef ARDUINO_ARCH_ESP32
    /**
     * Initialize I2C (ESP32 specific)
     * - If SDA/SCL provided → use custom pins
     * - If not → use default Wire pins
     */
    if (sda != 255 && scl != 255)
    {
        i2c->begin(sda, scl);
    }
    else
    {
        i2c->begin();
    }
#else
    /**
     * Initialize I2C (Other MCUs)
     * - Uses default Wire pins
     */
    i2c->begin();
#endif

    /**
     * Set I2C clock speed
     * - Typical: 100kHz / 400kHz
     */
    i2c->setClock(i2cClock);

    /**
     * Probe device on I2C bus
     * - Returns false if device not responding
     */
    i2c->beginTransmission(address);
    if (i2c->endTransmission(false) != 0)
        return false;

    /**
     * Ensure sensor is out of any active mode
     */
    if (!exitMode())
        return false;

    delay(10);

    /**
     * Perform soft reset
     * - Clears configuration
     * - Brings sensor to known state
     */
    if (!softReset())
        return false;

    delay(10);

    return true;
}

/**
 * Perform soft reset
 * - Sends reset command to sensor
 * - Clears configuration registers
 * - Sensor returns to default state
 */
bool MLX90393_7Semi::softReset()
{
    return sendCommand(CMD_RESET);
}

/**
 * Exit measurement / burst mode
 * - Required before accessing registers
 * - Ensures sensor is in idle state
 */
bool MLX90393_7Semi::exitMode()
{
    return sendCommand(CMD_EXIT_MODE);
}

/**
 * Start single measurement
 * - axisMask defines which axes to measure (X/Y/Z)
 * - Example: MLX90393_AXIS_ALL
 */
bool MLX90393_7Semi::startMeasurement(uint8_t axisMask)
{
    uint8_t cmd = CMD_START_MEASUREMENT | axisMask;
    return sendCommand(cmd);
}

/**
 * Start continuous (burst) mode
 * - Sensor continuously measures data
 * - Data can be read without triggering each time
 */
bool MLX90393_7Semi::startContinuous()
{
    return sendCommand(CMD_START_BURST_MODE);
}

/**
 * Compute measurement delay
 * - Depends on OSR and digital filter
 * - Higher values → slower but more accurate readings
 */
bool MLX90393_7Semi::getConversionDelay(uint8_t &conversionDelay)
{
    uint8_t filter, osr;

    if (!getFilter(filter))
        return false;

    if (!getOSR(osr))
        return false;
    if (osr + filter < 2)
    {
        if (!setFilter(MLX90393_FILTER_2))
            return false;
        if (!setOSR(MLX90393_OSR_1))
            return false;
        filter = osr = 1;
    }
    conversionDelay = (uint16_t)(base_conv_delay[filter] * (1 << osr) + 1.0f);
    return true;
}

/**
 * Wait for data ready (DRDY)
 * - Polls status register until data is ready
 * - Returns false if timeout occurs
 */
bool MLX90393_7Semi::waitDataReady(uint16_t timeout)
{
    uint32_t start = millis();

    while ((millis() - start) < timeout)
    {
        uint8_t status;

        /**
         * Read status byte
         * - If communication fails → exit
         */
        if (!readStatus(status))
            return false;

        /**
         * Check DRDY bit (bit 0)
         * - 1 → data ready
         * - 0 → still converting
         */
        if (status & MLX90393_STATUS_DRDY)
            return true;
    }

    /**
     * Timeout occurred
     */
    return false;
}

/**
 * Read status byte from sensor
 * - Returns 1 byte status
 * - Contains DRDY, ERROR flags, etc.
 */
bool MLX90393_7Semi::readStatus(uint8_t &status)
{
    /**
     * Request 1 byte from sensor
     */
    if (i2c->requestFrom(address, (uint8_t)1) != 1)
        return false;

    /**
     * Read status value
     */
    status = i2c->read();

    return true;
}

/**
 * Set sensor gain
 * - Controls measurement sensitivity
 * - Higher gain → higher sensitivity, lower range
 * - Lower gain → lower sensitivity, higher range
 *
 * Register:
 * - CONF1
 * - Bits: [7:5]
 */
bool MLX90393_7Semi::setGain(MLX90393_Gain gain)
{
    if (!writeBits(MLX90393_REG_CONF1,
                   MLX90393_GAIN_POSITION,
                   MLX90393_GAIN_BITS,
                   (uint8_t)gain))
        return false;
    if (!getSensitivity(sensitivity_x, sensitivity_y, sensitivity_z))
        return false;
    return true;
}

/**
 * Get current sensor gain
 * - Reads gain setting from CONF1 register
 * - Returns raw gain index (0–7)
 */
bool MLX90393_7Semi::getGain(uint8_t &gain)
{
    return readBits(MLX90393_REG_CONF1,
                    MLX90393_GAIN_POSITION,
                    MLX90393_GAIN_BITS,
                    gain);
}

/**
 * Enable / Disable Z-axis series mode
 * - Affects Z-axis measurement configuration
 *
 * true  → Series mode enabled
 * false → Series mode disabled
 */
bool MLX90393_7Semi::enableSeriesZ(bool enable)
{
    return writeBits(MLX90393_REG_CONF1,
                     MLX90393_Z_SERIES_POSITION,
                     MLX90393_Z_SERIES_BITS,
                     (uint8_t)enable);
}

/**
 * Get ANA reserved low bits
 * - Internal factory calibration bits
 *
 * Register:
 * - CONF1
 */
bool MLX90393_7Semi::getANAReservedLow(uint8_t &value)
{
    return readBits(MLX90393_REG_CONF1,
                    MLX90393_ANA_RESERVED_LOW_POSITION,
                    MLX90393_ANA_RESERVED_LOW_BITS,
                    value);
}

/**
 * Set burst axis selection
 * - Defines which axes are measured in burst mode
 * - Value range: 0–7
 */
bool MLX90393_7Semi::setBurstSel(uint8_t sel)
{
    sel &= 0x07;

    return writeBits(MLX90393_REG_CONF2,
                     MLX90393_BURST_SEL_POSITION,
                     MLX90393_BURST_SEL_BITS,
                     sel);
}

/**
 * Get burst axis selection
 */
bool MLX90393_7Semi::getBurstSel(uint8_t &sel)
{
    uint8_t value;

    if (!readBits(MLX90393_REG_CONF2,
                  MLX90393_BURST_SEL_POSITION,
                  MLX90393_BURST_SEL_BITS,
                  value))
        return false;

    sel = value;
    return true;
}

/**
 * Set burst data rate
 * - Controls measurement frequency in burst mode
 */
bool MLX90393_7Semi::setBurstDataRate(uint8_t bdr)
{
    return writeBits(MLX90393_REG_CONF2,
                     MLX90393_BURST_DATA_RATE_POSITION,
                     MLX90393_BURST_DATA_RATE_BITS,
                     (uint8_t)bdr);
}

/**
 * Get burst data rate
 */
bool MLX90393_7Semi::getBurstDataRate(uint8_t &bdr)
{
    return readBits(MLX90393_REG_CONF2,
                    MLX90393_BURST_DATA_RATE_POSITION,
                    MLX90393_BURST_DATA_RATE_BITS,
                    bdr);
}

/**
 * Configure TRIG/INT pin mode
 *
 * true  → Interrupt mode (INT)
 * false → Trigger input mode (TRIG)
 */
bool MLX90393_7Semi::setInterruptMode(bool interruptMode)
{
    return writeBits(MLX90393_REG_CONF2,
                     MLX90393_TRIG_INT_POSITION,
                     MLX90393_TRIG_INT_BITS,
                     (uint8_t)interruptMode);
}

/**
 * Enable / disable temperature compensation
 * - Improves accuracy over temperature variation
 */
bool MLX90393_7Semi::setTemperatureCompensation(bool state)
{
    return writeBits(MLX90393_REG_CONF2,
                     MLX90393_TCMP_EN_POSITION,
                     MLX90393_TCMP_EN_BITS,
                     (uint8_t)state);
}

/**
 * Get temperature compensation state
 */
bool MLX90393_7Semi::getTemperatureCompensation(bool &enabled)
{
    uint8_t value;

    if (!readBits(MLX90393_REG_CONF2,
                  MLX90393_TCMP_EN_POSITION,
                  MLX90393_TCMP_EN_BITS,
                  value))
        return false;

    enabled = (value != 0);
    return true;
}

/**
 * Enable / disable external trigger mode
 * - Allows measurement triggered by external pin
 */
bool MLX90393_7Semi::setExternalTrigger(bool state)
{
    return writeBits(MLX90393_REG_CONF2,
                     MLX90393_EXT_TRIG_POSITION,
                     MLX90393_EXT_TRIG_BITS,
                     (uint8_t)state);
}

/**
 * Get external trigger state
 */
bool MLX90393_7Semi::getExternalTrigger(bool &enabled)
{
    uint8_t value;

    if (!readBits(MLX90393_REG_CONF2,
                  MLX90393_EXT_TRIG_POSITION,
                  MLX90393_EXT_TRIG_BITS,
                  value))
        return false;

    enabled = (value != 0);
    return true;
}

/**
 * Set resolution for X, Y, Z axes
 * - Higher resolution → better precision, slower conversion
 * - Lower resolution → faster, less precise
 *
 * Register:
 * - CONF3
 * - Bits:
 *   - X: [1:0]
 *   - Y: [3:2]
 *   - Z: [5:4]
 */
bool MLX90393_7Semi::setResolution(MLX90393_Resolution x,
                                   MLX90393_Resolution y,
                                   MLX90393_Resolution z)
{
    uint16_t val;

    if (!readReg(MLX90393_REG_CONF3, val))
        return false;

    // Clear all resolution bits at once
    val &= ~(
        (0x03 << MLX90393_RES_X_POSITION) |
        (0x03 << MLX90393_RES_Y_POSITION) |
        (0x03 << MLX90393_RES_Z_POSITION));

    // Set new values
    val |= ((uint16_t)x << MLX90393_RES_X_POSITION);
    val |= ((uint16_t)y << MLX90393_RES_Y_POSITION);
    val |= ((uint16_t)z << MLX90393_RES_Z_POSITION);

    if (!writeReg(MLX90393_REG_CONF3, val))
        return false;

    if (!getSensitivity(sensitivity_x, sensitivity_y, sensitivity_z))
        return false;
    resolution_x = x;
    resolution_y = y;
    resolution_z = z;

    return true;
}

/**
 * Get resolution for X, Y, Z axes
 */
bool MLX90393_7Semi::getResolution(MLX90393_Resolution &res_x,
                                   MLX90393_Resolution &res_y,
                                   MLX90393_Resolution &res_z)
{
    uint16_t reg_val;

    if (!readReg(MLX90393_REG_CONF3, reg_val))
        return false;

    res_x = (MLX90393_Resolution)((reg_val >> MLX90393_RES_X_POSITION) & 0x03);
    res_y = (MLX90393_Resolution)((reg_val >> MLX90393_RES_Y_POSITION) & 0x03);
    res_z = (MLX90393_Resolution)((reg_val >> MLX90393_RES_Z_POSITION) & 0x03);

    return true;
}

/**
 * Set digital filter
 * - Reduces noise in measurements
 * - Higher value → smoother data, slower response
 */
bool MLX90393_7Semi::setFilter(MLX90393_Filter filter)
{
    if (!writeBits(MLX90393_REG_CONF3,
                   MLX90393_FILTER_POSITION,
                   MLX90393_FILTER_BITS,
                   (uint8_t)filter))
        return false;

    /**
     * Store internally for timing calculation
     */
    if (!getConversionDelay(conversion_delay))
        return false;

    return true;
}

/**
 * Get digital filter setting
 */
bool MLX90393_7Semi::getFilter(uint8_t &filter)
{
    return readBits(MLX90393_REG_CONF3,
                    MLX90393_FILTER_POSITION,
                    MLX90393_FILTER_BITS,
                    filter);
}

/**
 * Set oversampling ratio (OSR)
 * - Controls measurement averaging
 * - Higher OSR → better accuracy, slower conversion
 */
bool MLX90393_7Semi::setOSR(MLX90393_OSR osr)
{
    if (!writeBits(MLX90393_REG_CONF3,
                   MLX90393_OSR_POSITION,
                   MLX90393_OSR_BITS,
                   (uint8_t)osr))
        return false;

    /**
     * Store internally for delay calculation
     */
    if (!getConversionDelay(conversion_delay))
        return false;

    return true;
}

/**
 * Get oversampling ratio (OSR)
 */
bool MLX90393_7Semi::getOSR(uint8_t &osr)
{
    return readBits(MLX90393_REG_CONF3,
                    MLX90393_OSR_POSITION,
                    MLX90393_OSR_BITS,
                    osr);
}

/**
 * Set temperature compensation sensitivity factor
 * - Adjusts sensitivity over temperature variation
 * - Value is device-specific (from datasheet or calibration)
 *
 * Register:
 * - SENS_TC
 */
bool MLX90393_7Semi::setSensitivityFactorTC(uint8_t factor)
{
    return writeReg(MLX90393_REG_SENS_TC, factor);
}

/**
 * Get temperature compensation sensitivity factor
 */
bool MLX90393_7Semi::getSensitivityFactorTC(uint16_t &factor)
{
    return readReg(MLX90393_REG_SENS_TC, factor);
}

/**
 * Set offset for X, Y, Z axes
 * - Used for calibration (zero-field correction)
 * - Values are signed (2's complement)
 *
 * Example:
 * - After calibration → store offsets here
 */
bool MLX90393_7Semi::setOffset(int16_t offset_x,
                               int16_t offset_y,
                               int16_t offset_z)
{
    /**
     * Write signed offsets (cast to uint16_t for register)
     */
    if (!writeReg(MLX90393_REG_OFFSET_X, (uint16_t)offset_x))
        return false;

    if (!writeReg(MLX90393_REG_OFFSET_Y, (uint16_t)offset_y))
        return false;

    if (!writeReg(MLX90393_REG_OFFSET_Z, (uint16_t)offset_z))
        return false;

    return true;
}

/**
 * Get offset values for X, Y, Z axes
 * - Converts register values back to signed integers
 */
bool MLX90393_7Semi::getOffset(int16_t &offset_x,
                               int16_t &offset_y,
                               int16_t &offset_z)
{
    uint16_t val;

    /**
     * Read X offset
     */
    if (!readReg(MLX90393_REG_OFFSET_X, val))
        return false;
    offset_x = (int16_t)val;

    /**
     * Read Y offset
     */
    if (!readReg(MLX90393_REG_OFFSET_Y, val))
        return false;
    offset_y = (int16_t)val;

    /**
     * Read Z offset
     */
    if (!readReg(MLX90393_REG_OFFSET_Z, val))
        return false;
    offset_z = (int16_t)val;

    return true;
}

/**
 * Set threshold values
 * - Used for interrupt / event triggering
 *
 * threshold_xy → combined X/Y threshold
 * threshold_z  → Z-axis threshold
 * threshold_tot → total magnetic field threshold
 */
bool MLX90393_7Semi::setThreshold(uint16_t threshold_xy,
                                  uint16_t threshold_z,
                                  uint16_t threshold_tot)
{
    if (!writeReg(MLX90393_REG_THRESHOLD_XY, threshold_xy))
        return false;

    if (!writeReg(MLX90393_REG_THRESHOLD_Z, threshold_z))
        return false;

    if (!writeReg(MLX90393_REG_THRESHOLD_T, threshold_tot))
        return false;

    return true;
}

/**
 * Get threshold values
 */
bool MLX90393_7Semi::getThreshold(uint16_t &threshold_xy,
                                  uint16_t &threshold_z,
                                  uint16_t &threshold_tot)
{
    uint16_t val;

    if (!readReg(MLX90393_REG_THRESHOLD_XY, val))
        return false;
    threshold_xy = val;

    if (!readReg(MLX90393_REG_THRESHOLD_Z, val))
        return false;
    threshold_z = val;

    if (!readReg(MLX90393_REG_THRESHOLD_T, val))
        return false;
    threshold_tot = val;

    return true;
}

/**
 * Read raw magnetic data (X, Y, Z)
 * - Performs single measurement
 * - Returns signed 16-bit values
 */
bool MLX90393_7Semi::readMagRaw(int16_t &x, int16_t &y, int16_t &z)
{
    // if(!exitMode())
    // return false;
    /**
     * Start measurement for all axes
     */
    if (!startMeasurement(MLX90393_AXIS_ALL))
        return false;

    /**
     * Wait for conversion
     * - Uses dynamic delay based on OSR + filter
     */
    delay(conversion_delay);
    /**
     * Send read measurement command
     */
    i2c->beginTransmission(address);
    i2c->write(CMD_READ_MEASUREMENT | MLX90393_AXIS_ALL);

    if (i2c->endTransmission(true) != 0)
        return false;

    delay(2);

    /**
     * Read response (status + XYZ)
     */
    uint8_t rx[7];
    if (i2c->requestFrom(address, (uint8_t)7) != 7)
        return false;

    for (uint8_t i = 0; i < 7; i++)
        rx[i] = i2c->read();

    /**
     * Check status byte
     * - Bit 4 = error
     */
 
    if (rx[0] & MLX90393_STATUS_ERROR)
        return false;
    /**
     * Convert to signed values
     */
    z = (int16_t)((rx[1] << 8) | rx[2]);
    y = (int16_t)((rx[3] << 8) | rx[4]);
    x = (int16_t)((rx[5] << 8) | rx[6]);
    if (resolution_x == MLX90393_RES_18BIT)
        x -= 0x8000;
    if (resolution_x == MLX90393_RES_19BIT)
        x -= 0x4000;

    if (resolution_y == MLX90393_RES_18BIT)
        y -= 0x8000;
    if (resolution_y == MLX90393_RES_19BIT)
        y -= 0x4000;

    if (resolution_z == MLX90393_RES_18BIT)
        z -= 0x8000;
    if (resolution_z == MLX90393_RES_19BIT)
        z -= 0x4000;

    return true;
}

/**
 * Read magnetic field in microtesla (µT)
 * - Fully corrected for gain + resolution + axis
 */
bool MLX90393_7Semi::readMag(float &x, float &y, float &z)
{
    int16_t rx, ry, rz;

    if (!readMagRaw(rx, ry, rz))
        return false;

    x = rx * sensitivity_x; // µT
    y = ry * sensitivity_y; // µT
    z = rz * sensitivity_z; // µT

    return true;
}

/**
 * Get sensitivity (µT/LSB)
 * - Formula-based (datasheet derived)
 * - Uses base sensitivity at RES=0
 */
bool MLX90393_7Semi::getSensitivity(float &sens_x, float &sens_y, float &sens_z)
{
    uint8_t gain;
    MLX90393_Resolution res_x, res_y, res_z;

    if (!getGain(gain) || !getResolution(res_x, res_y, res_z))
    {
        return false;
    }

    uint8_t g = gain & 0x07;

    /**
     * Resolution scaling
     * - Each step doubles sensitivity
     */
    float scale_x = (float)(1 << res_x);
    float scale_y = (float)(1 << res_y);
    float scale_z = (float)(1 << res_z);

    sensitivity_x = base_xy[g] * scale_x;
    sensitivity_y = base_xy[g] * scale_y;
    sensitivity_z = base_z[g] * scale_z;
    return true;
}

/**
 * Send command to sensor
 * - Returns false if I2C or sensor error
 */
bool MLX90393_7Semi::sendCommand(uint8_t cmd)
{
    i2c->beginTransmission(address);
    i2c->write(cmd);

    /**
     * I2C transmission check
     */
    if (i2c->endTransmission(true) != 0)
        return false;

    /**
     * Read status byte
     */
    if (i2c->requestFrom(address, (uint8_t)1) != 1)
        return false;

    uint8_t status = i2c->read();

    /**
     * Check error bit
     */
    if (status & MLX90393_STATUS_ERROR)
        return false;

    return true;
}

/**
 * Read 16-bit register
 * - Uses MLX90393 RR command
 */
bool MLX90393_7Semi::readReg(uint8_t reg, uint16_t &value)
{
    /**
     * Ensure sensor is not in measurement mode
     */
    if (!exitMode())
        return false;

    i2c->beginTransmission(address);
    i2c->write(CMD_READ_REG);
    i2c->write(reg << 2);

    if (i2c->endTransmission(true) != 0)
        return false;

    delay(2);

    /**
     * Read STATUS + DATA (3 bytes)
     */
    if (i2c->requestFrom(address, (uint8_t)3) != 3)
        return false;

    uint8_t status = i2c->read();
    uint8_t msb = i2c->read();
    uint8_t lsb = i2c->read();

    /**
     * Check error flag
     */
    if (status & MLX90393_STATUS_ERROR)
        return false;

    value = (msb << 8) | lsb;

    if (!exitMode())
        return false;

    return true;
}

/**
 * Write 16-bit register
 */
bool MLX90393_7Semi::writeReg(uint8_t reg, uint16_t value)
{
    /**
     * Ensure safe register access
     */
    if (!exitMode())
        return false;

    i2c->beginTransmission(address);
    i2c->write(CMD_WRITE_REG);
    i2c->write(value >> 8);
    i2c->write(value & 0xFF);
    i2c->write(reg << 2);

    if (i2c->endTransmission(true) != 0)
        return false;

    if (i2c->requestFrom(address, (uint8_t)1) != 1)
        return false;
    uint8_t status = i2c->read();

    if (status & MLX90393_STATUS_ERROR)
        return false;
    if (!exitMode())
        return false;
    return true;
}

/**
 * Read specific bits from register
 */
bool MLX90393_7Semi::readBits(uint8_t reg, uint8_t pos, uint8_t len, uint8_t &value)
{
    uint16_t reg_val;

    if (!readReg(reg, reg_val))
        return false;

    uint16_t mask = ((1 << len) - 1);

    value = (reg_val >> pos) & mask;

    return true;
}

/**
 * Write specific bits to register
 */
bool MLX90393_7Semi::writeBits(uint8_t reg, uint8_t pos, uint8_t len, uint8_t value)
{
    uint16_t reg_val;

    if (!readReg(reg, reg_val))
        return false;

    uint16_t mask = ((1 << len) - 1);

    /**
     * Clear target bits
     */
    reg_val &= ~(mask << pos);

    /**
     * Set new value
     */
    reg_val |= ((value & mask) << pos);

    return writeReg(reg, reg_val);
}