/**
* 7Semi MLX90393 Arduino Library
*
* A lightweight, high-performance Arduino library for the Melexis MLX90393
* 3-axis magnetometer sensor.
*
* Features:
* - Read magnetic field (X, Y, Z) in microtesla (µT)
* - Configurable gain, resolution, OSR, and digital filter
* - ESP32 custom I2C pin support
*
* Supported Platforms:
* * ESP32
* * Arduino AVR (UNO, Nano, Mega)
* * Any board with TwoWire (I2C) support
*
* ---
* Copyright (c) 2026 7Semi
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
* ---
*
* Author: 7Semi
* Website: https://7semi.com
* Version: 1.0.0
*
* ---

*/
#ifndef _7SEMI_MLX90393_H_
#define _7SEMI_MLX90393_H_

#include <Arduino.h>
#include <Wire.h>

// Default I2C address
#define MLX90393_DEFAULT_ADDR 0x0C

// MLX90393 Commands
#define CMD_START_BURST_MODE 0x10
#define CMD_START_WAKE_UP_MODE 0x20
#define CMD_START_MEASUREMENT 0x30
#define CMD_READ_MEASUREMENT 0x40
#define CMD_READ_REG 0x50
#define CMD_WRITE_REG 0x60
#define CMD_EXIT_MODE 0x80
#define CMD_MEM_RECALL 0xD0
#define CMD_MEM_STORE 0xE0
#define CMD_RESET 0xF0
#define CMD_STORE 0x70

#define MLX90393_X_AXIS 0x02
#define MLX90393_Y_AXIS 0x04
#define MLX90393_Z_AXIS 0x08
#define MLX90393_AXIS_ALL 0x0E

// MLX90393 Register Map
#define MLX90393_REG_CONF1 0x00
#define MLX90393_REG_CONF2 0x01
#define MLX90393_REG_CONF3 0x02
#define MLX90393_REG_SENS_TC 0x03
#define MLX90393_REG_OFFSET_X 0x04
#define MLX90393_REG_OFFSET_Y 0x05
#define MLX90393_REG_OFFSET_Z 0x06
#define MLX90393_REG_THRESHOLD_XY 0x07
#define MLX90393_REG_THRESHOLD_Z 0x08
#define MLX90393_REG_THRESHOLD_T 0x09
#define MLX90393_REG_BURST_CTRL 0x0A
#define MLX90393_REG_HALL_CONF 0x0C
#define MLX90393_REG_BIST 0x0D
#define MLX90393_REG_TRIG_INT 0x0E

// CONF1 register bits
#define MLX90393_GAIN_POSITION 4
#define MLX90393_Z_SERIES_POSITION 7
#define MLX90393_ANA_RESERVED_LOW_POSITION 9

// CONF2 register bits
#define MLX90393_BURST_SEL_POSITION 6
#define MLX90393_BURST_DATA_RATE_POSITION 0
#define MLX90393_TRIG_INT_POSITION 15
#define MLX90393_TCMP_EN_POSITION 10
#define MLX90393_EXT_TRIG_POSITION 11

// CONF3 register bits
#define MLX90393_OSR_POSITION 0
#define MLX90393_FILTER_POSITION 2
#define MLX90393_RES_X_POSITION 5
#define MLX90393_RES_Y_POSITION 7
#define MLX90393_RES_Z_POSITION 9
#define MLX90393_OSR2_POSITION 11

#define MLX90393_GAIN_BITS 3
#define MLX90393_Z_SERIES_BITS 1
#define MLX90393_ANA_RESERVED_LOW_BITS 7
#define MLX90393_BURST_DATA_RATE_BITS 5
#define MLX90393_BURST_SEL_BITS 3
#define MLX90393_TRIG_INT_BITS 1
#define MLX90393_EXT_TRIG_BITS 1
#define MLX90393_TCMP_EN_BITS 1
#define MLX90393_RES_BITS 2
#define MLX90393_OSR_BITS 2
#define MLX90393_FILTER_BITS 3
#define MLX90393_STATUS_DRDY 0x01
#define MLX90393_STATUS_ERROR 0x10

// Resolution selection enum
enum MLX90393_Resolution
{
    MLX90393_RES_16BIT = 0,
    MLX90393_RES_17BIT = 1,
    MLX90393_RES_18BIT = 2,
    MLX90393_RES_19BIT = 3
};

// Gain selection enum
enum MLX90393_Gain
{
    MLX90393_GAIN_0 = 0,
    MLX90393_GAIN_1 = 1,
    MLX90393_GAIN_2 = 2,
    MLX90393_GAIN_3 = 3,
    MLX90393_GAIN_4 = 4,
    MLX90393_GAIN_5 = 5,
    MLX90393_GAIN_6 = 6,
    MLX90393_GAIN_7 = 7
};

// Oversampling ratio (OSR)
enum MLX90393_OSR
{
    MLX90393_OSR_0 = 0,
    MLX90393_OSR_1 = 1,
    MLX90393_OSR_2 = 2,
    MLX90393_OSR_3 = 3
};

// Digital filter setting
enum MLX90393_Filter
{
    MLX90393_FILTER_0 = 0,
    MLX90393_FILTER_1 = 1,
    MLX90393_FILTER_2 = 2,
    MLX90393_FILTER_3 = 3,
    MLX90393_FILTER_4 = 4,
    MLX90393_FILTER_5 = 5,
    MLX90393_FILTER_6 = 6,
    MLX90393_FILTER_7 = 7
};

class MLX90393_7Semi
{
public:
    MLX90393_7Semi();

    /**
     * Initialize MLX90393 sensor with custom pins (ESP32 specific)
     * - Allows custom SDA and SCL pin assignment
     * - Pass 255 to use default pins
     */
    bool begin(uint8_t i2cAddress = 0x0C,
               TwoWire &i2cPort = Wire,
               uint32_t i2cClock = 100000,
               uint8_t sda = 255,
               uint8_t scl = 255);

    /**
     * Perform soft reset
     * - Resets internal configuration registers
     * - Sensor returns to default state
     */
    bool softReset();

    /**
     * Exit measurement / burst mode
     * - Required before writing configuration
     * - Ensures safe register access
     */
    bool exitMode();

    /**
     * Start single measurement
     * - axisMask defines which axes are measured
     * - Example: MLX90393_AXIS_ALL
     */
    bool startMeasurement(uint8_t axisMask);

    /**
     * Start continuous (burst) mode
     * - Sensor continuously updates data
     */
    bool startContinuous();

    /**
     * Compute conversion delay
     * - Based on OSR and digital filter settings
     * - Used when DRDY polling is not used
     */
    bool getConversionDelay(uint8_t &conversionDelay);

    /**
     * Wait until data is ready
     * - Polls DRDY bit from status register
     * - Returns false on timeout
     */
    bool waitDataReady(uint16_t timeout = 50);

    /**
     * Read sensor status byte
     * - Provides DRDY, ERROR flags
     */
    bool readStatus(uint8_t &status);

    /**
     * Set sensor gain
     * - Controls sensitivity and measurement range
     */
    bool setGain(MLX90393_Gain gain);

    /**
     * Get current gain setting
     * - Returns gain index (0–7)
     */
    bool getGain(uint8_t &gain);

    /**
     * Enable / disable Z-axis series mode
     * - true  → enable
     * - false → disable
     */
    bool enableSeriesZ(bool enable);

    /**
     * Get reserved ANA bits (read-only)
     */
    bool getANAReservedLow(uint8_t &value);

    /**
     * Set burst axis selection
     */
    bool setBurstSel(uint8_t sel);

    /**
     * Get burst axis selection
     */
    bool getBurstSel(uint8_t &sel);

    /**
     * Set burst data rate
     */
    bool setBurstDataRate(uint8_t bdr);

    /**
     * Get burst data rate
     */
    bool getBurstDataRate(uint8_t &bdr);

    /**
     * Configure TRIG/INT pin mode
     */
    bool setInterruptMode(bool interruptMode);

    /**
     * Enable / disable temperature compensation
     */
    bool setTemperatureCompensation(bool state);

    /**
     * Get temperature compensation state
     */
    bool getTemperatureCompensation(bool &enabled);

    /**
     * Enable / disable external trigger
     */
    bool setExternalTrigger(bool state);

    /**
     * Get external trigger state
     */
    bool getExternalTrigger(bool &enabled);

    /**
     * Set resolution for X, Y, Z axes
     */
    bool setResolution(MLX90393_Resolution x,
                       MLX90393_Resolution y,
                       MLX90393_Resolution z);

    /**
     * Get resolution for X, Y, Z axes
     */
    bool getResolution(MLX90393_Resolution &res_x,
                       MLX90393_Resolution &res_y,
                       MLX90393_Resolution &res_z);

    /**
     * Set digital filter
     */
    bool setFilter(MLX90393_Filter filter);

    /**
     * Get digital filter
     */
    bool getFilter(uint8_t &filter);

    /**
     * Set oversampling ratio (OSR)
     */
    bool setOSR(MLX90393_OSR sampling);

    /**
     * Get oversampling ratio (OSR)
     */
    bool getOSR(uint8_t &sampling);

    /**
     * Set temperature compensation sensitivity factor
     */
    bool setSensitivityFactorTC(uint8_t factor);

    /**
     * Get temperature compensation sensitivity factor
     */
    bool getSensitivityFactorTC(uint16_t &factor);

    /**
     * Set offset values for X, Y, Z axes
     */
    bool setOffset(int16_t offset_x,
                   int16_t offset_y,
                   int16_t offset_z);

    /**
     * Get offset values for X, Y, Z axes
     */
    bool getOffset(int16_t &offset_x,
                   int16_t &offset_y,
                   int16_t &offset_z);

    /**
     * Set threshold values
     */
    bool setThreshold(uint16_t threshold_xy,
                      uint16_t threshold_z,
                      uint16_t threshold_tot);

    /**
     * Get threshold values
     */
    bool getThreshold(uint16_t &threshold_xy,
                      uint16_t &threshold_z,
                      uint16_t &threshold_tot);

    /**
     * Read raw magnetic data (X, Y, Z)
     */
    bool readMagRaw(int16_t &x, int16_t &y, int16_t &z);

    /**
     * Read magnetic field in µT
     */
    bool readMag(float &x, float &y, float &z);

    /**
     * Get sensitivity factor
     */
    bool getSensitivity(float &sens_x, float &sens_y, float &sens_z);

private:
    TwoWire *i2c;
    uint8_t address;

    uint8_t conversion_delay = 5;
    uint8_t resolution_x = 0;
    uint8_t resolution_y = 0;
    uint8_t resolution_z = 0;

    /**
     * Cached sensitivity (µT/LSB)
     */
    float sensitivity_x = 0.751;
    float sensitivity_y = 0.751;
    float sensitivity_z = 1.210;

    /**
     * Read magnetic field (µT)
     */
    bool readData(float &x, float &y, float &z);

    /**
     * Read 16-bit register
     */
    bool readReg(uint8_t reg, uint16_t &value);

    /**
     * Write 16-bit register
     */
    bool writeReg(uint8_t reg, uint16_t value);

    /**
     * Send command to sensor
     */
    bool sendCommand(uint8_t cmd);

    /**
     * Read bits from register
     */
    bool readBits(uint8_t reg, uint8_t pos, uint8_t len, uint8_t &value);

    /**
     * Write bits to register
     */
    bool writeBits(uint8_t reg, uint8_t pos, uint8_t len, uint8_t value);
};

#endif
