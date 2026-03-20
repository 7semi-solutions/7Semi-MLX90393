# 7Semi MLX90393 Arduino Library

![Arduino](https://img.shields.io/badge/platform-Arduino-blue.svg)
![Sensor](https://img.shields.io/badge/sensor-MLX90393-green.svg)
![License](https://img.shields.io/badge/license-MIT-orange.svg)

Arduino driver for the **MLX90393 3-axis magnetometer sensor**.

The **MLX90393** is a high-performance magnetic field sensor with an **I²C interface**, capable of measuring magnetic field strength in **X, Y, and Z axes**. It is suitable for applications like compasses, position sensing, and industrial measurements.

This library provides a simple interface for configuring the sensor and reading magnetic field values in **microtesla (µT)**.

---

# Features

* 3-axis magnetic field measurement (X, Y, Z)
* Raw data reading
* Configurable gain
* Resolution control (16–19 bit)
* Oversampling (OSR) configuration
* Digital filter configuration
* Offset calibration support
* Register-level access (advanced users)
* ESP32 custom I²C pin support

---

# Supported Platforms

* Arduino UNO / Mega
* ESP32
* Any board supporting the **Wire (I²C) library**

---

# Hardware

Supported sensor:

**7Semi 3-Axis Magnetometer I2C Breakout - MLX90393**

---

# Connection

The **MLX90393 communicates using I²C**.

## I²C Connection

| MLX90393 Pin | MCU Pin | Description  |
| ------------ | ------- | ------------ |
| VCC          | 3.3V    | Sensor power |
| GND          | GND     | Ground       |
| SCL          | SCL     | I²C clock    |
| SDA          | SDA     | I²C data     |

### I²C Notes

Default sensor address:

```
0x0C
```

Recommended I²C speed:

```
100 kHz – 400 kHz
```

---

# Installation

## Arduino Library Manager

1. Open **Arduino IDE**
2. Go to **Library Manager**
3. Search for **7Semi MLX90393**
4. Click **Install**

---

## Manual Installation

1. Download this repository as ZIP
2. Arduino IDE → **Sketch → Include Library → Add .ZIP Library**

---

# Example

```
#include <7Semi_MLX90393.h>

MLX90393_7Semi mag;

void setup()
{
  Serial.begin(115200);

  if(!mag.begin())
  {
    Serial.println("MLX90393 not detected");
    while(1);
  }
}

void loop()
{
  float x, y, z;

  if(mag.readMag(x, y, z))
  {
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Z: ");
    Serial.println(z);
  }

  delay(500);
}
```

---

# Library Overview

## Reading Magnetic Field (µT)

```
float x, y, z;

mag.readMag(x, y, z);

Serial.print("X: ");
Serial.print(x);
Serial.print(" Y: ");
Serial.print(y);
Serial.print(" Z: ");
Serial.println(z);
```

Returns magnetic field values in **microtesla (µT)**.

---

## Reading Raw Data

```
int16_t x, y, z;

mag.readMagRaw(x, y, z);

Serial.print("Raw X: ");
Serial.println(x);
```

Returns **raw ADC values**.

---

# Sensor Configuration

## Gain

```
mag.setGain(MLX90393_GAIN_0_751);
```

Available gain settings:

```
MLX90393_GAIN_0_751
MLX90393_GAIN_0_601
MLX90393_GAIN_0_451
MLX90393_GAIN_0_376
MLX90393_GAIN_0_300
MLX90393_GAIN_0_250
MLX90393_GAIN_0_200
MLX90393_GAIN_0_150
```

---

## Resolution

```
mag.setResolution(
  MLX90393_RES_19BIT,
  MLX90393_RES_19BIT,
  MLX90393_RES_19BIT);
```

Resolution options:

```
MLX90393_RES_16BIT
MLX90393_RES_17BIT
MLX90393_RES_18BIT
MLX90393_RES_19BIT
```

Higher resolution increases **precision** but reduces **speed**.

---

## Oversampling (OSR)

```
mag.setOSR(MLX90393_OSR_3);
```

Available OSR settings:

```
MLX90393_OSR_0
MLX90393_OSR_1
MLX90393_OSR_2
MLX90393_OSR_3
```

Higher OSR improves **accuracy** but slows down measurements.

---

## Digital Filter

```
mag.setFilter(MLX90393_FILTER_7);
```

Available filter settings:

```
MLX90393_FILTER_0 to MLX90393_FILTER_7
```

Higher filter values provide **smoother readings**.

---

# Offset Calibration

```
mag.setOffset(10, -5, 3);
```

Used for **zero-field calibration**.

---

# Example Applications

Typical applications include:

* Digital compass
* Position sensing
* Magnetic field detection
* Industrial monitoring
* Robotics orientation
* Automotive sensing

---

# License

MIT License

---

# Author

7Semi
