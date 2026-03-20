/**
* 7Semi MLX90393 Example
*
* * 7Semi MLX90393 Arduino Library
* * Supports magnetic field (X, Y, Z) measurement in µT
*
* Example Features:
* * Basic sensor initialization
* * Magnetic field reading (X, Y, Z)
*
* Hardware Connection (I2C):
* * VCC → 3.3V
* * GND → GND
* * SDA → SDA pin (ESP32: GPIO21, UNO: A4)
* * SCL → SCL pin (ESP32: GPIO22, UNO: A5)
*
* Default I2C Address:
* * 0x0C
*
* Author: 7Semi
*/

#include <7Semi_MLX90393.h>

MLX90393_7Semi mag;

void setup() {
  Serial.begin(115200);

  // Initialize sensor
  if (!mag.begin(0x0C, Wire, 100000)) {
    Serial.println("MLX90393 not found!");
    while (1)
      ;
  }
  /*
  * Apply configuration
  * * Gain: highest sensitivity
  * * Resolution: maximum (19-bit)
  * * Filter + OSR: highest accuracy (slowest)
  */
  mag.setGain(MLX90393_GAIN_0);

  mag.setResolution(
    MLX90393_RES_19BIT,
    MLX90393_RES_19BIT,
    MLX90393_RES_19BIT);

  if (!mag.setFilter(MLX90393_FILTER_7))
    Serial.println("Filter set failed");

  if (!mag.setOSR(MLX90393_OSR_3))
    Serial.println("OSR set failed");

  delay(50);

  uint8_t gain, filter, osr;
  bool tcmp;

  if (mag.getGain(gain)) {
    Serial.print("Gain: ");
    Serial.println(gain);
  }

  if (mag.getFilter(filter)) {
    Serial.print("Filter: ");
    Serial.println(filter);
  }

  if (mag.getOSR(osr)) {
    Serial.print("OSR: ");
    Serial.println(osr);
  }

  if (mag.getTemperatureCompensation(tcmp)) {
    Serial.print("Temp Compensation: ");
    Serial.println(tcmp ? "ON" : "OFF");
  }

  MLX90393_Resolution rx, ry, rz;
  if (mag.getResolution(rx, ry, rz)) {
    Serial.print("Resolution X/Y/Z: ");
    Serial.print(rx);
    Serial.print(" / ");
    Serial.print(ry);
    Serial.print(" / ");
    Serial.println(rz);
  }

  Serial.println("----------------------\n");
}

void loop() {
  float x, y, z;

  if (mag.readMag(x, y, z)) {
    Serial.print("X: ");
    Serial.print(x);
    Serial.print("  Y: ");
    Serial.print(y);
    Serial.print("  Z: ");
    Serial.println(z);
  } else {
    Serial.println("⚠️ Read failed!");
  }
  delay(200);
}