/*!
 *  @file Adafruit_CAP1106.cpp
 *
 *  @mainpage Adafruit CAP1106 I2C/SPI 8-chan Capacitive Sensor
 *
 *  @section intro_sec Introduction
 *
 * 	This is a library for the Adafruit CAP1106 I2C/SPI 8-chan Capacitive
 * Sensor http://www.adafruit.com/products/1602
 *
 *  These sensors use I2C/SPI to communicate, 2+ pins are required to
 *  interface
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Dadafruit_CAP1106.h"

/*!
 *    @brief  Instantiates a new CAP1106 class using hardware I2C
 *    @param  resetpin
 *            number of pin where reset is connected
 *
 */
Adafruit_CAP1106::Adafruit_CAP1106() {
  // I2C
}

/*!
 *    @brief  Setups the i2c depending on selected mode (I2C / SPI, Software /
 * Hardware). Displays useful debug info, as well as allow multiple touches
 * (CAP1106_MTBLK), links leds to touches (CAP1106_LEDLINK), and increase the
 * cycle time value (CAP1106_STANDBYCFG)
 *    @param  i2caddr
 *            optional i2caddres (default to 0x28)
 *    @param  theWire
 *            optional wire object
 *    @return True if initialization was successful, otherwise false.
 */
boolean Adafruit_CAP1106::begin(uint8_t i2caddr, TwoWire *theWire) {

  // I2C
  i2c_dev = new Adafruit_I2CDevice(i2caddr, theWire);
  uint8_t res = i2c_dev->begin();
  //if (!res) return false;

  readRegister(CAP1106_PRODID);

  // Useful debugging info. Comment/uncomment as needed.

  Serial.print("Product ID: 0x");
  Serial.println(readRegister(CAP1106_PRODID), HEX);
  Serial.print("Manuf. ID: 0x");
  Serial.println(readRegister(CAP1106_MANUID), HEX);
  Serial.print("Revision: 0x");
  Serial.println(readRegister(CAP1106_REV), HEX);

  if ((readRegister(CAP1106_PRODID) != 0x55) ||
      (readRegister(CAP1106_MANUID) != 0x5D) ||
      (readRegister(CAP1106_REV) != 0x83)) {
    return false;
  }
  // allow multiple touches
  writeRegister(CAP1106_MTBLK, 0);
  // speed up a bit
  // writeRegister(CAP1106_STANDBYCFG, 0x30);
  // set Gain
  writeRegister(CAP1106_SENSITIVITY, 0x1F);
  // Each Sensor Input X Threshold register is updated individually
  // writeRegister(0x2F, 0x0A); 

  return true;
}

/*!
 *   @brief  Reads the touched status (CAP1106_SENINPUTSTATUS)
 *   @return Returns read from CAP1106_SENINPUTSTATUS where 1 is touched, 0 not
 * touched.
 */
uint8_t Adafruit_CAP1106::touched() {
  uint8_t t = readRegister(CAP1106_SENINPUTSTATUS);
  if (t) {
    writeRegister(CAP1106_MAIN, readRegister(CAP1106_MAIN) & ~CAP1106_MAIN_INT);
  }
  return t;
}

/*!
 *    @brief  Reads from selected register
 *    @param  reg
 *            register address
 *    @return
 */
uint8_t Adafruit_CAP1106::readRegister(uint8_t reg) {
  uint8_t buffer[3] = {reg, 0, 0};
  if (i2c_dev) {
    i2c_dev->write_then_read(buffer, 1, buffer, 1);
  } 
  return buffer[0];
}

/*!
 *   @brief  Writes 8-bits to the specified destination register
 *   @param  reg
 *           register address
 *   @param  value
 *           value that will be written at selected register
 */
void Adafruit_CAP1106::writeRegister(uint8_t reg, uint8_t value) {
  uint8_t buffer[4] = {reg, value, 0, 0};
  if (i2c_dev) {
    i2c_dev->write(buffer, 2);
  }
}
