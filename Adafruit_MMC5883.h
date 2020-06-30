/*!
 * @file Adafruit_MMC5883.h
 */
#ifndef __MMC5883_H__
#define __MMC5883_H__

#include "Arduino.h"
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_Sensor.h>

/*!
 * @brief I2C address/bits
 */
#define MMC5883_ADDRESS_MAG 0x30

/*!
 @brief Registers
 */
typedef enum {
  MMC5883_REGISTER_OUT_X_L = 0x00,
  MMC5883_REGISTER_STATUS = 0x07,
  MMC5883_REGISTER_INTCTRL0 = 0x08,
  MMC5883_REGISTER_INTCTRL1 = 0x09,
  MMC5883_REGISTER_INTCTRL2 = 0x0A,
  MMC5883_REGISTER_PRODID = 0x2F,
} mmc5883MagRegisters_t;

/*!
 * @brief Magnetometer continuous frequency settings
 */
typedef enum {
  MMC5883_CMFREQ_ONESHOT = 0x00, // One shot mode
  MMC5883_CMFREQ_14HZ = 0x01, // Continuous 14 Hz
  MMC5883_CMFREQ_5HZ = 0x02, // Continuous 5 Hz
  MMC5883_CMFREQ_2_2HZ = 0x03, // Continuous 2.2 Hz
  MMC5883_CMFREQ_1HZ = 0x04, // Continuous 1 Hz
  MMC5883_CMFREQ_0_5HZ = 0x05, // Continuous 1/2 Hz
  MMC5883_CMFREQ_0_25HZ = 0x06, // Continuous 1/4 Hz
  MMC5883_CMFREQ_0_125HZ = 0x07, // Continuous 1/8 Hz
  MMC5883_CMFREQ_0_0625HZ = 0x08, // Continuous 1/16 Hz
  MMC5883_CMFREQ_0_003125HZ = 0x09, // Continuous 1/32 Hz
  MMC5883_CMFREQ_0_015625HZ = 0x0A, // Continuous 1/16 Hz
} mmc5883CMFreq;

/*!
 * @brief Chip ID
 */
#define MMC5883_ID (0b11010100)

//! Unified sensor driver for the magnetometer ///
class Adafruit_MMC5883 : public Adafruit_Sensor {
public:
  /*!
   * @param sensorID sensor ID, -1 by default
   */
  Adafruit_MMC5883(int32_t sensorID = -1);

  bool begin(uint8_t i2c_addr=MMC5883_ADDRESS_MAG, TwoWire *theWire = &Wire);
  void reset(void);
  void setContinuousFreq(mmc5883CMFreq freq);
  void setInterrupt(bool motion, bool meas);

  bool getEvent(sensors_event_t *);
  void getSensor(sensor_t *);

private:
  Adafruit_I2CDevice *i2c_dev = NULL;
  Adafruit_BusIO_Register *ctrl0_reg=NULL, 
    *ctrl1_reg=NULL, 
    *ctrl2_reg=NULL,
    *status_reg=NULL;
  uint16_t mag_x, mag_y, mag_z; // Last read magnetometer data will be available here
  int32_t _sensorID;

  void read(void);
};

#endif
