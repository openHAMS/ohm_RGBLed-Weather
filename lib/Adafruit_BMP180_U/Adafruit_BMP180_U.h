/***************************************************************************
  This is a library for the BMP085 pressure sensor

  Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution

  Refactored by László Székely-Tóth, based on TSL2561 adafruit library.
 ***************************************************************************/
#ifndef _BMP180_H_
#define _BMP180_H_

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <Adafruit_Sensor.h>
#include <Wire.h>

// I2C address
#define BMP180_ADDRESS (0x77)

// Registers
enum
{
  BMP180_REGISTER_CAL_AC1            = 0xAA,  // R   Calibration data (16 bits)
  BMP180_REGISTER_CAL_AC2            = 0xAC,  // R   Calibration data (16 bits)
  BMP180_REGISTER_CAL_AC3            = 0xAE,  // R   Calibration data (16 bits)
  BMP180_REGISTER_CAL_AC4            = 0xB0,  // R   Calibration data (16 bits)
  BMP180_REGISTER_CAL_AC5            = 0xB2,  // R   Calibration data (16 bits)
  BMP180_REGISTER_CAL_AC6            = 0xB4,  // R   Calibration data (16 bits)
  BMP180_REGISTER_CAL_B1             = 0xB6,  // R   Calibration data (16 bits)
  BMP180_REGISTER_CAL_B2             = 0xB8,  // R   Calibration data (16 bits)
  BMP180_REGISTER_CAL_MB             = 0xBA,  // R   Calibration data (16 bits)
  BMP180_REGISTER_CAL_MC             = 0xBC,  // R   Calibration data (16 bits)
  BMP180_REGISTER_CAL_MD             = 0xBE,  // R   Calibration data (16 bits)
  BMP180_REGISTER_ID                 = 0xD0,
  BMP180_REGISTER_VERSION            = 0xD1,
  BMP180_REGISTER_SOFTRESET          = 0xE0,
  BMP180_REGISTER_CONTROL            = 0xF4,
  BMP180_REGISTER_TEMPDATA           = 0xF6,
  BMP180_REGISTER_PRESSUREDATA       = 0xF6,
  BMP180_REGISTER_READTEMPCMD        = 0x2E,
  BMP180_REGISTER_READPRESSURECMD    = 0x34
};

// Mode settings
typedef enum
{
  BMP180_MODE_5MS                    = 0,
  BMP180_MODE_8MS                    = 1,
  BMP180_MODE_14MS                   = 2,
  BMP180_MODE_26MS                   = 3
} bmp180_mode_t;

// Calibration data
typedef struct
{
  int16_t  ac1;
  int16_t  ac2;
  int16_t  ac3;
  uint16_t ac4;
  uint16_t ac5;
  uint16_t ac6;
  int16_t  b1;
  int16_t  b2;
  int16_t  mb;
  int16_t  mc;
  int16_t  md;
} bmp180_calib_data;


class Adafruit_BMP180_Unified : public Adafruit_Sensor
{
  public:
    Adafruit_BMP180_Unified(int32_t sensorID = -1);
    bool  begin(bmp180_mode_t mode = BMP180_MODE_26MS);
    /* BMP180 Functions */
    void  getTemperature(float *temp);
    void  getPressure(float *pressure);
    float pressureToAltitude(float seaLvel, float atmospheric);
    float seaLevelForAltitude(float altitude, float atmospheric);
    float pressureToAltitude(float seaLevel, float atmospheric, float temp);
    float seaLevelForAltitude(float altitude, float atmospheric, float temp);

    /* Unified Sensor API Functions */
    bool  getEvent(sensors_event_t*);
    bool getTempEvent(sensors_event_t *event);
    void  getSensor(sensor_t*);

  private:
    int8_t _addr;
    void readCoefficients(void);
    void readRawTemperature(int32_t *temperature);
    void readRawPressure(int32_t *pressure);
    int32_t computeB5(int32_t ut);
    int32_t _bmp180SensorID;

    void     write8 (uint8_t reg, uint32_t value);
    uint8_t  read8 (uint8_t reg);
    uint16_t read16 (uint8_t reg);
};

#endif
