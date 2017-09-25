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
 ***************************************************************************/
#include <math.h>
#include <limits.h>

#include "Adafruit_BMP180_U.h"

static bmp180_calib_data _bmp180_coeffs;   // Last read accelerometer data will be available here
static uint8_t           _bmp180Mode;

#define BMP180_USE_DATASHEET_VALS (0) /* Set to 1 for sanity check */

/*========================================================================*/
/*                          PRIVATE FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    @brief  Writes a register and an 8 bit value over I2C
*/
/**************************************************************************/
void Adafruit_BMP180_Unified::write8(uint8_t reg, uint32_t value)
{
  Wire.beginTransmission(_addr);
  #if ARDUINO >= 100
  Wire.write(reg);
  Wire.write(value & 0xFF);
  #else
  Wire.send(reg);
  Wire.send(value & 0xFF);
  #endif
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads an 8 bit value over I2C
*/
/**************************************************************************/
uint8_t Adafruit_BMP180_Unified::read8(uint8_t reg)
{
  Wire.beginTransmission(_addr);
  #if ARDUINO >= 100
  Wire.write(reg);
  #else
  Wire.send(reg);
  #endif
  Wire.endTransmission();

  Wire.requestFrom(_addr, 1);
  #if ARDUINO >= 100
  return Wire.read();
  #else
  return Wire.receive();
  #endif
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
uint16_t Adafruit_BMP180_Unified::read16(uint8_t reg)
{
  uint16_t x1; uint16_t x2;

  Wire.beginTransmission(_addr);
  #if ARDUINO >= 100
  Wire.write(reg);
  #else
  Wire.send(reg);
  #endif
  Wire.endTransmission();

  Wire.requestFrom(_addr, 2);
  #if ARDUINO >= 100
  // the TSL2561 driver is reading 16 bits in reversed order
  x1 = Wire.read();
  x2 = Wire.read();
  #else
  x1 = Wire.receive();
  x2 = Wire.receive();
  #endif
  x1 <<= 8;
  x1 |= x2;
  return x1;
}

/**************************************************************************/
/*!
    @brief  Reads the factory-set coefficients
*/
/**************************************************************************/
void Adafruit_BMP180_Unified::readCoefficients(void)
{
  #if BMP180_USE_DATASHEET_VALS
    _bmp180_coeffs.ac1 = 408;
    _bmp180_coeffs.ac2 = -72;
    _bmp180_coeffs.ac3 = -14383;
    _bmp180_coeffs.ac4 = 32741;
    _bmp180_coeffs.ac5 = 32757;
    _bmp180_coeffs.ac6 = 23153;
    _bmp180_coeffs.b1  = 6190;
    _bmp180_coeffs.b2  = 4;
    _bmp180_coeffs.mb  = -32768;
    _bmp180_coeffs.mc  = -8711;
    _bmp180_coeffs.md  = 2868;
    _bmp180Mode        = 0;
  #else
    _bmp180_coeffs.ac1 = read16(BMP180_REGISTER_CAL_AC1);
    _bmp180_coeffs.ac2 = read16(BMP180_REGISTER_CAL_AC2);
    _bmp180_coeffs.ac3 = read16(BMP180_REGISTER_CAL_AC3);
    _bmp180_coeffs.ac4 = read16(BMP180_REGISTER_CAL_AC4);
    _bmp180_coeffs.ac5 = read16(BMP180_REGISTER_CAL_AC5);
    _bmp180_coeffs.ac6 = read16(BMP180_REGISTER_CAL_AC6);
    _bmp180_coeffs.b1  = read16(BMP180_REGISTER_CAL_B1);
    _bmp180_coeffs.b2  = read16(BMP180_REGISTER_CAL_B2);
    _bmp180_coeffs.mb  = read16(BMP180_REGISTER_CAL_MB);
    _bmp180_coeffs.mc  = read16(BMP180_REGISTER_CAL_MC);
    _bmp180_coeffs.md  = read16(BMP180_REGISTER_CAL_MD);
  #endif
}

/**************************************************************************/
/*!
    READ RAW TEMPERATURE
*/
/**************************************************************************/
void Adafruit_BMP180_Unified::readRawTemperature(int32_t *temperature)
{
  #if BMP180_USE_DATASHEET_VALS
    *temperature = 27898;
  #else
    write8(BMP180_REGISTER_CONTROL, BMP180_REGISTER_READTEMPCMD);
    delay(5);
    *temperature = read16(BMP180_REGISTER_TEMPDATA);
  #endif
}

/**************************************************************************/
/*!
    READ RAW PRESSURE
*/
/**************************************************************************/
#include <Arduino.h>
void Adafruit_BMP180_Unified::readRawPressure(int32_t *pressure)
{
  #if BMP180_USE_DATASHEET_VALS
    *pressure = 23843;
  #else
    uint8_t  p8;
    uint16_t p16;
    int32_t  p32;

    write8(BMP180_REGISTER_CONTROL, BMP180_REGISTER_READPRESSURECMD + (_bmp180Mode << 6));
    switch(_bmp180Mode)
    {
      case BMP180_MODE_5MS:
        delay(5);
        break;
      case BMP180_MODE_8MS:
        delay(8);
        break;
      case BMP180_MODE_14MS:
        delay(14);
        break;
      case BMP180_MODE_26MS:
      default:
        delay(26);
        break;
    }

    p16 = read16(BMP180_REGISTER_PRESSUREDATA);
    p32 = (uint32_t)p16 << 8;
    p8 = read8(BMP180_REGISTER_PRESSUREDATA+2);
    p32 += p8;
    p32 >>= (8 - _bmp180Mode);

    *pressure = p32;
  #endif
}

/**************************************************************************/
/*!
    @brief  Compute B5 coefficient used in temperature & pressure calcs.
*/
/**************************************************************************/
int32_t Adafruit_BMP180_Unified::computeB5(int32_t ut) {
  int32_t X1 = (ut - (int32_t)_bmp180_coeffs.ac6) * ((int32_t)_bmp180_coeffs.ac5) >> 15;
  int32_t X2 = ((int32_t)_bmp180_coeffs.mc << 11) / (X1+(int32_t)_bmp180_coeffs.md);
  return X1 + X2;
}


/*========================================================================*/
/*                            CONSTRUCTORS                                */
/*========================================================================*/

/**************************************************************************/
/*!
    Constructor
*/
/**************************************************************************/
Adafruit_BMP180_Unified::Adafruit_BMP180_Unified(int32_t sensorID)
{
  _addr = BMP180_ADDRESS;
  _bmp180SensorID = sensorID;
}

/*========================================================================*/
/*                           PUBLIC FUNCTIONS                             */
/*========================================================================*/

/**************************************************************************/
/*!
    Initializes I2C and configures the sensor (call this function before
    doing anything else)
*/
/**************************************************************************/
bool Adafruit_BMP180_Unified::begin(bmp180_mode_t mode)
{
  // Enable I2C
  Wire.begin();

  /* Mode boundary check */
  if ((mode > BMP180_MODE_26MS) || (mode < 0))
  {
    mode = BMP180_MODE_26MS;
  }

  /* Make sure we're actually connected */
  uint8_t id = read8(BMP180_REGISTER_ID);
  if(id != 0x55)
  {
    return false;
  }

  /* Set the mode indicator */
  _bmp180Mode = mode;

  /* Coefficients need to be read once */
  readCoefficients();

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the compensated pressure level in kPa
*/
/**************************************************************************/
void Adafruit_BMP180_Unified::getPressure(float *pressure)
{
  int32_t  ut = 0, up = 0, compp = 0;
  int32_t  x1, x2, b5, b6, x3, b3, p;
  uint32_t b4, b7;

  /* Get the raw pressure and temperature values */
  readRawTemperature(&ut);
  readRawPressure(&up);

  /* Temperature compensation */
  b5 = computeB5(ut);

  /* Pressure compensation */
  b6 = b5 - 4000;
  x1 = (_bmp180_coeffs.b2 * ((b6 * b6) >> 12)) >> 11;
  x2 = (_bmp180_coeffs.ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((int32_t) _bmp180_coeffs.ac1) * 4 + x3) << _bmp180Mode) + 2) >> 2;
  x1 = (_bmp180_coeffs.ac3 * b6) >> 13;
  x2 = (_bmp180_coeffs.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (_bmp180_coeffs.ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t) (up - b3) * (50000 >> _bmp180Mode));

  if (b7 < 0x80000000)
  {
    p = (b7 << 1) / b4;
  }
  else
  {
    p = (b7 / b4) << 1;
  }

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  compp = p + ((x1 + x2 + 3791) >> 4);

  /* Assign compensated pressure value */
  *pressure = compp;
}

/**************************************************************************/
/*!
    @brief  Reads the temperatures in degrees Celsius
*/
/**************************************************************************/
void Adafruit_BMP180_Unified::getTemperature(float *temp)
{
  int32_t UT, X1, X2, B5;     // following ds convention
  float t;

  readRawTemperature(&UT);

  #if BMP180_USE_DATASHEET_VALS
    // use datasheet numbers!
    UT = 27898;
    _bmp180_coeffs.ac6 = 23153;
    _bmp180_coeffs.ac5 = 32757;
    _bmp180_coeffs.mc = -8711;
    _bmp180_coeffs.md = 2868;
  #endif

  B5 = computeB5(UT);
  t = (B5+8) >> 4;
  t /= 10;

  *temp = t;
}

/**************************************************************************/
/*!
    Calculates the altitude (in meters) from the specified atmospheric
    pressure (in hPa), and sea-level pressure (in hPa).

    @param  seaLevel      Sea-level pressure in hPa
    @param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float Adafruit_BMP180_Unified::pressureToAltitude(float seaLevel, float atmospheric)
{
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

/**************************************************************************/
/*!
    Calculates the pressure at sea level (in hPa) from the specified altitude
    (in meters), and atmospheric pressure (in hPa).

    @param  altitude      Altitude in meters
    @param  atmospheric   Atmospheric pressure in hPa
*/
/**************************************************************************/
float Adafruit_BMP180_Unified::seaLevelForAltitude(float altitude, float atmospheric)
{
  // Equation taken from BMP180 datasheet (page 17):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  return atmospheric / pow(1.0 - (altitude/44330.0), 5.255);
}

/**************************************************************************/
/*!
    @brief  Reads the sensor and returns the data as a sensors_event_t
*/
/**************************************************************************/
bool Adafruit_BMP180_Unified::getEvent(sensors_event_t *event)
{
  float pressure_kPa;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = 1;
  event->sensor_id = _bmp180SensorID;
  event->type      = SENSOR_TYPE_PRESSURE;
  event->timestamp = 0;
  getPressure(&pressure_kPa);
  event->pressure = pressure_kPa / 100.0F;

  return true;
}

bool Adafruit_BMP180_Unified::getTempEvent(sensors_event_t *event)
{
  float temperature_C;

  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  event->version   = 1;
  event->sensor_id = _bmp180SensorID;
  event->type      = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  event->timestamp = 0;
  //getPressure(&pressure_kPa);
  getTemperature(&temperature_C);
  event->temperature = temperature_C;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Adafruit_BMP180_Unified::getSensor(sensor_t *sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "BMP180", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _bmp180SensorID;
  sensor->type        = SENSOR_TYPE_PRESSURE;
  sensor->min_delay   = 0;
  sensor->max_value   = 1100.0F;               // 300..1100 hPa
  sensor->min_value   = 300.0F;
  sensor->resolution  = 0.01F;                // Datasheet states 0.01 hPa resolution
}
