#include "LSM6.h"
#include <wiringPi.h>
#include <math.h>
#include <wiringPiI2C.h>
#include <unistd.h>

/*
Copyright(c) 2016 Pololu Corporation.For more information, see

http ://www.pololu.com/
http://forum.pololu.com/

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files(the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions :

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

Adapted for raspberry pi by Sami Sallinen (ssallinen@gmail.com)

*/

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define DS33_SA0_HIGH_ADDRESS 0b1101011
#define DS33_SA0_LOW_ADDRESS  0b1101010

#define TEST_REG_ERROR -1

#define DS33_WHO_ID    0x69

// Constructors ////////////////////////////////////////////////////////////////

LSM6::LSM6(void)
{
  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in readAcc(), readGyro(), or read() since the last call to timeoutOccurred()?
bool LSM6::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

void LSM6::setTimeout(uint16_t timeout)
{
  io_timeout = timeout;
}

uint16_t LSM6::getTimeout()
{
  return io_timeout;
}

bool LSM6::init(deviceType device, sa0State sa0)
{
  // perform auto-detection unless device type and SA0 state were both specified
  if (device == device_auto || sa0 == sa0_auto)
  {
    // check for LSM6DS33 if device is unidentified or was specified to be this type
    if (device == device_auto || device == device_DS33)
    {
      // check SA0 high address unless SA0 was specified to be low
      if (sa0 != sa0_low && testReg(DS33_SA0_HIGH_ADDRESS, WHO_AM_I) == DS33_WHO_ID)
      {
        sa0 = sa0_high;
        if (device == device_auto) { device = device_DS33; }
      }
      // check SA0 low address unless SA0 was specified to be high
      else if (sa0 != sa0_high && testReg(DS33_SA0_LOW_ADDRESS, WHO_AM_I) == DS33_WHO_ID)
      {
        sa0 = sa0_low;
        if (device == device_auto) { device = device_DS33; }
      }
    }

    // make sure device and SA0 were successfully detected; otherwise, indicate failure
    if (device == device_auto || sa0 == sa0_auto)
    {
      return false;
    }
  }

  _device = device;

  switch (device)
  {
    case device_DS33:
      address = (sa0 == sa0_high) ? DS33_SA0_HIGH_ADDRESS : DS33_SA0_LOW_ADDRESS;
      break;
    default:
      break;
  }
  m_deviceFd = wiringPiI2CSetup(address);
  if (m_deviceFd < 0)
    return false;
  return true;
}

/*
Enables the LSM6's accelerometer and gyro. Also:
- Sets sensor full scales (gain) to default power-on values, which are
  +/- 2 g for accelerometer and 245 dps for gyro
- Selects 1.66 kHz (high performance) ODR (output data rate) for accelerometer
  and 1.66 kHz (high performance) ODR for gyro. (These are the ODR settings for
  which the electrical characteristics are specified in the datasheet.)
- Enables automatic increment of register address during multiple byte access
Note that this function will also reset other settings controlled by
the registers it writes to.
*/
void LSM6::enableDefault(void)
{
  if (_device == device_DS33)
  {
    // Accelerometer

    // 0x80 = 0b10000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
    writeReg(CTRL1_XL, 0x80);

    // Gyro

    // 0x80 = 0b010000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 dps)
    writeReg(CTRL2_G, 0x80);

    // Common

    // 0x04 = 0b00000100
    // IF_INC = 1 (automatically increment register address)
    writeReg(CTRL3_C, 0x04);
  }
}

void LSM6::writeReg(uint8_t reg, uint8_t value)
{

  int ret = wiringPiI2CWriteReg8(m_deviceFd, reg, value);
  if (ret < 0)
    last_status = 4;
  else
    last_status = 0;
}

uint8_t LSM6::readReg(uint8_t reg)
{
  uint8_t value = (uint8_t)wiringPiI2CReadReg8(m_deviceFd, reg);
  return value;
}

// Reads the 3 accelerometer channels and stores them in vector a
void LSM6::readAcc(void)
{  
  uint8_t xla = (uint8_t)wiringPiI2CReadReg8(m_deviceFd, OUTX_L_XL);
  uint8_t xha = (uint8_t)wiringPiI2CReadReg8(m_deviceFd, OUTX_H_XL);
  uint8_t yla = (uint8_t)wiringPiI2CReadReg8(m_deviceFd, OUTY_L_XL);
  uint8_t yha = (uint8_t)wiringPiI2CReadReg8(m_deviceFd, OUTY_H_XL);
  uint8_t zla = (uint8_t)wiringPiI2CReadReg8(m_deviceFd, OUTZ_L_XL);
  uint8_t zha = (uint8_t)wiringPiI2CReadReg8(m_deviceFd, OUTZ_H_XL);

  // combine high and low bytes
  a.x = (int16_t)(xha << 8 | xla);
  a.y = (int16_t)(yha << 8 | yla);
  a.z = (int16_t)(zha << 8 | zla);
}

// Reads the 3 gyro channels and stores them in vector g
void LSM6::readGyro(void)
{  
  uint8_t xlg = (uint8_t)wiringPiI2CReadReg8(m_deviceFd, OUTX_L_G);
  uint8_t xhg = (uint8_t)wiringPiI2CReadReg8(m_deviceFd, OUTX_H_G);
  uint8_t ylg = (uint8_t)wiringPiI2CReadReg8(m_deviceFd, OUTY_L_G);
  uint8_t yhg = (uint8_t)wiringPiI2CReadReg8(m_deviceFd, OUTY_H_G);
  uint8_t zlg = (uint8_t)wiringPiI2CReadReg8(m_deviceFd, OUTZ_L_G);
  uint8_t zhg = (uint8_t)wiringPiI2CReadReg8(m_deviceFd, OUTZ_H_G);

  // combine high and low bytes
  g.x = (int16_t)(xhg << 8 | xlg);
  g.y = (int16_t)(yhg << 8 | ylg);
  g.z = (int16_t)(zhg << 8 | zlg);
}

// Reads all 6 channels of the LSM6 and stores them in the object variables
void LSM6::read(void)
{
  readAcc();
  readGyro();
}

void LSM6::vector_normalize(vector3<float> *a)
{
  float mag = (float)sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

// Private Methods //////////////////////////////////////////////////////////////

int16_t LSM6::testReg(uint8_t address, regAddr reg)
{
  int fd = wiringPiI2CSetup(address);
  if (fd == -1) {
    return TEST_REG_ERROR;
  }
  int ret = wiringPiI2CReadReg8(fd, reg);
  close(fd);
  if (ret < 0) 
    return TEST_REG_ERROR;
  return (int16_t)ret;
}