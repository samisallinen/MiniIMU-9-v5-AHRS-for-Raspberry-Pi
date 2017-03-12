#ifndef LSM6_h
#define LSM6_h

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

#include <cstdint>

class LSM6
{
  public:
    template <typename T> struct vector3
    {
      T x, y, z;
    };

    enum deviceType { device_DS33, device_auto };
    enum sa0State { sa0_low, sa0_high, sa0_auto };

    // register addresses
    enum regAddr
    {
      FUNC_CFG_ACCESS   = 0x01,

      FIFO_CTRL1        = 0x06,
      FIFO_CTRL2        = 0x07,
      FIFO_CTRL3        = 0x08,
      FIFO_CTRL4        = 0x09,
      FIFO_CTRL5        = 0x0A,
      ORIENT_CFG_G      = 0x0B,

      INT1_CTRL         = 0x0D,
      INT2_CTRL         = 0x0E,
      WHO_AM_I          = 0x0F,
      CTRL1_XL          = 0x10,
      CTRL2_G           = 0x11,
      CTRL3_C           = 0x12,
      CTRL4_C           = 0x13,
      CTRL5_C           = 0x14,
      CTRL6_C           = 0x15,
      CTRL7_G           = 0x16,
      CTRL8_XL          = 0x17,
      CTRL9_XL          = 0x18,
      CTRL10_C          = 0x19,

      WAKE_UP_SRC       = 0x1B,
      TAP_SRC           = 0x1C,
      D6D_SRC           = 0x1D,
      STATUS_REG        = 0x1E,

      OUT_TEMP_L        = 0x20,
      OUT_TEMP_H        = 0x21,
      OUTX_L_G          = 0x22,
      OUTX_H_G          = 0x23,
      OUTY_L_G          = 0x24,
      OUTY_H_G          = 0x25,
      OUTZ_L_G          = 0x26,
      OUTZ_H_G          = 0x27,
      OUTX_L_XL         = 0x28,
      OUTX_H_XL         = 0x29,
      OUTY_L_XL         = 0x2A,
      OUTY_H_XL         = 0x2B,
      OUTZ_L_XL         = 0x2C,
      OUTZ_H_XL         = 0x2D,

      FIFO_STATUS1      = 0x3A,
      FIFO_STATUS2      = 0x3B,
      FIFO_STATUS3      = 0x3C,
      FIFO_STATUS4      = 0x3D,
      FIFO_DATA_OUT_L   = 0x3E,
      FIFO_DATA_OUT_H   = 0x3F,
      TIMESTAMP0_REG    = 0x40,
      TIMESTAMP1_REG    = 0x41,
      TIMESTAMP2_REG    = 0x42,

      STEP_TIMESTAMP_L  = 0x49,
      STEP_TIMESTAMP_H  = 0x4A,
      STEP_COUNTER_L    = 0x4B,
      STEP_COUNTER_H    = 0x4C,

      FUNC_SRC          = 0x53,

      TAP_CFG           = 0x58,
      TAP_THS_6D        = 0x59,
      INT_DUR2          = 0x5A,
      WAKE_UP_THS       = 0x5B,
      WAKE_UP_DUR       = 0x5C,
      FREE_FALL         = 0x5D,
      MD1_CFG           = 0x5E,
      MD2_CFG           = 0x5F,
    };

    vector3<int16_t> a; // accelerometer readings
    vector3<int16_t> g; // gyro readings

    uint8_t last_status; // status of last I2C transmission

    LSM6(void);

    bool init(deviceType device = device_auto, sa0State sa0 = sa0_auto);
    deviceType getDeviceType(void) { return _device; }

    void enableDefault(void);


    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);

    void readAcc(void);
    void readGyro(void);
    void read(void);

    void setTimeout(uint16_t timeout);
    uint16_t getTimeout(void);
    bool timeoutOccurred(void);

    // vector functions
    template <typename Ta, typename Tb, typename To> static void vector_cross(const vector3<Ta> *a, const vector3<Tb> *b, vector3<To> *out);
    template <typename Ta, typename Tb> static float vector_dot(const vector3<Ta> *a, const vector3<Tb> *b);
    static void vector_normalize(vector3<float> *a);

  private:
    deviceType _device; // chip type
    uint8_t address;
    int m_deviceFd = -1;   

    uint16_t io_timeout;
    bool did_timeout;

    int16_t testReg(uint8_t address, regAddr reg);
};


template <typename Ta, typename Tb, typename To> void LSM6::vector_cross(const vector3<Ta> *a, const vector3<Tb> *b, vector3<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float LSM6::vector_dot(const vector3<Ta> *a, const vector3<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

#endif