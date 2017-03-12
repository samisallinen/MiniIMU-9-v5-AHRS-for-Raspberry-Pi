/*

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011-2016 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/


#include "lsm6/LSM6.h"
#include "lis3mdl/LIS3MDL.h"
#include "MiniIMU-9v5-ahrs-internal.h"

namespace MiniIMu9 {

  LSM6 gyro_acc;
  LIS3MDL mag;



  void Gyro_Init()
  {
    // Accel_Init() should have already called gyro_acc.init() and enableDefault()
    gyro_acc.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale
  }

  void Read_Gyro()
  {

    gyro_acc.readGyro();

    AN[0] = gyro_acc.g.x;
    AN[1] = gyro_acc.g.y;
    AN[2] = gyro_acc.g.z;

    gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
    gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
    gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
  }

  void Accel_Init()
  {
    gyro_acc.init();
    gyro_acc.enableDefault();
    gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale
  }

  // Reads x,y and z accelerometer registers
  void Read_Accel()
  {
    gyro_acc.readAcc();

    AN[3] = gyro_acc.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
    AN[4] = gyro_acc.a.y >> 4;
    AN[5] = gyro_acc.a.z >> 4;

    accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
    accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
    accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
  }

  void Compass_Init()
  {
    mag.init();
    mag.enableDefault();
  }

  void Read_Compass()
  {
    mag.read();

    magnetom_x = SENSOR_SIGN[6] * mag.m.x;
    magnetom_y = SENSOR_SIGN[7] * mag.m.y;
    magnetom_z = SENSOR_SIGN[8] * mag.m.z;
  }
};