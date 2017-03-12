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

#include "MiniIMU-9v5-ahrs-internal.h"
#include "MiniIMU-9v5-ahrs.h"
#include <wiringPi.h>
#include <thread>
#include <mutex>

namespace MiniIMu9 {

  // X axis pointing forward
  // Y axis pointing to the right
  // and Z axis pointing down.
  // Positive pitch : nose up
  // Positive roll : right wing down
  // Positive yaw : clockwise
  int SENSOR_SIGN[9] = { 1,1,1,-1,-1,-1,1,1,1 }; //Correct directions x,y,z - gyro, accelerometer, magnetometer

  float G_Dt = 0.02f;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

  long timer = 0;   //general purpose timer
  long timer_old;
  long timer24 = 0; //Second timer used to print values
  int AN[6]; //array that stores the gyro and accelerometer data
  int AN_OFFSET[6] = { 0,0,0,0,0,0 }; //Array that stores the Offset of the sensors

  int gyro_x;
  int gyro_y;
  int gyro_z;
  int accel_x;
  int accel_y;
  int accel_z;
  int magnetom_x;
  int magnetom_y;
  int magnetom_z;
  float c_magnetom_x;
  float c_magnetom_y;
  float c_magnetom_z;
  float MAG_Heading;

  float Accel_Vector[3] = { 0,0,0 }; //Store the acceleration in a vector
  float Gyro_Vector[3] = { 0,0,0 };//Store the gyros turn rate in a vector
  float Omega_Vector[3] = { 0,0,0 }; //Corrected Gyro_Vector data
  float Omega_P[3] = { 0,0,0 };//Omega Proportional correction
  float Omega_I[3] = { 0,0,0 };//Omega Integrator
  float Omega[3] = { 0,0,0 };

  // Euler angles
  float roll;
  float pitch;
  float yaw;

  float errorRollPitch[3] = { 0,0,0 };
  float errorYaw[3] = { 0,0,0 };

  unsigned int counter = 0;

  float DCM_Matrix[3][3] = {
    {
      1,0,0  }
    ,{
      0,1,0  }
    ,{
      0,0,1  }
  };
  float Update_Matrix[3][3] = { {0,1,2},{3,4,5},{6,7,8} }; //Gyros here


  float Temporary_Matrix[3][3] = {
    {
      0,0,0  }
    ,{
      0,0,0  }
    ,{
      0,0,0  }
  };

  void setup()
  {
    Accel_Init();
    Compass_Init();
    Gyro_Init();

    delay(20);

    /*
    for (int i = 0; i < 128; i++)    // We take some readings...
    {
      Read_Gyro();
      Read_Accel();
      for (int y = 0; y < 6; y++)   // Cumulate values
        AN_OFFSET[y] += AN[y];
      delay(20);
    }

    for (int y = 0; y < 6; y++) {
      AN_OFFSET[y] = AN_OFFSET[y] / 128;
      fprintf(stderr, "AN_OFFSET[%d]=%d;\n", y, AN_OFFSET[y]);
    };

    */

    AN_OFFSET[0] = 27;
    AN_OFFSET[1] = -60;
    AN_OFFSET[2] = -26;
    AN_OFFSET[3] = 2;
    AN_OFFSET[4] = -1;
    AN_OFFSET[5] = -255;


    AN_OFFSET[5] -= GRAVITY*SENSOR_SIGN[5];


    delay(20);

    timer = millis();
    delay(20);
    counter = 0;
  }

  static std::unique_ptr<std::thread> s_thread;
  static std::mutex s_valuesAccessMutex;
  static bool s_bRun = false;


  void AHRSThread() //Main Loop
  {
    setup();
    while (s_bRun) {
      while ((millis() - timer) < 20) {// Main loop runs at 50Hz
        delay(1);
      }
      
      counter++;
      timer_old = timer;
      timer = millis();
      if (timer > timer_old)
      {
        G_Dt = (timer - timer_old) / 1000.0f;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
        if (G_Dt > 0.2f)
          G_Dt = 0; // ignore integration times over 200 ms
      } else {
        G_Dt = 0;
      }

      // *** DCM algorithm
      // Data adquisition
      Read_Gyro();   // This read gyro data
      Read_Accel();     // Read I2C accelerometer

      if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
      {
        counter = 0;
        Read_Compass();    // Read I2C magnetometer
        Compass_Heading(); // Calculate magnetic heading
      }

      // Calculations...
      Matrix_update();
      Normalize();
      Drift_correction();
      {
        std::unique_lock<std::mutex> l(s_valuesAccessMutex);
        Euler_angles();
      }
      
    }
  }

  bool Launch() {
    if (s_thread)
      return false;
    s_bRun = true;
    s_thread = std::unique_ptr<std::thread>(new std::thread(AHRSThread));
    return true;
  }

  void Stop() {
    if (!s_thread)
      return;
    s_bRun = false;
    s_thread->join();
  }


  void GetEulerAngles(float *pitchOut, float *yawOut, float *rollOut) {    
      std::unique_lock<std::mutex> l(s_valuesAccessMutex);
      *pitchOut = pitch;
      *yawOut = yaw;
      *rollOut = roll;
  }
}