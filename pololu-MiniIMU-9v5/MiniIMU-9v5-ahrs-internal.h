#pragma once

namespace MiniIMu9 {

  extern int SENSOR_SIGN[9];


  // accelerometer: 8 g sensitivity
  // 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

                                               // gyro: 2000 dps full scale
                                               // 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

                                               // LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
                                               // the Pololu LSM303 or LIS3MDL library to find the right values for your board


  //{ -2138, -3855, -9359}   max: { +4600, +3019, -2165}

#define M_X_MIN -2200
#define M_Y_MIN -3900
#define M_Z_MIN -9400
#define M_X_MAX +4700
#define M_Y_MAX +3100
#define M_Z_MAX +2200

#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

                                               /*For debugging purposes*/
                                               //OUTPUTMODE=1 will print the corrected data,
                                               //OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13

  extern float G_Dt;

  extern int AN[6];
  extern int AN_OFFSET[6];

  extern int gyro_x;
  extern int gyro_y;
  extern int gyro_z;
  extern int accel_x;
  extern int accel_y;
  extern int accel_z;
  extern int magnetom_x;
  extern int magnetom_y;
  extern int magnetom_z;
  extern float c_magnetom_x;
  extern float c_magnetom_y;
  extern float c_magnetom_z;
  extern float MAG_Heading;

  extern float Accel_Vector[3];
  extern float Gyro_Vector[3];
  extern float Omega_Vector[3];
  extern float Omega_P[3];
  extern float Omega_I[3];
  extern float Omega[3];

  // Euler angles
  extern float roll;
  extern float pitch;
  extern float yaw;

  extern float errorRollPitch[3];
  extern float errorYaw[3];

  extern unsigned int counter;

  extern float DCM_Matrix[3][3];

  extern float Update_Matrix[3][3];
  extern float Temporary_Matrix[3][3];

  // i2c.cpp
  void Accel_Init();
  void Compass_Init();
  void Gyro_Init();


  void Read_Accel();
  void Read_Compass();
  void Read_Gyro();

  // compass.cpp
  void Compass_Heading();

  // DCM.cpp
  void Matrix_update(void);
  void Normalize(void);
  void Drift_correction(void);
  void Euler_angles(void);

  // Vector.cpp
  float Vector_Dot_Product(float vector1[3], float vector2[3]);
  void Vector_Cross_Product(float vectorOut[3], float v1[3], float v2[3]);
  void Vector_Scale(float vectorOut[3], float vectorIn[3], float scale2);
  void Vector_Add(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]);

  // matrix.cpp
  void Matrix_Multiply(float a[3][3], float b[3][3], float mat[3][3]);

}