# MiniIMU-9-v5-AHRS-for-Raspberry-Pi
This pile of code is the Pololu arduino samples for MiniIMU 9 v5  modified to work on the Raspberry PI.

Be sure to calibrate your magnetometer, e.g. find out the minimum and maximum values of all 3 magnetometer axes you get when rotating the imu in all orientations. Put the values to M_X_MIN, M_X_MAX etc. in MiniIMU9v5-ahrs-internal.h.

The file MiniIMU-9v5-ahrs.h" provides the public interface to the ahrs.

