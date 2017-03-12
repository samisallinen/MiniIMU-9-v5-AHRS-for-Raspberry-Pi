#pragma once

namespace MiniIMu9 {
  bool Launch();
  void Stop();
  void GetEulerAngles(float *pitch, float *yaw, float *roll);
};

#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
