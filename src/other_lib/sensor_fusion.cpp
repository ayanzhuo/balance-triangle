// 传感器融合与角度估计：利用Kalman与互补/积分方法估计摆角，提供加速度转角、角度约束与状态更新接口。
#include "sensor_fusion.h"
#include "Kalman.h"
#include "my_mpu6050.h"
Kalman kalmanZ;
constexpr float GYROZ_OFF = -0.19f; // 与原代码一致
float gyroZangle = 0.0f;
float compAngleZ = 0.0f;
float kalAngleZ = 0.0f;
uint32_t lastMicros = 0;

void initWithPitch(float pitchDeg)
{
  kalmanZ.setAngle(pitchDeg);
  gyroZangle = pitchDeg;
  lastMicros = micros();
}

float acc2rotation(float x, float y)
{
  float tmp = (atan(x / y) / 1.570796f * 90.0f);
  if (y < 0)
    return (tmp + 180.0f);
  else if (x < 0)
  {
    if (!isnan(kalAngleZ) && (tmp + 360.0f - kalAngleZ) > 100.0f)
    {
      if (tmp < 0 && kalAngleZ < 0)
        return tmp;
      else
        return tmp;
    }
    else
      return (tmp + 360.0f);
  }
  else
    return tmp;
}

float constrainAngle(float x)
{
  float a = 0;
  if (x < 0)
  {
    a = 120 + x;
    if (a < fabs(x))
      return a;
  }
  return x;
}

void kalman_update()
{
  uint32_t now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;
  float pitch = acc2rotation(now_angleX, now_angleY);
  kalAngleZ = kalmanZ.getAngle(pitch, now_gyroZ + GYROZ_OFF, dt);
  // gyroZangle += (now_gyroZ + GYROZ_OFF) * dt;
  compAngleZ = 0.93f * (compAngleZ + (now_gyroZ + GYROZ_OFF) * dt) + 0.07f * pitch;
  // if (gyroZangle < -180.0f || gyroZangle > 180.0f)
  // gyroZangle = kalAngleZ;
}
