// 平衡控制：根据摆角/角速度/电机速度判断是否稳定，自适应目标角并使用两组LQR增益输出速度目标，提供稳定态标志。
#include "my_control.h"
#include "my_foc.h"
#include "my_mpu6050.h"
#include "my_web.h"
#include "sensor_fusion.h"
bool stable = false;
uint32_t last_unstable_time = 0;
uint32_t last_stable_time = 0;

MotionControlType mode = torque; // torque or velocity
float motion_target = 0.0f;      // voltage if torque, velocity if velocity
bool isStable = false;           // 当前是否稳态，用于外部切换PI参数

float LQR_K3_1 = 12.0f;
float LQR_K3_2 = 1.9f;
float LQR_K3_3 = 1.8f;
float LQR_K4_1 = 4.4f;
float LQR_K4_2 = 1.5f;
float LQR_K4_3 = 1.42f;

float v_p_1 = 0.25f;
float v_i_1 = 15.0f;
float v_p_2 = 0.10f;
float v_i_2 = 10.0f;

float target_angle = 60.0f;
float swing_up_voltage = 1.0f;
float swing_up_angle = 18.0f;

float err_angle = 0;

bool blance_swingup = false;
bool testmode_active = false;
void move_update()
{
  // err_angle = constrainAngle(fmod(kalAngleZ - angleZ0, 120.0f) - target_angle);
  // float angle0 = kalAngleZ - angleZ0;
  // err_angle = constrainAngle(angle0 - target_angle);
  float kalAngleZ1 = kalAngleZ;
  float target_angle1 = target_angle;
  if (abs(target_angle1) > 90)
    target_angle1 = target_angle1 + _sign(target_angle1) * 90;
  if (abs(kalAngleZ1) > 90)
    kalAngleZ1 = kalAngleZ1 + _sign(kalAngleZ1) * 90;
  err_angle = target_angle1 - kalAngleZ1;
  if (abs(err_angle) > 90)
    err_angle = err_angle + _sign(err_angle) * 90;

  // =============================================
  if (abs(err_angle) < swing_up_angle) // 如果角度小于摆动角度阈值，执行平衡控制
    blance_compute();
  else
    swingup_compute();
  testmode();
  if (!robot_run)
  {
    motion_target = 0;
    motor.controller = torque;
    angleZ0 = kalAngleZ; // 关闭以后自适应
    target_angle = 60;
  }
}
// 测试模式处理
void testmode()
{
  Serial.print(testmode_enabled);
  Serial.print(';');
  Serial.print(testmode_active);
  Serial.print(';');
  Serial.print(testmode_motor_mode);
  Serial.print(';');
  Serial.println(testmode_value);
  if (testmode_enabled)
  {
    float x = 1.0f; // 测试模式系数
    if (testmode_motor_mode == 0)
      motor.controller = velocity;
    else if (testmode_motor_mode == 1)
      motor.controller = torque;
    else if (testmode_motor_mode == 2)
      motor.controller = angle;
    motion_target = testmode_value * x;
    testmode_active = true;
  }
  if (testmode_active && !testmode_enabled) // 停止测试模式
  {
    motion_target = 0.0f;
    motor.controller = torque; // 恢复为力矩控制
    testmode_active = false;
  }
}
// 学生需要实现的函数1
void blance_compute()
{
  uint32_t now_time = millis();
  if (fabs(err_angle) > 8) // 误差角太大
  {
    stable = false;
    last_unstable_time = now_time;
  }
  if ((now_time - last_unstable_time) > 1000 && !stable) // 处于不稳定状态超过1秒，开始调整目标角度
  {
    target_angle += err_angle * 0.5f;
    stable = true;
  }

  if ((now_time - last_stable_time) > 2500 && stable) // 稳定时间超过2.5秒，开始调整目标角度
  {
    if (fabs(motion_target) > 5)
    {
      last_stable_time = millis();
      target_angle -= _sign(motion_target) * 0.2;
    }
  }

  isStable = stable;

  if (!stable)
  {
    motor.PID_velocity.P = v_p_1;
    motor.PID_velocity.I = v_i_1;
    motion_target = LQR_K3_1 * err_angle + LQR_K3_2 * now_gyroZ + LQR_K3_3 * motor.shaft_velocity;
  }
  else
  {
    motor.PID_velocity.P = v_p_2;
    motor.PID_velocity.I = v_i_2;
    motion_target = LQR_K4_1 * err_angle + LQR_K4_2 * now_gyroZ + LQR_K4_3 * motor.shaft_velocity;
  }

  if (abs(motion_target) > 120)
    motion_target = _sign(motion_target) * 120;
  motor.controller = velocity;
  blance_swingup = true; // 平衡为true
}
// 学生需要实现的函数2
void swingup_compute()
{
  motor.controller = torque;
  motion_target = -_sign(now_gyroZ) * swing_up_voltage;
  blance_swingup = false; // 摇摆为false
}