// 电机FOC配置与控制：初始化BLDC驱动与传感器、加载/保存零电角、运行loopFOC，提供移动与控制模式切换接口，导出全局BLDCMotor对象。
#include "my_foc.h"
#include "my_io.h"
#include "my_control.h"
// ===== 手动定义 MT6701 的 SPI 配置 =====
// MagneticSensorSPIConfig_s MT6701_SPI = {
//     .spi_mode = SPI_MODE0,    // MT6701 要用 MODE1 (CPOL=0, CPHA=1)
//     .clock_speed = 1000000,   // SPI 时钟频率 1MHz
//     .bit_resolution = 16,     // MT6701 输出 14bit
//     .angle_register = 0xFFFF, // 直接读寄存器，不需要命令
//     .command_rw_bit = 0,      // 无需
//     .command_parity_bit = 0,  // 无需
// };

static BLDCDriver3PWM driver(32, 33, 25);
// static MagneticSensorSPI sensor = MagneticSensorSPI(MT6701_SPI, 22); // 使用 MT6701 磁传感器配置，CS 引脚为 22
static GenericSensor sensor(encoderGenericRead, encoderGenericInit);
BLDCMotor motor = BLDCMotor(7);

void motor_init()
{
  sensor.init();
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  motor.foc_modulation = SpaceVectorPWM;
  motor.controller = torque;

  motor.PID_velocity.P = v_p_1;
  motor.PID_velocity.I = v_i_1;

  motor.voltage_limit = 12;
  motor.LPF_velocity.Tf = 0.01f;
  motor.velocity_limit = 40;

  motor.useMonitoring(Serial);
  motor.init();
  motor.initFOC();
}
void motor_update()
{
  motor.target = motion_target;
  motor.loopFOC();
  motor.move();
}
float encoderGenericRead()
{
  digitalWrite(22, 0);
  digitalWrite(5, 0);
  uint16_t ag = 0;
  for (int i = 0; i < 16; i++)
  {
    digitalWrite(5, 1);
    delayMicroseconds(1);
    ag = (uint16_t)(ag << 1);
    ag = (uint16_t)(ag + (uint16_t)digitalRead(23));
    digitalWrite(5, 0);
    delayMicroseconds(1);
  }
  digitalWrite(22, 1);
  ag = (uint16_t)(ag >> 2);
  digitalWrite(5, 1);
  float rad = (float)ag * 2.0f * PI / 16384.0f;
  if (rad < 0)
    rad += 2.0f * PI;
  return rad;
}
void encoderGenericInit()
{
  pinMode(5, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(23, INPUT);
}
