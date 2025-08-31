// FOC电机控制接口：导出BLDCMotor对象与初始化、FOC循环、目标输出与控制模式切换的函数原型。
#ifndef MOTOR_FOC_MODULE_H
#define MOTOR_FOC_MODULE_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include "SPI.h"
// 对外暴露电机对象以便其它模块读取 shaft_velocity 等
extern BLDCMotor motor;

void motor_init();
void motor_update();

void encoderGenericInit();
float encoderGenericRead();

#endif // MOTOR_FOC_MODULE_H