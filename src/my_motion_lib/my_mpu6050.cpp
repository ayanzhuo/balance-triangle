//
// Created by 22480 on 24-12-12.
//
#include "my_mpu6050.h"
#include "my_io.h"
// MPU6050实例

MPU6050 mpu6050(my_wire);
// 初始状态
float angleZ0 = 0;
float angleY0 = 0;
float angleX0 = 0;
// 当前状态
float now_angleZ = 0;
float now_angleY = 0;
float now_angleX = 0;
float now_gyroY = 0;
float now_gyroZ = 0;
float now_gyroX = 0;
void mpu6050_init()
{
    // mpu6050设置
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    Serial.println("MPU6050初始化完成");
    // 让IMU稳定一下再锁定初始航向（可按需要延长/多次均值）
    delay(200);
    mpu6050_update();
    angleX0 = now_angleX;
    angleY0 = now_angleY;
    angleZ0 = now_angleZ;
    Serial.println("MPU6050初始状态已更新");
}
// MPU6050更新,获取状态 角度制
void mpu6050_update()
{
    mpu6050.update();
    now_angleX = mpu6050.getAngleX();
    now_angleY = mpu6050.getAngleY();
    now_angleZ = mpu6050.getAngleZ();
    now_gyroX = mpu6050.getGyroX();
    now_gyroY = mpu6050.getGyroY();
    now_gyroZ = mpu6050.getGyroZ();
}
