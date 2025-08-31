//
// Created by 22480 on 24-12-12.
//

#include <MPU6050_tockn.h>
// MPU6050实例
void mpu6050_init();
void mpu6050_update();

extern float angleZ0;
extern float angleY0;
extern float angleX0;

extern float now_angleZ;
extern float now_angleY;
extern float now_angleX;
extern float now_gyroY;
extern float now_gyroZ;
extern float now_gyroX;
