// 一维Kalman滤波实现：融合角度与角速度测量，提供滤波角、无偏角速度与可调噪声参数接口。
#include <Arduino.h>
#include "Kalman.h"

// Kalman类的构造函数实现。
// 作用：设置默认的噪声协方差参数和初始状态。
// 为什么这样写：这些默认值基于典型IMU传感器的噪声特性，选择合适的值能获得良好滤波效果。
Kalman::Kalman()
{
    // 构造函数初始化
    Q_angle = 0.001f;  // 过程噪声协方差（角度）
    Q_bias = 0.003f;   // 过程噪声协方差（偏差）
    R_measure = 0.03f; // 测量噪声协方差

    angle = 0.0f; // 初始化角度
    bias = 0.0f;  // 初始化偏差

    // 初始化误差协方差矩阵P
    P[0][0] = 0.0f; // 角度估计误差
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 0.0f;
}

// 核心滤波方法
// 作用：实现Kalman滤波的预测和更新步骤，融合新测量数据。
// 为什么这样写：遵循标准Kalman滤波公式，简化为一维以减少计算量。
float Kalman::getAngle(float newAngle, float newRate, float dt)
{
    // 1. 时间更新步骤（预测阶段）
    rate = newRate - bias; // 去除偏差的角速度
    angle += dt * rate;    // 预测当前角度

    // 2. 更新协方差矩阵P
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // 3. 测量更新步骤（校正阶段）
    float S = P[0][0] + R_measure; // 计算创新协方差
    float K[2];                    // 卡尔曼增益向量

    K[0] = P[0][0] / S; // 计算卡尔曼增益
    K[1] = P[1][0] / S;

    float y = newAngle - angle; // 计算测量残差
    angle += K[0] * y;          // 更新角度估计
    bias += K[1] * y;           // 更新偏差估计

    // 4. 更新协方差矩阵
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle; // 返回滤波后的角度
}

// 设置角度（用于初始化）
// 作用：直接设置滤波器的角度状态。
// 为什么这样写：用于系统启动时设置初始姿态。
void Kalman::setAngle(float angle)
{
    this->angle = angle;
}

// 获取校正后的角速度
// 作用：返回内部计算的无偏角速度。
// 为什么这样写：允许外部访问滤波结果的另一个部分。
float Kalman::getRate()
{
    return this->rate;
}

// 参数调优方法
// 作用：设置过程噪声协方差（角度）。
// 为什么这样写：允许动态调整以适应不同传感器或环境。
void Kalman::setQangle(float Q_angle)
{
    this->Q_angle = Q_angle;
}

void Kalman::setQbias(float Q_bias)
{
    this->Q_bias = Q_bias;
}

void Kalman::setRmeasure(float R_measure)
{
    this->R_measure = R_measure;
}

// 获取当前参数值
float Kalman::getQangle()
{
    return this->Q_angle;
}

float Kalman::getQbias()
{
    return this->Q_bias;
}

float Kalman::getRmeasure()
{
    return this->R_measure;
}