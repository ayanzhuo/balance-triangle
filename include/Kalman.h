// Kalman滤波器接口：声明一维Kalman类的API（获取角度、设置初值、调参与读取参数），用于姿态估计。
#ifndef _Kalman_h_
#define _Kalman_h_

// 定义Kalman类，用于实现一维Kalman滤波器，主要用于角度估计。
// 作用：在自平衡系统中，融合加速计和陀螺仪数据，减少噪声，提供准确的角度估计。
// 为什么这样写：Kalman滤波是标准算法，这里简化为一维版本，适合嵌入式实时计算。
class Kalman
{
public:
    // 构造函数。
    // 作用：初始化Kalman滤波器的参数和状态。
    // 为什么这样写：默认参数值基于经验，适合大多数IMU传感器。
    Kalman();

    // 获取滤波后的角度。
    // 参数：newAngle - 来自加速计的角度；newRate - 来自陀螺仪的角速度；dt - 时间间隔。
    // 作用：执行Kalman滤波算法，融合两个传感器数据返回估计角度。
    // 为什么这样写：这是Kalman滤波的核心，实现预测和更新步骤。
    float getAngle(float newAngle, float newRate, float dt);

    // 设置初始角度。
    // 作用：用于初始化滤波器的角度状态。
    // 为什么这样写：允许外部设置起始角度，例如从静止状态开始。
    void setAngle(float angle); // Used to set angle, this should be set as the starting angle

    // 获取无偏角速度。
    // 作用：返回滤波后的角速度（去除偏差）。
    // 为什么这样写：提供访问内部状态的方法，便于调试或进一步使用。
    float getRate(); // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    // 设置过程噪声协方差（角度）。
    // 作用：调整滤波器对过程噪声的敏感度。
    // 为什么这样写：允许用户根据具体传感器调优性能。
    void setQangle(float Q_angle);
    /**
     * setQbias(float Q_bias)
     * Default value (0.003f) is in Kalman.cpp.
     * Raise this to follow input more closely,
     * lower this to smooth result of kalman filter.
     */
    void setQbias(float Q_bias);

    // 设置测量噪声协方差。
    // 作用：调整滤波器对测量噪声的信任度。
    // 为什么这样写：高值表示测量不可靠，滤波更平滑。
    void setRmeasure(float R_measure);

    // 获取当前参数值的方法。
    // 作用：允许读取当前调优参数。
    // 为什么这样写：便于调试和参数管理。
    float getQangle();
    float getQbias();
    float getRmeasure();

private:
    /* Kalman filter variables */
    float Q_angle;   // Process noise variance for the accelerometer
    float Q_bias;    // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias;  // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate;  // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
};

#endif
