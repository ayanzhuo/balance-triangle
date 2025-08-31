// 程序入口：初始化存储、UI/触摸与RGB、IMU与传感器融合、FOC电机、命令系统与WiFi/OTA；主循环执行FOC、UI触摸、周期性电池检测、IMU更新与平衡控制，并根据模式驱动电机与遥测。
#include <Arduino.h>
#include <SimpleFOC.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensor_fusion.h"

#include "my_io.h"
#include "my_foc.h"
#include "my_mpu6050.h"
#include "my_control.h"
#include "my_web.h"
#include "my_led.h"
#include "my_bat.h"
static TaskHandle_t data_send_TaskHandle = nullptr; // 遥测 FreeRTOS 任务
static TaskHandle_t control_TaskHandle = nullptr;   // 控制 FreeRTOS 任务
static TaskHandle_t led_TaskHandle = nullptr;       // LED FreeRTOS 任务
static TaskHandle_t bat_TaskHandle = nullptr;       // LED FreeRTOS 任务

// 三个图表的名称和三个通道的名称
ChartConfig chart_config[3] = {
    {"角速度", {"", "Y", ""}},
    {"角度跟踪", {"误差角度", "当前角度", "电机目标"}},
    {"其他", {"电池电压", "控制频率(100hz)", ""}},
};
// 12个滑块组的名称
SliderGroup slider_group[4] = {
    {"摇摆模式", {"晃动强度系数", "未使用", "未使用"}},
    {"未使用", {"未使用", "未使用", "未使用"}},
    {"未使用", {"未使用", "未使用", "未使用"}},
    {"未使用", {"未使用", "未使用", "未使用"}},
};
// 网页参数推送
void data_send_Task(void *)
{
  for (;;)
  {
    uint32_t dt_ms = my_web_data_update();
    // 这个地方可以放自己想看的参数
    // 姿态角
    send_msg[0] = now_angleX; // 摆角
    send_msg[1] = now_angleY; // 侧倾角
    send_msg[2] = now_angleZ; // 纵向角
    // 图1
    send_msg[3] = 0;         // 角速度X
    send_msg[4] = now_gyroZ; // 角速度Y
    send_msg[5] = 0;         // 角速度Z
    // 图2
    send_msg[6] = err_angle;           // 误差角度
    send_msg[7] = kalAngleZ - angleZ0; // 当前角度
    send_msg[8] = motion_target;       // 电机目标
    // 图3
    send_msg[9] = bat_voltage;
    send_msg[10] = control_true_hz / 100;
    send_msg[11] = 0;
    // 状态
    send_fall = blance_swingup; // 摇摆检测
    vTaskDelay(pdMS_TO_TICKS(dt_ms));
  }
}
// 运动控制
void robot_control_Task(void *)
{
  for (;;)
  {
    // 读取IMU
    mpu6050_update();
    // 卡尔曼滤波更新角度
    kalman_update();
    // 运动控制更新
    move_update();
    // 电机运动
    motor_update();
    // 控制频率计数
    freq_update();
    vTaskDelay(1.2); // 必须：避免饿死 idle
  }
}
// LED
void led_Task(void *)
{
  for (;;)
  {
    // 控制LED状态
    leds_update();
    vTaskDelay(pdMS_TO_TICKS(led_config.interval));
  }
}
// BAT检测
void bat_Task(void *)
{
  for (;;)
  {
    // 控制LED状态
    voltage_detection();
    vTaskDelay(pdMS_TO_TICKS(led_config.interval));
  }
}
void setup()
{
  Serial.begin(115200);
  // wifi初始
  Serial.println("初始化启动");
  my_wifi_init();
  Serial.println("WiFi初始化完成");
  // web初始化
  my_web_asyn_init();
  Serial.println("web初始化完成");
  // web ui文字传入
  my_web_ui_init(slider_group, chart_config);
  Serial.println("webui初始化完成");
  // io初始化
  my_io_init();
  Serial.println("IO初始化完成");
  // led初始化
  leds_init();
  Serial.println("led初始化完成");
  // 6050 初始化
  mpu6050_init();
  initWithPitch(acc2rotation(angleX0, angleY0));
  kalman_update();
  angleZ0 = kalAngleZ;
  Serial.println("6050初始化完成");
  // FOC 初始化
  motor_init();
  Serial.println("foc初始化完成");
  // 启动网页和控制任务
  xTaskCreatePinnedToCore(robot_control_Task, "control", 8192, nullptr, 12, &control_TaskHandle, 1);
  xTaskCreatePinnedToCore(data_send_Task, "websend", 8192, nullptr, 5, &data_send_TaskHandle, 0);
  xTaskCreatePinnedToCore(led_Task, "led", 1024, nullptr, 3, &led_TaskHandle, 0);
  xTaskCreatePinnedToCore(bat_Task, "bat", 1024, nullptr, 3, &bat_TaskHandle, 0);
}

void loop()
{
}
