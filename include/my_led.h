#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// 硬件参数
constexpr uint8_t LED_PIN = 16;
constexpr uint16_t NUM_LEDS = 21;
constexpr uint8_t LED_TYPE = NEO_GRB + NEO_KHZ800;

struct LedConfig
{
    int mode = 0;              // 模式
    int8_t dir = 1;            // 方向
    uint16_t interval = 40;    // 间隔
    uint8_t brightness = 80;   // 亮度
    uint32_t color = 0x1EC8FF; // 0xRRGGBB
    uint8_t tailDecay = 48;    // 尾迹衰减
    uint8_t theaterStep = 3;   // 剧院模式步进
    uint8_t breathSpeed = 8;   // 呼吸模式速度
    uint8_t rainbowSpeed = 4;  // 彩虹模式速度
    bool powerOn = false;
};

extern LedConfig led_config;

void leds_init();
void leds_update(); // 每帧调用（非阻塞）
void leds_apply();

void leds_power(bool on);
