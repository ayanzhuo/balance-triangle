#include "my_led_config.h"
// 追逐模式
void led_mode_chase()
{
    strip.clear();
    strip.setPixelColor(wrap(idx), led_config.color);
    strip.show();
    idx = wrap(idx + dir);
}

// 彗星模式
void led_mode_comet()
{
    fadeAll(led_config.tailDecay);
    strip.setPixelColor(wrap(idx), led_config.color);
    strip.show();
    idx = wrap(idx + dir);
}

// 反弹模式
void led_mode_bounce()
{
    strip.clear();
    strip.setPixelColor(wrap(idx), led_config.color);
    strip.show();
    idx += dir;
    if (idx >= (int)NUM_LEDS - 1)
    {
        idx = NUM_LEDS - 1;
        dir = -1;
    }
    else if (idx <= 0)
    {
        idx = 0;
        dir = 1;
    }
}

// 剧院模式
void led_mode_theater()
{
    strip.clear();
    for (uint16_t i = 0; i < NUM_LEDS; i++)
        if (((i + idx) % led_config.theaterStep) == 0)
            strip.setPixelColor(i, led_config.color);
    strip.show();
    idx = wrap(idx + dir);
}

// 彩虹模式
void led_mode_rainbow()
{
    for (uint16_t i = 0; i < NUM_LEDS; i++)
    {
        uint8_t hue = (uint8_t)((i * 256 / NUM_LEDS + phase) & 0xFF);
        strip.setPixelColor(i, wheel(hue));
    }
    strip.show();
    phase = (phase + led_config.rainbowSpeed) & 0xFFFF;
}

// 呼吸模式
void led_mode_breath()
{
    phase = (phase + led_config.breathSpeed) & 0x03FF; // 0..1023
    float t = (float)phase / 1024.0f;
    float s = 0.5f - 0.5f * cosf(2.0f * PI * t);
    uint8_t r = ((led_config.color >> 16) & 0xFF) * s;
    uint8_t g = ((led_config.color >> 8) & 0xFF) * s;
    uint8_t b = ((led_config.color) & 0xFF) * s;
    for (uint16_t i = 0; i < NUM_LEDS; i++)
        strip.setPixelColor(i, r, g, b);
    strip.show();
}
// 自定义模式1
void led_mode_custom1()
{
    // 自定义模式的实现
}
// 自定义模式2
void led_mode_custom2()
{
    // 自定义模式的实现
} // 自定义模式3
void led_mode_custom3()
{
    // 自定义模式的实现
}