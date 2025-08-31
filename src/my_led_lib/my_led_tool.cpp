#include "my_led_config.h"

uint16_t wrap(int32_t x)
{
    int32_t n = NUM_LEDS;
    int32_t m = x % n;
    return (m < 0) ? (m + n) : m;
}
uint32_t wheel(uint8_t pos)
{
    pos = 255 - pos;
    if (pos < 85)
        return strip.Color(255 - pos * 3, 0, pos * 3);
    if (pos < 170)
    {
        pos -= 85;
        return strip.Color(0, pos * 3, 255 - pos * 3);
    }
    pos -= 170;
    return strip.Color(pos * 3, 255 - pos * 3, 0);
}
void fadeAll(uint8_t decay)
{
    for (uint16_t i = 0; i < NUM_LEDS; i++)
    {
        uint32_t c = strip.getPixelColor(i);
        uint8_t r = (c >> 16) & 0xFF, g = (c >> 8) & 0xFF, b = c & 0xFF;
        r = (r > decay) ? (r - decay) : 0;
        g = (g > decay) ? (g - decay) : 0;
        b = (b > decay) ? (b - decay) : 0;
        strip.setPixelColor(i, r, g, b);
    }
}
