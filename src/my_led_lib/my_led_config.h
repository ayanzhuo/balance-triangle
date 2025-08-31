#include "my_led.h"
#include "Adafruit_NeoPixel.h"

// 模式定义

extern Adafruit_NeoPixel strip;
extern uint32_t tPrev;
extern int32_t idx;
extern int8_t dir;
extern uint16_t phase;

void led_mode_chase();
void led_mode_comet();
void led_mode_bounce();
void led_mode_theater();
void led_mode_rainbow();
void led_mode_breath();
void led_mode_custom1();
void led_mode_custom2();
void led_mode_custom3();

uint16_t wrap(int32_t x);
uint32_t wheel(uint8_t pos);
void fadeAll(uint8_t decay);