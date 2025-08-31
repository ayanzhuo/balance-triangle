#include "my_io.h"

TwoWire my_wire = TwoWire(0); // 使用默认 I2C 总线 0
// SPIClass my_spi(VSPI);

void my_io_init()
{
    // I2C 初始化
    my_wire.begin(19, 18, 400000);
    // SPI 初始化
    // pinMode(PIN_CS, OUTPUT);
    // pinMode(PIN_SCK, OUTPUT);
    // pinMode(PIN_MISO, INPUT);
    // digitalWrite(PIN_CS, HIGH);
    // my_spi.begin(5, 23, -1, 22); // SCK, MISO, MOSI(-1 means not used), CS
    // my_spi.setFrequency(1000000);
    // my_spi.setBitOrder(SPI_MSBFIRST);
    // my_spi.setDataMode(SPI_MODE0);
}

float control_true_hz = 0;  // 全局变量，记录频率
uint64_t last_us = 0;
uint32_t count   = 0;
void freq_update()
{
    uint64_t now = esp_timer_get_time();  // 当前时间 (us)
    count++;
    if (now - last_us >= 1000000ULL) {   // 每 1 秒统计一次
        control_true_hz = (float)count * 1000000.0f / (now - last_us);
        count = 0;
        last_us = now;
    }
}