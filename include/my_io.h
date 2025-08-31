#include "Wire.h"
#include "SPI.h"

#define ACTIVE_PIN 4
#define BAT_VOLTAGE_SENSE_PIN 34

extern TwoWire my_wire;
extern SPIClass my_spi;
extern float control_true_hz;

void my_io_init();
void freq_update();