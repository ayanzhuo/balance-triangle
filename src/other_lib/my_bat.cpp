#include "my_bat.h"
#include "Arduino.h"
#include "my_io.h"

double bat_voltage;
const double min_voltage = 9;     //电池检测最低电压
bool battery_low = 0;
const double R1_VOLTAGE = 62000;  //62K
const double R2_VOLTAGE = 10000;  //10K


double return_voltage_value(int pin_no) {
  double tmp;
  double ADCVoltage;
  double inputVoltage;
  analogSetPinAttenuation(pin_no, ADC_6db);

  for (int i = 0; i < 20; i++) {
    ADCVoltage = analogReadMilliVolts(pin_no) / 1000.0;
    inputVoltage = (ADCVoltage * R1_VOLTAGE) / R2_VOLTAGE;

    tmp = tmp + inputVoltage + ADCVoltage;  // formula for calculating voltage in i.e. GND
  }
  inputVoltage = tmp / 20;
  if (inputVoltage != 0)
    inputVoltage = inputVoltage + 0.001;
  return inputVoltage;
}
void voltage_detection() {
  bat_voltage = return_voltage_value(BAT_VOLTAGE_SENSE_PIN);
  //driver.voltage_power_supply = bat_voltage;
  //Serial.println(driver.voltage_power_supply);
  if (bat_voltage < min_voltage && !battery_low) {
    battery_low = 1;  
}else  {
    battery_low = 0;
  }
  
}