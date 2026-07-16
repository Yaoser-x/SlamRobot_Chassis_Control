#include "app_hardware_init.h"

#include "line_sensor_driver.h"
#include "ssd1306.h"
#include "status_led_driver.h"

void AppHardware_InitStatusLed(void)
{
    StatusLedDriver_Init();
}

void AppHardware_InitLineSensor(const uint16_t threshold_raw[8], uint8_t active_low)
{
    LineSensorDriver_Init();
    LineSensorDriver_SetThresholdConfig(threshold_raw, active_low);
    LineSensorDriver_InitSensor();
}

void AppHardware_InitDisplay(void)
{
    SSD1306_Init();
}
