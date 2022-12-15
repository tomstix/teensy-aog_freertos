#include "sensors.hpp"
#include "settings.hpp"
#include <Wire.h>

void sensors_task(void *)
{
    xEventGroupWaitBits(settings_loaded_event, 0x01, pdFALSE, pdFALSE, portMAX_DELAY);
    Log.infoln("Starting Sensors Task...");
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void init_sensors()
{
    Wire.begin(400000);
    xTaskCreate(sensors_task, "Sensors Task", 1024, nullptr, 4, nullptr);
}