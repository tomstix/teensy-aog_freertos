#include "comms_aog.hpp"
#include "main.hpp"
#include "settings.hpp"

void autosteer_task(void *)
{
    xEventGroupWaitBits(settings_loaded_event, 0x01, pdFALSE, pdFALSE, portMAX_DELAY);
    Log.infoln("Autosteer Task started!");
    AOG_SteerData steerData;
    Log.verboseln("Output Pins: ENA: %u, ENB: %u, INA: %u, INB: %u, PWM: %u", hardwareConfiguration.output_pin_ena, hardwareConfiguration.output_pin_enb, hardwareConfiguration.output_pin_ina, hardwareConfiguration.output_pin_inb, hardwareConfiguration.output_pin_pwm);
    while(1)
    {
        xQueueReceive(aogSteerDataQueue, &steerData, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void init_autosteer()
{
    xTaskCreate(autosteer_task, "Autosteer Task", 2048, nullptr, 3, nullptr);
}