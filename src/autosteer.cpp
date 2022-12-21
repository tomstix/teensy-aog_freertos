#include "comms_aog.hpp"
#include "main.hpp"
#include "settings.hpp"
#include "autosteer.hpp"
#include "sensors.hpp"

uint16_t steer_timeout_ms = 1000;

void autosteer_task(void *)
{
    xEventGroupWaitBits(settings_loaded_event, 0x01, pdFALSE, pdFALSE, portMAX_DELAY);
    Log.infoln("Autosteer Task started!");
    AOG_SteerData steerData;
    AOG_FromAutoSteer aogFromAutosteer;
    vTaskDelay(pdMS_TO_TICKS(500));

    if (hardwareConfiguration.outputType == OutputType::PWM)
    {
        if (hardwareConfiguration.output_pin_ena != 0)
            pinMode(hardwareConfiguration.output_pin_ena, arduino::OUTPUT);
        if (hardwareConfiguration.output_pin_enb != 0)
            pinMode(hardwareConfiguration.output_pin_enb, arduino::OUTPUT);
        if (hardwareConfiguration.output_pin_ina != 0)
            pinMode(hardwareConfiguration.output_pin_ina, arduino::OUTPUT);
        if (hardwareConfiguration.output_pin_inb != 0)
            pinMode(hardwareConfiguration.output_pin_inb, arduino::OUTPUT);
        pinMode(hardwareConfiguration.output_pin_pwm, arduino::OUTPUT);

        analogWriteRes(8);
    }

    if (hardwareConfiguration.steerswitchType == SteerswitchType::STEERSWITCH_PIN)
    {
        if (hardwareConfiguration.steerswitch_pin != 0)
        {
            pinMode(hardwareConfiguration.steerswitch_pin, arduino::INPUT);
            Log.verboseln("Steerswitch set to Pin %u.", hardwareConfiguration.steerswitch_pin);
        }
        else
        {
            Log.warningln("Steerswitch Pin selected but Pin is 0!");
        }
    }
    if (hardwareConfiguration.workswitchType == WorkswitchType::WORKSWITCH_PIN
        || hardwareConfiguration.workswitchType == WorkswitchType::WORKSWITCH_ANALOG)
    {
        if (hardwareConfiguration.workswitch_pin != 0)
        {
            pinMode(hardwareConfiguration.workswitch_pin, arduino::INPUT);
            Log.verboseln("Workswitch set to Pin %u.", hardwareConfiguration.steerswitch_pin);
        }
        else
        {
            Log.warningln("Workswitch Pin selected but Pin is 0!");
        }
    }

    TickType_t time_interval = pdMS_TO_TICKS(10);
    auto lastWakeTime = xTaskGetTickCount();
    while (1)
    {
        float angle_act;
        xQueueReceive(aogSteerDataQueue, &steerData, 0);
        xQueueReceive(queueWAStoAutosteer, &angle_act, pdMS_TO_TICKS(3));

        aogFromAutosteer.steer_angle = angle_act;

        if (millis() - steerData.timestamp_ms > steer_timeout_ms)
        {
            steerData.guidanceStatus = false;
        }

        //Steer
        switch (hardwareConfiguration.outputType)
        {
        case (OutputType::PWM):
        {
            if (steerData.guidanceStatus)
            {
                float angle_err = steerData.steerAngleSetpoint - angle_act;
                int16_t p_gain = (int)(angle_err * (float)aog_steerSettings.kp);
                uint16_t p_gain_abs = abs(p_gain) + aog_steerSettings.minPWM;
                bool dir = p_gain > 1;
                if (aog_steerConfig.MotorDriveDirection)
                    dir = !dir;
                float highLowPerDeg = ((float)(aog_steerSettings.highPWM - aog_steerSettings.lowPWM)) / 5.0;
                uint8_t max_pwm = aog_steerSettings.highPWM;
                if (abs(angle_err) < 5.0)
                {
                    max_pwm = (abs(angle_err) * highLowPerDeg) + aog_steerSettings.lowPWM;
                }
                if (p_gain_abs > max_pwm)
                    p_gain_abs = max_pwm;

                analogWrite(hardwareConfiguration.output_pin_pwm, p_gain_abs);
                digitalWriteFast(hardwareConfiguration.output_pin_ena, 1);
                digitalWriteFast(hardwareConfiguration.output_pin_enb, 1);
                digitalWriteFast(hardwareConfiguration.output_pin_ina, dir);
                digitalWriteFast(hardwareConfiguration.output_pin_inb, !dir);
                aogFromAutosteer.pwm_display = p_gain_abs;
                break;
            }
            else
            {
                analogWrite(hardwareConfiguration.output_pin_pwm, 0);
                digitalWriteFast(hardwareConfiguration.output_pin_ena, 0);
                digitalWriteFast(hardwareConfiguration.output_pin_enb, 0);
                digitalWriteFast(hardwareConfiguration.output_pin_ina, 0);
                digitalWriteFast(hardwareConfiguration.output_pin_inb, 0);
                aogFromAutosteer.pwm_display = 0;
            }
        }
        default:
            break;
        }

        //Read Switches

        xQueueOverwrite(aogFromAutosteerQueue, &aogFromAutosteer);

        xTaskDelayUntil(&lastWakeTime, time_interval);
    }
}

void init_autosteer()
{
    xTaskCreate(autosteer_task, "Autosteer Task", 2048, nullptr, 3, nullptr);
}