#include "comms_aog.hpp"
#include "main.hpp"
#include "settings.hpp"
#include "autosteer.hpp"
#include "sensors.hpp"

uint16_t STEER_TIMEOUT_MS = 1000;

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
            pinMode(hardwareConfiguration.output_pin_ena, OUTPUT);
        if (hardwareConfiguration.output_pin_enb != 0)
            pinMode(hardwareConfiguration.output_pin_enb, OUTPUT);
        if (hardwareConfiguration.output_pin_ina != 0)
            pinMode(hardwareConfiguration.output_pin_ina, OUTPUT);
        if (hardwareConfiguration.output_pin_inb != 0)
            pinMode(hardwareConfiguration.output_pin_inb, OUTPUT);
        pinMode(hardwareConfiguration.output_pin_pwm, OUTPUT);

        analogWriteRes(8);
    }

    if (hardwareConfiguration.steerswitchType == SteerswitchType::STEERSWITCH_PIN)
    {
        if (hardwareConfiguration.steerswitch_pin != 0)
        {
            pinMode(hardwareConfiguration.steerswitch_pin, INPUT_PULLUP);
            Log.verboseln("Steerswitch set to Pin %u.", hardwareConfiguration.steerswitch_pin);
        }
        else
        {
            Log.warningln("Steerswitch Pin selected but Pin is 0!");
        }
    }
    if (hardwareConfiguration.workswitchType == WorkswitchType::WORKSWITCH_PIN || hardwareConfiguration.workswitchType == WorkswitchType::WORKSWITCH_ANALOG)
    {
        if (hardwareConfiguration.workswitch_pin != 0)
        {
            pinMode(hardwareConfiguration.workswitch_pin, INPUT_PULLUP);
            Log.verboseln("Workswitch set to Pin %u.", hardwareConfiguration.workswitch_pin);
        }
        else
        {
            Log.warningln("Workswitch Pin selected but Pin is 0!");
        }
    }

    TickType_t TASK_INTERVAL_AUTOSTEER = pdMS_TO_TICKS(10);
    auto lastWakeTime = xTaskGetTickCount();
    while (1)
    {
        float angle_act;
        xQueueReceive(aogSteerDataQueue, &steerData, 0);
        xQueueReceive(queueWAStoAutosteer, &angle_act, pdMS_TO_TICKS(3));

        aogFromAutosteer.steer_angle = angle_act;

        if (millis() - steerData.timestamp_ms > STEER_TIMEOUT_MS)
        {
            steerData.guidanceStatus = false;
        }

        // Steer
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

        // Read Switches
        if (hardwareConfiguration.workswitchType == WorkswitchType::WORKSWITCH_PIN)
        {
            if (digitalReadFast(hardwareConfiguration.workswitch_pin) == LOW)
            {
                xEventGroupClearBits(switchesEventGroup, 0x01);
            }
            else
            {
                xEventGroupSetBits(switchesEventGroup, 0x01);
            }
        }

        if (hardwareConfiguration.steerswitchType == SteerswitchType::STEERSWITCH_PIN)
        {
            static uint32_t last_steerswitch_ms;
            bool steerswitch_state = (xEventGroupGetBits(switchesEventGroup) >> 1) & 0x01;
            auto set_steerswitch = [](bool on)
            {
                last_steerswitch_ms = millis();
                // Steerswitch at bit 1
                if (on)
                {
                    xEventGroupClearBits(switchesEventGroup, 0x02);
                    Log.verboseln("Steerswitch on!");
                }
                else
                {
                    xEventGroupSetBits(switchesEventGroup, 0x02);
                    Log.verboseln("Steerswitch off!");
                }
            };
            if (aog_steerConfig.SteerButton)
            {
                static bool previous_state;
                bool pressed = digitalReadFast(hardwareConfiguration.steerswitch_pin) == LOW;
                if (pressed && (pressed != previous_state) && (millis() - last_steerswitch_ms) > 50)
                {
                    set_steerswitch(steerswitch_state);
                }
                previous_state = pressed;
            }
            else if (aog_steerConfig.SteerSwitch)
            {
                if (digitalReadFast(hardwareConfiguration.steerswitch_pin) != steerswitch_state)
                {
                    set_steerswitch(steerswitch_state);
                }
            }
        }

        aogFromAutosteer.switches = (uint8_t)xEventGroupGetBits(switchesEventGroup);

        xQueueOverwrite(aogFromAutosteerQueue, &aogFromAutosteer);

        xTaskDelayUntil(&lastWakeTime, TASK_INTERVAL_AUTOSTEER);
    }
}

void init_autosteer()
{
    xTaskCreate(autosteer_task, "Autosteer Task", 2048, nullptr, 3, nullptr);
}