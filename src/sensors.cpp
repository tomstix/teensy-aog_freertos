#include "sensors.hpp"
#include "settings.hpp"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <CMPS14.h>

Adafruit_ADS1115 ads;
CMPS14_TS cmps;

QueueHandle_t queueWAStoAutosteer;
QueueHandle_t imuToGNSSQueue;

void sensors_task(void *) // Task to poll I2C Sensors and Teensy Pins
{
    xEventGroupWaitBits(settings_loaded_event, 0x01, pdFALSE, pdFALSE, portMAX_DELAY); // wait for settings to be loaded
    Log.infoln("Starting Sensors Task...");

    TickType_t imu_poll_interval = pdMS_TO_TICKS(10);
    TickType_t last_imu_poll = xTaskGetTickCount();
    IMUData imuData;

    switch (hardwareConfiguration.wasType)
    {
    case (WASType::TEENSY):
    {
        if (hardwareConfiguration.teensy_was_pin_number < 1)
        {
            Log.errorln("When Teensy is selected as WAS Type, Pin Number can not be 0!");
        }
        else
        {
            pinMode(hardwareConfiguration.teensy_was_pin_number, arduino::INPUT);
            analogReadResolution(12);
            Log.traceln("WAS Input set to Teensy pin %u", hardwareConfiguration.teensy_was_pin_number);
        }
        break;
    }
    case (WASType::ADS1115):
    {
        if (!ads.begin())
        {
            Log.errorln("Failed to initialize ADS!");
        }
        ads.setGain(GAIN_ONE);
        auto ads_mode = ADS1X15_REG_CONFIG_MUX_SINGLE_0;
        if (hardwareConfiguration.ads1115_differential_mode)
        {
            switch (hardwareConfiguration.ads1115_was_pin)
            {
            case (0):
                ads_mode = ADS1X15_REG_CONFIG_MUX_DIFF_0_1;
                break;
            case (2):
                ads_mode = ADS1X15_REG_CONFIG_MUX_DIFF_2_3;
                break;
            default:
                Log.warningln("Invalid ADS Pin Setting in Differential Mode!");
            }
        }
        else
        {
            switch (hardwareConfiguration.ads1115_was_pin)
            {
            case 0:
                ads_mode = ADS1X15_REG_CONFIG_MUX_SINGLE_0;
                break;
            case 1:
                ads_mode = ADS1X15_REG_CONFIG_MUX_SINGLE_1;
                break;
            case 2:
                ads_mode = ADS1X15_REG_CONFIG_MUX_SINGLE_2;
                break;
            case 3:
                ads_mode = ADS1X15_REG_CONFIG_MUX_SINGLE_3;
                break;
            default:
                Log.warningln("Invalid ADS Pin Settin in Single Mode!");
            }
        }
        ads.startADCReading(ads_mode, true);
        break;
    }
    case (WASType::ISOBUS_GMS):
    {
        // TODO
        break;
    }
    case (WASType::NO_WAS):
    {
        Log.warningln("No WAS Type selected!");
        break;
    }
    }

    switch (hardwareConfiguration.imuType)
    {
    case (ImuType::BNO085):
    {
        // TODO
        break;
    }
    case (ImuType::CMPS14):
    {
        auto ver = cmps.begin();
        Log.infoln("CMPS Version: %u", ver);
        break;
    }
    case (ImuType::LSM6DS):
    {
        // TODO
        break;
    }
    case (ImuType::NO_IMU):
    {
        Log.warningln("No IMU Type selected!");
        break;
    }
    }

    while (1)
    {
        switch (hardwareConfiguration.wasType)
        {
        case (WASType::TEENSY):
        {
            int16_t was_raw = analogRead(hardwareConfiguration.teensy_was_pin_number) - 2048;
            float was_value = (float)(was_raw + aog_steerSettings.wasOffset) / (float)aog_steerSettings.steerSensorCounts;
            was_value = aog_steerConfig.InvertWAS ? -was_value : was_value;
            if (was_value < 0)
                was_value *= aog_steerSettings.ackermannFix;
            xQueueOverwrite(queueWAStoAutosteer, &was_value);
            break;
        }
        case (WASType::ADS1115):
        {
            int16_t was_raw = ads.getLastConversionResults() - 13200;
            was_raw = aog_steerConfig.InvertWAS ? -was_raw : was_raw;
            float was_value = (float)(was_raw + aog_steerSettings.wasOffset * 2) / (float)(aog_steerSettings.steerSensorCounts * 2);
            if (was_value < 0)
                was_value *= aog_steerSettings.ackermannFix;
            xQueueOverwrite(queueWAStoAutosteer, &was_value);
            break;
        }
        default:
            break;
        }

        if (xTaskGetTickCount() - last_imu_poll > imu_poll_interval)
        {
            last_imu_poll = xTaskGetTickCount();
            switch (hardwareConfiguration.imuType)
            {
            case (ImuType::CMPS14):
            {
                imuData.yaw = cmps.get_bearing();
                imuData.pitch = cmps.get_pitch();
                imuData.roll = cmps.get_roll();
                xQueueOverwrite(imuToGNSSQueue, &imuData);
            }
            default:
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void init_sensors()
{
    Wire.begin(400000);
    queueWAStoAutosteer = xQueueCreate(1, sizeof(float));
    imuToGNSSQueue = xQueueCreate(1, sizeof(IMUData));
    xTaskCreate(sensors_task, "Sensors Task", 1024, nullptr, 4, nullptr);
}