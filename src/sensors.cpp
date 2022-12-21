#include "sensors.hpp"
#include "settings.hpp"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <CMPS14.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_AHRS_NXPFusion.h>

Adafruit_ADS1115 ads;
CMPS14_TS cmps;
Adafruit_LSM6DSOX lsm6dsox;
Adafruit_LIS3MDL lis3mdl;
Adafruit_NXPSensorFusion sensorfusion;

QueueHandle_t queueWAStoAutosteer;
QueueHandle_t imuToGNSSQueue;

float mag_offsets[3] = {-5.75F, -8.88F, 9.33F};
// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = {{1.003, 0.025, -0.013},
                                   {0.025, 1.004, 0.004},
                                   {-0.013, 0.004, 0.994}};
float mag_field_strength = 59.49F;
// Offsets applied to compensate for gyro zero-drift error for x/y/z
// Raw values converted to rad/s based on 250dps sensitiviy (1 lsb = 0.00875 rad/s)
float rawToDPS = 0.00875F;
float dpsToRad = 0.017453293F;
float gyro_zero_offsets[3] = {0.0F * rawToDPS * dpsToRad,
                              0.0F * rawToDPS *dpsToRad,
                              0.0F * rawToDPS *dpsToRad};

void sensors_task(void *) // Task to poll I2C Sensors and Teensy Pins
{
    xEventGroupWaitBits(settings_loaded_event, 0x01, pdFALSE, pdFALSE, portMAX_DELAY); // wait for settings to be loaded
    Log.infoln("Starting Sensors Task...");

    TickType_t sensors_task_interval = pdMS_TO_TICKS(10);
    TickType_t last_sensors_task_t = 0;
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
        if (!lsm6dsox.begin_I2C())
        {
            Log.errorln("Failed to find LSM6DSOX!");
        }
        else
        {
            Log.infoln("LSM6DSOX found!");
            lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
            lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
        }
        if (!lis3mdl.begin_I2C())
        {
            Log.errorln("Failed to find LIS3MDL!");
        }
        else
        {
            Log.infoln("LIS3MDL found!");
        }
        sensorfusion.begin(100);
        break;
    }
    case (ImuType::NO_IMU):
    {
        Log.warningln("No IMU Type selected!");
        break;
    }
    }

    if (hardwareConfiguration.workswitchType == WorkswitchType::WORKSWITCH_PIN ||
        hardwareConfiguration.workswitchType == WorkswitchType::WORKSWITCH_ANALOG)
    {
        pinMode(hardwareConfiguration.workswitch_pin, arduino::OUTPUT);
    }
    if (hardwareConfiguration.steerswitchType == SteerswitchType::STEERSWITCH_PIN)
    {
        pinMode(hardwareConfiguration.steerswitch_pin, arduino::OUTPUT);
    }

    while (1)
    {
        last_sensors_task_t = xTaskGetTickCount();
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

        switch (hardwareConfiguration.imuType)
        {
        case (ImuType::CMPS14):
        {
            imuData.yaw = cmps.get_bearing();
            imuData.pitch = cmps.get_pitch();
            imuData.roll = cmps.get_roll();
            xQueueOverwrite(imuToGNSSQueue, &imuData);
            break;
        }
        case (ImuType::LSM6DS):
        {
            sensors_event_t gyro_event;
            sensors_event_t accel_event;
            sensors_event_t mag_event;
            sensors_event_t temp_event;
            lsm6dsox.getEvent(&accel_event, &gyro_event, &temp_event);
            lis3mdl.getEvent(&mag_event);
            // Apply mag offset compensation (base values in uTesla)
            float x = mag_event.magnetic.x - mag_offsets[0];
            float y = mag_event.magnetic.y - mag_offsets[1];
            float z = mag_event.magnetic.z - mag_offsets[2];
            // Apply mag soft iron error compensation
            float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
            float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
            float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
            // Apply gyro zero-rate error compensation
            float gx = gyro_event.gyro.x - gyro_zero_offsets[0];
            float gy = gyro_event.gyro.y - gyro_zero_offsets[1];
            float gz = gyro_event.gyro.z - gyro_zero_offsets[2];
            // The filter library expects gyro data in degrees/s, but adafruit sensor
            // uses rad/s so we need to convert them first (or adapt the filter lib
            // where they are being converted)
            gx *= 57.2958F;
            gy *= 57.2958F;
            gz *= 57.2958F;
            // Update the filter
            sensorfusion.update(gx, gy, gz,
                                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                                mx, my, mz);
            imuData.yaw = 360.0F - sensorfusion.getYaw();
            imuData.pitch = sensorfusion.getPitch();
            imuData.roll = sensorfusion.getRoll();
            xQueueOverwrite(imuToGNSSQueue, &imuData);
            break;
        }
        default:
            break;
        }

        
        if (hardwareConfiguration.workswitchType == WorkswitchType::WORKSWITCH_PIN)
        {

        }

        xTaskDelayUntil(&last_sensors_task_t, sensors_task_interval);
    }
}

void init_sensors()
{
    Wire.begin(400000);
    queueWAStoAutosteer = xQueueCreate(1, sizeof(float));
    imuToGNSSQueue = xQueueCreate(1, sizeof(IMUData));
    xTaskCreate(sensors_task, "Sensors Task", 1024, nullptr, 4, nullptr);
}