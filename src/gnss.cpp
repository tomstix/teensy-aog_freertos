#include "gnss.hpp"
#include "settings.hpp"
#include "sensors.hpp"
#include "comms_aog.hpp"

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <panda.h>

#define GNSS_PORT Serial4

SFE_UBLOX_GNSS gnss;

char panda_buf[256];

void pvt_callback(UBX_NAV_PVT_data_t* pvt)
{
    IMUData imuData;
    xQueueReceive(imuToGNSSQueue, &imuData, 0);
    gnss.getHPPOSLLH(5);
    int8_t latHp = gnss.getHighResLatitudeHp();
    int8_t lonHp = gnss.getHighResLongitudeHp();
    makePANDA(panda_buf, sizeof(panda_buf), pvt, latHp, lonHp, imuData.yaw, imuData.pitch, imuData.roll);
    xMessageBufferSend(sendNMEABuffer, panda_buf, sizeof(panda_buf), pdMS_TO_TICKS(10));
}


void gnss_task(void *)
{
    xEventGroupWaitBits(settings_loaded_event, 0x01, pdFALSE, pdFALSE, portMAX_DELAY);
    Log.traceln("Starting GNSS...");
    GNSS_PORT.begin(115200);
    if (gnss.begin(GNSS_PORT) == false)
    {
        Log.fatalln("Failed to initialize GNSS!");
        while (1)
            ;
    }
    Log.traceln("GNSS initialized!");
    gnss.setUART1Output(COM_TYPE_UBX);
    gnss.setNavigationFrequency(10);
    gnss.setAutoPVT(true);
    gnss.setAutoHPPOSLLH(true);
    gnss.setAutoPVTcallbackPtr(pvt_callback);

    while (1)
    {
        gnss.checkUblox();
        gnss.checkCallbacks();
        if ((xStreamBufferIsEmpty(ntripStreamBuffer) == pdFALSE) && GNSS_PORT.availableForWrite())
        {
            auto available = GNSS_PORT.availableForWrite();
            uint8_t buf[available];
            auto written = xStreamBufferReceive(ntripStreamBuffer, buf, available, 0);
            GNSS_PORT.write(buf, written);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void init_gnss()
{
    xTaskCreate(gnss_task, "GNSS Task", 4096, nullptr, 5, nullptr);
}