#include "comms_aog.hpp"
#include "settings.hpp"

constexpr uint32_t AOG_PORT_STEER_RECEIVE = 8888; // Port to listen to
constexpr uint32_t AOG_SEND_PORT = 9999;          // Port to send to
constexpr uint32_t AOG_NTRIP_PORT = 2233;         // Port to listen for NMEA

QueueHandle_t aogSteerDataQueue;
QueueHandle_t aogSteerConfigQueue;
QueueHandle_t aogSteerSettingsQueue;
QueueHandle_t aogFromAutosteerQueue;
MessageBufferHandle_t sendNMEABuffer;
StreamBufferHandle_t ntripStreamBuffer;

using namespace qindesign::network;

AOG_SteerData::AOG_SteerData() {}

AOG_SteerData::AOG_SteerData(const uint8_t *buf, size_t len)
{
    this->parse(buf, len);
}

int AOG_SteerData::parse(const uint8_t *buf, size_t len)
{
    if (len != 14)
    {
        Log.errorln("AOG SteerData to parse doesn't match expected length!");
        return EXIT_FAILURE;
    }
    speed = ((float)(buf[5] | buf[6] << 8)) * 0.1;
    guidanceStatus = buf[7];
    steerAngleSetpoint = (float)((int16_t)(buf[8] | buf[9] << 8)) * 0.01;
    tramline = buf[10];
    sections = buf[11] << 8 | buf[12];

    return EXIT_SUCCESS;
}

int AOG_FromAutoSteer::get_buf(uint8_t *buf, const size_t len)
{
    if (len != 14)
    {
        Log.errorln("Buffer provided for Message From AutoSteer has incorrect size!");
        return EXIT_FAILURE;
    }
    buf[0] = 0x80;
    buf[1] = 0x81;
    buf[2] = 0x7F;
    buf[3] = 0xFD;
    buf[4] = 8;
    int16_t angle = (int16_t)(this->steer_angle * 100);
    buf[5] = (uint8_t)angle;
    buf[6] = angle >> 8;
    buf[7] = (uint8_t)9999;
    buf[8] = 9999 >> 8;
    buf[9] = (uint8_t)8888;
    buf[10] = 8888 >> 8;
    buf[11] = this->switches;
    buf[12] = this->pwm_display;

    int chk = 0;
    for (uint8_t i = 2; i < 13; i++)
    {
        chk = (chk + buf[i]);
    }
    buf[13] = chk;
    return EXIT_SUCCESS;
}

void ethernet_task(void *)
{
    xEventGroupWaitBits(settings_loaded_event, 0x01, pdFALSE, pdFALSE, portMAX_DELAY);
    Log.traceln("Ethernet Task started!");
    uint8_t mac[6];
    Ethernet.macAddress(mac);
    Log.infoln("MAC Address: %x::%x::%x::%x::%x::%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Log.infoln("IP Address: %p", networkConfiguration.ip);
    Ethernet.onLinkState(
        [](bool state)
        {
            if (state)
                Log.infoln("Ethernet Link detected. Running at %u Mbps.", Ethernet.linkSpeed());
            else
                Log.errorln("Ethernet Link lost!");
        });
    Ethernet.begin(networkConfiguration.ip, networkConfiguration.netmask, networkConfiguration.gateway);
    while (1)
    {
        Ethernet.loop();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void aog_udp_task(void *)
{
    xEventGroupWaitBits(settings_loaded_event, 0x01, pdFALSE, pdFALSE, portMAX_DELAY);
    while (!Ethernet.linkState())
        vTaskDelay(1);
    EthernetUDP aogUDP(10);
    aogUDP.begin(AOG_PORT_STEER_RECEIVE);
    EthernetUDP ntripUDP(10);
    ntripUDP.begin(AOG_NTRIP_PORT);

    Log.infoln("AOG UDP Task started!");

    AOG_SteerData steerData;
    AOG_SteerConfig steerConfig;
    AOG_SteerSettings steerSettings;

    char panda[256];

    while (1)
    {
        if (xMessageBufferReceive(sendNMEABuffer, panda, 256, pdMS_TO_TICKS(10)))
        {
            aogUDP.send(Ethernet.broadcastIP(), AOG_SEND_PORT, (const uint8_t *)panda, strlen(panda));
        }

        auto aog_packet_size = aogUDP.parsePacket();
        if (aog_packet_size >= 0)
        {
            static uint8_t aog_rx_buf[256];
            aogUDP.read(aog_rx_buf, sizeof(aog_rx_buf));
            if (aog_rx_buf[0] == 0x80 && aog_rx_buf[1] == 0x81 && aog_rx_buf[2] == 0x7F)
            {
                auto pgn = aog_rx_buf[3];
                switch (pgn)
                {
                case (AOG_SteerData::pgn):
                {
                    if (steerData.parse(aog_rx_buf, aog_packet_size) != EXIT_SUCCESS)
                        break;
                    steerData.timestamp_ms = millis();
                    if (xQueueSend(aogSteerDataQueue, &steerData, 0) != pdTRUE)
                    {
                        Log.warningln("AOG Steer Data Queue Full!");
                    }
                    AOG_FromAutoSteer aogFromAutosteer;
                    xQueueReceive(aogFromAutosteerQueue, &aogFromAutosteer, pdMS_TO_TICKS(10));
                    uint8_t buf[14];
                    aogFromAutosteer.get_buf(buf, sizeof(buf));
                    aogUDP.send(Ethernet.broadcastIP(), AOG_SEND_PORT, buf, sizeof(buf));
                    break;
                }
                case (AOG_SteerConfig::pgn):
                {
                    steerConfig.parse(aog_rx_buf);
                    xQueueSend(aogSteerConfigQueue, &steerConfig, pdMS_TO_TICKS(1000));
                    break;
                }
                case (AOG_SteerSettings::pgn):
                {
                    steerSettings.parse(aog_rx_buf);
                    xQueueSend(aogSteerSettingsQueue, &steerSettings, pdMS_TO_TICKS(1000));
                    break;
                }
                }
            }
            else
            {
                Log.warningln("Received AOG UDP packet, that doesn't match 0x80817F - scheme.");
            }
        }

        auto ntrip_packetsize = ntripUDP.parsePacket();
        if (ntrip_packetsize >= 0)
        {
            static uint8_t ntrip_buf[512];
            ntripUDP.read(ntrip_buf, sizeof(ntrip_buf));
            xStreamBufferSend(ntripStreamBuffer, &ntrip_buf, ntrip_packetsize, pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void init_aog_comms()
{
    aogSteerDataQueue = xQueueCreate(1, sizeof(AOG_SteerData));
    if (aogSteerDataQueue == NULL)
        Log.errorln("Failed to create aogSteerDataQueue!");
    aogSteerSettingsQueue = xQueueCreate(1, sizeof(AOG_SteerSettings));
    if (aogSteerSettingsQueue == NULL)
        Log.errorln("Failed to create aogSteerSettingsQueue!");
    aogSteerConfigQueue = xQueueCreate(1, sizeof(AOG_SteerConfig));
    if (aogSteerConfigQueue == NULL)
        Log.errorln("Failed to create aogSteerDataQueue!");
    aogFromAutosteerQueue = xQueueCreate(1, sizeof(AOG_FromAutoSteer));
    if (aogFromAutosteerQueue == NULL)
        Log.errorln("Failed to create aogFromAutosteerQueue!");
    sendNMEABuffer = xMessageBufferCreate(512);
    if (sendNMEABuffer == NULL)
        Log.errorln("Failed to create sendNMEABuffer!");
    ntripStreamBuffer = xStreamBufferCreate(1024, 16);
    if (ntripStreamBuffer == NULL)
        Log.errorln("Failed to create ntripStreamBuffer!");

    xTaskCreate(ethernet_task, "Ethernet Task", 1024, nullptr, 4, nullptr);
    xTaskCreate(aog_udp_task, "AOG UDP Task", 2048, nullptr, 3, nullptr);
}