#pragma once

#include "main.hpp"

#include <queue.h>
#include <message_buffer.h>
#include <QNEthernet.h>

extern QueueHandle_t aogSteerDataQueue;
extern QueueHandle_t aogSteerConfigQueue;
extern QueueHandle_t aogSteerSettingsQueue;
extern QueueHandle_t aogFromAutosteerQueue;
extern MessageBufferHandle_t sendNMEABuffer;
extern StreamBufferHandle_t ntripStreamBuffer;

struct AOG_SteerData
{
    const static uint16_t pgn = 0xFE;

    AOG_SteerData();
    AOG_SteerData(const uint8_t* buf, size_t len);

    float speed = 0.0;
    bool guidanceStatus = 0;
    float steerAngleSetpoint = 0.0;
    uint8_t tramline = 0;
    uint16_t sections = 0;

    int parse(const uint8_t* buf, size_t len);
};

struct AOG_FromAutoSteer
{
    float steer_angle = 69.0;
    float imu_heading = 0.0;
    float imu_roll = 0.0;
    uint8_t switches = 0;
    uint8_t pwm_display = 1;

    int get_buf(uint8_t* buf, const size_t len);
};

void init_aog_comms();