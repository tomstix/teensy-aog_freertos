#pragma once
#include "main.hpp"

#include <queue.h>

extern QueueHandle_t queueWAStoAutosteer;
extern QueueHandle_t queueIMUtoGNSS;

struct IMUData
{
    float yaw;
    float pitch;
    float roll;
};

void init_sensors();