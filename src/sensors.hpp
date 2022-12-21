#pragma once
#include "main.hpp"

#include <queue.h>
#include <event_groups.h>

extern QueueHandle_t queueWAStoAutosteer;
extern QueueHandle_t imuToGNSSQueue;
extern EventGroupHandle_t switchesEventGroup;

struct IMUData
{
    float yaw;
    float pitch;
    float roll;
};

void init_sensors();