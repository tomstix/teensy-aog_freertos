#pragma once

struct SteerSetpoints
{
    uint16_t was_raw = 0;
    bool guidance_status = 0;
    float angle_requested = 0.0;
    float angle_actual = 0.0;
    uint16_t pid_output = 0;
    float speed = 0.0;
};

void init_autosteer();