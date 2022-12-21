#pragma once

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>


void makePANDA(char* panda, size_t len, UBX_NAV_PVT_data_t *pvt, int8_t latHp, int8_t lonHp, float yaw, float pitch, float roll);