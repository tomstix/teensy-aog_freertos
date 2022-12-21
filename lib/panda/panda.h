#pragma once

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>


void makePANDA(char *panda, const size_t len, const UBX_NAV_PVT_data_t *pvt, const int8_t latHp, const int8_t lonHp, const float yaw, const float pitch, const float roll);