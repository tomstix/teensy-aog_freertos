#pragma once
#include <Wire.h>

class CMPS14_TS
{
public:
    uint8_t begin(int address = 0x60);
    float get_bearing();
    float get_pitch();
    float get_roll();

private:
    uint8_t _version;
    int _address = 0x60;
};