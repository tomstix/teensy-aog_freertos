#include "CMPS14.h"

uint16_t request_uint16(int address, int readReg)
{
    Wire.beginTransmission(address);
    Wire.write(readReg);
    Wire.endTransmission();
    Wire.requestFrom(address, 2);
    uint16_t value = Wire.read() << 8 | Wire.read();
    return value;
}

int8_t request_int8(int address, int readReg)
{
    Wire.beginTransmission(address);
    Wire.write(readReg);
    Wire.endTransmission();
    Wire.requestFrom(address, 1);
    return Wire.read();
}

uint8_t request_uint8(int address, int readReg)
{
    Wire.beginTransmission(address);
    Wire.write(readReg);
    Wire.endTransmission();
    Wire.requestFrom(address, 1);
    return Wire.read();
}

int16_t request_int16(int address, int readReg)
{
    Wire.beginTransmission(address);
    Wire.write(readReg);
    Wire.endTransmission();
    Wire.requestFrom(address, 2);
    int16_t value = (Wire.read() << 8) | Wire.read();
    return value;
}

uint8_t CMPS14_TS::begin(int address)
{
    _address = address;
    _version = request_uint8(address, 0x00);
    return _version;
}

float CMPS14_TS::get_bearing()
{
    return (float)request_uint16(_address, 0x02) / 10.0;
}
float CMPS14_TS::get_pitch()
{
    return (float)request_int16(_address, 0x1A) / 10.0;
}
float CMPS14_TS::get_roll()
{
    return (float)request_int16(_address, 0x1C) / 10.0;
}