#pragma once

#include "main.hpp"

#include <event_groups.h>
extern EventGroupHandle_t settings_loaded_event;

#include <json.hpp>
using json = nlohmann::json;

#include <QNEthernet.h>
using namespace qindesign::network;

const char settings_file[] = "settings.json";

enum ImuType
{
	NO_IMU,
	CMPS14,
	BNO085,
	LSM6DS
};
NLOHMANN_JSON_SERIALIZE_ENUM(ImuType, {{NO_IMU, "NO_IMU"},
									   {CMPS14, "CMPS14"},
									   {BNO085, "BNO085"},
									   {LSM6DS, "LSM6DS"}})
enum WASType
{
	NO_WAS,
	ADS1115,
	ISOBUS,
	TEENSY
};
NLOHMANN_JSON_SERIALIZE_ENUM(WASType, {{NO_WAS, "NO_WAS"},
									   {ADS1115, "ADS1115"},
									   {ISOBUS, "ISOBUS"},
									   {TEENSY, "TEENSY"}})
enum OutputType
{
	NO_OUTPUT,
	PWM,
	PWM2,
	FENDT_VBUS,
};
NLOHMANN_JSON_SERIALIZE_ENUM(OutputType, {{NO_OUTPUT, "NO_OUTPUT"},
										  {PWM, "PWM"},
										  {PWM2, "PWM2"},
										  {FENDT_VBUS, "FENDT_VBUS"}})

struct HardwareConfiguration
{
	ImuType imuType = ImuType::NO_IMU;
	WASType wasType = WASType::TEENSY;
	int8_t teensy_was_pin_number = 0;
	OutputType outputType = OutputType::NO_OUTPUT;
};
extern HardwareConfiguration hardwareConfiguration;
void to_json(json &j, const HardwareConfiguration &config);
void from_json(const json &j, HardwareConfiguration &config);

struct NetworkConfiguration
{
	IPAddress ip = IPAddress(192, 168, 1, 177);
	IPAddress netmask = IPAddress(255, 255, 255, 0);
	IPAddress gateway = IPAddress(192, 168, 1, 10);
};
extern NetworkConfiguration networkConfiguration;
void to_json(json &j, const NetworkConfiguration &config);
void from_json(const json &j, NetworkConfiguration &config);

struct AOG_SteerSettings
{
	const static uint16_t pgn = 0xFC;

	uint8_t kp = 100; // proportional gain
	uint8_t highPWM = 60; // max PWM value
	uint8_t lowPWM = 10;  // band of no action
	uint8_t minPWM = 9;
	uint16_t steerSensorCounts = 100;
	int16_t wasOffset = 0;
	float ackermannFix = 1.0; // sent as percent

	void parse(uint8_t *buf);
};
extern AOG_SteerSettings aog_steerSettings;
void to_json(json &j, const AOG_SteerSettings &settings);
void from_json(const json &j, AOG_SteerSettings &settings);

struct AOG_SteerConfig
{
	const static uint16_t pgn = 0xFB;

	bool InvertWAS = 0;
	bool IsRelayActiveHigh = 0; // if zero, active low (default)
	bool MotorDriveDirection = 0;
	bool SingleInputWAS = 1;
	bool CytronDriver = 1;
	bool SteerSwitch = 0;
	bool SteerButton = 0;
	bool ShaftEncoder = 0;
	bool PressureSensor = 0;
	bool CurrentSensor = 0;
	uint8_t PulseCountMax = 1;
	bool IsDanfoss = 0;

	void parse(uint8_t *buf);
};
extern AOG_SteerConfig aog_steerConfig;
void to_json(json &j, const AOG_SteerConfig &settings);
void from_json(const json &j, AOG_SteerConfig &settings);

void load_settings();