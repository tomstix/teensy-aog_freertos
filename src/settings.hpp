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
	ISOBUS_GMS,
	TEENSY
};
NLOHMANN_JSON_SERIALIZE_ENUM(WASType, {{NO_WAS, "NO_WAS"},
									   {ADS1115, "ADS1115"},
									   {ISOBUS_GMS, "ISOBUS_GMS"},
									   {TEENSY, "TEENSY"}})
enum OutputType
{
	NO_OUTPUT,
	PWM,
	FENDT_VBUS,
};
NLOHMANN_JSON_SERIALIZE_ENUM(OutputType, {{NO_OUTPUT, "NO_OUTPUT"},
										  {PWM, "PWM"},
										  {FENDT_VBUS, "FENDT_VBUS"}})

enum SteerswitchType
{
	NO_STEERSWITCH,
	STEERSWITCH_PIN,
	STEERSWITCH_ISOBUS
};
NLOHMANN_JSON_SERIALIZE_ENUM(SteerswitchType, {{NO_STEERSWITCH, "NO_STEERSWITCH"},
											   {STEERSWITCH_PIN, "STEERSWITCH_PIN"},
											   {STEERSWITCH_ISOBUS, "STEERSWITCH_ISOBUS"}})

enum WorkswitchType
{
	NO_WORKSWITCH,
	WORKSWITCH_PIN,
	WORKSWITCH_ANALOG,
	ISOBUS_HITCH,
	ISOBUS_PTO
};
NLOHMANN_JSON_SERIALIZE_ENUM(WorkswitchType, {{NO_WORKSWITCH, "NO_WORKSWITCH"},
											  {WORKSWITCH_PIN, "WORKSWITCH_PIN"},
											  {WORKSWITCH_ANALOG, "WORKSWITCH_ANALOG"},
											  {ISOBUS_HITCH, "ISOBUS_HITCH"},
											  {ISOBUS_PTO, "ISOBUS_PTO"}})

struct HardwareConfiguration
{
	ImuType imuType = ImuType::NO_IMU;
	WASType wasType = WASType::TEENSY;
	SteerswitchType steerswitchType = SteerswitchType::NO_STEERSWITCH;
	WorkswitchType workswitchType = WorkswitchType::NO_WORKSWITCH;

	uint8_t steerswitch_pin = 0;
	uint8_t workswitch_pin = 0;

	uint8_t teensy_was_pin_number = 0;
	uint8_t ads1115_was_pin = 0;
	bool ads1115_differential_mode = false;

	OutputType outputType = OutputType::NO_OUTPUT;
	uint8_t output_pin_ena = 27;
	uint8_t output_pin_enb = 26;
	uint8_t output_pin_ina = 34;
	uint8_t output_pin_inb = 35;
	uint8_t output_pin_pwm = 36;
	uint8_t input_pin_csense = 24;
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

	uint8_t kp = 100;	  // proportional gain
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