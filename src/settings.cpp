#include "settings.hpp"
#include "sd_card.hpp"
#include "comms_aog.hpp"

#include <string>
#include <vector>

HardwareConfiguration hardwareConfiguration;
NetworkConfiguration networkConfiguration;
AOG_SteerSettings aog_steerSettings;
AOG_SteerConfig aog_steerConfig;

EventGroupHandle_t settings_loaded_event;

void to_json(json &j, const HardwareConfiguration &config)
{
    Log.verboseln("Making HardwareConfiguration JSON...");
    j["IMU Type"] = config.imuType;
    j["WAS"]["Type"] = config.wasType;
    j["WAS"]["Teensy Pin"] = config.teensy_was_pin_number;
    j["WAS"]["ADS1115 Pin"] = config.ads1115_was_pin;
    j["WAS"]["ADS1115 Differential"] = config.ads1115_differential_mode;
    j["Output"]["Type"] = config.outputType;
    j["Output"]["Pins"] = {
                            {"ENA", config.output_pin_ena},
                            {"ENB", config.output_pin_enb},
                            {"INA", config.output_pin_ina},
                            {"INB", config.output_pin_inb},
                            {"PWM", config.output_pin_pwm},
                            {"CSENSE", config.input_pin_csense}
    };
    j["Switches"]["Steer Switch Type"] = config.steerswitchType;
    j["Switches"]["Steer Switch Pin"] = config.steerswitch_pin;
    j["Switches"]["Work Switch Type"] = config.workswitchType;
    j["Switches"]["Work Switch Pin"] = config.workswitch_pin;
}
void from_json(const json &j, HardwareConfiguration &config)
{
    if (!j.at("IMU Type").get_to(config.imuType))
        Log.errorln("Tried parsing Invalid IMU Type from JSON");
    if (!j.at("WAS").at("Type").get_to(config.wasType))
        Log.errorln("Tried parsing Invalid WAS Type from JSON");
    if (!j.at("WAS").at("Teensy Pin").get_to(config.teensy_was_pin_number))
        Log.errorln("Tried parsing Invalid WAS Pin from JSON");
    j.at("WAS").at("ADS1115 Pin").get_to(config.ads1115_was_pin);
    j.at("WAS").at("ADS1115 Differential").get_to(config.ads1115_differential_mode);
    if (!j.at("Output").at("Type").get_to(config.outputType))
        Log.errorln("Tried parsing Invalid Output Type from JSON");
    auto pins = j.at("Output").at("Pins");
    pins.at("ENA").get_to(config.output_pin_ena);
    pins.at("ENB").get_to(config.output_pin_enb);
    pins.at("INA").get_to(config.output_pin_ina);
    pins.at("INB").get_to(config.output_pin_inb);
    pins.at("PWM").get_to(config.output_pin_pwm);
    pins.at("CSENSE").get_to(config.input_pin_csense);

    auto switches = j.at("Switches");
    switches.at("Steer Switch Type").get_to(config.steerswitchType);
    switches.at("Steer Switch Pin").get_to(config.steerswitch_pin);
    switches.at("Work Switch Type").get_to(config.workswitchType);
    switches.at("Work Switch Pin").get_to(config.workswitch_pin);
}

void to_json(json &j, const NetworkConfiguration &config)
{
    Log.verboseln("Making NetworkConfiguration JSON...");
    j["IP Address"] = {config.ip[0], config.ip[1], config.ip[2], config.ip[3]};
    j["Netmask"] = {config.netmask[0], config.netmask[1], config.netmask[2], config.netmask[3]};
    j["Gateway"] = {config.gateway[0], config.gateway[1], config.gateway[2], config.gateway[3]};
}
void from_json(const json &j, NetworkConfiguration &config)
{
    std::vector<uint8_t> ip_vec;
    std::vector<uint8_t> mask_vec;
    std::vector<uint8_t> gw_vec;
    j.at("IP Address").get_to(ip_vec);
    j.at("Netmask").get_to(mask_vec);
    j.at("Gateway").get_to(gw_vec);
    auto ip_from_vector([](std::vector<uint8_t> vec)
                        {
        if(vec.size() != 4)
        {
            Log.errorln("Error Converting JSON to NetworkConfiguration! Vector doesn't have 4 components.");
            return IPAddress(0,0,0,0);
        }
        IPAddress ret_ip;
        for (int i = 0; i < 4; i++)
        {
            ret_ip[i] = vec[i];
        }
        return ret_ip; });
    config.ip = ip_from_vector(ip_vec);
    config.netmask = ip_from_vector(mask_vec);
    config.gateway = ip_from_vector(gw_vec);
}

bool read_bit(const uint8_t value, const uint8_t bit)
{
    return (value >> bit) & 0x01;
}

void AOG_SteerSettings::parse(uint8_t *buf)
{
    kp = ((float)buf[5]);
    highPWM = buf[6];
    lowPWM = buf[7];
    minPWM = buf[8];
    steerSensorCounts = buf[9];
    wasOffset = buf[10];
    wasOffset |= buf[11] << 8;
    ackermannFix = (float)buf[12] / 100.0;

    Log.traceln("Parsed new AOG steer settings!");
}
void to_json(json &j, const AOG_SteerSettings &settings)
{
    j = json{
        {"Kp", settings.kp},
        {"highPWM", settings.highPWM},
        {"lowPWM", settings.lowPWM},
        {"minPWM", settings.minPWM},
        {"steerSensorCounts", settings.steerSensorCounts},
        {"wasOffset", settings.wasOffset},
        {"ackermannFix", settings.ackermannFix}};
}
void from_json(const json &j, AOG_SteerSettings &settings)
{
    j.at("Kp").get_to(settings.kp);
    j.at("highPWM").get_to(settings.highPWM);
    j.at("lowPWM").get_to(settings.lowPWM);
    j.at("minPWM").get_to(settings.minPWM);
    j.at("steerSensorCounts").get_to(settings.steerSensorCounts);
    j.at("wasOffset").get_to(settings.wasOffset);
}

void AOG_SteerConfig::parse(uint8_t *buf)
{
    uint8_t sett = buf[5];

    InvertWAS = read_bit(sett, 0);
    IsRelayActiveHigh = read_bit(sett, 1);
    MotorDriveDirection = read_bit(sett, 2);
    SingleInputWAS = read_bit(sett, 3);
    CytronDriver = read_bit(sett, 4);
    SteerSwitch = read_bit(sett, 5);
    SteerButton = read_bit(sett, 6);
    ShaftEncoder = read_bit(sett, 7);

    PulseCountMax = buf[6];

    sett = buf[8];

    IsDanfoss = read_bit(sett, 0);
    PressureSensor = read_bit(sett, 1);
    CurrentSensor = read_bit(sett, 2);

    Log.traceln("Parsed new AOG steer config!");
}
void to_json(json &j, const AOG_SteerConfig &settings)
{
    j = json{
        {"InvertWAS", settings.InvertWAS},
        {"IsRelayActiveHigh", settings.IsRelayActiveHigh},
        {"MotorDriveDirection", settings.MotorDriveDirection},
        {"SingleInputWAS", settings.SingleInputWAS},
        {"CytronDriver", settings.CytronDriver},
        {"SteerSwitch", settings.SteerSwitch},
        {"SteerButton", settings.SteerButton},
        {"ShaftEncoder", settings.ShaftEncoder},
        {"PulseCountMax", settings.PulseCountMax},
        {"IsDanfoss", settings.IsDanfoss},
        {"PressureSensor", settings.PressureSensor},
        {"CurrentSensor", settings.CurrentSensor}};
}
void from_json(const json &j, AOG_SteerConfig &settings)
{
    j.at("InvertWAS").get_to(settings.InvertWAS);
    j.at("IsRelayActiveHigh").get_to(settings.IsRelayActiveHigh);
    j.at("MotorDriveDirection").get_to(settings.MotorDriveDirection);
    j.at("SingleInputWAS").get_to(settings.SingleInputWAS);
    j.at("CytronDriver").get_to(settings.CytronDriver);
    j.at("SteerSwitch").get_to(settings.SteerSwitch);
    j.at("SteerButton").get_to(settings.SteerButton);
    j.at("ShaftEncoder").get_to(settings.ShaftEncoder);
    j.at("PulseCountMax").get_to(settings.PulseCountMax);
    j.at("IsDanfoss").get_to(settings.IsDanfoss);
    j.at("PressureSensor").get_to(settings.PressureSensor);
    j.at("CurrentSensor").get_to(settings.CurrentSensor);
}

void save_settings()
{
    json j;
    j = {
        {"Hardware Configuration", hardwareConfiguration},
        {"Network Configuration", networkConfiguration},
        {"AOG Steer Settings", aog_steerSettings},
        {"AOG Steer Config", aog_steerConfig}};
    store_json_file(SETTINGS_FILE_NAME, j);
}

void settings_task(void *)
{
    Log.infoln("Loading Settings...");

    json settings_json;
    if (get_json_from_file(SETTINGS_FILE_NAME, settings_json) == EXIT_FAILURE)
    {
        Log.warningln("Failed to read settings file. Using default settings.");
        save_settings();
    }
    else
    {
        settings_json.at("Hardware Configuration").get_to(hardwareConfiguration);
        settings_json.at("Network Configuration").get_to(networkConfiguration);
        settings_json.at("AOG Steer Settings").get_to(aog_steerSettings);
        settings_json.at("AOG Steer Config").get_to(aog_steerConfig);
        Log.infoln("Settings loaded!");
    }
    xEventGroupSetBits(settings_loaded_event, 0x01);
    while (1)
    {
        if (xQueueReceive(aogSteerConfigQueue, &aog_steerConfig, pdMS_TO_TICKS(50)) ||
            xQueueReceive(aogSteerSettingsQueue, &aog_steerSettings, pdMS_TO_TICKS(50)))
        {
            save_settings();
        }
    }
}

void load_settings()
{
    settings_loaded_event = xEventGroupCreate();
    xTaskCreate(settings_task, "Settings Task", 4096, nullptr, 1, nullptr);
}