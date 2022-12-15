#pragma once

#include "main.hpp"
#include "semphr.h"

#include <json.hpp>
using json = nlohmann::json;

void init_sd_card();

int get_json_from_file(const char* filename, json& j);
int store_json_file(const char* filename, const json& j);

class SDLog : Print
{
    SDLog(const char* filename, const char* directory = "/");
    ~SDLog();
public:
    size_t write(uint8_t b);
    size_t write(const uint8_t *buffer, size_t size);
    int availableForWrite(void);
    void flush();
private:
    File root;
    File logfile;
};