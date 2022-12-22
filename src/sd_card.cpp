#include "sd_card.hpp"

#include <SD.h>
#include <string>

SemaphoreHandle_t sd_mutex;

int get_json_from_file(const char* filename, json& j)
{
    Log.verboseln(F("Trying to read from %s."), filename);
    if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(10000)) != pdTRUE)
    {
        Log.errorln(F("Failed to get SD Mutex while reading file %s"), filename);
        return EXIT_FAILURE;
    }
    Log.verboseln(F("Opening file for read: %s"), filename);
    if (FsFile json_file = SD.sdfs.open(filename, O_READ))
    {
        Log.verboseln(F("Opened file %s"), filename);
        unsigned int len = json_file.fileSize();
        if (len == 0)
        {
            Log.warningln(F("JSON file %s is empty!"), filename);
            json_file.close();
            xSemaphoreGive(sd_mutex);
            return EXIT_FAILURE;
        }
        std::string json_str;
        Log.verboseln(F("Reading from json file..."));
        int i = 0;
        while(json_file.available())
        {
            json_str += json_file.read();
            i++;
        }
        Log.verboseln(F("Read %i bytes."), i);
        j = json::parse(json_str);
        json_file.close();
        Log.verboseln(F("%s closed."), filename);
        xSemaphoreGive(sd_mutex);
        return EXIT_SUCCESS;
    }
    Log.warningln(F("JSON File %s does not exist!"), filename);
    xSemaphoreGive(sd_mutex);
    return EXIT_FAILURE;
}

int store_json_file(const char* filename, const json& j)
{
    Log.traceln(F("Storing JSON file %s."), filename);
    if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(5000)) != pdTRUE)
    {
        Log.errorln(F("Failed to get SD Mutex while saving file %s"), filename);
        return EXIT_FAILURE;
    }
    if (FsFile json_file = SD.sdfs.open(filename, O_WRITE | O_TRUNC | O_CREAT))
    {
        auto json_str = j.dump().c_str();
        if (!json_file.preAllocate(strlen(json_str)))
        {
            Log.errorln(F("Failed to allocte Memory to write %s."), filename);
            xSemaphoreGive(sd_mutex);
            return EXIT_FAILURE;
        }
        for (size_t i = 0; i < strlen(json_str); i++)
        {
            json_file.write(json_str[i]);
        }
        json_file.close();  
        xSemaphoreGive(sd_mutex);
        return EXIT_SUCCESS;      
    }
    Log.errorln(F("Failed to open %s."), filename);
    xSemaphoreGive(sd_mutex);
    return EXIT_FAILURE;
}

SDLog::SDLog(const char* filename, const char* directory)
{
    root = SD.open(directory);
    auto file = SD.open(filename);
}

void sd_card_task(void *)
{
    Log.verboseln(F("Initializing SD Card."));
    sd_mutex = xSemaphoreCreateMutex();
    xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(5000));
    if (SD.sdfs.begin(SdioConfig(DMA_SDIO)))
    {
        Log.traceln(F("SD Card successfully initialized!"));
        xSemaphoreGive(sd_mutex);
    }
    else
        Log.errorln(F("Failed to initialize SD Card!"));
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void init_sd_card()
{
    xTaskCreate(sd_card_task, "SD Card Task", 4096, nullptr, 2, nullptr);
}