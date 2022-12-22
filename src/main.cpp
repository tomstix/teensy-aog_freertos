#include "main.hpp"
#include "settings.hpp"
#include "comms_aog.hpp"
#include "sd_card.hpp"
#include "autosteer.hpp"
#include "sensors.hpp"
#include "gnss.hpp"

void heartbeat_task(void *)
{
  Log.infoln(F("Heartbeat Task started!"));
  while(1)
  {
    for( int i = 0; i < 256; i++ )
    {
      analogWrite(13, i);
      vTaskDelay(pdMS_TO_TICKS(4));
    }
    for( int i = 254; i > 1; i-- )
    {
      analogWrite(13, i);
      vTaskDelay(pdMS_TO_TICKS(4));
    }
  }
}

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, 1);
  Serial.begin(115200);
  while(!Serial & (millis() < 5000));
  digitalWrite(13, 0);
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);

  Log.infoln(F("Starting teensy-aog Version %u.%u ..."), VERSION_MAJOR, VERSION_MINOR);
  Log.infoln(F("Running FreeRTOS kernel " tskKERNEL_VERSION_NUMBER ". Built by gcc " __VERSION__ "."));

  Serial.flush();

  xTaskCreate(heartbeat_task, "Heartbeat", 128, nullptr, 1, nullptr);

  init_sd_card();
  load_settings();
  init_gnss();
  init_aog_comms();
  init_sensors();
  init_autosteer();

  vTaskStartScheduler();
}

void loop()
{}