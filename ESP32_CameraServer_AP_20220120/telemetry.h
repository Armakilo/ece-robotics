#ifndef TELEMETRY_H
#define TELEMETRY_H

// Data logger for the UNO's telemetry messages:
//   {T,time_ms,heading_actual,heading_predicted,state}   (headings in tenths of a degree)
// Keeps the newest 30 s of messages (oldest dropped first) in RAM, saved to
// flash (SPIFFS /log.bin) every 5 s so they survive an ESP32 reset.
// Other UNO messages ({STATE ...}, {E-STOP ...}, errors) are printed to Serial for debugging.
//
// Web endpoints (port 80):
//   /data       latest values as JSON (polled by the Robot Data panel)
//   /log.csv    download the last 30 s as CSV
//   /log_clear  delete the logged rows
//   /estop?on=1 engage the UNO's emergency stop, ?on=0 releases it

#include <Arduino.h>
#include "esp_http_server.h"

void telemetry_init(void);                               // mount flash and open the log, call once in setup()
void telemetry_handle_uno_msg(const String &msg);        // call with every {...} message received from the UNO
void telemetry_register_handlers(httpd_handle_t server); // add the endpoints above to the web server

#endif
