#ifndef TELEMETRY_H
#define TELEMETRY_H

// Data logger for the UNO's telemetry messages:
//   {T,time_ms,heading_gyro,heading_pred,x_pred,y_pred,heading_est,x_est,y_est,lap_ds,lap_cm,laps,state}
//   (headings in tenths of a degree, positions in millimetres; pred = model only, est = fused;
//    lap time in tenths of a second, lap distance in cm)
// and events, attached to the next row:
//   {SG,stop_x,stop_y,sign_x,sign_y,fix_dx,fix_dy,match}  stopped at a stop sign (mm)
//   {LP,lap,lap_ms,lap_mm}                                 lap completed
// Keeps the newest 90 s of messages (oldest dropped first) in RAM, saved to
// flash (SPIFFS /log.bin) every 5 s so they survive an ESP32 reset.
// Other UNO messages ({STATE ...}, {E-STOP ...}, errors) are printed to Serial for debugging.
//
// Web endpoints (port 80):
//   /data       latest values as JSON (polled by the Robot Data panel)
//   /log.csv    download the last 90 s as CSV
//   /log_clear  delete the logged rows
//   /estop?on=1 engage the UNO's emergency stop, ?on=0 releases it
//   /pose_reset set the kinematic model's origin to the car's current position

#include <Arduino.h>
#include "esp_http_server.h"

void telemetry_init(void);                               // mount flash and open the log, call once in setup()
void telemetry_handle_uno_msg(const String &msg);        // call with every {...} message received from the UNO
void telemetry_register_handlers(httpd_handle_t server); // add the endpoints above to the web server

#endif
