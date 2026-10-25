/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-12-18 14:14:35
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0
 * @FilePath:
 */
#include <avr/wdt.h>
#include "ApplicationFunctionSet_xxx0.h"

void setup()
{
  // put your setup code here, to run once:
  Application_FunctionSet.ApplicationFunctionSet_Init();
  wdt_enable(WDTO_2S);
  // FIX: the old line here had "==" instead of "=", no semicolon, and used a
  // variable this file can't see, so the sketch didn't compile.
  Application_FunctionSet.ApplicationFunctionSet_ForceTrackingMode();
}

void loop()
{
  //put your main code here, to run repeatedly :
  wdt_reset();

  Application_FunctionSet.ApplicationFunctionSet_SerialPortDataAnalysis();

  // Kinematic model: integrate the pose from the PWM applied since the last loop.
  // Before the E-stop check so time keeps running while stopped.
  Kinematics_Update();

  // E-stop engaged (E-STOP button on the web page): skip tracking, red-stop logic and all motor commands
  bool estop = Application_FunctionSet.ApplicationFunctionSet_EStop();

  // Send heading + state to the ESP32 data logger (keeps running during an E-stop)
  Application_FunctionSet.ApplicationFunctionSet_Telemetry(estop);

  if (estop)
    return;

  // Start of a run (power-on or E-stop released): stay stopped until the camera sees red, then green
  if (Application_FunctionSet.ApplicationFunctionSet_TrafficLight())
    return;

  // Keep the car in line-tracking mode (see the comment on this function in the .cpp)
  Application_FunctionSet.ApplicationFunctionSet_ForceTrackingMode();

  Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();
  // Application_FunctionSet.ApplicationFunctionSet_KeyCommand();
  Application_FunctionSet.ApplicationFunctionSet_RGB();
  // Application_FunctionSet.ApplicationFunctionSet_Follow();
  // Application_FunctionSet.ApplicationFunctionSet_Obstacle();
  Application_FunctionSet.ApplicationFunctionSet_Tracking();
  // Application_FunctionSet.ApplicationFunctionSet_Rocker();
  Application_FunctionSet.ApplicationFunctionSet_Standby();
  // Application_FunctionSet.ApplicationFunctionSet_IRrecv();


  // The Elegoo phone-app command handlers (CMD_..._xxx0) were removed from the loop:
  // the car is always in line-tracking mode, so they never ran. Saves flash.
}
