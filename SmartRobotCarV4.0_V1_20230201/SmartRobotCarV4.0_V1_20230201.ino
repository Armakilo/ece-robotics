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

  // E-stop engaged (OK on the IR remote): skip tracking, red-stop logic and all motor commands
  if (Application_FunctionSet.ApplicationFunctionSet_EStop())
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


  Application_FunctionSet.CMD_ServoControl_xxx0();
  Application_FunctionSet.CMD_MotorControl_xxx0();
  Application_FunctionSet.CMD_CarControlTimeLimit_xxx0();
  Application_FunctionSet.CMD_CarControlNoTimeLimit_xxx0();
  Application_FunctionSet.CMD_MotorControlSpeed_xxx0();
  Application_FunctionSet.CMD_LightingControlTimeLimit_xxx0();
  Application_FunctionSet.CMD_LightingControlNoTimeLimit_xxx0();
  Application_FunctionSet.CMD_ClearAllFunctions_xxx0();
}
