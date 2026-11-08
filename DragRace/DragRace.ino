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
#include "DRf.h"

float yawinit;

void setup()
{
  // put your setup code here, to run once:
  Application_FunctionSet.ApplicationFunctionSet_Init();
  wdt_enable(WDTO_2S);
  // FIX: the old line here had "==" instead of "=", no semicolon, and used a
  // variable this file can't see, so the sketch didn't compile.
  Application_FunctionSet.ApplicationFunctionSet_ForceTrackingMode();

  AppServo.DeviceDriverSet_Servo_control(80); // rhs angle, just small offset
  AppMPU6050getdata.MPU6050_dveGetEulerAngles(&yawinit);


}

void loop()
{

  wdt_reset();

  Application_FunctionSet.ApplicationFunctionSet_SerialPortDataAnalysis();

  // Kinematic model: integrate the pose from the PWM applied since the last loop.
  // Before the E-stop check so time keeps running while stopped.
  Kinematics_Update();

  DragRace();

  // E-stop engaged (E-STOP button on the web page): skip tracking, red-stop logic and all motor commands
  bool estop = Application_FunctionSet.ApplicationFunctionSet_EStop();

  // Send heading + state to the ESP32 data logger (keeps running during an E-stop)
  Application_FunctionSet.ApplicationFunctionSet_Telemetry(estop);

  if (estop)
    return;


  Application_FunctionSet.ApplicationFunctionSet_SensorDataUpdate();

  Application_FunctionSet.ApplicationFunctionSet_Standby();
}
