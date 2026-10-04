#include "ApplicationFunctionSet_xxx0.h"
#include <arduino.h>

//normal loop behaviour for the project
//written by Adam Bateman




// might be completley unessecary - really just for documentation/understanding purposes.

// stop sign activity already implemented. might want to look for red light behaviour

// inner loop uses sensor data
// outer loop uses data from line follower
// add a way to follow an object

//ApplicationFunctionSet_SmartRobotCarLinearMotionControl controls the proportional control

ApplicationFunctionSet.ApplicationFunctionSet_Tracking(void) //Line tracking function, motor already uses the 
ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); // function that sets the motor direction. Uses P control

void routine::track(void)
{
  static boolean timestamp = true;
  static boolean BlindDetection = true;
  static unsigned long MotorRL_time = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
  {
    if (Car_LeaveTheGround == false) //Check if the car leaves the ground
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }

    int getAnaloguexxx_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    int getAnaloguexxx_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    int getAnaloguexxx_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
#if _Test_print
    static unsigned long print_time = 0;
    if (millis() - print_time > 500)
    {
      print_time = millis();
      Serial.print("ITR20001_getAnaloguexxx_L=");
      Serial.println(getAnaloguexxx_L);
      Serial.print("ITR20001_getAnaloguexxx_M=");
      Serial.println(getAnaloguexxx_M);
      Serial.print("ITR20001_getAnaloguexxx_R=");
      Serial.println(getAnaloguexxx_R);
    }
#endif
    if (function_xxx(TrackingData_M, TrackingDetection_S, TrackingDetection_E))
    {
      /*Achieve straight and uniform speed movement*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
      timestamp = true;
      BlindDetection = true;
    }
    else if (function_xxx(TrackingData_R, TrackingDetection_S, TrackingDetection_E))
    {
      /*Turn right*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
      timestamp = true;
      BlindDetection = true;
    }
    else if (function_xxx(TrackingData_L, TrackingDetection_S, TrackingDetection_E))
    {
      /*Turn left*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
      timestamp = true;
      BlindDetection = true;
    }
    else ////The car is not on the black line. execute Blind scan
    {
      if (timestamp == true) //acquire timestamp
      {
        timestamp = false;
        MotorRL_time = millis();
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
      /*Blind Detection*/
      if ((function_xxx((millis() - MotorRL_time), 0, 200) || function_xxx((millis() - MotorRL_time), 1600, 2000)) && BlindDetection == true)
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
      }
      else if (((function_xxx((millis() - MotorRL_time), 200, 1600))) && BlindDetection == true)
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100);
      }
      else if ((function_xxx((millis() - MotorRL_time), 3000, 3500))) // Blind Detection ...s ?
      {
        BlindDetection = false;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
    }
  }
  else if (false == timestamp)
  {
    BlindDetection = true;
    timestamp = true;
    MotorRL_time = 0;
  }
}

/*
  Obstacle Avoidance Mode
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Obstacle(void)
{
  static boolean first_is = true;
  if (Application_SmartRobotCarxxx0.Functional_Mode == ObstacleAvoidance_mode)
  {
    uint8_t switc_ctrl = 0;
    uint16_t get_Distance;
    if (Car_LeaveTheGround == false)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }
    if (first_is == true) //Enter the mode for the first time, and modulate the steering gear to 90 degrees
    {
      AppServo.DeviceDriverSet_Servo_control(90 /*Position_angle*/);
      first_is = false;
    }

    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance /*out*/);
    if (function_xxx(get_Distance, 0, 20))
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

      for (uint8_t i = 1; i < 6; i += 2) //1、3、5 Omnidirectional detection of obstacle avoidance status
      {
        AppServo.DeviceDriverSet_Servo_control(30 * i /*Position_angle*/);
        delay_xxx(1);
        AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&get_Distance /*out*/);

        if (function_xxx(get_Distance, 0, 20))
        {
          ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
          if (5 == i)
          {
            ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, 150);
            delay_xxx(500);
            ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
            delay_xxx(50);
            first_is = true;
            break;
          }
        }
        else
        {
          switc_ctrl = 0;
          switch (i)
          {
          case 1:
            ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
            break;
          case 3:
            ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
            break;
          case 5:
            ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 150);
            break;
          }
          delay_xxx(50);
          first_is = true;
          break;
        }
      }
    }
    else //if (function_xxx(get_Distance, 20, 50))
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 150);
    }
  }
  else
  {
    first_is = true;
  }
}