#include "movement.h"

// File for writing the bug algorithm and drag race to



// bug algorithm




void bug_pass(uint8_t side){
  uint16_t distance = 150; //distance in CM
  uint16_t distance_thresh = 0;


  uint32_t TrackingData_R;
  uint32_t TrackingData_M;
  uint32_t TrackingData_L;

  switch(side){
    case(1): //LHS
      AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&distance /*out*/);

      if(distance < distance_thresh){

        while(distance < 10){
        SmartRobotCarMotionControl(Left, 70);
        AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&distance /*out*/);
        delay(50);
        }
        SmartRobotCarMotionControl(Forward, 70);
        delay(1000);

        AppServo.DeviceDriverSet_Servo_control(180);
        
        while(distance < 10){
          SmartRobotCarMotionControl(Forward, 70);
          AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&distance /*out*/);

        //look for a black line
          TrackingData_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
          TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
          TrackingData_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
            
          bool seen_M = function_xxx(TrackingData_M, TrackingDetection_S, TrackingDetection_E);
          bool seen_R = function_xxx(TrackingData_R, TrackingDetection_S, TrackingDetection_E);
          bool seen_L = function_xxx(TrackingData_L, TrackingDetection_S, TrackingDetection_E);

          if(seen_M || seen_R || seen_L){
            return;
          }

        }

        AppServo.DeviceDriverSet_Servo_control(90);

        SmartRobotCarMotionControl(Right, 70);

        delay(100);
        SmartRobotCarMotionControl(Forward, 70);
        break;

      }
      else{
        return;
      }
      case(2):
      Serial.println("right away!");



  }

  



}