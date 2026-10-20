// Servo and ultrasonic checks


#include "movement.h"

// File for writing the bug algorithm and drag race to



// bug algorithm

void delay_kinematics(unsigned long ms)
{
  unsigned long start = millis();
  while (millis() - start < ms) {
    wdt_reset();
    Kinematics_Update();
    // optional, keeps Heading_Gyro fresh for the fusion term
    Application_FunctionSet.ApplicationFunctionSet_Telemetry(false); //Gets gyro data or something
    //function could probably be named better
  }
}

bool checkLine(void){

  uint32_t TrackingData_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
  uint32_t TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
  uint32_t TrackingData_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
            
  bool seen_M = function_xxx(TrackingData_M, Application_FunctionSet.TrackingDetection_S, Application_FunctionSet.TrackingDetection_E);
  bool seen_R = function_xxx(TrackingData_R, Application_FunctionSet.TrackingDetection_S, Application_FunctionSet.TrackingDetection_E);
  bool seen_L = function_xxx(TrackingData_L, Application_FunctionSet.TrackingDetection_S, Application_FunctionSet.TrackingDetection_E);

  if(seen_M || seen_R || seen_L){
    return true;
  }
  
  return false;
  
}

bool crossLineFix(SmartRobotCarMotionControl dir){ //Fixes center of obj
  if(checkLine()){ //Checks for a line (corner cutting, etc.) assumes that the "turning point is the centrod of the object"
    
    uint32_t TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    if(function_xxx(TrackingData_M, Application_FunctionSet.TrackingDetection_S, Application_FunctionSet.TrackingDetection_E)){
      ApplicationFunctionSet_SmartRobotCarMotionControl(dir, 100);
      delay_kinematics(3000); //for testing
      return true;
    }
  }
  return false;
}






static void turn_to_heading(float target_deg, unsigned long timeout_ms)
{
  unsigned long t0 = millis();
  while (millis() - t0 < timeout_ms) {
    float err = target_deg - Heading_Gyro - 20.0f; //the -20 is for tuning purposes
    float er  = err * (PI / 180.0f);
    err = atan2f(sinf(er), cosf(er)) * (180.0f / PI);   // wrap to ±180

    if (fabs(err) < 3.0f) break;

    // Pulse and speed both shrink with error, so the last pulse can't overshoot
    

    if (err > 0)
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left,  100);
    else
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);

    delay_kinematics(200);

    
    
  }
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  
}

void bug_pass(uint8_t goto_side){

  Serial.println("trying to pass");
  uint16_t distance = 150; //distance in CM
  uint16_t distance_thresh = 20;
  uint16_t sideview_thresh = 20;

  bool look4line = false;

  side pass_side = goto_side;

  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&distance /*out*/);

  Serial.println(distance);
  if(distance > distance_thresh || distance == 0) return;

  const bool     is_lhs     = (pass_side == LHS);
  const SmartRobotCarMotionControl spin       = is_lhs ? Left  : Right;
  const SmartRobotCarMotionControl spin_rev = is_lhs ? Right : Left;
  const uint8_t  look_angle = is_lhs ? 0 : 180;

  delay_kinematics(10); //could find more elegant way to impliement this
  float yaw_0 = Heading_Gyro;

  

  ApplicationFunctionSet_SmartRobotCarMotionControl(spin, 100);

  uint32_t t_departed1 = millis();
  
  delay_kinematics(300);
  
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);

  uint32_t first_time = millis() - t_departed1;

  AppServo.DeviceDriverSet_Servo_control(look_angle); //move this around

  
  
  
  

  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&distance /*out*/);
  
  while(distance < sideview_thresh && (first_time < 500)){
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&distance /*out*/);
    delay_kinematics(50);
    first_time = millis() - t_departed1;
  }

  delay_kinematics(1200); //could add line seeing here

  first_time = millis() - t_departed1;

  uint32_t t_departed2 = millis();
  

  ApplicationFunctionSet_SmartRobotCarMotionControl(spin_rev, 100);
  delay_kinematics(600);

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
  AppServo.DeviceDriverSet_Servo_control(90);
  uint32_t second_time = millis() - t_departed2;
  while(first_time > second_time){
    if(look4line) crossLineFix(spin);
    delay_kinematics(50);
    second_time = millis() - t_departed2; 
       
    }

  // ApplicationFunctionSet_SmartRobotCarMotionControl(spin_rev, 100);
  // delay_kinematics(300);

  turn_to_heading(yaw_0,3000);

  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  delay_kinematics(1000);

}