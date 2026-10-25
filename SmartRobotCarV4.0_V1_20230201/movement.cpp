// Servo and ultrasonic checks


#include "movement.h"

// File for writing the bug algorithm and drag race to



// bug algorithm

bool Overtaking = false;

// Returns false (early) if the web E-stop was pressed during the wait
bool delay_kinematics(unsigned long ms)
{
  unsigned long start = millis();
  while (millis() - start < ms) {
    wdt_reset();
    // Read the ESP32 here too, so the E-stop works in the middle of a pass
    Application_FunctionSet.ApplicationFunctionSet_SerialPortDataAnalysis();
    bool estop = Application_FunctionSet.ApplicationFunctionSet_EStop(); // also cuts the motors
    if (estop || (Overtaking && EStop_Pressed)) // pressed during the pass, even if already released
      return false;
    Kinematics_Update();
    // optional, keeps Heading_Gyro fresh for the fusion term
    Application_FunctionSet.ApplicationFunctionSet_Telemetry(false); //Gets gyro data or something
    //function could probably be named better
  }
  return true;
}

extern Servo myservo; // DeviceDriverSet_xxx0.cpp

// E-stop pressed during bug_pass(): give up the pass (the car stays stopped)
static void abort_pass(void)
{
  Overtaking = false;
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  Serial.print(F("{OVERTAKE ABORTED - E-STOP}"));

  // Point the servo (camera + ultrasonic) back to the middle. DeviceDriverSet_Servo_control()
  // can't be used here: its 450 ms wait (delay_kinematics) ends at once while the E-stop is on.
  // The ESP32 isn't read during this wait, so a quick E-stop release can't start the car mid-move.
  myservo.attach(PIN_Servo_z);
  myservo.write(90);
  unsigned long t0 = millis();
  while (millis() - t0 < 450)
  {
    wdt_reset();
    Kinematics_Update();
    Application_FunctionSet.ApplicationFunctionSet_Telemetry(true); // web page keeps showing ESTOP
  }
  myservo.detach();
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






// Returns false if the E-stop was pressed
static bool turn_to_heading(float target_deg, unsigned long timeout_ms)
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

    if (!delay_kinematics(200)) return false;

    
    
  }
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  return true;
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

  Overtaking = true;
  EStop_Pressed = false; // only presses from now on cancel this pass
  uint32_t overtake_start = millis();

  const bool     is_lhs     = (pass_side == LHS);
  const SmartRobotCarMotionControl spin       = is_lhs ? Left  : Right;
  const SmartRobotCarMotionControl spin_rev = is_lhs ? Right : Left;
  const uint8_t  look_angle = is_lhs ? 0 : 180;

  if (!delay_kinematics(10)) return abort_pass(); //could find more elegant way to impliement this
  float yaw_0 = Heading_Gyro;

  

  ApplicationFunctionSet_SmartRobotCarMotionControl(spin, 100);

  uint32_t t_departed1 = millis();
  
  if (!delay_kinematics(300)) return abort_pass();
  
  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);

  uint32_t first_time = millis() - t_departed1;

  AppServo.DeviceDriverSet_Servo_control(look_angle); //move this around
  if (EStop_Pressed) return abort_pass(); // E-stop during the servo move

  
  
  
  

  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&distance /*out*/);
  
  while(distance < sideview_thresh && (first_time < 500)){
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&distance /*out*/);
    if (!delay_kinematics(50)) return abort_pass();
    first_time = millis() - t_departed1;
  }

  if (!delay_kinematics(1200)) return abort_pass(); //could add line seeing here

  first_time = millis() - t_departed1;

  uint32_t t_departed2 = millis();
  

  ApplicationFunctionSet_SmartRobotCarMotionControl(spin_rev, 100);
  if (!delay_kinematics(600)) return abort_pass();

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
  AppServo.DeviceDriverSet_Servo_control(90);
  if (EStop_Pressed) return abort_pass(); // E-stop during the servo move
  uint32_t second_time = millis() - t_departed2;
  while(first_time > second_time){
    if(look4line) crossLineFix(spin);
    if (!delay_kinematics(50)) return abort_pass();
    second_time = millis() - t_departed2; 
       
    }

  // ApplicationFunctionSet_SmartRobotCarMotionControl(spin_rev, 100);
  // delay_kinematics(300);

  if (!turn_to_heading(yaw_0,3000)) return abort_pass();

  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  if (!delay_kinematics(1000)) return abort_pass();

  Overtaking = false;
  Serial.print(F("{OVERTAKE DONE,"));
  Serial.print(millis() - overtake_start); // how long the pass took, ms
  Serial.print('}');
}