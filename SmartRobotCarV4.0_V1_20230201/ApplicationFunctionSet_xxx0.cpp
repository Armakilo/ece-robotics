/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2021-01-05 09:30:14
 * @LastEditors: Changhua
 * @Description: Smart Robot Car V4.0
 * @FilePath: 
 */


// Modified Oct 14, 2026 by Adam Bateman
// Still needs to be debugged, might run into some memory issues

#include <avr/wdt.h>
//#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"
#include "movement.h"

// ArduinoJson removed (hand parser in SerialPortDataAnalysis, saves flash)
#include "MPU6050_getdata.h"

#define _is_print 1
#define _Test_print 0

ApplicationFunctionSet Application_FunctionSet;

/*Hardware device object list*/
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_RBGLED AppRBG_LED;
DeviceDriverSet_Key AppKey;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Voltage AppVoltage;

DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_ULTRASONIC AppULTRASONIC;
DeviceDriverSet_Servo AppServo;
DeviceDriverSet_IRrecv AppIRrecv;
/*f(x) int */
boolean function_xxx(long x, long s, long e) //f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}
static void
delay_xxx(uint16_t _ms)
{
  wdt_reset();
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}



/*Mode Control List*/
enum SmartRobotCarFunctionalModel
{
  Standby_mode,           /*Standby Mode*/
  TraceBased_mode,        /*Line Tracking Mode*/
  ObstacleAvoidance_mode, /*Obstacle Avoidance Mode*/
  Follow_mode,            /*Following Mode*/
  Rocker_mode,            /*Rocker Control Mode*/
  CMD_inspect,
  CMD_Programming_mode,                   /*Programming Mode*/
  CMD_ClearAllFunctions_Standby_mode,     /*Clear All Functions And Enter Standby Mode*/
  CMD_ClearAllFunctions_Programming_mode, /*Clear All Functions And Enter Programming Mode*/
  CMD_MotorControl,                       /*Motor Control Mode*/
  CMD_CarControl_TimeLimit,               /*Car Movement Direction Control With Time Limit*/
  CMD_CarControl_NoTimeLimit,             /*Car Movement Direction Control Without Time Limit*/
  CMD_MotorControl_Speed,                 /*Motor Speed Control*/
  CMD_ServoControl,                       /*Servo Motor Control*/
  CMD_LightingControl_TimeLimit,          /*RGB Lighting Control With Time Limit*/
  CMD_LightingControl_NoTimeLimit,        /*RGB Lighting Control Without Time Limit*/

};

/*Application Management list*/
struct Application_xxx
{
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};
Application_xxx Application_SmartRobotCarxxx0;

// red object detection and integration - still must be tested
// need to send red object data, then check for a stop sign

//might want a struct for an obstacle

typedef struct  // only will really be used to track one object at a time.
{
  // the raw data for the object coming in
  char type; //'S' = stop sign, 'O' = obstacle, 'C' = car, 'N' = none
  uint16_t cent_x;
  uint16_t cent_y;
  uint16_t width; //estimate of width of bbox
  uint16_t height; //estimate of height of bbox unnecessary, but kept as a legacy feature
  char colour; //colour of object, 'R' = red, 'B' = blue, 'G' = green, '?' = none. Based off of values sent from the ESP
  uint16_t sizePX; //new
  int16_t dist_cm; // FIX: was uint8_t, but the ESP sends -1 for "too far", which became 255
  unsigned long lastUpdate;

} object_t;

object_t obj_now = {'N',0,0,0,0,'?',0,0,0};
object_t obj_prev = {'N',0,0,0,0,'?',0,0,0};

/*---------------- Red object / stop sign logic (from ESP32 camera) ----------------
  The ESP32 sends D1 = 'N' (not checked yet), 'S' (checked: stop sign), 'X' (checked: NOT a stop sign).
  Flow:
    DRIVE      -> close red object seen                        -> RED_WAIT   (stop)
    RED_WAIT   -> camera confirms stop sign ('S')              -> SIGN_HOLD  (start 3 s timer)
    RED_WAIT   -> camera says "not a stop sign" ('X') twice    -> RED_IGNORE (drive on)
    RED_WAIT   -> red no longer seen                           -> DRIVE
    RED_IGNORE -> drive, ignoring this red; a stop sign ('S')  -> SIGN_HOLD
    RED_IGNORE -> red no longer seen                           -> DRIVE
    (stop sign confirmed: if it matches a sign in STOP_MAP the car first drives on to that
     sign's stop line at the junction (TO_STOP_LINE), using the position estimate; otherwise
     it holds right where it is)
    TO_STOP_LINE -> estimate reaches the stop line             -> SIGN_HOLD
    SIGN_HOLD  -> 3 s passed                                   -> COOLDOWN
    COOLDOWN   -> drive, ignore red for 5 s so we can pass the sign -> DRIVE
*/
#define STOP_SIGN_DEBUG        1     // 1 = print state changes, 0 = off
#define STOP_SIGN_HOLD_MS      3000  // how long to stop at a confirmed stop sign
#define STOP_SIGN_COOLDOWN_MS  5000  // ignore red/stop signs this long after the hold, so the car can drive past
#define RED_FRESH_MS           2000  // red counts as "still there" if seen within this time
                                     // (the ESP32 sends about every 0.5-1.7 s, so keep this above ~1.7 s)
#define RED_STOP_MIN_PX        100   // stop for red blobs at least this big (= close enough). Was 150.
                                     // KEEP THIS EQUAL TO STOP_CHECK_MIN_PX ON THE ESP32
#define RED_REJECTS_TO_GO      2     // drive on after this many "not a stop sign" answers in a row
                                     // (1 = faster, but a stop sign missed once at a bad angle gets skipped)
#define RED_HARD_STOP_PX       800   // FIX 4: a red blob this big is very close (~10 cm by the ESP32's
                                     // dist_class table). It's treated as a stop sign even if the model
                                     // said "not a stop sign", so RED_IGNORE can't drive into the sign.
                                     // Raise it to stop closer, lower it to stop farther away.

enum RedStopState
{
  RS_DRIVE,
  RS_RED_WAIT,
  RS_RED_IGNORE,
  RS_SIGN_HOLD,
  RS_COOLDOWN,
  RS_TO_STOP_LINE // stop sign confirmed and on the map: drive to its stop line, then hold
};
static RedStopState red_state = RS_DRIVE;

// What the line tracker is doing (for telemetry)
enum TrackState
{
  TS_FOLLOW, // on the line
  TS_SEARCH, // lost the line, sweeping left/right to find it
  TS_LOST,   // sweep failed, stopped
  TS_LIFTED  // all sensors see nothing: car picked up
};
static TrackState track_state = TS_FOLLOW;
static unsigned long red_state_since = 0;
static bool stop_sign_pending = false; // set when a "D1":"S" message arrives
static uint8_t red_rejects = 0;        // "D1":"X" answers received while in RED_WAIT

static int8_t Camera_SignFix(void);                  // camera fusion, defined after the kinematic model
static bool StopLine_Reached(int8_t m);             // true once the estimate is at map sign m's stop line
#define TO_STOP_LINE_TIMEOUT_MS 6000                // hold anyway if the stop line isn't reached by then
static int8_t approach_sign = -1;                   // STOP_MAP entry being approached

static void RedStop_SetState(RedStopState s)
{
  red_state = s;
  red_state_since = millis();
  red_rejects = 0;
#if STOP_SIGN_DEBUG
  switch (s)
  {
  case RS_DRIVE:      Serial.print(F("{STATE DRIVE}")); break;
  case RS_RED_WAIT:   Serial.print(F("{STATE RED_WAIT - red seen, stopped}")); break;
  case RS_RED_IGNORE: Serial.print(F("{STATE RED_IGNORE - not a stop sign, driving on}")); break;
  case RS_SIGN_HOLD:  Serial.print(F("{STATE SIGN_HOLD - stop sign, 3s}")); break;
  case RS_COOLDOWN:   Serial.print(F("{STATE COOLDOWN - driving past}")); break;
  case RS_TO_STOP_LINE: Serial.print(F("{STATE TO_STOP_LINE - driving to the junction}")); break;
  }
#endif
}

// True if the camera has recently seen a red object big (= close) enough to check for a stop sign
static bool RedAhead(void)
{
  return obj_now.colour == 'R' &&
         obj_now.sizePX >= RED_STOP_MIN_PX &&
         (millis() - obj_now.lastUpdate) < RED_FRESH_MS;
}

// FIX 4: true if a red object is so close the car must stop no matter what the model said
static bool RedTooClose(void)
{
  return RedAhead() && obj_now.sizePX >= RED_HARD_STOP_PX;
}

// A stop sign was confirmed: correct the position with the camera (Camera_SignFix), then either
// drive on to the sign's stop line (sign on the map) or hold right here (unknown sign).
static void RedStop_SignConfirmed(void)
{
  approach_sign = Camera_SignFix();
  if (approach_sign >= 0 && !StopLine_Reached(approach_sign))
    RedStop_SetState(RS_TO_STOP_LINE);
  else
    RedStop_SetState(RS_SIGN_HOLD);
}

static void RedStop_HardStop(void)
{
#if STOP_SIGN_DEBUG
  Serial.print(F("{CLOSE RED - hard stop}"));
#endif
  RedStop_SignConfirmed();
}

// Called by the message parser each time a camera message arrives
static void StopSign_Check(void)
{
  if (obj_now.type == 'S')
    stop_sign_pending = true;
  else if (obj_now.type == 'X' && red_state == RS_RED_WAIT && red_rejects < 255)
    red_rejects++;
}

// Called every time the car is about to move. Updates the state and
// returns true if the car must stay stopped.
static bool RedStop_MustStop(void)
{
  unsigned long in_state = millis() - red_state_since;

  switch (red_state)
  {
  case RS_DRIVE:
    if (stop_sign_pending)
    {
      stop_sign_pending = false;
      RedStop_SignConfirmed();
    }
    else if (RedTooClose())
      RedStop_HardStop();
    else if (RedAhead())
      RedStop_SetState(RS_RED_WAIT);
    break;

  case RS_RED_WAIT:
    if (stop_sign_pending)
    {
      stop_sign_pending = false;
      RedStop_SignConfirmed(); // confirmed: go to the stop line / start the 3 s timer
    }
    else if (red_rejects >= RED_REJECTS_TO_GO && RedTooClose())
      RedStop_HardStop(); // model says no, but it's right in front of us: stop anyway
    else if (red_rejects >= RED_REJECTS_TO_GO)
      RedStop_SetState(RS_RED_IGNORE); // background red, not a stop sign
    else if (!RedAhead())
      RedStop_SetState(RS_DRIVE); // red went away without being a stop sign
    break;

  case RS_RED_IGNORE:
    if (stop_sign_pending)
    {
      stop_sign_pending = false;
      RedStop_SignConfirmed(); // a real stop sign showed up
    }
    else if (RedTooClose())
      RedStop_HardStop(); // the ignored red got very close: it's probably the sign after all
    else if (!RedAhead())
      RedStop_SetState(RS_DRIVE); // that red is out of view, watch for new red again
    break;

  case RS_TO_STOP_LINE:
    stop_sign_pending = false; // it's the sign we're already handling
    if (StopLine_Reached(approach_sign) || in_state >= TO_STOP_LINE_TIMEOUT_MS)
      RedStop_SetState(RS_SIGN_HOLD);
    break;

  case RS_SIGN_HOLD:
    if (in_state >= STOP_SIGN_HOLD_MS)
      RedStop_SetState(RS_COOLDOWN);
    break;

  case RS_COOLDOWN:
    stop_sign_pending = false; // ignore the sign we're driving past
    if (in_state >= STOP_SIGN_COOLDOWN_MS)
      RedStop_SetState(RS_DRIVE);
    break;
  }

  return red_state == RS_RED_WAIT || red_state == RS_SIGN_HOLD;
}

bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void);
void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit);


void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  bool res_error = true;
  Serial.begin(9600);
  AppVoltage.DeviceDriverSet_Voltage_Init();
  AppMotor.DeviceDriverSet_Motor_Init();
  AppServo.DeviceDriverSet_Servo_Init(90);
  AppKey.DeviceDriverSet_Key_Init();
  AppRBG_LED.DeviceDriverSet_RBGLED_Init(20);
  // AppIRrecv.DeviceDriverSet_IRrecv_Init(); // IR remote not used (saves flash)
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Init();
  AppITR20001.DeviceDriverSet_ITR20001_Init();
  res_error = AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();

  // while (Serial.read() >= 0)
  // {
  //   /*Clear serial port buffer...*/
  // }
  Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
}

/*ITR20001 Check if the car leaves the ground*/
static bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void)
{
  if (AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M() > Application_FunctionSet.TrackingDetection_V &&
      AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L() > Application_FunctionSet.TrackingDetection_V)
  {
    Application_FunctionSet.Car_LeaveTheGround = false;
    return false;
  }
  else
  {
    Application_FunctionSet.Car_LeaveTheGround = true;
    return true;
  }
}
/*
  Straight line movement control：For dual-drive motors, due to frequent motor coefficient deviations and many external interference factors, 
  it is difficult for the car to achieve relative Straight line movement. For this reason, the feedback of the yaw control loop is added.
  direction：only forward/backward
  directionRecord：Used to update the direction and position data (Yaw value) when entering the function for the first time.
  speed：the speed range is 0~255
  Kp：Position error proportional constant（The feedback of improving location resuming status，will be modified according to different mode），improve damping control.
  UpperLimit：Maximum output upper limit control
*/
static void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit)
{
  static float Yaw; //Yaw
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;
  if (en != directionRecord || millis() - is_time > 10)
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    is_time = millis();
  }
  //if (en != directionRecord)
  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false)
  {
    en = directionRecord;
    yaw_So = Yaw;
  }
  //Add proportional constant Kp to increase rebound effect
  int R = (Yaw - yaw_So) * Kp + speed;
  if (R > UpperLimit)
  {
    R = UpperLimit;
  }
  else if (R < 10)
  {
    R = 10;
  }
  int L = (yaw_So - Yaw) * Kp + speed;
  if (L > UpperLimit)
  {
    L = UpperLimit;
  }
  else if (L < 10)
  {
    L = 10;
  }
  if (direction == Forward) //Forward
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ R,
                                           /*direction_B*/ direction_just, /*speed_B*/ L, /*controlED*/ control_enable);
  }
  else if (direction == Backward) //Backward
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ L,
                                           /*direction_B*/ direction_back, /*speed_B*/ R, /*controlED*/ control_enable);
  }
}
/*
  Movement Direction Control:
  Input parameters:     1# direction:Forward（1）、Backward（2）、 Left（3）、Right（4）、LeftForward（5）、LeftBackward（6）、RightForward（7）RightBackward（8）
                        2# speed(0--255)
*/
void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed)
{
  ApplicationFunctionSet Application_FunctionSet;
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;

  if (direction != stop_it && RedStop_MustStop()) // red object / stop sign: override any movement
  {
    direction = stop_it;
  }

  //Control mode that requires straight line movement adjustment（Car will has movement offset easily in the below mode，the movement cannot achieve the effect of a relatively straight direction
  //so it needs to add control adjustment）
  switch (Application_SmartRobotCarxxx0.Functional_Mode)
  {
  case Rocker_mode:
    Kp = 5;
    UpperLimit = 255;
    break;
  case ObstacleAvoidance_mode:
    Kp = 2;
    UpperLimit = 180;
    break;
  case Follow_mode:
    Kp = 2;
    UpperLimit = 180;
    break;
  case CMD_CarControl_TimeLimit:
    Kp = 2;
    UpperLimit = 180;
    break;
  case CMD_CarControl_NoTimeLimit:
    Kp = 2;
    UpperLimit = 180;
    break;
  default:
    Kp = 10;
    UpperLimit = 255;
    break;
  }
  switch (direction)
  {
  case /* constant-expression */
      Forward:
    /* code */
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //When moving forward, enter the direction and position approach control loop processing
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Forward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 1;
    }

    break;
  case /* constant-expression */ Backward:
    /* code */
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                             /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //When moving backward, enter the direction and position approach control loop processing
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Backward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 2;
    }

    break;
  case /* constant-expression */ Left:
    /* code */
    directionRecord = 3;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ Right:
    /* code */
    directionRecord = 4;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftForward:
    /* code */
    directionRecord = 5;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftBackward:
    /* code */
    directionRecord = 6;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightForward:
    /* code */
    directionRecord = 7;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightBackward:
    /* code */
    directionRecord = 8;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ stop_it:
    /* code */
    directionRecord = 9;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control

    break;
  default:
    directionRecord = 10;
    break;
  }
}
/*
 Robot car update sensors' data:Partial update (selective update)
*/
void ApplicationFunctionSet::ApplicationFunctionSet_SensorDataUpdate(void)
{

  // AppMotor.DeviceDriverSet_Motor_Test();
  { /*Battery voltage status update*/
    static unsigned long VoltageData_time = 0;
    static int VoltageData_number = 1;
    if (millis() - VoltageData_time > 10) //read and update the data per 10ms
    {
      VoltageData_time = millis();
      VoltageData_V = AppVoltage.DeviceDriverSet_Voltage_getAnalogue();
      if (VoltageData_V < VoltageDetection)
      {
        VoltageData_number++;
        if (VoltageData_number == 500) //Continuity to judge the latest voltage value multiple 
        {
          VoltageDetectionStatus = true;
          VoltageData_number = 0;
        }
      }
      else
      {
        VoltageDetectionStatus = false;
      }
    }
  }

  // { /*value updation for the ultrasonic sensor：for the Obstacle Avoidance mode*/
  //   AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&UltrasoundData_cm /*out*/);
  //   UltrasoundDetectionStatus = function_xxx(UltrasoundData_cm, 0, ObstacleDetection);
  // }

  { /*value updation for the IR sensors on the line tracking module：for the line tracking mode*/
    TrackingData_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
    TrackingDetectionStatus_R = function_xxx(TrackingData_R, TrackingDetection_S, TrackingDetection_E);
    TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    TrackingDetectionStatus_M = function_xxx(TrackingData_M, TrackingDetection_S, TrackingDetection_E);
    TrackingData_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    TrackingDetectionStatus_L = function_xxx(TrackingData_L, TrackingDetection_S, TrackingDetection_E);
    //ITR20001 Check if the car leaves the ground
    ApplicationFunctionSet_SmartRobotCarLeaveTheGround();
  }

  // acquire timestamp
  // static unsigned long Test_time;
  // if (millis() - Test_time > 200)
  // {
  //   Test_time = millis();
  //   //AppITR20001.DeviceDriverSet_ITR20001_Test();
  // }
}
/*
  Startup operation requirement：
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Bootup(void)
{
  Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
}

/*
  FIX: the .ino tried to set Application_SmartRobotCarxxx0.Functional_Mode directly,
  but that variable and TraceBased_mode only exist inside this .cpp file, so the
  sketch didn't compile. The .ino calls this function instead.
  It's called every loop on purpose: the ESP32 sends {"N":100} ("stop, go to standby")
  whenever no phone app is connected, which would otherwise knock the car out of
  line-tracking mode.
*/
void ApplicationFunctionSet::ApplicationFunctionSet_ForceTrackingMode(void)
{
  Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
}

/*Emergency stop: set by the web page's E-STOP button, which the ESP32 sends as
  {"N":300,"D1":1} (engage) or {"N":300,"D1":0} (release), see SerialPortDataAnalysis.
  Returns true while engaged so loop() can skip all driving code.*/
static bool estop_active = false;
#define RESET_ON_ESTOP_RELEASE 1 // 1 = releasing the E-stop zeroes the position and starts the lap timer
                                 // (so an E-stop in the middle of a run restarts the coordinates there)
bool ApplicationFunctionSet::ApplicationFunctionSet_EStop(void)
{
  if (estop_active)
  {
    // PWM 0 and STBY LOW: motor driver disabled
    AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable);
  }
  return estop_active;
}

float Heading_Predicted = NAN; // degrees, set by Kinematics_Update()

/*---------------- Kinematic model (dead reckoning from the motor commands) ----------------
  Inertial frame: origin = where the pose was last reset (the start / traffic light),
  X = the car's starting direction, Y = to its left, heading psi = 0 at reset, + = turning left.

  Wheel speed from PWM (measured with Kinematics_Calibration.ino, PWM 100/130/160 straight runs
  and a PWM 100 spin):   w_wheel = K_WHEEL * (|PWM| - PWM_DEAD)   for |PWM| >= PWM_STALL
    v   = r (wR + wL) / 2          psi_dot = r (wR - wL) / L
    X  += v cos(psi) dt            Y      += v sin(psi) dt          (midpoint psi)
  The input is the same PWM that drives the wheels (g_cmdR / g_cmdL, recorded in
  DeviceDriverSet_Motor_control).*/
#define WHEEL_R_M  0.0325f  // wheel radius (m), 65 mm wheels
#define PWM_STALL  35       // below this the wheels don't start turning
#define PWM_DEAD   5.3f     // zero point of the speed line (3-point fit)
#define MOTOR_TAU  0.15f    // wheel speed time constant (s): a wheel takes ~0.15 s to reach ~2/3 of
                            // its new speed. Without this, the tracker's short steering pulses were
                            // counted as full-speed turns. (The calibration distances were measured
                            // to where the car STOPPED, so spin-up loss and coasting cancel and the
                            // calibrated K_WHEEL is already the steady-state value.)
// Wheel rad/s per PWM count above PWM_DEAD. Calibration floor (3-point fit, 100/130/160): 0.1267.
// On the track the model came out ~5% long (legs 3.30/1.59/2.37 m vs 3.20/1.51/2.20 m) and turned
// ~6% too much, so 0.1267 / 1.05 = 0.1207 (lowering K fixes both: turn rate scales with K / L).
#define K_WHEEL    0.1207f
// EFFECTIVE wheelbase (m) from the PWM 100 spin test (skid steer, so ~2x the real 14 cm width).
// Checked on the track: model/gyro = 98% and 109% on the two corner turns.
#define TRACK_L_M  0.289f
// Left/right motor mismatch: at PWM 100 the car veers RIGHT at 2.4 deg/s (calibration straight
// runs), i.e. the left side is faster by  e = w_veer * L / v = 0.0419 * 0.289 / 0.390 = 0.031.
// Without this the model only saw the tracker's LEFT correction pulses and drifted left at
// 2.4 deg/s on straight line (log: +27 deg predicted vs +1 deg gyro over 11 s).
#define TRIM_LEFT_FASTER 0.031f

// 1 if turning the car LEFT by hand makes the gyro heading go UP (calibration sketch 'g'),
// -1 if it goes DOWN. Makes the logged gyro heading use the same sign as the model.
#define GYRO_SIGN  -1   // set from the g test: turning left by hand made the raw heading go DOWN

extern int16_t g_cmdR, g_cmdL; // DeviceDriverSet_xxx0.cpp

float Pose_X = 0;   // m
float Pose_Y = 0;   // m
float Pose_Psi = 0; // rad

/*---------------- Gyro fusion ----------------
  Estimated pose = the kinematic model, but its heading is continuously pulled toward the gyro
  heading (complementary filter):
      psi_est += w_model * dt                               (predict with the model)
      psi_est += dt / (FUSION_T + dt) * (psi_gyro - psi_est) (correct with the gyro)
  and the estimated X/Y are integrated with psi_est. The model alone over-counts the tracker's
  short steering pulses (~+2 deg/s false left drift on straight line); the gyro was within ~3 deg
  of the real track on the corners, so it gets most of the weight (short FUSION_T).
  Hand-check on the 14:52 run: corners came out within ~0.1 m of the track instead of 0.5-0.7 m.*/
#define FUSION_T   0.5f     // s: how fast the estimate follows the gyro. Smaller = trust gyro more.

float Heading_Gyro = 0; // deg, same sign convention as the model (+ = left), 0 at pose reset
float Est_X = 0;        // m
float Est_Y = 0;        // m
float Est_Psi = 0;      // rad

/*---------------- Lap time / distance ----------------
  The lap timer starts the first time the motors turn after a pose reset (so press Reset position
  with the car on the start line, then release the E-stop). A lap is complete when the car has
  driven at least LAP_MIN_M and the fused estimate passes closest to the start point (within
  LAP_RADIUS_M). Average speed = lap distance / lap time (stop-sign holds included).*/
#define LAP_MIN_M     6.0f   // the figure 8 is 8.59 m, so this rejects false laps near the start
#define LAP_RADIUS_M  0.4f   // how close to the start the estimate must come

static bool lap_running = false;
static unsigned long lap_start = 0; // millis() when the current lap started
float Lap_Dist = 0;                 // m driven in the current lap (odometer, from the model)
uint8_t Lap_Count = 0;              // laps completed since the pose reset

static float Kinematics_PwmToWheel(int16_t pwm) // rad/s, signed
{
  int16_t a = abs(pwm);
  if (a < PWM_STALL)
    return 0;
  float w = K_WHEEL * (a - PWM_DEAD);
  return pwm < 0 ? -w : w;
}

// Sets the inertial-frame origin to where the car is now (call at the start line / green light)
/*---------------- Start pose ----------------
  Where the car is when the E-stop is released (or Reset position is pressed), in the TRACK frame:
  origin = bottom-right corner of the figure 8 (where the 1.25 m section meets the long diagonal),
  X = up the long diagonal, Y = to the left of it, heading + = left. All the map data (corners,
  stop sign, stop line) is in this frame, so if you are told to start somewhere else, only change
  these three numbers. Default: at the origin, pointing up the diagonal.
  Example: on the 1.2 m section, 0.3 m past the top-right corner, facing the crossing:
    START_X_M 1.95, START_Y_M -0.78, START_PSI_DEG 120*/
#define START_X_M     0.0f
#define START_Y_M     0.0f
#define START_PSI_DEG 0.0f

void Kinematics_Reset(void)
{
  Pose_X = START_X_M;
  Pose_Y = START_Y_M;
  Pose_Psi = START_PSI_DEG * (PI / 180.0f);
  Heading_Predicted = START_PSI_DEG;
  Est_X = START_X_M;
  Est_Y = START_Y_M;
  Est_Psi = START_PSI_DEG * (PI / 180.0f);
  AppMPU6050getdata.agz = 0; // gyro heading starts at START_PSI_DEG together with the model
  Heading_Gyro = START_PSI_DEG;
  lap_running = false; // lap timer starts when the car next moves
  Lap_Dist = 0;
  Lap_Count = 0;
}

// Call every loop. Uses the PWM that was applied since the previous call.
void Kinematics_Update(void)
{
  static unsigned long last = 0;
  unsigned long now = micros();
  if (last == 0)
  {
    last = now;
    return;
  }
  float dt = (now - last) * 1e-6f;
  last = now;
  if (dt > 0.2f) // loop stalled (e.g. a servo delay): don't integrate a huge step
    dt = 0.2f;

  // Wheels can't change speed instantly: first-order lag toward the commanded speed
  static float wR = 0, wL = 0;
  float a = dt / (MOTOR_TAU + dt);
  wR += a * (Kinematics_PwmToWheel(g_cmdR) * (1.0f - 0.5f * TRIM_LEFT_FASTER) - wR);
  wL += a * (Kinematics_PwmToWheel(g_cmdL) * (1.0f + 0.5f * TRIM_LEFT_FASTER) - wL);
  float v = WHEEL_R_M * (wR + wL) / 2.0f;
  float w = WHEEL_R_M * (wR - wL) / TRACK_L_M;

  float psi_mid = Pose_Psi + 0.5f * w * dt;
  Pose_X += v * cos(psi_mid) * dt;
  Pose_Y += v * sin(psi_mid) * dt;
  Pose_Psi += w * dt;

  // Fused estimate: same motion, heading corrected toward the gyro
  float est_mid = Est_Psi + 0.5f * w * dt;
  Est_X += v * cos(est_mid) * dt;
  Est_Y += v * sin(est_mid) * dt;
  Est_Psi += w * dt;
  Est_Psi += (dt / (FUSION_T + dt)) * (Heading_Gyro * (PI / 180.0f) - Est_Psi);

  Heading_Predicted = Pose_Psi * (180.0f / PI);

  // Lap timing
  if (!lap_running && (g_cmdR != 0 || g_cmdL != 0))
  {
    lap_running = true;
    lap_start = millis();
    Lap_Dist = 0;
  }
  if (lap_running)
  {
    static float prev_d2 = 1e9f;
    Lap_Dist += fabs(v) * dt;
    float d2 = (Est_X - START_X_M) * (Est_X - START_X_M) +
               (Est_Y - START_Y_M) * (Est_Y - START_Y_M); // squared distance from the start point
    // lap done at the closest approach to the start: inside the radius and moving away again
    if (Lap_Dist > LAP_MIN_M && d2 < LAP_RADIUS_M * LAP_RADIUS_M && d2 > prev_d2 + 1e-6f)
    {
      unsigned long t = millis() - lap_start;
      Lap_Count++;
      // {LP,lap number,lap time ms,lap distance mm}
      Serial.print(F("{LP,"));
      Serial.print(Lap_Count);
      Serial.print(',');
      Serial.print(t);
      Serial.print(',');
      Serial.print((long)(Lap_Dist * 1000));
      Serial.print('}');
      lap_start = millis(); // next lap starts here
      Lap_Dist = 0;
      d2 = 1e9f;
    }
    prev_d2 = d2;
  }
}

/*---------------- Camera fusion: stop signs as position landmarks ----------------
  The car stops for a stop sign when the sign's red blob reaches a fixed size in the camera
  image (RED_STOP_MIN_PX), so it always stops the same distance from the sign. Each sign
  therefore marks a known stopping point on the track (STOP_MAP). When the car stops at a
  confirmed stop sign:
    1. find the map point closest to the fused estimate, with a similar heading (gate)
    2. pull the estimate toward it:  est += CAM_GAIN * (map - est)
       The gyro fixes heading but not distance; this fixes the distance (along-track) error.
    3. estimate where the sign itself is: range from the blob size (pinhole camera: blob
       pixels ~ 1/range^2) and bearing from the blob's x position in the image
  and send {SG,stop_x,stop_y,sign_x,sign_y,fix_dx,fix_dy,match} (mm) to the logger.
  stop_x/stop_y is the estimate BEFORE the fix: after a run, put the averages for each sign
  into STOP_MAP (or measure the stopping points on the track with a tape).*/
#define CAM_FUSION    1      // 0 = only log the signs, don't correct the position
#define CAM_GAIN      0.7f   // 0..1: how much to trust the map point over the estimate
#define CAM_GATE_M    0.5f   // ignore map points farther than this from the estimate
#define CAM_GATE_DEG  45     // ... or with a heading more than this different
#define CAM_HFOV_DEG  60.0f  // camera horizontal field of view (deg); adjust for a wide lens
#define CAM_IMG_W     160    // QQVGA image width (pixels), D2 = blob centre x in this image
#define CAM_RANGE_K   18.0f  // m*sqrt(px): range = K / sqrt(blob px)  (pinhole: px ~ 1/range^2)
                             // Was 4.14 from the ESP32's dist_class table (275 px = 0.25 m), which put
                             // the real sign 0.21 m away when it was ~0.92 m away (16:4x run), so
                             // K = 4.14 * 0.92 / 0.21 = 18. Measure once with a tape to refine.

struct StopPoint
{
  float x, y;           // m, where the camera confirms the sign (car stops in RED_WAIT here)
  int16_t psi;          // deg, car heading there (wrapped to -180..180)
  float line_x, line_y; // m, the stop line at the junction, where the car holds for 3 s
};
// Real stop sign at the crossing (1.50, 0), facing the 1.2 m section of the long return leg.
// Detection point from the 16:4x run: the camera confirmed it at (1.99, -0.80), heading 140 deg
// (the previous run gave ~(1.94, -0.80)). Check it with a tape: car centre ~0.92 m before the
// crossing, ~0.3 m past the top-right corner.
// Stop line: STOP_LINE_GAP_M before the crossing along that leg (direction 120 deg):
//   (1.50, 0) - 0.20 * (cos 120, sin 120) = (1.60, -0.17)
#define STOP_LINE_GAP_M 0.20f // car centre this far before the crossing centre
static const StopPoint STOP_MAP[] = {
    {1.99f, -0.80f, 140, 1.50f + 0.5f * STOP_LINE_GAP_M, -0.866f * STOP_LINE_GAP_M},
};
#define STOP_MAP_N (sizeof(STOP_MAP) / sizeof(STOP_MAP[0]))

// Distance still to drive to the stop line, measured along the approach direction
// (detection point -> stop line). <= 0 once the estimate reaches or passes it.
static bool StopLine_Reached(int8_t m)
{
  if (m < 0 || m >= (int8_t)STOP_MAP_N)
    return true;
  float ux = STOP_MAP[m].line_x - STOP_MAP[m].x;
  float uy = STOP_MAP[m].line_y - STOP_MAP[m].y;
  float len = hypot(ux, uy);
  if (len < 0.01f)
    return true;
  float remaining = ((STOP_MAP[m].line_x - Est_X) * ux + (STOP_MAP[m].line_y - Est_Y) * uy) / len;
  return remaining <= 0.02f;
}

static int8_t Camera_SignFix(void)
{
  float x0 = Est_X, y0 = Est_Y; // estimate before the fix
  float dx = 0, dy = 0;
  int8_t match = -1;
#if CAM_FUSION
  float psi_deg = Est_Psi * (180.0f / PI);
  float best = CAM_GATE_M;
  for (uint8_t i = 0; i < STOP_MAP_N; i++)
  {
    float dpsi = psi_deg - STOP_MAP[i].psi;
    while (dpsi > 180)
      dpsi -= 360;
    while (dpsi < -180)
      dpsi += 360;
    if (fabs(dpsi) > CAM_GATE_DEG)
      continue;
    float d = hypot(STOP_MAP[i].x - x0, STOP_MAP[i].y - y0);
    if (d < best)
    {
      best = d;
      match = i;
    }
  }
  if (match >= 0)
  {
    dx = CAM_GAIN * (STOP_MAP[match].x - x0);
    dy = CAM_GAIN * (STOP_MAP[match].y - y0);
    Est_X += dx;
    Est_Y += dy;
  }
#endif
  // Where is the sign? Range and bearing from the latest red blob (default 0.4 m straight ahead)
  float range = 0.9f, bearing = 0;
  if (obj_now.colour == 'R' && obj_now.sizePX > 0)
  {
    range = CAM_RANGE_K / sqrt((float)obj_now.sizePX);
    range = constrain(range, 0.1f, 2.5f);
    // image x grows to the right, bearing is + to the left
    bearing = ((CAM_IMG_W / 2.0f) - obj_now.cent_x) * (CAM_HFOV_DEG * PI / 180.0f / CAM_IMG_W);
  }
  float sx = Est_X + range * cos(Est_Psi + bearing);
  float sy = Est_Y + range * sin(Est_Psi + bearing);

  Serial.print(F("{SG,"));
  Serial.print((long)(x0 * 1000));
  Serial.print(',');
  Serial.print((long)(y0 * 1000));
  Serial.print(',');
  Serial.print((long)(sx * 1000));
  Serial.print(',');
  Serial.print((long)(sy * 1000));
  Serial.print(',');
  Serial.print((long)(dx * 1000));
  Serial.print(',');
  Serial.print((long)(dy * 1000));
  Serial.print(',');
  Serial.print(match);
  Serial.print('}');
  return match;
}

/*Telemetry for the ESP32 data logger, 5x per second:
    {T,time_ms,heading_gyro,heading_pred,x_pred,y_pred,heading_est,x_est,y_est,lap_ds,lap_cm,laps,state}
  lap_ds: current lap time in tenths of a second, lap_cm: current lap distance in cm,
  laps: laps completed. Events: {SG,...} stop sign (Camera_SignFix), {LP,...} lap done.
  Headings in tenths of a degree, positions in millimetres (integers, to avoid the float print code).
  heading_gyro: integrated gyro yaw (0 at pose reset).  *_pred: kinematic model only.
  *_est: model fused with the gyro heading.*/
void ApplicationFunctionSet::ApplicationFunctionSet_Telemetry(bool estop_active)
{
  static unsigned long last_send = 0;
  static float heading_actual = 0;
  // Integrate the gyro on a fixed 10 ms step instead of every loop (fewer I2C reads, steadier dt).
  // While the motors have been off for 0.4 s the car is still, so instead of integrating, the gyro
  // offset is re-measured (zero-velocity update). The power-on offset was off by ~8 deg/s, which
  // the old dead-zone hid by throwing away all slow turning.
  static unsigned long last_gyro = 0;
  static unsigned long last_moving = 0;
  if (g_cmdR != 0 || g_cmdL != 0)
    last_moving = millis();
  if (millis() - last_gyro >= 10)
  {
    last_gyro = millis();
    if (millis() - last_moving > 400)
    {
      AppMPU6050getdata.MPU6050_dveStill();
      heading_actual = AppMPU6050getdata.agz; // FIX: also refresh while still (e.g. right after a reset)
    }
    else
      AppMPU6050getdata.MPU6050_dveGetEulerAngles(&heading_actual);
    Heading_Gyro = START_PSI_DEG + GYRO_SIGN * heading_actual; // used by the fusion in Kinematics_Update
  }
  if (millis() - last_send < 200)
    return;
  last_send = millis();

  const __FlashStringHelper *state;
  if (estop_active)
    state = F("ESTOP");
  else if (red_state == RS_RED_WAIT)
    state = F("RED_WAIT");
  else if (red_state == RS_SIGN_HOLD)
    state = F("STOP_SIGN");
  else if (red_state == RS_TO_STOP_LINE)
    state = F("TO_STOP_LINE");
  else if (track_state == TS_LIFTED)
    state = F("LIFTED");
  else if (track_state == TS_SEARCH)
    state = F("LINE_SEARCH");
  else if (track_state == TS_LOST)
    state = F("LINE_LOST");
  else if (red_state == RS_RED_IGNORE)
    state = F("RED_IGNORE");
  else if (red_state == RS_COOLDOWN)
    state = F("COOLDOWN");
  else
    state = F("LINE_FOLLOW");

  Serial.print(F("{T,"));
  Serial.print(last_send);
  Serial.print(',');
  Serial.print((long)(Heading_Gyro * 10));
  Serial.print(',');
  if (!isnan(Heading_Predicted))
    Serial.print((long)(Heading_Predicted * 10));
  Serial.print(',');
  Serial.print((long)(Pose_X * 1000));
  Serial.print(',');
  Serial.print((long)(Pose_Y * 1000));
  Serial.print(',');
  Serial.print((long)(Est_Psi * (1800.0f / PI)));
  Serial.print(',');
  Serial.print((long)(Est_X * 1000));
  Serial.print(',');
  Serial.print((long)(Est_Y * 1000));
  Serial.print(',');
  Serial.print(lap_running ? (long)((millis() - lap_start) / 100) : 0L); // lap time, tenths of s
  Serial.print(',');
  Serial.print((long)(Lap_Dist * 100)); // lap distance, cm
  Serial.print(',');
  Serial.print(Lap_Count);
  Serial.print(',');
  Serial.print(state);
  Serial.print('}');
}

static void CMD_Lighting(uint8_t is_LightingSequence, int8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B)
{
  switch (is_LightingSequence)
  {
  case 0:
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(NUM_LEDS, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 1: /*Left*/
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(3, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 2: /*Forward*/
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(2, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 3: /*Right*/
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(1, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 4: /*Back*/
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(0, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  case 5: /*Middle*/
    AppRBG_LED.DeviceDriverSet_RBGLED_Color(4, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    break;
  default:
    break;
  }
}

/*RBG_LED set*/
void ApplicationFunctionSet::ApplicationFunctionSet_RGB(void)
{
  static unsigned long getAnalogue_time = 0;
  FastLED.clear(true);
  if (true == VoltageDetectionStatus) //Act on low power state？
  {
    if ((millis() - getAnalogue_time) > 3000)
    {
      getAnalogue_time = millis();
    }
  }
  unsigned long temp = millis() - getAnalogue_time;
  if (function_xxx((temp), 0, 500) && VoltageDetectionStatus == true)
  {
    switch (temp)
    {
    case /* constant-expression */ 0 ... 49:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 50 ... 99:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 100 ... 149:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 150 ... 199:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 200 ... 249:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 250 ... 299:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 300 ... 349:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 350 ... 399:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    case /* constant-expression */ 400 ... 449:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
      break;
    case /* constant-expression */ 450 ... 499:
      /* code */
      AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
      break;
    default:
      break;
    }
  }
  else if (((function_xxx((temp), 500, 3000)) && VoltageDetectionStatus == true) || VoltageDetectionStatus == false)
  {
    switch (Application_SmartRobotCarxxx0.Functional_Mode) //Act on mode control sequence
    {
    case /* constant-expression */ Standby_mode:
      /* code */
      {
        if (VoltageDetectionStatus == true)
        {
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Red);
          delay(30);
          AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Black);
          delay(30);
        }
        else
        {
          static uint8_t setBrightness = 0;
          static boolean et = false;
          static unsigned long time = 0;

          if ((millis() - time) > 10)
          {
            time = millis();
            if (et == false)
            {
              setBrightness += 1;
              if (setBrightness == 100)
                et = true;
            }
            else if (et == true)
            {
              setBrightness -= 1;
              if (setBrightness == 0)
                et = false;
            }
          }
          // AppRBG_LED.leds[1] = CRGB::Blue;
          AppRBG_LED.leds[0] = CRGB::Violet;
          FastLED.setBrightness(setBrightness);
          FastLED.show();
        }
      }
      break;
    case /* constant-expression */ CMD_Programming_mode:
      /* code */
      {
      }
      break;
    case /* constant-expression */ TraceBased_mode:
      /* code */
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Green);
      }
      break;
    case /* constant-expression */ ObstacleAvoidance_mode:
      /* code */
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Yellow);
      }
      break;
    case /* constant-expression */ Follow_mode:
      /* code */
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Blue);
      }
      break;
    case /* constant-expression */ Rocker_mode:
      /* code */
      {
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, 2 /*Traversal_Number*/, CRGB::Violet);
      }
      break;
    default:
      break;
    }
  }
}

/*Rocker control mode*/
void ApplicationFunctionSet::ApplicationFunctionSet_Rocker(void)
{
  if (Application_SmartRobotCarxxx0.Functional_Mode == Rocker_mode)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(Application_SmartRobotCarxxx0.Motion_Control /*direction*/, Rocker_CarSpeed /*speed*/);
  }
}

/*Line tracking mode
  Line search (all three sensors off the line), steered by the gyro:
    1. turn toward the side the line most likely went, up to SEARCH_FIRST_DEG
    2. turn back the other way, up to SEARCH_BACK_DEG past the starting heading
    3. turn back to the starting heading, then give up (LINE_LOST) and stop.
  Which side first (Search_FirstSide):
    a. near a corner of the track (fused position estimate within CORNER_NEAR_M): the direction
       that corner turns. The estimate was within ~0.1 m of every corner in the 16:4x run.
    b. otherwise the side sensor that saw the line in the last LAST_SIDE_FRESH_MS
    c. otherwise right first, like the original Elegoo search.
  History: the original search spun right 0.2 s then left, and every glimpse of the tape restarted
  it, so at the 90 deg LEFT corner it kept spinning right (16:2x, 16:4x). v9's first fix used only
  the last side sensor, which on the long diagonal was a stale LEFT correction (the car veers
  right), so at the first (RIGHT) corner it searched left and turned all the way around.
  A glimpse of the line shorter than GLIMPSE_MS no longer restarts the search, and the search
  direction is fixed when the search starts.*/
#define SEARCH_FIRST_DEG    140   // first sweep (corners are 30-137 deg; the line we came along is at 180)
#define SEARCH_BACK_DEG     90    // second sweep, the other way
#define SEARCH_TIMEOUT_MS   6000  // give up anyway after this long
#define LAST_SIDE_FRESH_MS  300   // a side sensor reading older than this is stale
#define GLIMPSE_MS          250   // line must be seen this long to end a search
#define CORNER_NEAR_M       0.6f

struct TrackCorner
{
  float x, y;
  int8_t dir; // +1 = the track turns left here, -1 = right
};
// Corners of the figure 8 (track drawing, inertial frame, start = (0,0) facing up the diagonal)
static const TrackCorner TRACK_CORNERS[] = {
    {3.20f, 0.00f, -1},  // top left, 136.6 deg right
    {2.10f, -1.04f, -1}, // top right, 103.4 deg right
    {1.00f, 0.87f, 1},   // end of the 1 m section, 30 deg left
    {0.63f, 1.08f, 1},   // 90 deg left
    {0.00f, 0.00f, 1},   // back at the start, 120 deg left
};
#define TRACK_CORNERS_N (sizeof(TRACK_CORNERS) / sizeof(TRACK_CORNERS[0]))

static int8_t line_last_side = -1;         // +1 = LEFT sensor saw the line last, -1 = RIGHT
static unsigned long line_last_side_ms = 0; // when

static int8_t Search_FirstSide(void)
{
  float best = CORNER_NEAR_M;
  int8_t side = 0;
  for (uint8_t i = 0; i < TRACK_CORNERS_N; i++)
  {
    float d = hypot(TRACK_CORNERS[i].x - Est_X, TRACK_CORNERS[i].y - Est_Y);
    if (d < best)
    {
      best = d;
      side = TRACK_CORNERS[i].dir;
    }
  }
  if (side != 0)
    return side;
  if (millis() - line_last_side_ms < LAST_SIDE_FRESH_MS)
    return line_last_side;
  return -1;
}

void ApplicationFunctionSet::ApplicationFunctionSet_Tracking(void)
{
  static boolean timestamp = true;
  static boolean BlindDetection = true;
  static unsigned long MotorRL_time = 0;
  static float search_psi0 = 0;   // gyro heading when the line was lost
  static uint8_t search_phase = 0;
  static int8_t search_side = -1; // direction of the first sweep, fixed for the whole search
  static bool on_line = false;
  static unsigned long on_line_since = 0;

  // FIX: update the red / stop sign states every loop. They used to advance only when the car
  // tried to move, so if the line search gave up during a stop-sign hold, the hold never ended
  // (the log showed STOP_SIGN forever while the car had really lost the line).
  
  //Avoid obstacles

  bug_pass(RHS);

  RedStop_MustStop();

  if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
  {
    if (Car_LeaveTheGround == false) //Check if the car leaves the ground
    {
      track_state = TS_LIFTED;
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }
    track_state = TS_FOLLOW; // changed below if the line is lost

    bool seen_M = function_xxx(TrackingData_M, TrackingDetection_S, TrackingDetection_E);
    bool seen_R = function_xxx(TrackingData_R, TrackingDetection_S, TrackingDetection_E);
    bool seen_L = function_xxx(TrackingData_L, TrackingDetection_S, TrackingDetection_E);
    if (seen_M || seen_R || seen_L)
    {
      if (seen_M)
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100); /*straight*/
      else if (seen_R)
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100); /*Turn right*/
        line_last_side = -1;
        line_last_side_ms = millis();
      }
      else
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 100); /*Turn left*/
        line_last_side = 1;
        line_last_side_ms = millis();
      }
      if (!on_line)
      {
        on_line = true;
        on_line_since = millis();
      }
      // Only a real find ends the search; a glimpse keeps the current search going
      if (millis() - on_line_since >= GLIMPSE_MS || timestamp)
      {
        timestamp = true;
        BlindDetection = true;
      }
    }
    else ////The car is not on the black line. execute Blind scan
    {
      on_line = false;
      track_state = BlindDetection ? TS_SEARCH : TS_LOST;
      if (timestamp == true) //acquire timestamp
      {
        timestamp = false;
        MotorRL_time = millis();
        search_psi0 = Heading_Gyro;
        search_phase = 0;
        search_side = Search_FirstSide();
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
      /*Blind Detection, steered by the gyro*/
      if (BlindDetection == true)
      {
        // degrees turned since the line was lost, + = toward the first-sweep side
        float turned = (Heading_Gyro - search_psi0) * search_side;
        if (search_phase == 0 && turned >= SEARCH_FIRST_DEG)
          search_phase = 1; // not found on that side: sweep back to the other side
        else if (search_phase == 1 && turned <= -SEARCH_BACK_DEG)
          search_phase = 2; // not there either: return to the starting heading
        else if (search_phase == 2 && turned >= 0)
          search_phase = 3;
        if (search_phase == 3 || millis() - MotorRL_time > SEARCH_TIMEOUT_MS)
        {
          BlindDetection = false; // give up: LINE_LOST, stopped
          ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
        }
        else
        {
          // phases 0 and 2 turn toward the first-sweep side, phase 1 away from it
          bool toward = (search_phase != 1);
          bool left = toward == (search_side > 0);
          ApplicationFunctionSet_SmartRobotCarMotionControl(left ? Left : Right, 100);
        }
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

/*
  Following mode：
*/
void ApplicationFunctionSet::ApplicationFunctionSet_Follow(void)
{
  static uint16_t ULTRASONIC_Get = 0;
  static unsigned long ULTRASONIC_time = 0;
  static uint8_t Position_Servo = 1;
  static uint8_t timestamp = 3;
  static uint8_t OneCycle = 1;
  if (Application_SmartRobotCarxxx0.Functional_Mode == Follow_mode)
  {

    if (Car_LeaveTheGround == false)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }
    AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&ULTRASONIC_Get /*out*/);
    if (false == function_xxx(ULTRASONIC_Get, 0, 20)) //There is no obstacle 20 cm ahead?
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      static unsigned long time_Servo = 0;
      static uint8_t Position_Servo_xx = 0;

      if (timestamp == 3)
      {
        if (Position_Servo_xx != Position_Servo) //Act on servo motor：avoid loop execution
        {
          Position_Servo_xx = Position_Servo; //Act on servo motor：rotation angle record

          if (Position_Servo == 1)
          {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(80 /*Position_angle*/);
          }
          else if (Position_Servo == 2)
          {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(20 /*Position_angle*/);
          }
          else if (Position_Servo == 3)
          {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(80 /*Position_angle*/);
          }
          else if (Position_Servo == 4)
          {
            time_Servo = millis();
            AppServo.DeviceDriverSet_Servo_control(150 /*Position_angle*/);
          }
        }
      }
      else
      {
        if (timestamp == 1)
        {
          timestamp = 2;
          time_Servo = millis();
        }
      }
      if (millis() - time_Servo > 1000) //Act on servo motor：stop at the current location for 2s
      {
        timestamp = 3;
        Position_Servo += 1;
        OneCycle += 1;
        if (OneCycle > 4)
        {
          Position_Servo = 1;
          OneCycle = 5;
        }
      }
    }
    else
    {
      OneCycle = 1;
      timestamp = 1;
      if ((Position_Servo == 1))
      { /*Move forward*/
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
      }
      else if ((Position_Servo == 2))
      { /*Turn right*/
        ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 150);
      }
      else if ((Position_Servo == 3))
      {
        /*Move forward*/
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 100);
      }
      else if ((Position_Servo == 4))
      { /*Turn left*/
        ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 150);
      }
    }
  }
  else
  {
    ULTRASONIC_Get = 0;
    ULTRASONIC_time = 0;
  }
}

/*Servo motor control*/
void ApplicationFunctionSet::ApplicationFunctionSet_Servo(uint8_t Set_Servo)
{
  static int z_angle = 9;
  static int y_angle = 9;
  uint8_t temp_Set_Servo = Set_Servo; 

  switch (temp_Set_Servo)
  {
  case 1 ... 2:
  {
    if (1 == temp_Set_Servo)
    {
      y_angle -= 1;
    }
    else if (2 == temp_Set_Servo)
    {
      y_angle += 1;
    }
    if (y_angle <= 3) //minimum control
    {
      y_angle = 3;
    }
    if (y_angle >= 11) //maximum control
    {
      y_angle = 11;
    }
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--y*/ 2, /*unsigned int Position_angle*/ y_angle);
  }
  break;

  case 3 ... 4:
  {
    if (3 == temp_Set_Servo)
    {
      z_angle += 1;
    }
    else if (4 == temp_Set_Servo)
    {
      z_angle -= 1;
    }

    if (z_angle <= 1) //minimum control
    {
      z_angle = 1;
    }
    if (z_angle >= 17) //maximum control
    {
      z_angle = 17;
    }
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ z_angle);
  }
  break;
  case 5:
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--y*/ 2, /*unsigned int Position_angle*/ 9);
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo--z*/ 1, /*unsigned int Position_angle*/ 9);
    break;
  default:
    break;
  }
}
/*Standby mode*/
void ApplicationFunctionSet::ApplicationFunctionSet_Standby(void)
{
  static bool is_ED = true;
  static uint8_t cout = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == Standby_mode)
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    if (true == is_ED) //Used to zero yaw raw data(Make sure the car is placed on a stationary surface!)
    {
      static unsigned long timestamp; //acquire timestamp
      if (millis() - timestamp > 20)
      {
        timestamp = millis();
        if (ApplicationFunctionSet_SmartRobotCarLeaveTheGround() /* condition */)
        {
          cout += 1;
        }
        else
        {
          cout = 0;
        }
        if (cout > 10)
        {
          is_ED = false;
          AppMPU6050getdata.MPU6050_calibration();
        }
      }
    }
  }
}

/* 
--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 * Begin:CMD
 * Graphical programming and command control module
 $ Elegoo & SmartRobot & 2020-06
*/

void ApplicationFunctionSet::CMD_inspect_xxx0(void)
{
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_inspect)
  {
    Serial.println("CMD_inspect");
    delay(100);
  }
}
/*
  N1:command
  CMD mode：Car receive the control commands from the APP<motor control command>,perform unidirectional drive of the motor
  Input parameters:     uint8_t CMD_is_MotorSelection,  Motor selection  1left motor  2right motor  0both motors
                        uint8_t CMD_is_MotorDirection,  rotation direction selection  1forward  2backward  0stop
                        uint8_t CMD_is_MotorSpeed,      motor speed  0-250
  No time limited
*/
void ApplicationFunctionSet::CMD_MotorControl_xxx0(uint8_t is_MotorSelection, uint8_t is_MotorDirection, uint8_t is_MotorSpeed)
{
  static boolean MotorControl = false;
  static uint8_t is_MotorSpeed_A = 0;
  static uint8_t is_MotorSpeed_B = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl)
  {
    MotorControl = true;
    if (0 == is_MotorDirection)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    }
    else
    {
      switch (is_MotorSelection) //motor selection
      {
      case 0:
      {
        is_MotorSpeed_A = is_MotorSpeed;
        is_MotorSpeed_B = is_MotorSpeed;
        if (1 == is_MotorDirection)
        { //turn forward
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else if (2 == is_MotorDirection)
        { //turn backward
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      case 1:
      {
        is_MotorSpeed_A = is_MotorSpeed;
        if (1 == is_MotorDirection)
        { //turn forward
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_void, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else if (2 == is_MotorDirection)
        { //turn backward
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_void, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      case 2:
      {
        is_MotorSpeed_B = is_MotorSpeed;
        if (1 == is_MotorDirection)
        { //turn forward
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else if (2 == is_MotorDirection)
        { //turn backward
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      default:
        break;
      }
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
      is_MotorSpeed_A = 0;
      is_MotorSpeed_B = 0;
    }
  }
}
void ApplicationFunctionSet::CMD_MotorControl_xxx0(void)
{
  static boolean MotorControl = false;
  static uint8_t is_MotorSpeed_A = 0;
  static uint8_t is_MotorSpeed_B = 0;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl)
  {
    MotorControl = true;
    if (0 == CMD_is_MotorDirection)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    }
    else
    {
      switch (CMD_is_MotorSelection) //motor selection
      {
      case 0:
      {
        is_MotorSpeed_A = CMD_is_MotorSpeed;
        is_MotorSpeed_B = CMD_is_MotorSpeed;
        if (1 == CMD_is_MotorDirection)
        { //turn forward
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else if (2 == CMD_is_MotorDirection)
        { //turn backward
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      case 1:
      {
        is_MotorSpeed_A = CMD_is_MotorSpeed;
        if (1 == CMD_is_MotorDirection)
        { //turn forward
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_void, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else if (2 == CMD_is_MotorDirection)
        { //turn backward
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_void, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      case 2:
      {
        is_MotorSpeed_B = CMD_is_MotorSpeed;
        if (1 == CMD_is_MotorDirection)
        { //turn forward
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_just, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else if (2 == CMD_is_MotorDirection)
        { //turn backward
          AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ is_MotorSpeed_A,
                                                 /*direction_B*/ direction_back, /*speed_B*/ is_MotorSpeed_B,
                                                 /*controlED*/ control_enable); //Motor control
        }
        else
        {
          return;
        }
      }
      break;
      default:
        break;
      }
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
      is_MotorSpeed_A = 0;
      is_MotorSpeed_B = 0;
    }
  }
}

static void CMD_CarControl(uint8_t is_CarDirection, uint8_t is_CarSpeed)
{
  switch (is_CarDirection)
  {
  case 1: 
    ApplicationFunctionSet_SmartRobotCarMotionControl(Left, is_CarSpeed);
    break;
  case 2: 
    ApplicationFunctionSet_SmartRobotCarMotionControl(Right, is_CarSpeed);
    break;
  case 3: /*movement direction mode forward*/
    ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, is_CarSpeed);
    break;
  case 4: /*movement direction mode backward*/
    ApplicationFunctionSet_SmartRobotCarMotionControl(Backward, is_CarSpeed);
    break;
  default:
    break;
  }
}
/*
  N2：command
  CMD mode：Receive the control commands from the APP,perform movement direction and speed control of the car
  Time limited
*/
void ApplicationFunctionSet::CMD_CarControlTimeLimit_xxx0(uint8_t is_CarDirection, uint8_t is_CarSpeed, uint32_t is_Timer)
{
  static boolean CarControl = false;
  static boolean CarControl_TE = false; //Time stamp
  static boolean CarControl_return = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_TimeLimit) //enter time-limited control mode
  {
    CarControl = true;
    if (is_Timer != 0) //#1 if the pre-set time is not ... (zero)
    {
      if ((millis() - Application_SmartRobotCarxxx0.CMD_CarControl_Millis) > (is_Timer)) //check the timestamp
      {
        CarControl_TE = true;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*set mode to programming mode<Waiting for the next set of control commands>*/
        if (CarControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          CarControl_return = true;
        }
      }
      else
      {
        CarControl_TE = false; //There still has time left
        CarControl_return = false;
      }
    }
    if (CarControl_TE == false)
    {
      CMD_CarControl(is_CarDirection, is_CarSpeed);
    }
  }
  else
  {
    if (CarControl == true)
    {
      CarControl_return = false;
      CarControl = false;
      Application_SmartRobotCarxxx0.CMD_CarControl_Millis = 0;
    }
  }
}

void ApplicationFunctionSet::CMD_CarControlTimeLimit_xxx0(void)
{
  static boolean CarControl = false;
  static boolean CarControl_TE = false; //Time stamp
  static boolean CarControl_return = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_TimeLimit) //enter time-limited control mode
  {
    CarControl = true;
    if (CMD_is_CarTimer != 0) //#1 if the pre-set time is not ... (zero)
    {
      if ((millis() - Application_SmartRobotCarxxx0.CMD_CarControl_Millis) > (CMD_is_CarTimer)) //check the timestamp
      {
        CarControl_TE = true;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);

        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*set mode to programming mode<Waiting for the next set of control commands>*/
        if (CarControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          CarControl_return = true;
        }
      }
      else
      {
        CarControl_TE = false; //There still has time left
        CarControl_return = false;
      }
    }
    if (CarControl_TE == false)
    {
      CMD_CarControl(CMD_is_CarDirection, CMD_is_CarSpeed);
    }
  }
  else
  {
    if (CarControl == true)
    {
      CarControl_return = false;
      CarControl = false;
      Application_SmartRobotCarxxx0.CMD_CarControl_Millis = 0;
    }
  }
}
/*
  N3 command
  CMD mode：Receive the control commands from the APP,perform movement direction and speed control of the car
  No time limited
*/
void ApplicationFunctionSet::CMD_CarControlNoTimeLimit_xxx0(uint8_t is_CarDirection, uint8_t is_CarSpeed)
{
  static boolean CarControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_NoTimeLimit) //enter no time-limited control mode
  {
    CarControl = true;
    CMD_CarControl(is_CarDirection, is_CarSpeed);
  }
  else
  {
    if (CarControl == true)
    {
      CarControl = false;
    }
  }
}
void ApplicationFunctionSet::CMD_CarControlNoTimeLimit_xxx0(void)
{
  static boolean CarControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_CarControl_NoTimeLimit) //enter no time-limited control mode
  {
    CarControl = true;
    CMD_CarControl(CMD_is_CarDirection, CMD_is_CarSpeed);
  }
  else
  {
    if (CarControl == true)
    {
      CarControl = false;
    }
  }
}

/*
  N4 command
  CMD mode：movement mode<motor control>
  Receive the control commands from the APP,perform motion control of the left and right motors
*/
void ApplicationFunctionSet::CMD_MotorControlSpeed_xxx0(uint8_t is_Speed_L, uint8_t is_Speed_R)
{
  static boolean MotorControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl_Speed)
  {
    MotorControl = true;
    if (is_Speed_L == 0 && is_Speed_R == 0)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    }
    else
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ is_Speed_L,
                                             /*direction_B*/ direction_just, /*speed_B*/ is_Speed_R,
                                             /*controlED*/ control_enable); //Motor control
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
    }
  }
}
void ApplicationFunctionSet::CMD_MotorControlSpeed_xxx0(void)
{
  static boolean MotorControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_MotorControl_Speed)
  {
    MotorControl = true;
    if (CMD_is_MotorSpeed_L == 0 && CMD_is_MotorSpeed_R == 0)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    }
    else
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ CMD_is_MotorSpeed_L,
                                             /*direction_B*/ direction_just, /*speed_B*/ CMD_is_MotorSpeed_R,
                                             /*controlED*/ control_enable); //Motor control
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
    }
  }
}

/*
  N5:command
  CMD mode：<servo motor control>
*/
void ApplicationFunctionSet::CMD_ServoControl_xxx0(void)
{
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_ServoControl)
  {
    AppServo.DeviceDriverSet_Servo_controls(/*uint8_t Servo*/ CMD_is_Servo, /*unsigned int Position_angle*/ CMD_is_Servo_angle / 10);
    Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*set mode to programming mode<Waiting for the next set of control commands>*/
  }
}
/*
  N7:command
  CMD mode：<Lighting Control>
  Time limited：Enter programming mode after the time is over
*/
void ApplicationFunctionSet::CMD_LightingControlTimeLimit_xxx0(uint8_t is_LightingSequence, uint8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B,
                                                               uint32_t is_LightingTimer)
{
  static boolean LightingControl = false;
  static boolean LightingControl_TE = false; //Time stamp
  static boolean LightingControl_return = false;

  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_TimeLimit) //enter time-limited control mode
  {
    LightingControl = true;
    if (is_LightingTimer != 0) //#1 if the pre-set time is not ... (zero)
    {
      if ((millis() - Application_SmartRobotCarxxx0.CMD_LightingControl_Millis) > (is_LightingTimer)) //Check the timestamp
      {
        LightingControl_TE = true;
        FastLED.clear(true);
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*set mode to programming mode<Waiting for the next set of control commands>*/
        if (LightingControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          LightingControl_return = true;
        }
      }
      else
      {
        LightingControl_TE = false; //There still has time left
        LightingControl_return = false;
      }
    }
    if (LightingControl_TE == false)
    {
      CMD_Lighting(is_LightingSequence, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
    }
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl_return = false;
      LightingControl = false;
      Application_SmartRobotCarxxx0.CMD_LightingControl_Millis = 0;
    }
  }
}

void ApplicationFunctionSet::CMD_LightingControlTimeLimit_xxx0(void)
{
  static boolean LightingControl = false;
  static boolean LightingControl_TE = false; //Time stamp
  static boolean LightingControl_return = false;

  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_TimeLimit) //Enter Lighting Control mode with time-limited
  {
    LightingControl = true;
    if (CMD_is_LightingTimer != 0) //#1 if the pre-set time is not ... (zero)
    {
      if ((millis() - Application_SmartRobotCarxxx0.CMD_LightingControl_Millis) > (CMD_is_LightingTimer)) //Check the timestamp
      {
        LightingControl_TE = true;
        FastLED.clear(true);
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*set mode to programming mode<Waiting for the next set of control commands>*/
        if (LightingControl_return == false)
        {

#if _is_print
          Serial.print('{' + CommandSerialNumber + "_ok}");
#endif
          LightingControl_return = true;
        }
      }
      else
      {
        LightingControl_TE = false; //There still has time left
        LightingControl_return = false;
      }
    }
    if (LightingControl_TE == false)
    {
      CMD_Lighting(CMD_is_LightingSequence, CMD_is_LightingColorValue_R, CMD_is_LightingColorValue_G, CMD_is_LightingColorValue_B);
    }
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl_return = false;
      LightingControl = false;
      Application_SmartRobotCarxxx0.CMD_LightingControl_Millis = 0;
    }
  }
}
/*
  N8:command
  CMD mode：<Lighting control>
  No time limited
*/
void ApplicationFunctionSet::CMD_LightingControlNoTimeLimit_xxx0(uint8_t is_LightingSequence, uint8_t is_LightingColorValue_R, uint8_t is_LightingColorValue_G, uint8_t is_LightingColorValue_B)
{
  static boolean LightingControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_NoTimeLimit) //Enter Lighting Control mode without time-limited
  {
    LightingControl = true;
    CMD_Lighting(is_LightingSequence, is_LightingColorValue_R, is_LightingColorValue_G, is_LightingColorValue_B);
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl = false;
    }
  }
}
void ApplicationFunctionSet::CMD_LightingControlNoTimeLimit_xxx0(void)
{
  static boolean LightingControl = false;
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_LightingControl_NoTimeLimit) //Enter Lighting Control mode without time-limited
  {
    LightingControl = true;
    CMD_Lighting(CMD_is_LightingSequence, CMD_is_LightingColorValue_R, CMD_is_LightingColorValue_G, CMD_is_LightingColorValue_B);
  }
  else
  {
    if (LightingControl == true)
    {
      LightingControl = false;
    }
  }
}

/*
  N100/N110:command
  CMD mode：Clear all functions
*/
void ApplicationFunctionSet::CMD_ClearAllFunctions_xxx0(void)
{
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_ClearAllFunctions_Standby_mode) //Command:N100 Clear all functions to enter standby mode
  {
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    FastLED.clear(true);
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, NUM_LEDS /*Traversal_Number*/, CRGB::Black);
    Application_SmartRobotCarxxx0.Motion_Control = stop_it;
    Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
  }
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_ClearAllFunctions_Programming_mode) //Command:N110 Clear all functions and enter programming mode
  {

    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    FastLED.clear(true);
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, NUM_LEDS /*Traversal_Number*/, CRGB::Black);
    Application_SmartRobotCarxxx0.Motion_Control = stop_it;
    Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode;
  }
}

/*
  N21:command
  CMD mode：The ultrasonic module receives and feeds back the status and ranging data according to the control command of APP terminal.
  Input：
*/
void ApplicationFunctionSet::CMD_UltrasoundModuleStatus_xxx0(uint8_t is_get)
{
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&UltrasoundData_cm /*out*/); //Ultrasonic data
  UltrasoundDetectionStatus = function_xxx(UltrasoundData_cm, 0, ObstacleDetection);
  if (1 == is_get) //ultrasonic sensor  is_get Start     true：has obstacle / false: no obstable
  {
    if (true == UltrasoundDetectionStatus)
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    }
    else
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }
  }
  else if (2 == is_get) //ultrasonic sensor is_get data
  {
    char toString[10];
    sprintf(toString, "%d", UltrasoundData_cm);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
  }
}
/*
  N22:command
  CMD mode：Tracking module receives and feeds back tracing status and data according to the control command of APP terminal
  Input：
*/
void ApplicationFunctionSet::CMD_TraceModuleStatus_xxx0(uint8_t is_get)
{
  char toString[10];
  if (0 == is_get) /*Get left IR sensor status*/
  {
    sprintf(toString, "%d", TrackingData_L);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
    /*
    if (true == TrackingDetectionStatus_L)
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    }
    else
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }*/
  }
  else if (1 == is_get) /*Get middle IR sensor status*/
  {
    sprintf(toString, "%d", TrackingData_M);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
    /*
    if (true == TrackingDetectionStatus_M)
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    }
    else
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }*/
  }
  else if (2 == is_get) /*Get right IR sensor status*/
  {
    sprintf(toString, "%d", TrackingData_R);
#if _is_print
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
#endif
    /*
        if (true == TrackingDetectionStatus_R)
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_true}");
#endif
    }
    else
    {
#if _is_print
      Serial.print('{' + CommandSerialNumber + "_false}");
#endif
    }*/
  }
  Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*set mode to programming mode<Waiting for the next set of control commands>*/
}

/* 
 * End:CMD
 * Graphical programming and command control module
 $ Elegoo & SmartRobot & 2020-06
 --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*Key command*/
void ApplicationFunctionSet::ApplicationFunctionSet_KeyCommand(void)
{
  uint8_t get_keyValue;
  static uint8_t temp_keyValue = keyValue_Max;
  AppKey.DeviceDriverSet_key_Get(&get_keyValue);

  if (temp_keyValue != get_keyValue)
  {
    temp_keyValue = get_keyValue;//Serial.println(get_keyValue);
    switch (get_keyValue)
    {
    case /* constant-expression */ 1:
      /* code */
      Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
      break;
    case /* constant-expression */ 2:
      /* code */
      Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
      break;
    case /* constant-expression */ 3:
      /* code */
      Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
      break;
    case /* constant-expression */ 4:
      /* code */
      Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
      break;
    default:

      break;
    }
  }
}
/*Infrared remote control*/
void ApplicationFunctionSet::ApplicationFunctionSet_IRrecv(void)
{
  uint8_t IRrecv_button;
  static bool IRrecv_en = false;
  if (AppIRrecv.DeviceDriverSet_IRrecv_Get(&IRrecv_button /*out*/))
  {
    IRrecv_en = true;
    //Serial.println(IRrecv_button);
  }
  if (true == IRrecv_en)
  {
    switch (IRrecv_button)
    {
    case /* constant-expression */ 1:
      /* code */
      Application_SmartRobotCarxxx0.Motion_Control = Forward;
      break;
    case /* constant-expression */ 2:
      /* code */
      Application_SmartRobotCarxxx0.Motion_Control = Backward;
      break;
    case /* constant-expression */ 3:
      /* code */
      Application_SmartRobotCarxxx0.Motion_Control = Left;
      break;
    case /* constant-expression */ 4:
      /* code */
      Application_SmartRobotCarxxx0.Motion_Control = Right;
      break;
    case /* constant-expression */ 5:
      /* code */
      Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
      break;
    case /* constant-expression */ 6:
      /* code */ Application_SmartRobotCarxxx0.Functional_Mode = TraceBased_mode;
      break;
    case /* constant-expression */ 7:
      /* code */ Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
      break;
    case /* constant-expression */ 8:
      /* code */ Application_SmartRobotCarxxx0.Functional_Mode = Follow_mode;
      break;
    case /* constant-expression */ 9:
      /* code */ if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode) //Adjust the threshold of the line tracking module to adapt the actual environment
      {
        if (TrackingDetection_S < 600)
        {
          TrackingDetection_S += 10;
        }
      }

      break;
    case /* constant-expression */ 10:
      /* code */ if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
      {
        TrackingDetection_S = 250;
      }
      break;
    case /* constant-expression */ 11:
      /* code */ if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
      {
        if (TrackingDetection_S > 30)
        {
          TrackingDetection_S -= 10;
        }
      }
      break;

    case /* constant-expression */ 12:
    {
      if (Rocker_CarSpeed < 255)
      {
        Rocker_CarSpeed += 5;
      }
    }
    break;
    case /* constant-expression */ 13:
    {
      Rocker_CarSpeed = 250;
    }
    break;
    case /* constant-expression */ 14:
    {
      if (Rocker_CarSpeed > 50)
      {
        Rocker_CarSpeed -= 5;
      }
    }
    break;

    default:
      Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
      break;
    }
    /*achieve time-limited control on movement direction part*/
    if (IRrecv_button < 5)
    {
      Application_SmartRobotCarxxx0.Functional_Mode = Rocker_mode;
      if (millis() - AppIRrecv.IR_PreMillis > 300)
      {
        IRrecv_en = false;
        Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
        AppIRrecv.IR_PreMillis = millis();
      }
    }
    else
    {
      IRrecv_en = false;
      AppIRrecv.IR_PreMillis = millis();
    }
  }
}
/*Data analysis on serial port
  The ESP32 only ever sends three commands the car uses (all flat JSON):
    {"N":200,"D1":"S","D2":cx,"D3":cy,"D4":w,"D5":h,"D6":"R","D7":px,"D8":cm}  camera object
    {"N":300,"D1":1}  E-stop engage (D1 0 = release)
    {"N":301}         reset position
  The Elegoo phone-app commands (N 1..110) were removed: the car is forced into
  line-tracking mode every loop, so they could never take effect. Parsing these
  three by hand instead of with ArduinoJson + String saves about 8 KB of flash.*/
// Number after "key": (e.g. key "\"D2\":"), or def if the key isn't there
static long Json_Long(const char *buf, const char *key, long def)
{
  const char *p = strstr(buf, key);
  return p ? atol(p + strlen(key)) : def;
}
// First letter of the string value after "key":" (e.g. key "\"D1\":\""), or def
static char Json_Char(const char *buf, const char *key, char def)
{
  const char *p = strstr(buf, key);
  return p ? p[strlen(key)] : def;
}
void ApplicationFunctionSet::ApplicationFunctionSet_SerialPortDataAnalysis(void)
{
  static char buf[96];
  static uint8_t len = 0;
  bool frame = false;
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == '{')
      len = 0; // start of a new frame (also drops any half-received junk)
    if (len < sizeof(buf) - 1)
      buf[len++] = c;
    if (c == '}')
    {
      frame = true;
      break;
    }
  }
  if (!frame)
    return;
  buf[len] = 0;
  len = 0;

  long n = Json_Long(buf, "\"N\":", -1);

  if (n == 300) /* E-stop from the web page: D1 1 = engage, 0 = release */
  {
    bool was_active = estop_active;
    estop_active = Json_Long(buf, "\"D1\":", 0) != 0;
    Serial.println(estop_active ? F("{E-STOP ENGAGED}") : F("{E-STOP RELEASED}"));
#if RESET_ON_ESTOP_RELEASE
    // Releasing the E-stop starts a run: put the origin (X = Y = heading = 0) and the lap timer
    // here, so the logged path always starts at the start line. (Pressing Reset position by hand
    // after the car had started moving put the origin 0.45-0.58 m up the track in two runs.)
    // The gyro offset was re-measured while the car stood still, so the heading starts clean.
    if (was_active && !estop_active)
    {
      Kinematics_Reset();
      Serial.print(F("{POSE RESET}"));
    }
#endif
  }
  else if (n == 301) /* "Reset position" button on the web page: car is at the origin now */
  {
    Kinematics_Reset();
    Serial.print(F("{POSE RESET}"));
  }
  else if (n == 200) /* object sign data from ESP32 camera */
  {
    obj_prev = obj_now;
    obj_now.type = Json_Char(buf, "\"D1\":\"", 'N');
    obj_now.cent_x = Json_Long(buf, "\"D2\":", 0);
    obj_now.cent_y = Json_Long(buf, "\"D3\":", 0);
    obj_now.width = Json_Long(buf, "\"D4\":", 0);
    obj_now.height = Json_Long(buf, "\"D5\":", 0);
    obj_now.colour = Json_Char(buf, "\"D6\":\"", '?');
    obj_now.sizePX = Json_Long(buf, "\"D7\":", 0);
    obj_now.dist_cm = Json_Long(buf, "\"D8\":", -1);
    obj_now.lastUpdate = millis();

#if STOP_SIGN_DEBUG
    // Echo every camera message the UNO understood, so you can check the link
    Serial.print(F("{UNO rx type="));
    Serial.print(obj_now.type);
    Serial.print(F(" col="));
    Serial.print(obj_now.colour);
    Serial.print(F(" x="));
    Serial.print(obj_now.cent_x);
    Serial.print(F(" d="));
    Serial.print(obj_now.dist_cm);
    Serial.print('}');
#endif
    StopSign_Check();
  }
  // anything else ({"N":100}, {Factory}, {WA_OK}, ...) is ignored
}
