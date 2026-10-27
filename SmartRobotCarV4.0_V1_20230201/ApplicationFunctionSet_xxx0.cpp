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
    DRIVE      -> close red object seen                        -> RED_WAIT   (stop, model checks it)
    RED_WAIT   -> camera confirms stop sign ('S')              -> TO_STOP_LINE
    RED_WAIT   -> camera says "not a stop sign" ('X') twice    -> RED_IGNORE (drive on)
    RED_WAIT   -> red no longer seen                           -> DRIVE
    RED_IGNORE -> drive, ignoring this red; a stop sign ('S')  -> TO_STOP_LINE
    RED_IGNORE -> red no longer seen                           -> DRIVE
    Stop sign confirmed: the car drives on to the NEXT stop line on its route (Next_StopLine: the one
    before the crossing on the section it is on, or that it is heading for), using the position
    estimate, and holds there. Only if its position on the track is unknown does it hold where it is.
    (Before, the stop line came from matching the sign through its rough camera range, so when that
     match failed the car held wherever it was - the inconsistent stops at the intersection.)
    TO_STOP_LINE -> estimate reaches the stop line             -> SIGN_HOLD
    SIGN_HOLD  -> 3 s passed                                   -> COOLDOWN
    COOLDOWN   -> drive, ignore red for 5 s so we can pass the sign -> DRIVE
*/
#define STOP_SIGN_DEBUG        0     // 1 = print state changes, 0 = off (off to save flash: the state is
                                     // already in every telemetry row / the CSV state column)
#define STOP_SIGN_HOLD_MS      3000  // how long to stop at a confirmed stop sign
#define STOP_SIGN_COOLDOWN_MS  5000  // ignore red/stop signs this long after the hold, so the car can drive past
#define RED_FRESH_MS           2000  // red counts as "still there" if seen within this time
                                     // (the ESP32 sends about every 0.5-1.7 s, so keep this above ~1.7 s)
#define RED_STOP_MIN_PX        60    // stop for red blobs at least this big (= close enough). Was 150, 100, then 70
                                     // (a sign off to the side only reached 100 px nearly in front of the camera).
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
static void Light_Calibrate(void);                   // traffic light range constant, at the start
static void Light_Fix(void);                         // logs the traffic light's estimated position once
static bool StopLine_Reached(int8_t m);             // true once the estimate is at map sign m's stop line
#define TO_STOP_LINE_TIMEOUT_MS 15000               // hold anyway if the stop line isn't reached by then
                                                    // (signs are seen up to ~2 m away, sometimes before a corner)
static int8_t approach_sign = -1;                   // STOP_MAP entry being approached
static int8_t Next_StopLine(void);                  // stop line the car is heading for (-1 = unknown)

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
  Camera_SignFix();                // position fix from the sign (and the log), when it matches
  approach_sign = Next_StopLine(); // then: the crossing the car is heading for
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

/*---------------- Traffic light at the start (red foam = stop, green foam = go) ----------------
  At power-on and every time the E-stop is released the car stays stopped:
    LIGHT_LOOK -> red blob seen      -> LIGHT_RED (light identified, wait for green)
    LIGHT_LOOK -> no red within 3 s  -> LIGHT_GO  (no red light there: drive)
    LIGHT_RED  -> green blob seen    -> LIGHT_GO  (drive)
  A green blob before any red is ignored: the light has to be identified first.
*/
#define LIGHT_MIN_PX 100     // blob size (px) for the red / green light to count (same as RED_STOP_MIN_PX)
#define LIGHT_LOOK_MS 3000   // look for a red light this long before driving off without one
enum LightState
{
  LIGHT_LOOK,
  LIGHT_RED,
  LIGHT_GO
};
static LightState light_state = LIGHT_LOOK;
static unsigned long light_look_start = 0; // millis() when LIGHT_LOOK started
static bool light_look_started = false;   // false = the 3 s haven't started yet
static unsigned long light_red_time = 0;   // millis() when the red light was identified

// True if a camera reading that arrived at or after 'since' saw a blob of this colour big (= close)
// enough to be the light. Readings from before the check started (e.g. seen during the E-stop) don't count.
static bool LightSeen(char colour, unsigned long since)
{
  return obj_now.colour == colour &&
         obj_now.sizePX >= LIGHT_MIN_PX &&
         (long)(obj_now.lastUpdate - since) >= 0 &&
         (millis() - obj_now.lastUpdate) < RED_FRESH_MS;
}

// Returns true while the car must wait at the light
bool ApplicationFunctionSet::ApplicationFunctionSet_TrafficLight(void)
{
  // Start the 3 s on the first check, not at power-on (setup itself takes a few seconds)
  // (FIX: this used to store millis() | 1, which is 1 ms in the future on an even millisecond, so
  //  millis() - light_look_start wrapped to ~4 billion and the 3 s check ended at once, ~half the time)
  if (light_state == LIGHT_LOOK && !light_look_started)
  {
    light_look_start = millis();
    light_look_started = true;
  }
  if (light_state == LIGHT_LOOK && LightSeen('R', light_look_start))
  {
    light_state = LIGHT_RED;
    light_red_time = millis();
    Serial.print(F("{LIGHT RED - waiting for green}"));
    Light_Calibrate(); // standing at the start: learn the light's blob size vs distance
  }
  else if (light_state == LIGHT_LOOK && millis() - light_look_start >= LIGHT_LOOK_MS)
  {
    light_state = LIGHT_GO;
    Serial.print(F("{LIGHT NONE - go}"));
  }
  else if (light_state == LIGHT_RED && LightSeen('G', light_red_time))
  {
    light_state = LIGHT_GO;
    RedStop_SetState(RS_COOLDOWN); // don't stop for the red foam while pulling away
    Serial.print(F("{LIGHT GREEN - go}"));
    Light_Calibrate();
  }
  if (light_state == LIGHT_GO)
    return false;
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  return true;
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

  // RED_WAIT stops at once (so there is time to check the red and still stop before the crossing);
  // SIGN_HOLD is the 3 s at the stop line
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
  // FIX: start the position estimate at the start pose at power-on too. Before, Pose/Est started
  // at (0, 0) and only moved to START_X_M/START_Y_M when the E-stop was released, so a run started
  // straight from power-on had the map offset by the start position (wrong corner turn directions).
  Kinematics_Reset();
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
  // (FIX: a local "ApplicationFunctionSet Application_FunctionSet;" was here. It was never used, but it
  //  built a whole copy of the object, String included, on every motor command.)
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;

  // red object / stop sign: override any movement (not during an overtaking manoeuvre: stopping
  // half-way through a turn out of the lane would leave the car beside the line)
  if (direction != stop_it && !Overtaking && RedStop_MustStop())
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
    { // (the Elegoo yaw-hold controller for the phone-app modes was removed to save flash:
      //  the car is always in line-tracking mode, so it never ran)
      AppMotor.DeviceDriverSet_Motor_control(direction_just, speed, direction_just, speed, control_enable);
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
    {
      AppMotor.DeviceDriverSet_Motor_control(direction_back, speed, direction_back, speed, control_enable);
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
    TrackingDetectionStatus_R = function_xxx(TrackingData_R, TrackingDetection_SR, TrackingDetection_E);
    TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    TrackingDetectionStatus_M = function_xxx(TrackingData_M, TrackingDetection_SM, TrackingDetection_E);
    TrackingData_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    TrackingDetectionStatus_L = function_xxx(TrackingData_L, TrackingDetection_SL, TrackingDetection_E);
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
bool EStop_Pressed = false;      // see ApplicationFunctionSet_xxx0.h
#define RESET_ON_ESTOP_RELEASE 1 // 1 = releasing the E-stop zeroes the position and starts the lap timer
                                 // (so an E-stop in the middle of a run restarts the coordinates there)
bool ApplicationFunctionSet::ApplicationFunctionSet_EStop(void)
{
  if (estop_active)
  {
    // PWM 0 and STBY LOW: motor driver disabled
    AppMotor.DeviceDriverSet_Motor_control(direction_void, 0, direction_void, 0, control_enable);
    // Re-arm the traffic light check every loop while stopped, so leaving the E-stop always starts
    // with the 3 s CHECK_LIGHT, however the release arrives (the log showed releases that skipped it)
    light_state = LIGHT_LOOK;
    light_look_started = false;
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

/*---------------- Position estimate: EKF (extended Kalman filter) ----------------
  State: X, Y (m), heading psi (rad), and s = speed scale of the wheel model (how far the car really
  goes per metre the PWM model predicts). s is learned from the landmark fixes, so the model adapts to
  the floor's traction and the battery; it is kept between runs (only a power cycle resets it to 1).
  Predict (every Kinematics_Update):  X += s*v*cos(psi)*dt,  Y += s*v*sin(psi)*dt,  psi += gyro step
    v = wheel model speed from the PWM; the heading uses the gyro (accurate) instead of the model.
    Process noise: wiggle/slip per metre driven, gyro drift per second and scale error per radian.
  Update (Ekf_PosUpdate): a measurement of where the car's centre is, from a landmark on the map:
    - track corners (exact): a line search at a corner that finds the next section (Corner_Fix)
    - the obstacle (moose, centre of the bottom section): ultrasonic range when a pass starts
    - the two stop signs at the crossing: camera range (blob size) + bearing when a sign is confirmed
    (the traffic light's position is only logged, once per run - its fixes were unreliable)
  Each update is gated (Mahalanobis distance), so a wrong match or a moved landmark can't throw the
  estimate off. Est_* = the EKF estimate; Pose_* = the model alone (for the comparison in the report).
  Every fix is logged with the estimate BEFORE the fix, so the report can show the raw estimate error.*/
#define EKF_SIG_POS0  0.03f                 // m, start position uncertainty (car placed by hand)
#define EKF_SIG_PSI0  (3.0f * PI / 180.0f)  // rad, start heading uncertainty
#define EKF_SIG_S0    0.10f                 // speed scale starts at 1 +- 10 %
#define EKF_Q_POS     0.0004f               // m^2 per m driven (2 cm per m: wiggle on the line, slip)
#define EKF_Q_PSI_T   2.7e-5f               // rad^2 per s (gyro drift ~0.3 deg/s)
#define EKF_Q_PSI_R   0.0009f               // rad^2 per rad turned (3 % gyro scale error)
#define EKF_Q_S       4e-6f                 // per s: lets the speed scale drift slowly
#define EKF_GATE      25.0f                 // reject a fix more than 5 sigma from the estimate

float Heading_Gyro = 0; // deg, same sign convention as the model (+ = left), 0 at pose reset
float Est_X = 0;        // m
float Est_Y = 0;        // m
float Est_Psi = 0;      // rad
float Ekf_Scale = 1.0f; // speed scale s
static float P[4][4];   // covariance of [X, Y, psi, s]
// Loop count for the 4x4 loops. volatile so the compiler can't unroll them: unrolled, the EKF
// doesn't fit in the UNO's flash (loops cost a few microseconds, which doesn't matter here).
static volatile uint8_t EKF_N = 4;
static float ekf_last_gyro = 0; // Heading_Gyro at the last predict step (deg)

// New run: the pose is known (start), the learned speed scale is kept
static void Ekf_ResetPose(void)
{
  float pss = P[3][3] > 0 ? P[3][3] : EKF_SIG_S0 * EKF_SIG_S0;
  memset(P, 0, sizeof(P));
  P[0][0] = P[1][1] = EKF_SIG_POS0 * EKF_SIG_POS0;
  P[2][2] = EKF_SIG_PSI0 * EKF_SIG_PSI0;
  P[3][3] = pss;
}

// v = model speed (m/s) over dt (s); the heading step comes from the gyro
static void Ekf_Predict(float v, float dt)
{
  float dpsi = (Heading_Gyro - ekf_last_gyro) * (PI / 180.0f);
  ekf_last_gyro = Heading_Gyro;
  float ds = v * dt; // distance by the model
  float c = cos(Est_Psi + 0.5f * dpsi), sn = sin(Est_Psi + 0.5f * dpsi);
  Est_X += Ekf_Scale * ds * c;
  Est_Y += Ekf_Scale * ds * sn;
  Est_Psi += dpsi;

  // P = F P F^T + Q with F = I + G; G only has G[0][2], G[0][3], G[1][2], G[1][3]
  if (ds != 0)
  {
    float a = -Ekf_Scale * ds * sn, b = ds * c, e = Ekf_Scale * ds * c, f = ds * sn;
    float g0[4], g1[4]; // rows 0 and 1 of G P
    for (uint8_t j = 0; j < EKF_N; j++)
    {
      g0[j] = a * P[2][j] + b * P[3][j];
      g1[j] = e * P[2][j] + f * P[3][j];
    }
    float q00 = g0[2] * a + g0[3] * b, q01 = g0[2] * e + g0[3] * f, q11 = g1[2] * e + g1[3] * f; // G P G^T
    for (uint8_t j = 0; j < EKF_N; j++)
    {
      P[0][j] += g0[j]; // + G P
      P[1][j] += g1[j];
    }
    for (uint8_t i = 0; i < EKF_N; i++)
    {
      P[i][0] += g0[i]; // + P G^T = (G P)^T
      P[i][1] += g1[i];
    }
    P[0][0] += q00;
    P[0][1] += q01;
    P[1][0] += q01;
    P[1][1] += q11;
  }
  float dist = fabs(ds * Ekf_Scale);
  P[0][0] += EKF_Q_POS * dist;
  P[1][1] += EKF_Q_POS * dist;
  P[2][2] += EKF_Q_PSI_T * dt + EKF_Q_PSI_R * fabs(dpsi);
  P[3][3] += EKF_Q_S * dt;
}

/* Fix: the car's centre was measured at (zx, zy), with standard deviation sa along the unit vector
   (ux, uy) and sc across it. Returns false if it failed the gate (not used). *dx, *dy = how far the
   estimate moved. All the state (heading and speed scale too) is corrected through the covariance.*/
bool Ekf_PosUpdate(float zx, float zy, float ux, float uy, float sa, float sc, float *dx, float *dy)
{
  *dx = *dy = 0;
  float r00 = sa * sa * ux * ux + sc * sc * uy * uy; // R = sa^2 u u^T + sc^2 n n^T
  float r01 = (sa * sa - sc * sc) * ux * uy;
  float r11 = sa * sa * uy * uy + sc * sc * ux * ux;
  float s00 = P[0][0] + r00, s01 = P[0][1] + r01, s11 = P[1][1] + r11; // S = H P H^T + R
  float det = s00 * s11 - s01 * s01;
  if (det <= 1e-12f)
    return false;
  float i00 = s11 / det, i01 = -s01 / det, i11 = s00 / det; // S^-1
  float ex = zx - Est_X, ey = zy - Est_Y;                   // innovation
  if (ex * (i00 * ex + i01 * ey) + ey * (i01 * ex + i11 * ey) > EKF_GATE)
    return false;
  float k[4][2]; // K = P H^T S^-1
  for (uint8_t i = 0; i < EKF_N; i++)
  {
    k[i][0] = P[i][0] * i00 + P[i][1] * i01;
    k[i][1] = P[i][0] * i01 + P[i][1] * i11;
  }
  *dx = k[0][0] * ex + k[0][1] * ey;
  *dy = k[1][0] * ex + k[1][1] * ey;
  Est_X += *dx;
  Est_Y += *dy;
  Est_Psi += k[2][0] * ex + k[2][1] * ey;
  Ekf_Scale = constrain(Ekf_Scale + k[3][0] * ex + k[3][1] * ey, 0.7f, 1.3f);
  float p0[4], p1[4]; // P = P - K H P
  memcpy(p0, P[0], sizeof(p0));
  memcpy(p1, P[1], sizeof(p1));
  for (uint8_t i = 0; i < EKF_N; i++)
    for (uint8_t j = 0; j < EKF_N; j++)
      P[i][j] -= k[i][0] * p0[j] + k[i][1] * p1[j];
  for (uint8_t i = 0; i < EKF_N; i++) // keep it symmetric
    for (uint8_t j = i + 1; j < EKF_N; j++)
      P[i][j] = P[j][i] = 0.5f * (P[i][j] + P[j][i]);
  return true;
}

static void Print_mm(float m)
{
  Serial.print(',');
  Serial.print((long)(m * 1000));
}

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
// Start used for the gray-floor track: on the long diagonal, between the crossing (X = 1.50) and the
// sharp right corner (X = 3.20), facing the sharp corner. 2.35 m = roughly halfway; measure the car
// centre's distance past the crossing centre and put 1.50 + that distance here.
#define START_X_M     2.35f
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
  ekf_last_gyro = START_PSI_DEG;
  Ekf_ResetPose();
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

  // EKF estimate: model speed (times the learned scale) with the gyro heading
  Ekf_Predict(v, dt);

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

/*---------------- Camera landmarks: stop signs and the traffic light ----------------
  Range from the blob size (pinhole camera: blob pixels ~ 1/range^2, range = K / sqrt(px)) and bearing
  from the blob's x position give where the landmark is relative to the car, so with the landmark's
  position on the map the car's position is known: car = landmark - range * (cos, sin)(psi + bearing).
  The range is only rough (blob size depends on glare, angle...), so the fix is trusted much less
  along the line of sight than across it.
  Logged with the estimate BEFORE the fix:
    {SG,car_x,car_y,sign_x,sign_y,fix_dx,fix_dy,match}  stop sign confirmed (mm; match -1 = no fix)
    {TL,car_x,car_y,light_x,light_y,fix_dx,fix_dy}      traffic light seen on the 1.7 m section, before it*/
#define CAM_HFOV_DEG  60.0f  // camera horizontal field of view (deg); adjust for a wide lens
#define CAM_IMG_W     160    // QQVGA image width (pixels), D2 = blob centre x in this image
#define CAM_RANGE_K   5.5f   // stop sign: m*sqrt(px), range = K / sqrt(blob px). Was 18, which put the signs
                             // ~3x too far away (robot_log_8: 4 sightings vs the measured sign positions
                             // gave K = 4.9 / 5.5 / 6.5 / 5.1). Re-measure: sign at a known distance d,
                             // read its pixel count N on the ESP32 serial, K = d * sqrt(N)
#define CAM_RANGE_SIG 0.25f  // range error, fraction of the range
#define CAM_GATE_DEG  45     // a sign only matches if the car's heading is within this of its approach
#define SIGN_MATCH_M  1.0f   // ... and the sign estimate is within this of it

// The two stop signs, just before the crossing (1.50, 0), off to the side of the tape.
// MEASURE: how far before the crossing centre (along the tape) and how far to the side of it.
#define SIGN_A_BEFORE_M 0.15f // sign facing the 1.2 m section (car approaching at 120 deg)
#define SIGN_A_SIDE_M   0.15f //   on the side toward the 1.5 m section
#define SIGN_B_BEFORE_M 0.15f // sign facing the 1.5 m section (car approaching at 0 deg)
#define SIGN_B_SIDE_M   0.15f //   on the side toward the 1.2 m section
#define STOP_LINE_GAP_M 0.35f // the car holds with its centre this far before the crossing centre
                              // (was 0.20: stopped a little too close to the intersection)

struct StopSign
{
  float x, y;           // m, the sign
  int16_t psi;          // deg, heading of a car approaching it
  float line_x, line_y; // m, where the car holds for 3 s (before the crossing)
};
static const StopSign STOP_MAP[] = {
    // 1.2 m section: direction 120 deg = (-0.5, 0.866); the side toward the 1.5 m section = (-0.866, -0.5)
    {1.50f + 0.5f * SIGN_A_BEFORE_M - 0.866f * SIGN_A_SIDE_M, -0.866f * SIGN_A_BEFORE_M - 0.5f * SIGN_A_SIDE_M,
     120, 1.50f + 0.5f * STOP_LINE_GAP_M, -0.866f * STOP_LINE_GAP_M},
    // 1.5 m section (the start of the long diagonal): direction 0 deg; the side toward the 1.2 m section = -Y
    {1.50f - SIGN_B_BEFORE_M, -SIGN_B_SIDE_M, 0, 1.50f - STOP_LINE_GAP_M, 0.0f},
};
#define STOP_MAP_N (sizeof(STOP_MAP) / sizeof(STOP_MAP[0]))

// Traffic light: just in front of the start, to the left of the tape (START_X_M, START_Y_M, facing
// the top-left corner). It used to stand straight past the top-left corner, but from the start it was
// too far away to see the red, and on lap 3 the car took it for an obstacle at that corner.
// MEASURE both from the car's centre at the start: make sure it is in view on /stream from the start.
#define LIGHT_AHEAD_M  0.40f // how far in front of the start position
#define LIGHT_LEFT_M   0.20f // how far to the left of the tape
#define LIGHT_X        (START_X_M + LIGHT_AHEAD_M)
#define LIGHT_Y        (START_Y_M + LIGHT_LEFT_M)
static float light_k = 0; // m*sqrt(px) for the light's blobs, measured at the start (Light_Calibrate)

static float wrap180f(float deg)
{
  deg = fmod(deg, 360.0f);
  if (deg > 180)
    deg -= 360;
  if (deg < -180)
    deg += 360;
  return deg;
}

// Bearing of the latest blob (rad, + = left)
static float Blob_Bearing(void)
{
  return ((CAM_IMG_W / 2.0f) - obj_now.cent_x) * (CAM_HFOV_DEG * PI / 180.0f / CAM_IMG_W);
}

// Car-position fix from a landmark at (lx, ly) seen at range r and bearing b. Prints the pre-fix
// car estimate, the landmark estimate from it, and the fix.
static bool Landmark_Fix(float lx, float ly, float r, float b, bool use, float *dx, float *dy)
{
  float ux = cos(Est_Psi + b), uy = sin(Est_Psi + b); // line of sight
  Print_mm(Est_X);
  Print_mm(Est_Y);
  Print_mm(Est_X + r * ux); // landmark position estimated from the car (before the fix)
  Print_mm(Est_Y + r * uy);
  bool ok = use && !Overtaking &&
            Ekf_PosUpdate(lx - r * ux, ly - r * uy, ux, uy, CAM_RANGE_SIG * r + 0.05f, 0.05f * r + 0.03f, dx, dy);
  Print_mm(*dx);
  Print_mm(*dy);
  return ok;
}

// The car stops at the stop line: distance still to drive along the approach direction <= 2 cm
static bool StopLine_Reached(int8_t m)
{
  if (m < 0 || m >= (int8_t)STOP_MAP_N)
    return true;
  float a = STOP_MAP[m].psi * (PI / 180.0f);
  return (STOP_MAP[m].line_x - Est_X) * cos(a) + (STOP_MAP[m].line_y - Est_Y) * sin(a) <= 0.02f;
}

// Stop sign confirmed: which mapped sign is it, and fix the position with it. Returns the sign (-1 = none).
static int8_t Camera_SignFix(void)
{
  float range = 0.9f, bearing = 0;
  if (obj_now.colour == 'R' && obj_now.sizePX > 0)
  {
    range = constrain(CAM_RANGE_K / sqrt((float)obj_now.sizePX), 0.1f, 3.0f);
    bearing = Blob_Bearing();
  }
  float sx = Est_X + range * cos(Est_Psi + bearing), sy = Est_Y + range * sin(Est_Psi + bearing);
  int8_t match = -1;
  float best = SIGN_MATCH_M;
  for (uint8_t i = 0; i < STOP_MAP_N; i++)
  {
    if (fabs(wrap180f(Est_Psi * (180.0f / PI) - STOP_MAP[i].psi)) > CAM_GATE_DEG)
      continue;
    float d = hypot(STOP_MAP[i].x - sx, STOP_MAP[i].y - sy);
    if (d < best)
    {
      best = d;
      match = i;
    }
  }
  float dx, dy;
  Serial.print(F("{SG"));
  bool ok = Landmark_Fix(match >= 0 ? STOP_MAP[match].x : 0, match >= 0 ? STOP_MAP[match].y : 0,
                         range, bearing, match >= 0, &dx, &dy);
  Serial.print(',');
  Serial.print(ok ? match : -1);
  Serial.print('}');
  return match; // the stop line is used even if the position fix failed the gate
}

// At the start (car standing at the known start pose): the light's blob size there gives its range
// constant, so later sightings give a range. Called when the light is identified / turns green.
static bool light_logged = false; // the light's position has been logged this run

static void Light_Calibrate(void)
{
  if (obj_now.sizePX == 0)
    return;
  light_k = hypot(LIGHT_X - Est_X, LIGHT_Y - Est_Y) * sqrt((float)obj_now.sizePX);
  light_logged = false; // new run: log the light again once
}

// The first red/green blob on the 1.7 m section after the start (heading for the top-left corner) =
// the traffic light: log where the car estimates it is, ONCE per run ({TL,...}). It is not used to
// correct the position: in robot_log_8 two of those fixes pulled the estimate 0.19 m and 0.38 m
// sideways while the car was on the tape (the corners and the moose correct it reliably).
static void Light_Fix(void)
{
  if (light_logged || light_k <= 0 || light_state != LIGHT_GO || Overtaking || obj_now.type == 'S' ||
      (obj_now.colour != 'R' && obj_now.colour != 'G') || obj_now.sizePX < LIGHT_MIN_PX ||
      fabs(wrap180f(Est_Psi * (180.0f / PI))) > 25 || Est_X < 1.6f || Est_X > LIGHT_X - 0.25f)
    return;
  light_logged = true;
  float range = constrain(light_k / sqrt((float)obj_now.sizePX), 0.1f, 3.0f), dx, dy;
  Serial.print(F("{TL"));
  Landmark_Fix(LIGHT_X, LIGHT_Y, range, Blob_Bearing(), false, &dx, &dy); // log only, no fix
  Serial.print('}');
}

/*Telemetry for the ESP32 data logger, 5x per second:
    {T,time_ms,heading_gyro,heading_pred,x_pred,y_pred,heading_est,x_est,y_est,lap_ds,lap_cm,laps,sig_mm,scale,state}
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
  else if (light_state == LIGHT_LOOK)
    state = F("CHECK_LIGHT"); // first 3 s: checking for the red stop light
  else if (light_state == LIGHT_RED)
    state = F("RED_LIGHT");
  else if (Overtaking)
    state = F("OVERTAKING");
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
  Print_mm(sqrt(P[0][0] + P[1][1])); // EKF position uncertainty (1 sigma), mm
  Serial.print(',');
  Serial.print((long)(Ekf_Scale * 1000)); // learned speed scale x 1000
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
/*Status LED. The car is always in line-tracking mode, so Elegoo's per-mode colours came down to:
  green, or (battery low) a red blink pattern for 0.5 s every 3 s. Rewritten to just that, and the LED
  is only re-sent when its colour changes (the old code cleared and re-sent it on every loop, and the
  per-mode code took ~0.6 KB of flash that the position estimate needs).*/
void ApplicationFunctionSet::ApplicationFunctionSet_RGB(void)
{
  static unsigned long blink_start = 0;
  static uint8_t shown = 255; // 0 = off, 1 = red, 2 = green
  if (VoltageDetectionStatus && millis() - blink_start > 3000)
    blink_start = millis();
  unsigned long t = millis() - blink_start;
  uint8_t c = (VoltageDetectionStatus && t < 500) ? (((t / 50) & 1) ? 0 : 1) : 2;
  if (c == shown)
    return;
  shown = c;
  AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, NUM_LEDS, c == 2 ? CRGB(CRGB::Green) : (c == 1 ? CRGB(CRGB::Red) : CRGB(CRGB::Black)));
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
    a. which leg of the figure 8 the car is on (Search_FirstSide): the leg whose direction matches
       the car's heading (within LEG_GATE_DEG) and that is closest to the position estimate. The
       search turns the way the corner at the END of that leg turns. Heading comes from the gyro and
       is accurate; the legs with similar headings are far apart, so this keeps working when the
       position estimate is 0.5-1 m off along the track (the old "within 0.6 m of a corner" check
       didn't, and fell back to "right first" at the 90 deg LEFT corner on the gray-floor track).
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
#define LEG_GATE_DEG        50    // the car's heading must be within this of a leg's direction
#define LEG_GATE_M          1.0f  // ... and the estimate within this of the leg

struct TrackLeg
{
  float x0, y0, x1, y1; // m, start and end of the leg (track frame, see START_X_M)
  int16_t psi;          // deg, direction of travel along the leg
  int8_t next_dir;      // the corner at the end of the leg: +1 = turns left, -1 = right
};
// The figure 8 in driving order (same corners as the track drawing)
static const TrackLeg TRACK_LEGS[] = {
    {0.00f, 0.00f, 3.20f, 0.00f, 0, -1},     // long diagonal, through the crossing -> 136.6 deg right
    {3.20f, 0.00f, 2.10f, -1.04f, -137, -1}, // -> 103.4 deg right
    {2.10f, -1.04f, 1.00f, 0.87f, 120, 1},   // 1.2 m section, stop sign, crossing, 1 m section -> 30 deg left
    {1.00f, 0.87f, 0.63f, 1.08f, 150, 1},    // -> 90 deg left
    {0.63f, 1.08f, 0.00f, 0.00f, -120, 1},   // 1.25 m section -> 120 deg left (start corner)
};
#define TRACK_LEGS_N (sizeof(TRACK_LEGS) / sizeof(TRACK_LEGS[0]))

// Distance from a point to a leg (m)
static float Seg_Dist(const TrackLeg &g, float x, float y)
{
  float dx = g.x1 - g.x0, dy = g.y1 - g.y0;
  float t = ((x - g.x0) * dx + (y - g.y0) * dy) / (dx * dx + dy * dy);
  t = constrain(t, 0.0f, 1.0f);
  return hypot(x - (g.x0 + t * dx), y - (g.y0 + t * dy));
}

// The leg the car is on: heading within gate_deg of the leg's direction, and the closest one to the
// position estimate within gate_m. -1 = none.
static int8_t Match_Leg(float gate_deg, float gate_m)
{
  float psi_deg = Est_Psi * (180.0f / PI);
  int8_t leg = -1;
  for (uint8_t i = 0; i < TRACK_LEGS_N; i++)
  {
    float dpsi = fmod(psi_deg - TRACK_LEGS[i].psi, 360.0f); // wrap to -180..180
    if (dpsi > 180)
      dpsi -= 360;
    if (dpsi < -180)
      dpsi += 360;
    if (fabs(dpsi) > gate_deg)
      continue;
    float d = Seg_Dist(TRACK_LEGS[i], Est_X, Est_Y);
    if (d < gate_m)
    {
      gate_m = d;
      leg = i;
    }
  }
  return leg;
}

static int8_t line_last_side = -1;         // +1 = LEFT sensor saw the line last, -1 = RIGHT
static unsigned long line_last_side_ms = 0; // when

static int8_t search_leg = -1;       // leg matched when the current search started (-1 = none)
static float search_x0, search_y0;   // estimate when it started

static int8_t Search_FirstSide(void)
{
  search_leg = -1;
  search_x0 = Est_X;
  search_y0 = Est_Y;
  // Just after an overtake the line is beside the car, not at a corner: use the side bug_pass() left
  if ((long)(Search_Hint_Until - millis()) > 0)
  {
    Serial.print(F("{SEARCH,"));
    Serial.print(Search_Hint_Side);
    Serial.print(F(",pass}"));
    return Search_Hint_Side;
  }
  int8_t leg = Match_Leg(LEG_GATE_DEG, LEG_GATE_M);
  int8_t side = leg >= 0 ? TRACK_LEGS[leg].next_dir : 0;
  search_leg = leg;
  if (side == 0)
  {
    if (millis() - line_last_side_ms < LAST_SIDE_FRESH_MS)
      side = line_last_side;
    else
      side = -1;
  }
  // {SEARCH,first side (+1 left / -1 right),leg (-1 = none matched)}: shows on the ESP32's debug serial
  Serial.print(F("{SEARCH,"));
  Serial.print(side);
  Serial.print(',');
  Serial.print(leg);
  Serial.print('}');
  return side;
}

/* Corner fix: a line search that started at the end of a mapped leg found the next leg on its first
   sweep (in the direction the map predicted), so the car is at that corner. It lost the line when
   the sensors passed the corner, so its centre was SENSOR_AHEAD_M before the corner on the leg it
   came along; it has only spun on the spot since (plus a little tracking, added back on).
   Logged: {CF,leg,car_x,car_y,fix_dx,fix_dy,used} (mm; car = estimate before the fix)*/
#define CORNER_SIG_M  0.03f // corners are exact; this covers where exactly the sensors lose the tape
#define CORNER_NEAR_M 0.50f // only if the estimate is this close to the corner (not a mid-leg wobble)

static void Corner_Fix(int8_t leg)
{
  const TrackLeg &g = TRACK_LEGS[leg];
  float L = hypot(g.x1 - g.x0, g.y1 - g.y0);
  float ux = (g.x1 - g.x0) / L, uy = (g.y1 - g.y0) / L;
  float zx = g.x1 - SENSOR_AHEAD_M * ux + (Est_X - search_x0); // + motion since the search started
  float zy = g.y1 - SENSOR_AHEAD_M * uy + (Est_Y - search_y0);
  float dx = 0, dy = 0;
  bool ok = hypot(zx - Est_X, zy - Est_Y) < CORNER_NEAR_M &&
            Ekf_PosUpdate(zx, zy, ux, uy, CORNER_SIG_M, CORNER_SIG_M, &dx, &dy);
  Serial.print(F("{CF,"));
  Serial.print(leg);
  Print_mm(Est_X - dx);
  Print_mm(Est_Y - dy);
  Print_mm(dx);
  Print_mm(dy);
  Serial.print(',');
  Serial.print(ok ? 1 : 0);
  Serial.print('}');
}

/* The stop line the car is heading for, from where it is on the track (route order):
     long diagonal before (or just past) the crossing           -> sign B's line (on that diagonal)
     long diagonal after it, top section, or 1.2 m section before the crossing -> sign A's line
     after the crossing on the 1 m section, the 90 deg section, the bottom section -> sign B's line
   "Just past" = up to STOP_PAST_M beyond the crossing, so a slightly late estimate still holds there.*/
#define STOP_PAST_M 0.30f
static int8_t Next_StopLine(void)
{
  int8_t leg = Match_Leg(LEG_GATE_DEG, LEG_GATE_M);
  if (leg < 0)
    return -1;
  if (leg == 0) // long diagonal from (0, 0) along +X, crossing at X = 1.50
    return Est_X < 1.50f + STOP_PAST_M ? 1 : 0;
  if (leg == 2) // from (2.10, -1.04) in direction 120 deg = (-0.5, 0.866), crossing 1.20 m along
    return (-0.5f * (Est_X - 2.10f) + 0.866f * (Est_Y + 1.04f)) < 1.20f + STOP_PAST_M ? 0 : 1;
  return leg == 1 ? 0 : 1;
}

// EKF position uncertainty (m, 1 sigma)
float Ekf_PosSigma(void)
{
  return sqrt(P[0][0] + P[1][1]);
}

// Distance (m) from a point to the nearest track section (used to ignore things off the track)
float Track_Dist(float x, float y)
{
  float best = 1e9f;
  for (uint8_t i = 0; i < TRACK_LEGS_N; i++)
    best = min(best, Seg_Dist(TRACK_LEGS[i], x, y));
  return best;
}

// Direction of the track section the car is on, in GYRO heading degrees (the frame turn_to_heading
// uses): the map direction moved into the gyro's frame with the EKF heading. Returns false if no
// section matches (heading within 30 deg and estimate within 0.3 m).
bool Track_LineHeading(float *gyro_deg)
{
  int8_t leg = Match_Leg(30, 0.3f);
  if (leg < 0)
    return false;
  const TrackLeg &g = TRACK_LEGS[leg];
  float leg_deg = atan2(g.y1 - g.y0, g.x1 - g.x0) * (180.0f / PI);
  *gyro_deg = Heading_Gyro + wrap180f(leg_deg - Est_Psi * (180.0f / PI));
  return true;
}

// Obstacle fix (called by bug_pass): the moose sits in the centre of the bottom section (leg 4), and
// the ultrasonic measured range_m to its near face. Returns false if this isn't the moose (heading or
// gate), e.g. another robot somewhere else.
bool Obstacle_Fix(float range_m, float *dx, float *dy)
{
  *dx = *dy = 0;
  const TrackLeg &g = TRACK_LEGS[4];
  if (fabs(wrap180f(Est_Psi * (180.0f / PI) - g.psi)) > 45)
    return false;
  float L = hypot(g.x1 - g.x0, g.y1 - g.y0);
  float ux = (g.x1 - g.x0) / L, uy = (g.y1 - g.y0) / L;
  float back = range_m + US_AHEAD_M + 0.5f * OBST_LENGTH_M; // car centre to the moose's centre
  return Ekf_PosUpdate(0.5f * (g.x0 + g.x1) - back * ux, 0.5f * (g.y0 + g.y1) - back * uy,
                       ux, uy, 0.04f, 0.03f, dx, dy);
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
  
  RedStop_MustStop();

  // Obstacle ahead -> overtake (movement.cpp). Only armed while really following the line: not
  // during a line search (the sensor sees walls while the car spins at corners), not while stopped
  // for red / a stop sign, and not right after finding the line again.
  bool can_pass = Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode &&
                  Car_LeaveTheGround && track_state == TS_FOLLOW &&
                  on_line && millis() - on_line_since >= 500 &&
                  (red_state == RS_DRIVE || red_state == RS_COOLDOWN || red_state == RS_RED_IGNORE);
  if (Obstacle_Ahead(can_pass))
  {
    bug_pass(Pass_Side());
    timestamp = true; // if the line isn't under the sensors, start a fresh search
    BlindDetection = true;
    on_line = false;
    if (estop_active || light_state != LIGHT_GO)
      return; // E-stop pressed during the pass: don't drive until released and the light check has run
  }

  if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
  {
    if (Car_LeaveTheGround == false) //Check if the car leaves the ground
    {
      track_state = TS_LIFTED;
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }
    track_state = TS_FOLLOW; // changed below if the line is lost

    bool seen_M = function_xxx(TrackingData_M, TrackingDetection_SM, TrackingDetection_E);
    bool seen_R = function_xxx(TrackingData_R, TrackingDetection_SR, TrackingDetection_E);
    bool seen_L = function_xxx(TrackingData_L, TrackingDetection_SL, TrackingDetection_E);
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
        // a corner search just found the next leg on its first sweep: the car is at that corner
        if (!timestamp && BlindDetection && search_phase == 0 && search_leg >= 0)
          Corner_Fix(search_leg);
        timestamp = true;
        BlindDetection = true;
        if (millis() - on_line_since >= GLIMPSE_MS)
          Search_Hint_Until = millis(); // back on the line: the post-pass hint is no longer needed
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
          Serial.print(F("{ok}"));
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
          Serial.print(F("{ok}"));
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
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, NUM_LEDS, CRGB::Black); // (was FastLED.clear)
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*set mode to programming mode<Waiting for the next set of control commands>*/
        if (LightingControl_return == false)
        {

#if _is_print
          Serial.print(F("{ok}"));
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
        AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, NUM_LEDS, CRGB::Black); // (was FastLED.clear)
        Application_SmartRobotCarxxx0.Functional_Mode = CMD_Programming_mode; /*set mode to programming mode<Waiting for the next set of control commands>*/
        if (LightingControl_return == false)
        {

#if _is_print
          Serial.print(F("{ok}"));
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
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, NUM_LEDS, CRGB::Black); // (was FastLED.clear)
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0 /*Duration*/, NUM_LEDS /*Traversal_Number*/, CRGB::Black);
    Application_SmartRobotCarxxx0.Motion_Control = stop_it;
    Application_SmartRobotCarxxx0.Functional_Mode = Standby_mode;
  }
  if (Application_SmartRobotCarxxx0.Functional_Mode == CMD_ClearAllFunctions_Programming_mode) //Command:N110 Clear all functions and enter programming mode
  {

    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    AppRBG_LED.DeviceDriverSet_RBGLED_xxx(0, NUM_LEDS, CRGB::Black); // (was FastLED.clear)
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
      Serial.print(F("{true}"));
#endif
    }
    else
    {
#if _is_print
      Serial.print(F("{false}"));
#endif
    }
  }
  else if (2 == is_get) //ultrasonic sensor is_get data
  {
    char toString[10];
    sprintf(toString, "%d", UltrasoundData_cm);
#if _is_print
    { Serial.print('{'); Serial.print(toString); Serial.print('}'); }
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
    { Serial.print('{'); Serial.print(toString); Serial.print('}'); }
#endif
    /*
    if (true == TrackingDetectionStatus_L)
    {
#if _is_print
      Serial.print(F("{true}"));
#endif
    }
    else
    {
#if _is_print
      Serial.print(F("{false}"));
#endif
    }*/
  }
  else if (1 == is_get) /*Get middle IR sensor status*/
  {
    sprintf(toString, "%d", TrackingData_M);
#if _is_print
    { Serial.print('{'); Serial.print(toString); Serial.print('}'); }
#endif
    /*
    if (true == TrackingDetectionStatus_M)
    {
#if _is_print
      Serial.print(F("{true}"));
#endif
    }
    else
    {
#if _is_print
      Serial.print(F("{false}"));
#endif
    }*/
  }
  else if (2 == is_get) /*Get right IR sensor status*/
  {
    sprintf(toString, "%d", TrackingData_R);
#if _is_print
    { Serial.print('{'); Serial.print(toString); Serial.print('}'); }
#endif
    /*
        if (true == TrackingDetectionStatus_R)
    {
#if _is_print
      Serial.print(F("{true}"));
#endif
    }
    else
    {
#if _is_print
      Serial.print(F("{false}"));
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
    if (estop_active)
    {
      EStop_Pressed = true;     // cancels a pass in progress, even if a release is read straight after
      light_state = LIGHT_LOOK; // armed on the press too, so the next run always starts at the light
      light_look_started = false;
    }
    if (was_active && !estop_active)
    {
      light_state = LIGHT_LOOK; // a new run starts at the traffic light
      light_look_started = false; // 3 s look starts now
    }
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

#if 0 // 1 = echo every camera message the UNO understood, to check the link (off: flash and serial time)
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
    Light_Fix();
  }
  // anything else ({"N":100}, {Factory}, {WA_OK}, ...) is ignored
}
