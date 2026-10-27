// Obstacle detection and the Bug-algorithm overtaking manoeuvre

#include "movement.h"

/*---------------- Overtaking (Bug algorithm) ----------------
  Detection (Obstacle_Ahead): the front ultrasonic must read OBST_MIN_CM..OBST_DETECT_CM on
  OBST_CONFIRM_N pings in a row, and only while the car is really following the line. Before, one
  bad ping started a pass, and the sensor was also read while the car spun in a line search, so it
  saw walls and chairs at the corners ("activates randomly").

  Side (Pass_Side): always the OUTSIDE of the track. Both loops of the figure 8 meet at the crossing
  (X = CROSSING_X): the right-turn loop is at X > CROSSING_X, so its outside is on the LEFT; the
  left-turn loop is at X < CROSSING_X, so its outside is on the RIGHT. The point checked is a little
  ahead of the car (where the obstacle is). Before, it always passed on the right.

  Manoeuvre (bug_pass), each step closed-loop on the gyro and the position estimate:
    1. turn PASS_OUT_DEG toward the pass side
    2. drive out until PASS_OFFSET_M beside the line: half the obstacle + half the car + clearance
    3. turn back parallel to the line, point the ultrasonic at the obstacle
    4. drive beside it until the car's rear is past the obstacle's far end (worked out from where the
       obstacle was measured and OBST_LENGTH_M) - the Bug "leave" point. If the side sensor still sees
       something there (a robot moving the same way), keep going while it does, up to PASS_EXTRA_MAX_M.
    5. turn PASS_OUT_DEG back toward the line
    6. drive until an IR sensor sees the line (gives up after PASS_RETURN_MAX_M)
    7. drive on SENSOR_AHEAD_M so the wheels are on the line, then turn parallel to it and hand back
       to the line tracker (with a search hint toward the line in case it isn't under the sensors)
  Before, the manoeuvre was timed, the side check never ran (first_time was already > 500 ms), and
  the way back was shorter than the way out, so it ended ~0.2 m beside the line and lost it.

  Events sent to the ESP32 (shown as "UNO: ..." on its debug serial):
    {OB,obstacle_x_mm,obstacle_y_mm,side,fix_dx,fix_dy,used}  pass started (side 0 = left, 1 = right;
                                           position before the moose fix; used = moose fix applied)
    {OV,time_ms,side,line_found}           pass finished
*/
#define OBST_DETECT_CM      20    // start a pass when the obstacle is this close ahead
#define OBST_MIN_CM         3     // readings below this are noise
#define OBST_CONFIRM_N      3     // close readings in a row needed to start a pass
#define OBST_PING_MS        60    // HC-SR04 needs ~60 ms between pings or it hears the last echo
#define OBST_AFTER_PASS_MS  3000  // no new pass this soon after the last one
#define OBST_OFF_TRACK_M    0.20f // ignore things farther than this from the tape (walls, the traffic light,
                                  // people past a corner - the light was taken for an obstacle on lap 3)

#define CROSSING_X          1.50f // m, X of the crossing in the track frame (see START_X_M / TRACK_LEGS)
#define SIDE_LOOK_AHEAD_M   0.30f // decide the side at this point ahead of the car (at the obstacle)

#define PASS_SPEED          100
#define PASS_OUT_DEG        45    // angle out of and back into the lane
// Size of the pass. Measure your obstacle and car and set these; everything below follows from them.
#define OBST_WIDTH_M        0.10f // obstacle size ACROSS the track
#define CAR_HALF_WIDTH_M    0.10f // half the car's width
#define CAR_HALF_LEN_M      0.13f // car centre to its back bumper
#define PASS_CLEARANCE_M    0.04f // gap kept between the car and the obstacle
// Car centre this far beside the line while passing: just enough for the car to miss the obstacle
#define PASS_OFFSET_M       (OBST_WIDTH_M / 2 + CAR_HALF_WIDTH_M + PASS_CLEARANCE_M)
#define PASS_OUT_MAX_M      (PASS_OFFSET_M * 1.41f + 0.10f) // give up driving out after this far (45 deg)
#define PASS_RETURN_MAX_M   (PASS_OFFSET_M * 1.41f + 0.15f) // drive at most this far looking for the line
#define PASS_SIDE_SEE_CM    25    // side ultrasonic: obstacle is still beside the car if closer than this
#define PASS_EXTRA_MAX_M    0.40f // keep going at most this much further while it is (moving robot)
#define HOLD_GAIN           3     // PWM per degree of heading error while driving beside the obstacle
#define SERVO_LEFT          180   // servo angles (Elegoo: 30 = right, 150 = left)
#define SERVO_RIGHT         0
#define SERVO_AHEAD         90

#define TURN_SPEED          100
#define TURN_LEAD_DEG       12    // stop turning this early: the car keeps turning ~10-15 deg after the
                                  // motors stop. If turns overshoot raise it, if they stop short lower it.
#define TURN_TIMEOUT_MS     2500
#define TURN_TOL_DEG        5     // after the turn, trim with short pulses until within this
#define TURN_PULSE_MS       40    // one trim pulse (~5 deg at PWM 100)

bool Overtaking = false;
int8_t Search_Hint_Side = 0;
unsigned long Search_Hint_Until = 0;
static unsigned long last_pass_end = 0;

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
    // keeps Heading_Gyro fresh and sends telemetry (state OVERTAKING) during the pass
    Application_FunctionSet.ApplicationFunctionSet_Telemetry(false);
  }
  return true;
}

extern Servo myservo; // DeviceDriverSet_xxx0.cpp

// E-stop pressed during bug_pass(): give up the pass (the car stays stopped)
static void abort_pass(void)
{
  Overtaking = false;
  last_pass_end = millis();
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  Serial.print(F("{PASS ABORTED}"));

  // Point the servo (camera + ultrasonic) back to the middle. DeviceDriverSet_Servo_control()
  // can't be used here: its 450 ms wait (delay_kinematics) ends at once while the E-stop is on.
  // The ESP32 isn't read during this wait, so a quick E-stop release can't start the car mid-move.
  myservo.attach(PIN_Servo_z);
  myservo.write(SERVO_AHEAD);
  unsigned long t0 = millis();
  while (millis() - t0 < 450)
  {
    wdt_reset();
    Kinematics_Update();
    Application_FunctionSet.ApplicationFunctionSet_Telemetry(true); // web page keeps showing ESTOP
  }
  myservo.detach();
}

static bool checkLine(void)
{
  uint32_t TrackingData_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
  uint32_t TrackingData_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
  uint32_t TrackingData_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();

  bool seen_M = function_xxx(TrackingData_M, Application_FunctionSet.TrackingDetection_SM, Application_FunctionSet.TrackingDetection_E);
  bool seen_R = function_xxx(TrackingData_R, Application_FunctionSet.TrackingDetection_SR, Application_FunctionSet.TrackingDetection_E);
  bool seen_L = function_xxx(TrackingData_L, Application_FunctionSet.TrackingDetection_SL, Application_FunctionSet.TrackingDetection_E);

  return seen_M || seen_R || seen_L;
}

static float wrap180(float deg)
{
  while (deg > 180)
    deg -= 360;
  while (deg < -180)
    deg += 360;
  return deg;
}

// One ultrasonic reading (cm), 0 = nothing in range. Rate-limited to one ping per OBST_PING_MS.
static uint16_t ping_cm(void)
{
  static unsigned long last = 0;
  static uint16_t last_cm = 0;
  if (millis() - last < OBST_PING_MS)
    return last_cm;
  last = millis();
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&last_cm /*out*/);
  return last_cm;
}

// Smoothed gyro heading while following the line (the tracker zig-zags +-10-15 deg around the line),
// used as the line's direction when the car isn't on a mapped section
static float line_dir = 0;
static bool line_dir_ok = false;

bool Obstacle_Ahead(bool armed)
{
  static uint8_t close_count = 0;
  static unsigned long last_checked = 0;
  static unsigned long last_avg = 0;
  if (!armed)
    line_dir_ok = false;
  else if (!line_dir_ok)
  {
    line_dir = Heading_Gyro;
    line_dir_ok = true;
    last_avg = millis();
  }
  else if (millis() - last_avg >= 20)
  {
    last_avg = millis();
    line_dir += 0.04f * wrap180(Heading_Gyro - line_dir); // ~0.5 s average
  }
  if (!armed || millis() - last_pass_end < OBST_AFTER_PASS_MS)
  {
    close_count = 0;
    return false;
  }
  if (millis() - last_checked < OBST_PING_MS)
    return false;
  last_checked = millis();
  uint16_t d = ping_cm();
  if (d >= OBST_MIN_CM && d <= OBST_DETECT_CM)
    close_count++;
  else
    close_count = 0;
  if (close_count < OBST_CONFIRM_N)
    return false;
  // Is it on the track? (e.g. approaching a corner, the sensor looks straight off the track)
  float r = d * 0.01f + US_AHEAD_M;
  float ox = Est_X + r * cos(Est_Psi), oy = Est_Y + r * sin(Est_Psi);
  // (tolerance grows with the position uncertainty, so a drifted estimate can't hide a real obstacle)
  float off = Track_Dist(ox, oy);
  if (off > OBST_OFF_TRACK_M + 2 * Ekf_PosSigma())
  {
    close_count = 0;
    Serial.print(F("{OBST OFF TRACK,"));
    Serial.print((long)(off * 1000)); // mm from the tape
    Serial.print('}');
    return false;
  }
  return true;
}

side Pass_Side(void)
{
  float x_ahead = Est_X + SIDE_LOOK_AHEAD_M * cos(Est_Psi);
  return (x_ahead > CROSSING_X) ? LHS : RHS; // right-turn loop: pass on the left; left-turn loop: on the right
}

// Wait until the car has stopped turning (gyro heading steady over 30 ms), at most 600 ms.
// (A fixed 150 ms wasn't enough: the car was still turning at ~1/3 speed, so the trim thought it
//  was done and the car then coasted another 5-10 deg.) Returns false if the E-stop was pressed.
static bool settle(void)
{
  unsigned long t0 = millis();
  float last = Heading_Gyro;
  do
  {
    if (!delay_kinematics(30))
      return false;
    float d = fabs(wrap180(Heading_Gyro - last));
    last = Heading_Gyro;
    if (d < 0.3f && millis() - t0 >= 100)
      break;
  } while (millis() - t0 < 600);
  return true;
}

// Turn on the spot to target_deg (gyro heading, + = left): turn until TURN_LEAD_DEG short (the car
// coasts on), let it settle, then trim with short pulses until within TURN_TOL_DEG (overshoot varies
// with the battery and the floor). Returns false if the E-stop was pressed.
static bool turn_to_heading(float target_deg)
{
  unsigned long t0 = millis();
  uint8_t pulses = 0; // 0 = main turn; then 1.. = trim pulses done + 1
  for (;;)
  {
    float err = wrap180(target_deg - Heading_Gyro);
    bool turn = pulses == 0 ? fabs(err) > TURN_LEAD_DEG : (fabs(err) > TURN_TOL_DEG && pulses <= 4);
    if (millis() - t0 > TURN_TIMEOUT_MS)
      turn = false;
    if (!turn && pulses > 0)
      break; // within TURN_TOL_DEG (or gave up), already stopped and settled
    ApplicationFunctionSet_SmartRobotCarMotionControl(turn ? (err > 0 ? Left : Right) : stop_it, TURN_SPEED);
    if (turn && pulses == 0) // main turn: keep going
    {
      if (!delay_kinematics(10))
        return false;
      continue;
    }
    if (turn && !delay_kinematics(TURN_PULSE_MS)) // one trim pulse
      return false;
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    if (!settle()) // the car coasts on for a few hundred ms: measure only once it has stopped turning
      return false;
    pulses++;
  }
  return true;
}

// Drive forward holding heading_deg (A = right side, B = left side). Call in a loop.
static void drive_holding(float heading_deg)
{
  int16_t corr = (int16_t)(HOLD_GAIN * wrap180(heading_deg - Heading_Gyro)); // + = steer left
  corr = constrain(corr, -40, 40);
  AppMotor.DeviceDriverSet_Motor_control(direction_just, PASS_SPEED + corr,
                                         direction_just, PASS_SPEED - corr, control_enable);
}

static float dist_from(float x0, float y0)
{
  return hypot(Est_X - x0, Est_Y - y0);
}

static void look(uint8_t angle)
{
  AppServo.DeviceDriverSet_Servo_control(angle); // 450 ms, through delay_kinematics
}

void bug_pass(side s)
{
  Overtaking = true;
  EStop_Pressed = false; // only presses from now on cancel this pass
  unsigned long t_start = millis();
  const int8_t sgn = (s == LHS) ? 1 : -1; // +1 = pass on the left (heading + = left)

  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  uint16_t d_cm = ping_cm();
  if (!delay_kinematics(150))
    return abort_pass();
  const float d_m = (d_cm >= OBST_MIN_CM && d_cm <= 60) ? d_cm * 0.01f : OBST_DETECT_CM * 0.01f;

  // Where is the obstacle's centre? (estimate BEFORE the fix, for the log / report)
  float back = d_m + US_AHEAD_M + 0.5f * OBST_LENGTH_M;
  float ob_x = Est_X + back * cos(Est_Psi);
  float ob_y = Est_Y + back * sin(Est_Psi);
  // If it is the moose (centre of the bottom section), it fixes the car's position
  float fx, fy;
  bool fixed = Obstacle_Fix(d_m, &fx, &fy);
  // {OB,obstacle_x,obstacle_y,side,fix_dx,fix_dy,used} (mm)
  Serial.print(F("{OB,"));
  Serial.print((long)(ob_x * 1000));
  Serial.print(',');
  Serial.print((long)(ob_y * 1000));
  Serial.print(',');
  Serial.print((int)s);
  Serial.print(',');
  Serial.print((long)(fx * 1000));
  Serial.print(',');
  Serial.print((long)(fy * 1000));
  Serial.print(',');
  Serial.print(fixed ? 1 : 0);
  Serial.print('}');

  // Direction of the line here. NOT the car's heading at this moment: line following zig-zags
  // +-10-15 deg around the line, which made one pass leave at ~60 deg instead of 45. The mapped
  // section's direction is used (in the gyro's frame), or the averaged heading off the map.
  float yaw0 = Heading_Gyro;
  if (!Track_LineHeading(&yaw0) && line_dir_ok)
    yaw0 = line_dir;
  const float x0 = Est_X, y0 = Est_Y;
  const float c0 = cos(yaw0 * (PI / 180.0f)), s0 = sin(yaw0 * (PI / 180.0f));
  #define LATERAL() (-(Est_X - x0) * s0 + (Est_Y - y0) * c0) // m beside the line, + = left
  #define ALONG() ((Est_X - x0) * c0 + (Est_Y - y0) * s0)    // m along the line from where it stopped
  // The car centre is level with the obstacle's far end + car half-length + clearance -> rear is clear
  const float pass_along = US_AHEAD_M + d_m + OBST_LENGTH_M + CAR_HALF_LEN_M + PASS_CLEARANCE_M;

  // 1. turn out toward the pass side
  if (!turn_to_heading(yaw0 + sgn * PASS_OUT_DEG))
    return abort_pass();

  // 2. drive out until PASS_OFFSET_M beside the line
  float px = Est_X, py = Est_Y;
  while (sgn * LATERAL() < PASS_OFFSET_M && dist_from(px, py) < PASS_OUT_MAX_M)
  {
    drive_holding(yaw0 + sgn * PASS_OUT_DEG);
    if (!delay_kinematics(10))
      return abort_pass();
  }

  // 3. parallel to the line, ultrasonic looking at the obstacle (on the side away from the pass)
  if (!turn_to_heading(yaw0))
    return abort_pass();
  look(s == LHS ? SERVO_RIGHT : SERVO_LEFT);
  if (EStop_Pressed)
    return abort_pass();

  // 4. drive beside the obstacle until the car is past it (Bug "leave" point). The distance comes
  //    from where the obstacle was measured and its size, so a small obstacle gives a short pass.
  //    If the side sensor still sees something there (another robot moving the same way), keep
  //    going while it does, up to PASS_EXTRA_MAX_M more.
  unsigned long last_ping = 0;
  bool beside = false;
  while (ALONG() < pass_along || (beside && ALONG() < pass_along + PASS_EXTRA_MAX_M))
  {
    drive_holding(yaw0);
    if (millis() - last_ping >= OBST_PING_MS)
    {
      last_ping = millis();
      uint16_t d = ping_cm();
      beside = d >= OBST_MIN_CM && d <= PASS_SIDE_SEE_CM;
    }
    if (!delay_kinematics(10))
      return abort_pass();
  }
  ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
  look(SERVO_AHEAD);
  if (EStop_Pressed)
    return abort_pass();

  // 5. turn back toward the line
  if (!turn_to_heading(yaw0 - sgn * PASS_OUT_DEG))
    return abort_pass();

  // 6. drive until a line sensor sees the tape
  px = Est_X;
  py = Est_Y;
  bool found = false;
  while (dist_from(px, py) < PASS_RETURN_MAX_M)
  {
    if (checkLine())
    {
      found = true;
      break;
    }
    drive_holding(yaw0 - sgn * PASS_OUT_DEG);
    if (!delay_kinematics(5))
      return abort_pass();
  }

  // 7. wheels onto the line, then line up with it
  if (found)
  {
    px = Est_X;
    py = Est_Y;
    while (dist_from(px, py) < SENSOR_AHEAD_M)
    {
      drive_holding(yaw0 - sgn * PASS_OUT_DEG);
      if (!delay_kinematics(5))
        return abort_pass();
    }
  }
  if (!turn_to_heading(yaw0))
    return abort_pass();

  // Line NOT found on the way back: for the next 3 s the tracker's line search turns toward the side
  // the line should be on first (still on the pass side of it -> back toward it; already past it -> the
  // other way). Not when it was found: then the next search is at a corner and must use the map
  // (with the hint, the corner right after the moose was searched the wrong way first).
  if (!found)
  {
    Search_Hint_Side = (sgn * LATERAL() > 0) ? -sgn : sgn;
    Search_Hint_Until = millis() + 3000;
  }
  #undef LATERAL
  #undef ALONG

  Overtaking = false;
  last_pass_end = millis();
  Serial.print(F("{OV,"));
  Serial.print(millis() - t_start); // how long the pass took, ms
  Serial.print(',');
  Serial.print((int)s);
  Serial.print(',');
  Serial.print(found ? 1 : 0);
  Serial.print('}');
}
