#ifndef MOVEMENT
#define MOVEMENT

#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"
#include <avr/wdt.h>


typedef enum side{
  LHS,
  RHS
}side;


extern DeviceDriverSet_ULTRASONIC AppULTRASONIC;
extern DeviceDriverSet_Servo AppServo;
extern DeviceDriverSet_ITR20001 AppITR20001;
extern DeviceDriverSet_Motor AppMotor;

bool delay_kinematics(unsigned long ms); // false if the E-stop was pressed during the wait

// Car / obstacle dimensions shared with the position estimate (MEASURE them)
#define SENSOR_AHEAD_M      0.10f // line sensors are this far ahead of the wheel axle (car centre)
#define US_AHEAD_M          0.10f // front ultrasonic is this far ahead of the car centre
#define OBST_LENGTH_M       0.10f // obstacle (moose) size ALONG the track

// Position fix from the moose (ApplicationFunctionSet_xxx0.cpp); false = not the moose / rejected
bool Obstacle_Fix(float range_m, float *dx, float *dy);
float Track_Dist(float x, float y);       // m from the point to the nearest track section
float Ekf_PosSigma(void);                 // m, position estimate uncertainty (1 sigma)
bool Track_LineHeading(float *gyro_deg);  // direction of the section the car is on (gyro frame), false = unknown

// Obstacle ahead, confirmed by several pings in a row. Call every loop; armed = false (not following
// the line, stopped, searching...) clears the count so the car can't start a pass then.
bool Obstacle_Ahead(bool armed);
side Pass_Side(void);   // outside of the track at the obstacle (LHS on the right-turn loop, RHS on the left-turn loop)
void bug_pass(side s);  // the overtaking manoeuvre

extern bool Overtaking; // true while bug_pass() is passing an obstacle (shown as OVERTAKING on the web page)

// After a pass the line is on a known side: the line search turns that way first until Search_Hint_Until
extern int8_t Search_Hint_Side;          // +1 = left, -1 = right
extern unsigned long Search_Hint_Until;  // millis()

#endif
