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
