#ifndef MOVEMENT
#define MOVEMENT

#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"
#include <avr/wdt.h>


typedef enum side{
  LHS,
  RHS
}side;


// extern uint8_t TrackingDetection_S;
// extern uint8_t TrackingDetection_E;

extern DeviceDriverSet_ULTRASONIC AppULTRASONIC;
extern DeviceDriverSet_Servo AppServo;
extern DeviceDriverSet_ITR20001 AppITR20001;

bool delay_kinematics(unsigned long ms); // false if the E-stop was pressed during the wait


void bug_pass(uint8_t side);

extern bool Overtaking; // true while bug_pass() is passing an obstacle (shown as OVERTAKING on the web page)



#endif