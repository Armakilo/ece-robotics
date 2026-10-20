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

void delay_kinematics(unsigned long ms);


void bug_pass(uint8_t side);



#endif