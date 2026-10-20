#ifndef MOVEMENT
#define MOVEMENT

#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

typedef enum side{
  LHS,
  RHS
}side;


extern uint8_t TrackingDetection_S;
extern uint8_t TrackingDetection_E;

extern DeviceDriverSet_ULTRASONIC AppULTRASONIC;
extern DeviceDriverSet_Servo AppServo;
extern DeviceDriverSet_ITR20001 AppITR20001;


void bug_pass(uint8_t side);



#endif