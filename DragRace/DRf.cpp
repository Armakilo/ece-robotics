#include "DRf.h"




// frame is 160 width x 120 height
// ignore height

// guiding objects will be red and be on the RHS of the car

// 20 px buffer, ignore the rightmost edges

uint8_t turn_count;
uint32_t turn_start = 0;
#define L_TURN_COOLDOWN 500
#define R_TURN_COOLDOWN 1500 //R. turn func will overcorrect to get the robot back on the lhs


//protip: don't forget the upload/cam switch

void DragRace(void)
{

  
  uint16_t u_dist;
  AppULTRASONIC.DeviceDriverSet_ULTRASONIC_Get(&u_dist /*out*/); //saftey mechanism/ Estop
  if((u_dist != 0) && (u_dist < 15)){
    ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    delay_kinematics(1000);
    return;
  }

  ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, 600);
  
  //if(obj_now.sizePX < 50) return;
  delay_kinematics(100);
  if(obj_now.colour == 'R'){
    int centroid = obj_now.cent_x;
    if((centroid > 100) && (millis()-turn_start > L_TURN_COOLDOWN) && (turn_count < 3)){ //s-shape around obstacles(kinda) LHS
      int turn_start = millis();
      ApplicationFunctionSet_SmartRobotCarMotionControl(LeftForward, 100);
      delay_kinematics(1000);
      ApplicationFunctionSet_SmartRobotCarMotionControl(RightForward, 100);
      delay_kinematics(1000);
      //turn_to_heading(yawinit); // fix l8r
      turn_count++;
      //if (fmod(turn_count, 2) == 0) AppMPU6050getdata.MPU6050_calibration();
      
      turn_count++;
    }
    else if((centroid < 60 && centroid > 20) && (millis()-turn_start > R_TURN_COOLDOWN) && (turn_count < 3)){ //RHS
      ApplicationFunctionSet_SmartRobotCarMotionControl(RightForward, 100);
      delay_kinematics(2000);
      ApplicationFunctionSet_SmartRobotCarMotionControl(LeftForward, 100);
      delay_kinematics(4000); //gets things back on track

      turn_to_heading(yawinit); //could try to implement later
      turn_count++;
      //if (fmod(turn_count, 2) == 0) AppMPU6050getdata.MPU6050_calibration();

      
    }

    if(turn_count >= 3){
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 60);
      delay_kinematics(3000);
      while(1);//blocks till wdt
    }
  }
  
}