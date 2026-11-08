/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-30 10:34:30
 * @LastEditors: Changhua
 * @Description: MPU6050 Data solution
 * @FilePath: 
 */

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "MPU6050_getdata.h"
#include <stdio.h>
#include <math.h>

MPU6050 accelgyro;
MPU6050_getdata MPU6050Getdata;

// static void MsTimer2_MPU6050getdata(void)
// {
//   sei();
//   int16_t ax, ay, az, gx, gy, gz;
//   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //Read the raw values of the six axes
//   float gyroz = -(gz - MPU6050Getdata.gzo) / 131 * 0.005f;
//   MPU6050Getdata.yaw += gyroz;
// }

bool MPU6050_getdata::MPU6050_dveInit(void)
{
  Wire.begin();
  uint8_t chip_id = 0x00;
  uint8_t cout = 0; // FIX: was never initialised, so the retry limit was random
  do
  {
    chip_id = accelgyro.getDeviceID();
    Serial.print("MPU6050_chip_id: ");
    Serial.println(chip_id);
    delay(10);
    cout += 1;
    if (cout > 10)
    {
      return true;
    }
  } while (chip_id == 0X00 || chip_id == 0XFF); //Ensure that the slave device is online（Wait forcibly to get the ID）
  accelgyro.initialize();
  // unsigned short times = 100; //Sampling times
  // for (int i = 0; i < times; i++)
  // {
  //   gz = accelgyro.getRotationZ();
  //   gzo += gz;
  // }
  // gzo /= times; //Calculate gyroscope offset
  return false;
}
bool MPU6050_getdata::MPU6050_calibration(void)
{
  unsigned short times = 100; //Sampling times
  float sum = 0;                // FIX: start from 0 (it used to add onto the old offset)
  for (int i = 0; i < times; i++)
  {
    gz = accelgyro.getRotationZ();
    sum += gz;
    delay(2);
  }
  gzo = sum / times; //Calculate gyroscope offset (float, keeps the fraction)
  lastTime = millis(); // so the first integration step doesn't use the time since power-on

  // gzo = accelgyro.getRotationZ();
  return false;
}
/* Zero-velocity update: call instead of MPU6050_dveGetEulerAngles while the motors are off and
   the car has had time to stop. Whatever the gyro reads now is offset (the car isn't turning),
   so the offset estimate is pulled toward it. Time constant ~0.5 s when called every 10 ms.
   Readings above ~15 deg/s are ignored, so turning the car by hand doesn't spoil the offset. */
void MPU6050_getdata::MPU6050_dveStill(void)
{
  lastTime = millis(); // so the next integration step doesn't include the still time
  gz = accelgyro.getRotationZ();
  if (fabs(gz - gzo) < 15.0 * 131.0)
    gzo += 0.02 * (gz - gzo);
}

bool MPU6050_getdata::MPU6050_dveGetEulerAngles(float *Yaw)
{
  unsigned long now = millis();           //Record the current time(ms)
  dt = (now - lastTime) / 1000.0;         //Caculate the derivative time(s)
  lastTime = now;                         //Record the last sampling time(ms)
  gz = accelgyro.getRotationZ();          //Read the raw values of the six axes
  float rate = -(gz - gzo) / 131.0;       //z-axis angular velocity (deg/s)
  // FIX: the old code zeroed any STEP smaller than 0.05 deg. Called every few ms, that
  // threw away every turn slower than ~10-25 deg/s (the figure-8 curves). Now the
  // dead-zone is on the RATE, so only sensor drift below 0.5 deg/s is ignored.
  if (fabs(rate) < 0.5)
  {
    rate = 0.0;
  }
  agz += rate * dt; //z-axis angular velocity integral
  *Yaw = agz;
  return false;
}
