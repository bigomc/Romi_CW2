#include "imu.h"
#include "pins.h"
#include <Wire.h>
#include <Arduino.h>

void Imu::init()
{
    if (!imu.init())
    {
        Serial.println("Failed to detect and initialize magnetometer!");
        while (1);
    }

    imu.enableDefault();
    imu.writeReg(LSM6::CTRL1_XL,0b01100000); //Setting accelerometer ODR to 416Hz
    imu.writeReg(LSM6::CTRL2_G,0b01100000); //Setting gyro ODR to 416Hz
}

void Imu::readRaw()
{

  imu.read();

  gx = imu.g.x;
  gy = imu.g.y;
  gz = imu.g.z;

  ax = imu.a.x;
  ay = imu.a.y;
  az = imu.a.z;

}

void Imu::readCalibrated()
{

  imu.read();
//Conversion factor - Readings in dps units
  gx = g_sensitivity * (imu.g.x - gx_offset)*0.001;
  gy = g_sensitivity * (imu.g.y - gy_offset)*0.001;
  gz = g_sensitivity * (imu.g.z - gz_offset)*0.001;
  //Conversion factor - Readings in g units
  ax = a_sensitivity * imu.a.x*0.001;
  ay = a_sensitivity * imu.a.y*0.001;
  az = a_sensitivity * imu.a.z*0.001;

}

void Imu::calibrate()
{
  analogWrite(BUZZER_PIN, 10);
  delay(500);
  digitalWrite( BUZZER_PIN, LOW );

  for (int i=0;i<NUM_CALIBRATIONS_IMU;i++)
  {
    imu.read();

    gx_offset += ((float)imu.g.x / NUM_CALIBRATIONS_IMU);
    gy_offset += ((float)imu.g.y / NUM_CALIBRATIONS_IMU);
    gz_offset += ((float)imu.g.z / NUM_CALIBRATIONS_IMU);
    analogWrite(BUZZER_PIN, 0);
    delay(50);
  }

  analogWrite(BUZZER_PIN, 10);
  delay(500);
  digitalWrite( BUZZER_PIN, LOW );
  delay(500);
  analogWrite(BUZZER_PIN, 10);
  delay(500);
  digitalWrite( BUZZER_PIN, LOW );
  delay(500);


}
