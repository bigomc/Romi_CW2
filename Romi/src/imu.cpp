#include "imu.h"
#include "pins.h"

void Imu::init()
{
    if (!imu.init())
    {
        Serial.println("Failed to detect and initialize magnetometer!");
        while (1);
    }

    imu.enableDefault();
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

  gx = g_sensitivity * (imu.g.x - gx_offset);
  gy = g_sensitivity * (imu.g.y - gy_offset);
  gz = g_sensitivity * (imu.g.z - gz_offset);

  ax = a_sensitivity * imu.a.x;
  ay = a_sensitivity * imu.a.y;
  az = a_sensitivity * imu.a.z;




}

void Imu::calibrate()
{

  for (int i=0;i<NUM_CALIBRATIONS_IMU;i++)
  {
    //analogWrite(BUZZER_PIN, 10);
    delay(50);

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