#include "imu.h"
#include "pins.h"
#include <Wire.h>
#include <Arduino.h>

void Imu::init()
{
    if (!imu.init())
    {
        Serial.println("Failed to detect and initialize accelerometer!");
        while (1);
    }
    imu.enableDefault();
	//imu.writeReg(LSM6::CTRL1_XL, 0b01011000); // 208 Hz, +/4 g
  //imu.writeReg(LSM6::CTRL1_XL,0b01100000); //Setting accelerometer ODR to 416Hz
  //imu.writeReg(LSM6::CTRL2_G,0b01100000); //Setting gyro ODR to 416Hz
	imu.writeReg(imu.CTRL1_XL, 0x40);
	imu.writeReg(imu.CTRL2_G, 0x40);
	imu.writeReg(imu.FIFO_CTRL5, 0x26);
	imu.writeReg(imu.FIFO_CTRL1, 0x01);
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
  gx = g_sensitivity * (imu.g.x - gx_offset);
  gy = g_sensitivity * (imu.g.y - gy_offset);
  gz = g_sensitivity * (imu.g.z - gz_offset);
 //Conversion factor - Readings in g units
  ax = a_sensitivity * imu.a.x;
  ay = a_sensitivity * imu.a.y;
  az = a_sensitivity * imu.a.z;

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


void Imu::getFiltered() {

	readCalibrated();

	//ax = (alpha*ax) + ((1 - alpha)*last_ax);
	//ay = (alpha*ay) + ((1 - alpha)*last_ay);
	//az = (alpha*az) + ((1 - alpha)*last_az);
	//gx = (alpha*gx) + ((1 - alpha)*last_gx);
	//gy = (alpha*gy) + ((1 - alpha)*last_gy);
	gz = (alpha*gz) + ((1 - alpha)*last_gz);

	//last_ax = ax;
	//last_ay = ay;
	//last_az = az;
	//last_gx = gx;
	//last_gy = gy;
	last_gz = gz;

  return gz;

}
