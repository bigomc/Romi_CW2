#include "magnetometer.h"
#include "pins.h"
#include <Arduino.h>
#include "utils.h"
#include <Wire.h>

void Magnetometer::init()
{
    if (!mag.init())
    {
        Serial.println("Failed to detect and initialize magnetometer!");
        while (1);
    }

    mag.enableDefault();
    mag.writeReg(LIS3MDL::CTRL_REG1,0b01010010); //Setting high performance mode with 300Hz ODR axes X and Y
    mag.writeReg(LIS3MDL::CTRL_REG4,0b00001000);  //Setting high performance mode with 300Hz ODR axis Z

}

void Magnetometer::readRaw()
{

  mag.read();

  x = mag.m.x;
  y = mag.m.y;
  z = mag.m.z;

}

void Magnetometer::readCalibrated()
{

  mag.read();

  x = sensitivity * (mag.m.x - x_offset) * x_scale;
  y = sensitivity * (mag.m.y - y_offset) * y_scale;
  z = sensitivity * (mag.m.z - z_offset) * z_scale;

}

void Magnetometer::calibrate()
{
    analogWrite(BUZZER_PIN, 10);
    delay(100);analogWrite(BUZZER_PIN, 0);
    delay(100);
    analogWrite(BUZZER_PIN, 10);
    delay(100);analogWrite(BUZZER_PIN, 0);
    delay(100);
    analogWrite(BUZZER_PIN, 10);
    delay(100);analogWrite(BUZZER_PIN, 0);
    delay(100);

  for (int i=0;i<NUM_CALIBRATIONS_MAG;i++)
  {

    mag.read();

    x_max = max(x_max, mag.m.x);
    y_max = max(y_max, mag.m.y);
    z_max = max(z_max, mag.m.z);

    x_min = min(x_min, mag.m.x);
    y_min = min(y_min, mag.m.y);
    z_min = min(z_min, mag.m.z);

    delay(50);
  }

  calculateOffsets();

  analogWrite(BUZZER_PIN, 10);
  delay(500);
  digitalWrite( BUZZER_PIN, LOW );
  delay(500);
  analogWrite(BUZZER_PIN, 10);
  delay(500);
  digitalWrite( BUZZER_PIN, LOW );
  delay(500);

}

void Magnetometer::calculateOffsets()
{

  x_offset = (x_max + x_min) / 2.0;
  y_offset = (y_max + y_min) / 2.0;
  z_offset = (z_max + z_min) / 2.0;

  x_scale = (x_max - x_min) / 2.0;
  y_scale = (y_max - y_min) / 2.0;
  z_scale = (z_max - z_min) / 2.0;

  float avg_scale = (x_scale + y_scale + z_scale) / 3.0;

  x_scale = avg_scale / x_scale;
  y_scale = avg_scale / y_scale;
  z_scale = avg_scale / z_scale;

  //Serial.print("X: ");
  //Serial.print(x_offset);
  //Serial.print("Y: ");
  //Serial.print(y_offset);
  //Serial.print("Z: ");
  //Serial.println(z_offset);

  //Serial.print("X: ");
  //Serial.print(x_scale);
  //Serial.print("Y: ");
  //Serial.print(y_scale);
  //Serial.print("Z: ");
  //Serial.println(z_scale);

}

void Magnetometer::set_zero()
{
  readCalibrated();
  heading_mag_zero = atan2(y,x);
}

float Magnetometer::getHeading()
{
  readCalibrated();
  heading = atan2(y,x);
  heading = heading - heading_mag_zero; //Relative to start position

  //Adjusting angle value
  if(heading < -PI){
    heading = 2*PI + heading;
  }
  if(heading > PI){
    heading = heading - 2*PI;
  }

  return rad2deg(heading);
}
