#include "magnetometer.h"
#include "pins.h"
#include <Arduino.h>

void Magnetometer::init()
{
    if (!mag.init())
    {
        Serial.println("Failed to detect and initialize magnetometer!");
        while (1);
    }

    mag.enableDefault();
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

    orientation = orientation_offset - atan2(y,x);
}

void Magnetometer::calibrate()
{
    analogWrite(BUZZER_PIN, 10);
    delay(50);analogWrite(BUZZER_PIN, 0);
    delay(50);
    analogWrite(BUZZER_PIN, 10);
    delay(50);analogWrite(BUZZER_PIN, 0);
    delay(50);
    analogWrite(BUZZER_PIN, 10);
    delay(50);analogWrite(BUZZER_PIN, 0);
    delay(50);
  for (int i=0;i<NUM_CALIBRATIONS_MAG;i++)
  {
        analogWrite(BUZZER_PIN, 10);
        delay(30);

        mag.read();

        x_max = max(x_max, mag.m.x);
        y_max = max(y_max, mag.m.y);
        z_max = max(z_max, mag.m.z);

        x_min = min(x_min, mag.m.x);
        y_min = min(y_min, mag.m.y);
        z_min = min(z_min, mag.m.z);

        analogWrite(BUZZER_PIN, 0);

        delay(50);
  }

  calculateOffsets();
}

void Magnetometer::calculateOffsets()
{

  x_offset = (x_max + x_min) / 2;
  y_offset = (y_max + y_min) / 2;
  z_offset = (z_max + z_min) / 2;

  x_scale = (x_max - x_min) / 2;
  y_scale = (y_max - y_min) / 2;
  z_scale = (z_max - z_min) / 2;

  float avg_scale = (x_scale + y_scale + z_scale) / 3;

  x_scale = avg_scale / x_scale;
  y_scale = avg_scale / y_scale;
  z_scale = avg_scale / z_scale;

}

void Magnetometer::setOrientationOffset() {
    for(int i = 0; i < 10; i++) {
        mag.read();

        x = sensitivity * (mag.m.x - x_offset) * x_scale;
        y = sensitivity * (mag.m.y - y_offset) * y_scale;
        z = sensitivity * (mag.m.z - z_offset) * z_scale;

        orientation_offset += atan2(y,x);
    }
    orientation_offset /= 10;
}


void Magnetometer::readFiltered() {
	readCalibrated();
	orientation = (alpha_m*orientation) + ((1 - alpha_m)*last_orientation);
	last_orientation = orientation;
}