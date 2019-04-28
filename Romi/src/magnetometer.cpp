#include "magnetometer.h"
#include "pins.h"
#include <Arduino.h>
#include "utils.h"

void Magnetometer::init()
{
    if (!mag.init())
    {
        Serial.println("Failed to detect and initialize magnetometer!");
        while (1);
    }

    mag.enableDefault();
    //mag.writeReg(LIS3MDL::CTRL_REG1,0b01010010); //Setting high performance mode with 300Hz ODR axes X and Y
    //mag.writeReg(LIS3MDL::CTRL_REG4,0b00001000);  //Setting high performance mode with 300Hz ODR axis Z
}

void Magnetometer::read()
{
    mag.read();
}

void Magnetometer::readRaw()
{
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
    delay(1000);

    analogWrite(BUZZER_PIN, 10);
    delay(500);
    digitalWrite( BUZZER_PIN, LOW );
    delay(500);
    analogWrite(BUZZER_PIN, 10);
    delay(500);
    digitalWrite( BUZZER_PIN, LOW );
    delay(500);

    for (int i=0;i<NUM_CALIBRATIONS_MAG;i++)
    {
        //analogWrite(BUZZER_PIN, 10);
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

}

void Magnetometer::set_zero() {
    heading_mag_zero = 0;
    for(int i = 0; i < 10; i++) {
        readCalibrated();
        heading_mag_zero += atan2(y,x);
    }
    heading_mag_zero /= 10.0;
}


float Magnetometer::getHeading()
{
    double aux = 0;

    //readCalibrated();
    heading = heading_mag_zero - atan2(y,x); //Relative to start position

    //Adjusting angle value
    if(heading < -PI){
        aux = 2 * PI;
    }
    if(heading > PI){
        aux = - 2 * PI;
    }
    heading += aux;

    return rad2deg(heading);
}


float Magnetometer::headingFiltered() {
  //Magnetometer reading and low pass Filter heading
	float unfiltered_h = getHeading();

  heading_filter_mag = (alpha*unfiltered_h) + ((1 - alpha)*last_heading);
	last_heading = heading_filter_mag;

  return heading_filter_mag;

}
