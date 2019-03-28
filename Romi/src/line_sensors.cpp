
#include "line_sensors.h"
#include "pins.h"

LineSensor::LineSensor(int line_pin)
{

	pin = line_pin;
	pinMode(pin, INPUT);

}


int LineSensor::readRaw()
{
	return analogRead(pin);
}


void LineSensor::calibrate()
{

	analogWrite(BUZZER_PIN, 255);

	for (int i = 0; i < NUM_CALIBRATIONS; i++)
	{
		calibration_offset += analogRead(pin);
		delay(10);
	}

	calibration_offset = calibration_offset / NUM_CALIBRATIONS;

	analogWrite(BUZZER_PIN, 0);

}

float LineSensor::readCalibrated()
{
	return (analogRead(pin) - calibration_offset);
}
