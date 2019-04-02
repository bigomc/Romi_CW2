
#include "line_sensors.h"
#include "pins.h"

LineSensor::LineSensor(int line_pin)
{

	pin = line_pin;
	pinMode(pin, INPUT);

}

int LineSensor::read() {
    last_value = analogRead(pin);

    return last_value;
}

int LineSensor::readRaw()
{
	return last_value;
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
	int p = map(last_value, calibration_offset, BLACK_THRESHOLD, 0, 100);
	p = min(100, max(0, p));

	return p;
}
