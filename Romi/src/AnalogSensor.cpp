#include "AnalogSensor.h"
#include "pins.h"

AnalogSensor::AnalogSensor(byte line_pin) {
	pin = line_pin;
	pinMode(pin, INPUT);
}

int AnalogSensor::read() {
    last_value = analogRead(pin);

    return last_value;
}

int AnalogSensor::readRaw() {
	return last_value;
}

float AnalogSensor::readCalibrated() {
	return last_value - calibration_offset;
}

void AnalogSensor::calibrate() {
	for (int i = 0; i < NUM_CALIBRATIONS; i++) {
        analogWrite(BUZZER_PIN, 255);
		calibration_offset += analogRead(pin);
		delay(10);
        analogWrite(BUZZER_PIN, 0);
	}

	calibration_offset = calibration_offset / NUM_CALIBRATIONS;
}
