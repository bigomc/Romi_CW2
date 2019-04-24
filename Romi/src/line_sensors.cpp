
#include "line_sensors.h"

LineSensor::LineSensor(byte line_pin) : AnalogSensor(line_pin) {
}

float LineSensor::readCalibrated() {
	int p = map(last_value, calibration_offset, BLACK_THRESHOLD, 0, 100);
	p = min(100, max(0, p));

	return p;
}
