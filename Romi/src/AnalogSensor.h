#ifndef Analog_Sensor_h
#define Analog_Sensor_h

#include <Arduino.h>

const int NUM_CALIBRATIONS = 20;

class AnalogSensor
{
    public:
        AnalogSensor(byte line_pin);
        int read();
        int readRaw();
        virtual float readCalibrated();
        void calibrate();

    protected:
        byte pin;
		int last_value;
        float calibration_offset=0;
};

#endif
