#ifndef _Line_follow_h
#define _Line_follow_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

const int NUM_CALIBRATIONS = 20;

class LineSensor
{
    public:
        LineSensor(int pin);
        void calibrate();
        int  readRaw();
        float  readCalibrated();
		int read();

    private:
		int last_value;
        float calibration_offset=0;
        int pin;

};
#endif
