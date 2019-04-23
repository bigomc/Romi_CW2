#ifndef Line_follow_h
#define Line_follow_h

#include "AnalogSensor.h"

#define BLACK_THRESHOLD 857

class LineSensor : public AnalogSensor {
    public:
		LineSensor(byte pin);
        float readCalibrated() override;
};
#endif
