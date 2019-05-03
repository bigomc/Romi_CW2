#ifndef _IRProximity_h
#define _IRProximity_h

#include "AnalogSensor.h"

class SharpIR : public AnalogSensor {
    public:
        SharpIR(byte pin);
        float readCalibrated() override;
    private:
        float last_distance = 0;
};

#endif
