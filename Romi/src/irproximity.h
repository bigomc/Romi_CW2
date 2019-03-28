#ifndef _IRProximity_h
#define _IRProximity_h

#include <Arduino.h>

class SharpIR
{
    public:
        SharpIR(byte pin);
        int  getDistanceRaw();
        float  getDistanceInMM();
        void calibrate();

    private:
        byte pin;
};

#endif
