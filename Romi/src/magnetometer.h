#ifndef _Magentometer
#define _Magentometer

#include <LIS3MDL.h>
#include <Arduino.h>

const int NUM_CALIBRATIONS_MAG = 500;

class Magnetometer
{
    public:
        void init();
        void  readRaw();
        void  readCalibrated();
        void  calibrate();
        void  calculateOffsets();
        void  set_zero();
        float getHeading();
        LIS3MDL mag;
        float x = 0;
        float y = 0;
        float z = 0;
        float heading_mag_zero = 0;

    private:
        float sensitivity = 1.0/6842.0; //default sensitivity for +/-4 Gauss
        int x_min = 32767;
        int y_min = 32767;
        int z_min = 32767;
        int x_max = -32768;
        int y_max = -32768;
        int z_max = -32768;
        float x_offset = 0;
        float y_offset = 0;
        float z_offset = 0;
        float x_scale = 0;
        float y_scale = 0;
        float z_scale = 0;
        float heading = 0;

};

#endif
