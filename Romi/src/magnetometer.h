#ifndef _Magentometer_h
#define _Magentometer_h

#include <LIS3MDL.h>

const int NUM_CALIBRATIONS_MAG = 200;

class Magnetometer
{
    public:
        void init();
        void  readRaw();
        void  readCalibrated();
        void  calibrate();
        void  calculateOffsets();
        void  set_sensitivity();
        void setOrientationOffset();
        LIS3MDL mag;
        float x = 0;
        float y = 0;
        float z = 0;
        float orientation = 0;

    private:
        float sensitivity = 1.0/6842.0; //This makes more sense if you look at data sheet of the sensor
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
        float orientation_offset = 0;
};

#endif
