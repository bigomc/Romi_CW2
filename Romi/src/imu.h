#ifndef _IMU_h
#define _IMU_h

#include <Arduino.h>
#include <LSM6.h>

const int NUM_CALIBRATIONS_IMU = 100;

class Imu
{
    public:
        void init();
        void  readRaw();
        void  readCalibrated();
        void  calibrate();
        LSM6 imu;
        float ax = 0;
        float ay = 0;
        float az = 0;
        float gx = 0;
        float gy = 0;
        float gz = 0;

    private:
        float a_sensitivity = 0.061;  //Corresponds to +/-2g default sensitivity
        float g_sensitivity = 8.75; //Corresponds to default sensitivity of +/-245 mdps
        float gx_offset = 0;
        float gy_offset = 0;
        float gz_offset = 0;


};

#endif
