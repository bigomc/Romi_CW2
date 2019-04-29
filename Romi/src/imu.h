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
		void  readFiltered();
        LSM6 imu;
        float ax = 0;
        float ay = 0;
        float az = 0;
        float gx = 0;
        float gy = 0;
        float gz = 0;
		float last_ax = 0;
		float last_ay = 0;
		float last_az = 0;
		float last_gx = 0;
		float last_gy = 0;
		float last_gz = 0;
		float alpha = 0.2;

    private:
        float a_sensitivity = 0.061/1000;
        float g_sensitivity = 8.75/1000;
        float gx_offset = 0;
        float gy_offset = 0;
        float gz_offset = 0;


};

#endif
