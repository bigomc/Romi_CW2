#ifndef _Motor_h
#define _Motor_h
#include <stdint.h>

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

//Pin definitions for motor

const byte default_max_power = 255;
const byte PWM_MAX = 255;

class Motor
{
    public:

        Motor(byte pwm, byte dir); //Constructor - stores pins for controlling the motor
        void setPower(float demand); //Sets the duty factor of the PWM signal sent to the Motor's H-Bridge

    private:

        byte pwm_pin;
        byte dir_pin;

        float max_power=default_max_power;
};

#endif

