#ifndef _Kinematics
#define _Kinematics_h

#include <Arduino.h>

#define RAD_PER_COUNT   4.363323129985824
#define GEAR_RATIO 120.0
#define COUNTS_PER_SHAFT_REVOLUTION 12.0
const float WHEEL_RADIUS = 35.0;
const float WHEEL_DISTANCE = 142.1; //Corrected to fit calculations below
const float COUNTS_PER_WHEEL_REVOLUTION = GEAR_RATIO * COUNTS_PER_SHAFT_REVOLUTION;
const float MM_PER_COUNT = ( 2 * WHEEL_RADIUS * PI ) / COUNTS_PER_WHEEL_REVOLUTION;

class Kinematics
{
     public:
         Kinematics(void);
         void  update();
         float getThetaDegrees();
         float getThetaRadians();
         float getX();
         float getY();
         void  resetPose();
         void  setPose(float X, float Y, float theta);
         void  printPose();
         void  setDebug(bool state);
         float getDistanceFromOrigin();
         float getAngularVelocity();
         float getLeftVelocity();
         float getRightVelocity();
         long  angle2counts(float ang);
         void predictAngularVelocity();
         void predictOrientation();
         void updateAngularVelocity(float measurement);
         void updateOrientation(float measurement);
         void updatePosition();

    private:

        float x=900;
        float y=900;
        float theta=0;
        float theta_hat = 0;
        float angular_velocity = 0;
        float angular_velocity_hat = 0;
        long  left_angular_velocity = 0;
        long  right_angular_velocity = 0;

        bool  debug = false;
        bool initialised = false;
        unsigned long last_update = 0;
        float time_elapsed = 0;

        float Q_w = 0.1;
        float K_w = 1;
        float P_w = 10;
        const float R_w = 0.1;

        float Q_t = 0.2;
        float K_t = 1;
        float P_t = 10;
        const float R_t = 11;
};

#endif
