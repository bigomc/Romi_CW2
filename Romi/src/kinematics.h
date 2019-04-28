#ifndef _Kinematics
#define _Kinematics_h

#include <Arduino.h>

#define RAD_PER_COUNT   4.363323129985824
#define GEAR_RATIO 120.0
#define COUNTS_PER_SHAFT_REVOLUTION 12.0
const float WHEEL_RADIUS = 35.0;
const float WHEEL_DISTANCE = 138.0; //Corrected to fit calculations below
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
         void  sensorFusion();
         float getTheta_fDegrees();
         float getTheta_fRadians();

    private:

         float x=900;
         float y=900;
         float theta=0;
         float angular_velocity = 0;
         long  left_angular_velocity = 0;
         long  right_angular_velocity = 0;
         bool  debug = false;
         unsigned long last_update = 0;
         unsigned long last_gh_update = 0;
         float heading_mag = 0;
         float gyro = 0;
         float g_high_filter = 0;
         float complementary_heading = 0;
         float alpha = 0.3;
         float g_h_heading = 0;
         float angular_rate = 0;
         float h = 0.0001;
         float g = 0.1;
         float theta_f = 0;
         bool initialised = false;

};

#endif
