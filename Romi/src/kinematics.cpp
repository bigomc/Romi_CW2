#include <Arduino.h>
#include "kinematics.h"
#include "Wheels.h"
#include "utils.h"

/*
 * Class constructor
 * This runs whenever we create an instance of the class
 */
Kinematics::Kinematics(void)
{
	setupEncoders();
}

void Kinematics::update()
{
    //Calculate delta since last update
    int16_t left_count = getCountLeft();
    int16_t right_count = getCountRight();
    float delta_sum = MM_PER_COUNT * (right_count + left_count);
    float delta_dif = MM_PER_COUNT * (right_count - left_count);
    float offset = 0;

    //Update position
    x += (cos(theta) * delta_sum / 2);
    y += (sin(theta) * delta_sum / 2);
    theta +=  (delta_dif / WHEEL_DISTANCE);

    //Wrap theta between -PI and PI.
    if(theta < -PI ) {
        offset = 2 * PI;
    }
    if(theta > PI ) {
        offset = -2 * PI;
    }
    theta += offset;

    float time_elapsed = millis() - last_update;
    last_update = millis();

    left_angular_velocity = (RAD_PER_COUNT * left_count / time_elapsed);
    right_angular_velocity = (RAD_PER_COUNT * left_count / time_elapsed);

    if (debug)
    {
        printPose();
    }

}

float Kinematics::getThetaDegrees()
{
    return rad2deg(theta);
}

float Kinematics::getThetaRadians()
{
    return (theta);
}

float Kinematics::getAngularVelocity() {
    return rad2deg( angular_velocity );
}

float Kinematics::getX()
{
    return x;
}

float Kinematics::getY()
{
    return y;
}

void Kinematics::resetPose()
{

    x = 0;
    y = 0;
    theta = 0;

}

void Kinematics::setPose(float newX, float newY, float newTheta)
{

    x = newX;
    y = newY;
    theta = newTheta;

}

void Kinematics::printPose()
{

    Serial.print(F("X: "));
    Serial.print(x);
    Serial.print(F(" Y: "));
    Serial.print(y);
    Serial.print(F(" H: "));
    Serial.println(rad2deg(theta));

}

void Kinematics::setDebug(bool state)
{
    debug = state;
}


float Kinematics::getDistanceFromOrigin()
{
    return sqrt(x*x + y*y);
}

float Kinematics::getLeftVelocity() {
    return left_angular_velocity;
}

float Kinematics::getRightVelocity() {
    return right_angular_velocity;
}
