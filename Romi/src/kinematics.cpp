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

void Kinematics::predictAngularVelocity() {
	if(!initialised) {
		initialised = true;

		enableInterrupts();
	}

    //Calculate delta since last update
    int16_t left_count = getCountLeft();
    int16_t right_count = getCountRight();
	float delta_dif = MM_PER_COUNT * (right_count - left_count);

    time_elapsed = millis() - last_update;
    last_update = millis();

    left_angular_velocity = ((RAD_PER_COUNT * left_count) / time_elapsed);
    right_angular_velocity = ((RAD_PER_COUNT * right_count) / time_elapsed);

	angular_velocity = (delta_dif / WHEEL_DISTANCE) * 1000 / time_elapsed; //Radians per second
	angular_velocity_hat = angular_velocity;
}

void Kinematics::updateAngularVelocity(float measurement) {
	float k_2;

	P_w = P_w + Q_w;
	K_w = P_w / (P_w + R_w);

	angular_velocity_hat = angular_velocity_hat + K_w * (measurement - angular_velocity);
	k_2 = (K_w * K_w);
	P_w = (P_w * (1 - (2 * K_w) + k_2)) + (k_2 * R_w);
}

void Kinematics::predictOrientation() {
	float dt;
	float offset = 0;

	theta = theta_hat;
	dt = angular_velocity_hat * time_elapsed;
	dt /= 1000;
	theta_hat += dt;

	if(theta_hat < -PI ) {
        offset = 2 * PI;
    }
    if(theta_hat > PI ) {
        offset = -2 * PI;
    }
    theta_hat += offset;
}

void Kinematics::updateOrientation(float measurement) {
	float k_2;

	P_t = P_t + Q_t;
	K_t = P_t / (P_t + R_t);

	theta_hat = theta_hat + K_t * (measurement - theta);
	k_2 = (K_t * K_t);
	P_t = (P_t * (1 - (2 * K_t) + k_2)) + (k_2 * R_t);
}

void Kinematics::updatePosition() {

	float sum = (left_angular_velocity + right_angular_velocity) * WHEEL_RADIUS / 2;

	sum *= time_elapsed;
	sum /= 1000;

	x += sum * cos(theta_hat);
	y += sum * sin(theta_hat);
}

float Kinematics::getThetaDegrees()
{
    return rad2deg(theta_hat);
}

float Kinematics::getThetaRadians()
{
    return theta_hat;
}

float Kinematics::getAngularVelocity() {
    return angular_velocity_hat;
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
    Serial.println(rad2deg(theta_hat));
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

//Function to convert rotation angle in degrees to encoder counts
long Kinematics::angle2counts(float ang){
  float ang_r = ang*PI/180; //convert to radians
  float distance = (ang_r*WHEEL_DISTANCE)/2; //value of equivalent rotation distance in mm - arc length
  float count_f = distance/MM_PER_COUNT; // Number of counts needed - float number
  long counts =round(count_f); //Target number of counts to produce an angle
  return counts;
}

