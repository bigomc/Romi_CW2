#include <Arduino.h>
#include "kinematics.h"
#include "Wheels.h"
#include "utils.h"
#include "imu.h"
#include "magnetometer.h"

extern Magnetometer Mag;
extern Imu Imu;

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
	if(!initialised) {
		initialised = true;

		enableInterrupts();
	}

    //Calculate delta since last update
    int16_t left_count = getCountLeft();
    int16_t right_count = getCountRight();
    float delta_sum = MM_PER_COUNT * (right_count + left_count);
    float delta_dif = MM_PER_COUNT * (right_count - left_count);
    float offset = 0;


    //Update position
    x += (cos(deg2rad(theta_f)) * delta_sum / 2);
    y += (sin(deg2rad(theta_f)) * delta_sum / 2);
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

    left_angular_velocity = ((RAD_PER_COUNT * left_count) / time_elapsed);
    right_angular_velocity = ((RAD_PER_COUNT * right_count) / time_elapsed);

		angular_velocity = ((right_angular_velocity - left_angular_velocity)*WHEEL_RADIUS)/WHEEL_DISTANCE; //Radians per second
		angular_velocity = rad2deg(angular_velocity); //Degrees per second

    if (debug)
    {
        printPose();
    }

}


void Kinematics::sensorFusion(){

	float time_elapsed = millis() - last_gh_update;
	last_gh_update = millis();
	//G-H Filter to improve heading accuracy
	angular_rate = angular_rate + h*(angular_velocity-angular_rate); //Degrees per second
	float h_prediction = g_h_heading + angular_rate*(time_elapsed*0.001); //Degrees

	//Serial.print("Ang vel: ");
	//Serial.println(angular_velocity);
	//Serial.print("h_prediction: ");
	//Serial.println(h_prediction);

	//Complementary heading filter calculations:
	heading_mag = Mag.headingFiltered(); //Filtered magnetometer reading
	//Gyroscope reading and high pass Filter
	//Imu.readCalibrated(); //Gyroscope reading in Degrees per second
	//gyro = Imu.getFiltered(); //Gz low pass filter
	//g_high_filter += ((Imu.gz - gyro)*time_elapsed)*0.001; //Gz after high pass filter

	//Putting all together in complementary filter
	complementary_heading = (1-alpha)*(complementary_heading + (Imu.gz*time_elapsed*0.001))+alpha*(heading_mag); //Complementary filter - Degrees

	//Wrapping angle value
	float offset = 0;
	if(complementary_heading < -180){
			offset = 360;
			}
	if(complementary_heading > 180){
			offset = - 360;
			}
	complementary_heading += offset;
	//Serial.print("complementary_heading: ");
	//Serial.println(complementary_heading);

	//g-h filter
	float residual = complementary_heading - h_prediction;
	g_h_heading = h_prediction + g*residual;
	theta_f = g_h_heading;

}



float Kinematics::getThetaDegrees()
{
    return rad2deg(theta);
}

float Kinematics::getTheta_fDegrees()
{
    return theta_f;
}

float Kinematics::getThetaRadians()
{
    return theta;
}

float Kinematics::getTheta_fRadians()
{
    return deg2rad(theta_f);
}

float Kinematics::getAngularVelocity() {
    return angular_velocity;
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

    x = 900;
    y = 900;
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
		Serial.print(F(" H_f: "));
    Serial.println(theta_f);

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
