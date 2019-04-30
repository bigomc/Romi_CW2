#include <Arduino.h>
#include "kinematics.h"
#include "Wheels.h"
#include "utils.h"
#include "imu.h"
#include "magnetometer.h"

extern Magnetometer Mag;
extern Imu imu;

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

    left_angular_velocity = ((RAD_PER_COUNT * left_count) / time_elapsed);
    right_angular_velocity = ((RAD_PER_COUNT * right_count) / time_elapsed);

	angular_velocity = ((right_angular_velocity - left_angular_velocity)*WHEEL_RADIUS)/WHEEL_DISTANCE; //Radians per second
	//angular_velocity = rad2deg(angular_velocity); //Degrees per second

	x_hat = x;
	y_hat = y;
	theta_hat = theta;

    if (debug)
    {
        printPose();
    }

}

void Kinematics::predictAngularVelocity() {
	if(!initialised) {
		initialised = true;

		enableInterrupts();
	}

    //Calculate delta since last update
    int16_t left_count = getCountLeft();
    int16_t right_count = getCountRight();

    time_elapsed = millis() - last_update;
    last_update = millis();

    left_angular_velocity = ((RAD_PER_COUNT * left_count) / time_elapsed);
    right_angular_velocity = ((RAD_PER_COUNT * right_count) / time_elapsed);

	angular_velocity = ((right_angular_velocity - left_angular_velocity)*WHEEL_RADIUS)/WHEEL_DISTANCE; //Radians per second
	angular_velocity_hat = angular_velocity;
}

void Kinematics::predictOrientation() {
	float dt;

	dt = angular_velocity_hat * time_elapsed;
	dt /= 1000;
	theta += dt;

	theta_hat = theta;
}

void Kinematics::updateAngularVelocity(float measurement) {
	float k_2;

	P_w = P_w + Q_w;
	K_w = P_w / (P_w + R_w);

	angular_velocity_hat = angular_velocity_hat + K_w * (measurement - angular_velocity);
	k_2 = (K_w * K_w);
	P_w = (P_w * (1 - (2 * K_w) + k_2)) + (k_2 * R_w);
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

void Kinematics::sensorFusion(){

	float time_elapsed = millis() - last_gh_update;
	//last_gh_update = millis();
	//G-H Filter to improve heading accuracy
	// angular_rate = angular_rate + h*(angular_velocity-angular_rate); //Degrees per second
	// float h_prediction = g_h_heading + angular_rate*(time_elapsed*0.001); //Degrees

	//Serial.print("Ang vel: ");
	//Serial.println(angular_velocity);
	//Serial.print("h_prediction: ");
	//Serial.println(h_prediction);

	//Complementary heading filter calculations:
	heading_mag = Mag.headingFiltered(); //Filtered magnetometer reading
	//Gyroscope reading and high pass Filter
	imu.getFiltered(); //Gyroscope reading in Degrees per second
	//gyro = imu.getFiltered(); //Gz low pass filter
	//g_high_filter += ((imu.gz - gyro)*time_elapsed)*0.001; //Gz after high pass filter

	//Putting all together in complementary filter
	complementary_heading = (1-alpha)*(complementary_heading + (deg2rad(imu.gz)*time_elapsed*0.001))+alpha*(heading_mag); //Complementary filter - Degrees

	//Wrapping angle value
	if(complementary_heading < -PI){
			complementary_heading += (2 * PI);
			}
	if(complementary_heading > 180){
			complementary_heading -= (2 * PI);
			}

	//Serial.print("complementary_heading: ");
	//Serial.println(complementary_heading);

	//g-h filter
	// float residual = complementary_heading - h_prediction;
	// g_h_heading = h_prediction + g*residual;
	// theta_f = g_h_heading;

}



float Kinematics::getThetaDegrees()
{
    return rad2deg(theta_hat);
}

float Kinematics::getThetaRadians()
{
    return theta;
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
