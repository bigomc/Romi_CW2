#include "motors.h"



Motor::Motor(byte pwm, byte dir)
{

	//store pin numbers
	pwm_pin = pwm;
	dir_pin = dir;

	//set pins as outputs
	pinMode(pwm_pin, OUTPUT);
	pinMode(dir_pin, OUTPUT);

	//set initial speed and direction
	digitalWrite(pwm_pin, LOW);
	digitalWrite(dir_pin, LOW);

}


void Motor::setPower(float demand)
{
	// Toggle direction based on sign of demand.
	digitalWrite(dir_pin, demand < 0 ? HIGH : LOW);

	// Write out absolute magnitude to pwm pin
	demand = abs(demand);
	demand = constrain(demand, 0, max_power);
	analogWrite(pwm_pin, demand);
}