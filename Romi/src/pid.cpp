// 
// 
// 

#include "pid.h"



/*
 * Class constructor
 * This runs whenever we create an instance of the class
 */
PID::PID(float P, float D, float I)
{
	//Store the gains
	setGains(P, D, I);
	//Set last_millis
	last_millis = millis();
}




/*
 * This function prints the individual contributions to the total contol signal
 * You can call this yourself for debugging purposes, or set the debug flag to true to have it called
 * whenever the update function is called.
 */
void PID::printComponents()
{
	Serial.print(F(" Proportional component: "));
	Serial.print(Kp_output);
	Serial.print(F(" Differential component: "));
	Serial.print(Kd_output);
	Serial.print(F(" Integral component: "));
	Serial.print(Ki_output);
	Serial.print(F(" Total: "));
	Serial.println(total);
}


/*
 * This function sets the gains of the PID controller
 */
void PID::setGains(float P, float D, float I)
{
	Kp = P;
	Kd = D;
	Ki = I;
}


/*
 * This is the update function.
 * This function should be called repeatedly.
 * It takes a measurement of a particular variable (ex. Position, speed, heading) and a desired value for that quantity as input
 * It returns an output; this can be sent directly to the motors,
 * combined with other control outputs
 * or sent as input to another controller
 */
float PID::update(float demand, float measurement)
{
	//Calculate how much time (in milliseconds) has passed since the last update call
	long time_now = millis();
	float time_delta = (float)(time_now - last_millis);
	last_millis = time_now;

	//This represents the error term
	float error = demand - measurement;

	//This represents the error derivative
	float error_delta = (last_error - error) / time_delta;

	//Update storage
	last_demand = demand;
	last_measurement = measurement;
	last_error = error;

	integral_error += (error * time_delta);

	//Calculate components
	Kp_output = Kp * error;
	Kd_output = Kd * error_delta;
	Ki_output = Ki * integral_error;

	//Add the three components to get the total output
	total = Kp_output + Kd_output + Ki_output;

	//Make sure we don't exceed the maximum output
	total = constrain(total, -max_output, max_output);

	//Print debugging information if required
	if (debug)
	{
		Serial.print(F("Error: "));
		Serial.print(error);
		Serial.print(F(" Error Delta:"));
		Serial.print(error_delta);
		Serial.print(F(" Error Integral:"));
		Serial.print(integral_error);
		printComponents();
	}

	//Print response if required
	if (show_response)
	{
		printResponse();
	}

	return total;
}


void PID::setMax(float new_max)
{
	if (new_max > 0)
	{
		max_output = new_max;
	}
	else
	{
		Serial.println(F("Max output must be positive"));
	}
}


void PID::setDebug(bool state)
{
	debug = state;
}


void PID::reset()
{

	last_error = 0;
	integral_error = 0;
	last_millis = millis();

}


//This function prints measurement / demand - Good for visualising the response on the Serial plotter
void PID::printResponse()
{

	float response = last_measurement / last_demand;
	Serial.println(response);

}


void PID::setShowResponse(bool state)
{
	show_response = state;
}
