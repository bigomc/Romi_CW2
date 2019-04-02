#ifndef _PID_h
#define _PID_h
#include <stdint.h>
#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

/*Here, the definition of the PID class begins. This is indicated by the keyword: "class"
This is a general description of the data and functions that the class contains.
To use a class, we must make a specific instance of the class by declaring it into the same way we declare a variable.
For example, to create a version of the PID class, in our main file we might write:

PID LeftWheelPID;
PID RightWheelPID;

This will create two instances of the PID class; one for the left wheel and one for the right wheel.
Each class will have a full copy of all the variables and functions defined for that particular class.
*/

class PID
{
	/* Public functions and variables are defined here. A public function / variable can be accessed from outside
	 * the class.
	 * For example, once we have made an instance of the PID class, we can call the update function by writing:
	 *
	 * LeftWheelPID.update();
	 *
	 * Note that this will only update the LeftWheelPID - RightWheelPID will not be updated unless we also call
	 * RightWheelPID.update()
	 */
public:

	PID(float P, float D, float I); // This is the class constructor. It is called whenever we create an instance of the PID class 
	void  setGains(float P, float D, float I); // This function updates the values of the gains
	void  reset(); //This function resets any stored values used by the integral or derative terms
	float update(float demand, float measurement); //This function calculates the PID control signal. It should be called in a loop
	void  printComponents(); //This function prints the individual components of the control signal and can be used for debugging
	void  setMax(float  new_max); //This function sets the maximum output the controller can ask for
	void  setDebug(bool state); //This function sets the debug flag;
	void  printResponse(); // This function prints the ratio of input to output in a way that is nicely interpreted by the Serial plotter
	void  setShowResponse(bool state); //This functions set the show_response flag

/* Private functions and variables are defined here. These functions / variables cannot be accessed from outside the class.
 * For example, if we try to set the value of Kp in the file "Romi.h", we will get an error (Try it out!)
 */
private:

	//Control gains
	float Kp; //Proportional
	float Ki; //Integral
	float Kd; //Derivative

	//We can use this to limit the output to a certain value
	float max_output = 255;

	//Output components
	//These are used for debugging purposes
	float Kp_output = 0;
	float Ki_output = 0;
	float Kd_output = 0;
	float total = 0;

	//Values to store
	float last_demand = 0; //For storing the previous input
	float last_measurement = 0; //For storing the last measurement
	float last_error = 0; //For calculating the derivative term
	float integral_error = 0; //For storing the integral of the error
	long  last_millis = 0;
	bool  debug = false; //This flag controls whether we print the contributions of each component when update is called
	bool  show_response = false; // This flag controls whether we print the response of the controller on each update

};

#endif
