#include "irproximity.h"


SharpIR::SharpIR(byte _pin)
{
  pin = _pin;
}

int SharpIR::read() {
    last_value = analogRead(pin);

    return last_value;
}

int SharpIR::getDistanceRaw()
{
    return last_value;
}


/*
 * This piece of code is quite crucial to mapping
 * obstacle distance accurately, so you are encouraged
 * to calibrate your own sensor by following the labsheet.
 * Also remember to make sure your sensor is fixed to your
 * Romi firmly and has a clear line of sight!
 */
float SharpIR::getDistanceInMM()
{

    float distance = (float)last_value;

    // map this to 0 : 5v range.
    distance *= 0.0048;
    //Original numbers provided gave distance in cm. Numbers below updated with
    //calibration values for my Romi with distance in mm
    const float exponent = (1/-0.7); //Calibration values for my Romi
    distance = pow( ( distance / 76.14 ), exponent);

    return distance;
}
