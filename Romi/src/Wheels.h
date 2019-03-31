//
//  Wheels.h
//
//
//  Created by Rodrigo Moreno on 13/02/2019.
//

#ifndef Wheels_h
#define Wheels_h

#ifdef __cplusplus
extern "C" {
#endif

#define E1_A_PIN  7
#define E1_B_PIN  23
#define E0_A_PIN  26
    // Pin definitions, to make the
    // code easier to read.
#define R_PWM_PIN 9
#define R_DIR_PIN 15
#define L_PWM_PIN 10
#define L_DIR_PIN 16

    void setupEncoders();
    long getCountRight();
    long getCountLeft();
    void setLeftSpeed(int speed);
    void setRightSpeed(int speed);
    int getLeftSpeed();
    int getRightSpeed();
    long getAbsoluteCountRight();
    long getAbsoluteCountLeft();

#ifdef __cplusplus
}
#endif

#endif /* Wheels_h */
