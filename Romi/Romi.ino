
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Library Includes.                                                             *
 * Be sure to check each of these to see what variables/functions are made        *
 * global and accessible.                                                        *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "src/pid.h"
#include <Wire.h>
#include "src/pins.h"
#include "src/utils.h"
#include "src/motors.h"
#include "src/pid.h"
#include "src/kinematics.h"
#include "src/line_sensors.h"
#include "src/irproximity.h"
#include "src/mapping.h"
#include "src/RF_Interface.h"
#include "src/imu.h"
#include "src/magnetometer.h"
#include "src/Pushbutton.h"
#include "src/Scheduler.h"
#include "src/wheels.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Definitions.  Other definitions exist in the .h files above.                  *
 * Also ensure you check pins.h for pin/device definitions.                      *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define BAUD_RATE 115200
#define SAMPLING_TICK_PERIOD    5
#define MAX_VELOCITY    3
#define TIME_LIMIT  100000
#define LINE_CONFIDENCE 70
#define VMAX    3

struct Point_tag {
    float x;
    float y;
} typedef Point_t;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Class Instances.                                                              *
 * This list is complete for all devices supported in this code.                 *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
Kinematics    Pose; //Kinematics class to store position and heading

LineSensor    LineLeft(LINE_LEFT_PIN); //Left line sensor
LineSensor    LineCentre(LINE_CENTRE_PIN); //Centre line sensor
LineSensor    LineRight(LINE_RIGHT_PIN); //Right line sensor

SharpIR       DistanceSensor(SHARP_IR_PIN); //Distance sensor

Imu           _imu;

Magnetometer  Mag; // Class for the magnetometer

Motor         LeftMotor(MOTOR_PWM_L, MOTOR_DIR_L);
Motor         RightMotor(MOTOR_PWM_R, MOTOR_DIR_R);

//These work for our Romi - We strongly suggest you perform your own tuning
PID           LeftSpeedControl( 10, 0.1, 1 );
PID           RightSpeedControl( 10, 0.1, 1 );
PID           HeadingControl( 5, 0, 1 );
PID           TurningControl( 3, 0, 2 );

Mapper        Map; //Class for representing the map

Pushbutton    ButtonB( BUTTON_B, DEFAULT_STATE_HIGH);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Global variables.                                                             *
 * These global variables are not mandatory, but are used for the example loop() *
 * routine below.                                                                *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

 // Variables of the position goal
 float x_goal;
 float y_goal;
 float x_error;
 float y_error;
 float orientation_error;
 float position_error;
 const float Ks = 0.5;

 // Planning Variables
 bool goal_reached = false;
 const Point_t points[] = {{700, 900}, {36, 1764}, {36, 36}, {1764, 36}};
 int point_index = 0;

//Use these variables to set the demand of the speed controller
 float left_speed_demand = 0;
 float right_speed_demand = 0;;

//Mapping variables
unsigned long count_mapping = 0;
bool stop_mapping = false;

//Heading Flag
bool heading = false;
float target_rot = 0;
float zero_rot = 0;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * This setup() routine initialises all class instances above and peripherals.   *
 * It is recommended:                                                            *
 * - You keep this sequence of setup calls if you are to use all the devices.    *
 * - Comment out those you will not use.                                         *
 * - Insert new setup code after the below sequence.                             *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void setup()
{
  //Set speed control maximum outputs to match motor
  // LeftSpeedControl.setMax(100);
  // RightSpeedControl.setMax(100);

  // Initialise Serial communication
  Serial.begin( BAUD_RATE );
  delay(1000);

  Serial.println("Board Reset");

  // Romi will wait for you to press a button and then print
  // the current map.
  ButtonB.waitForButton();
  Map.printMap();

  // Set the initial goal point
  x_goal = points[point_index].x;
  y_goal = points[point_index].y;

  //Setup RFID card
  //setupRFID();

  // Calibration code
  Serial.println("Calibrating line sensors");
  LineCentre.calibrate();
  LineLeft.calibrate();
  LineRight.calibrate();

  // The magnetometer calibration routine require you to move
  // your robot around  in space.
  // See related lab sheets for more information.
  Serial.println("Initialising Magnetometer");
  Wire.begin();
  Mag.init();
  Serial.println("Press button to calibrate Magnetometer");
  ButtonB.waitForButton();
  LeftMotor.setPower(30);
  RightMotor.setPower(-30);
  Mag.calibrate();
  LeftMotor.setPower(0);
  RightMotor.setPower(0);

  _imu.init();
  _imu.calibrate();

  // Set the random seed for the random number generator
  // from A0, which should itself be quite random.
  randomSeed(analogRead(A0));

  //// Watch for second button press, then begin autonomous mode.
  Serial.println("Press button to begin autonomous mode");
  ButtonB.waitForButton();

  // Your extra setup code is best placed here:
  // ...
  Map.resetMap();
  Mag.set_zero();
  // ...
  // but not after the following:

  // Because code flow has been blocked, we need to reset the
  // last_time variable of the PIDs, otherwise we update the
  // PID with a large time elapsed since the class was
  // initialised, which will cause a big intergral term.
  // If you don't do this, you'll see the Romi accelerate away
  // very fast!
    HeadingControl.setMax(2 * VMAX);
    TurningControl.setMax(VMAX);
    LeftSpeedControl.setMax(2 * VMAX);
    RightSpeedControl.setMax(2 * VMAX);
    TurningControl.reset();
    HeadingControl.reset();
    LeftSpeedControl.reset();
    RightSpeedControl.reset();

    //Initialise simple scheduler
    initScheduler();

    createTask(UpdateTask, SAMPLING_TICK_PERIOD);
    createTask(ControlSpeed, 10);
    createTask(ControlPosition, 10);
    //createTask(doMovement, 20);
    //createTask(doTurn, 40);
    createTask(SensorsTask, 50);
    createTask(MappingTask, 50);
    createTask(PlanningTask, 100);
    createTask(PrintTask, 500);
    count_mapping = millis ();
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * This loop() demonstrates all devices being used in a basic sequence.
 * The Romi should:
 * - move forwards with random turns
 * - log lines, RFID and obstacles to the map.
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void loop() {
    Scheduler();
}

void UpdateTask() {
    Pose.update();
}

void SensorsTask() {
    // The aim of this task is to perform all sensor readings and only return
    // the real value when needed instead of read everytime, this reduces
    // latency and speeds up the program execution

    DistanceSensor.read();
    LineCentre.read();
    LineLeft.read();
    LineRight.read();
    Mag.readCalibrated();
    _imu.readFiltered();
}

void PrintTask() {
    Serial.print(" ");
    Serial.print("[");
    Serial.print(Pose.getX());
    Serial.print(", ");
    Serial.print(Pose.getY());
    Serial.print(", ");
    Serial.print(Pose.getThetaRadians());
    Serial.print("] [(");
    Serial.print(Pose.getLeftVelocity());
    Serial.print(", ");
    Serial.print(Pose.getRightVelocity());
    Serial.print(") (");
    Serial.print(left_speed_demand);
    Serial.print(", ");
    Serial.print(right_speed_demand);
    Serial.print(")] [");
    Serial.print(DistanceSensor.readCalibrated());
    Serial.print(", ");
    Serial.print(Mag.headingFiltered());
    Serial.print(", ");
    Serial.print(_imu.gz);
    Serial.print("] (");
    Serial.print(x_goal);
    Serial.print(", ");
    Serial.print(y_goal);
    Serial.println("]");
}

void ControlSpeed() {
    if(!stop_mapping && !heading){ //
        float left_speed_control_signal = LeftSpeedControl.update(left_speed_demand, Pose.getLeftVelocity());
        float right_speed_control_signal = RightSpeedControl.update(right_speed_demand, Pose.getRightVelocity());
        left_speed_control_signal += (6.66 * left_speed_demand);
        right_speed_control_signal += (6.66 * right_speed_demand);

        LeftMotor.setPower(left_speed_control_signal);
        RightMotor.setPower(right_speed_control_signal);
    } else if (stop_mapping) {
        LeftMotor.setPower(0);
        RightMotor.setPower(0);
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
* This function controls the left and right velocities in order to make romi
* arrive to a goal position
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void ControlPosition() {
    float sat;
    float offset = 0;
    float turning;
    float ahead;

    x_error = x_goal - Pose.getX();
    y_error = y_goal - Pose.getY();

    position_error = sqrt(x_error*x_error + y_error*y_error);
    orientation_error = atan2(y_error, x_error) - Pose.getThetaRadians();
    if(orientation_error < -PI ){
        orientation_error += (2 * PI);
    }
    if(orientation_error > PI){
        orientation_error -= (2 * PI);
    }

    if(position_error > 50) {
        sat = min(Ks, max(-Ks, orientation_error));

        turning = TurningControl.update(orientation_error, 0);
        ahead = HeadingControl.update(position_error, 0);
        ahead *= (1 - (abs(sat) / Ks));

        left_speed_demand = ahead - turning;
        right_speed_demand = ahead + turning;
    } else {
        goal_reached = true;

        left_speed_demand = 0;
        right_speed_demand = 0;
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
* This function iterates over a list of points or coordetates to change the
* goal position and make Romi explore the map
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void PlanningTask() {
    int size = sizeof(points)/sizeof(Point_t);

    // Changes the goal when the current goal has reached
    if(goal_reached) {

        // Verify the size of the goals
        if(point_index < (size - 1)) {
            goal_reached = false;

            point_index++;
            x_goal = points[point_index].x;
            y_goal = points[point_index].y;
        }
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * We have implemented a random walk behaviour for you
 * with a *very* basic obstacle avoidance behaviour.
 * It is enough to get the Romi to drive around.  We
 * expect that in your first week, should should get a
 * better obstacle avoidance behaviour implemented for
 * your Experiment Day 1 baseline test.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void doMovement() {

    // Static means this variable will keep
    // its value on each call from loop()
    static unsigned long walk_update = millis();

    // used to control the forward and turn
    // speeds of the robot.
    float forward_bias;
    float turn_bias;
    int obs_dect = DistanceSensor.readRaw();

if (Pose.getThetaDegrees() <=90 && Pose.getThetaDegrees() >-5 ){
    float forward_bias=0;
      float turn_bias=3;
      left_speed_demand = forward_bias - turn_bias;
              right_speed_demand = forward_bias + turn_bias;
                } else {
              stop_mapping =1;
            }

 }

//Function to turn the robot a specific target angle
void doTurn (){

    if(heading){
      float current_rot = Pose.getThetaDegrees() - zero_rot;
      if((target_rot-current_rot>=2) && current_rot>-10){
        long heading_counts = Pose.angle2counts(2);
        long targetCounts = getAbsoluteCountRight() + heading_counts; // Turning CCW
        float rot_demand = HeadingControl.update(targetCounts,getAbsoluteCountRight());
        LeftMotor.setPower(-rot_demand);
        RightMotor.setPower(rot_demand);
        Serial.print(current_rot);
        Serial.print(" ");
        Serial.println(target_rot);
        delay(100);

      } else if (((MAP_X-Pose.getX())< C_HALF_WIDTH) || ((MAP_Y-Pose.getY())< C_HALF_WIDTH) || (Pose.getX()<C_HALF_WIDTH) || (Pose.getY()<C_HALF_WIDTH)){
            float forward_speed = 10;
            LeftMotor.setPower(forward_speed);
            RightMotor.setPower(forward_speed);
            }

        else {
              heading =  false;
              Serial.print("heading: ");
              Serial.println(heading);
              Serial.print(current_rot);
              Serial.print(" ");
              Serial.println(target_rot);
              }
    }
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * This function groups up our sensor checks, and then
 * encodes into the map.  To get you started, we are
 * simply placing a character into the map.  However,
 * you might want to look using a bitwise scheme to
 * encode more information.  Take a look at mapping.h
 * for more information on how we are reading and
 * writing to eeprom memory.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void MappingTask() {

    if (stop_mapping && (Pose.getLeftVelocity() == 0) && (Pose.getRightVelocity() == 0)){
        ButtonB.waitForButton();
        Map.printMap();
    }

    if ((millis() - count_mapping) > TIME_LIMIT){
        Serial.println("Stopping");
        stop_mapping = true;
    }


    // Read the IR Sensor and determine distance in
    // mm.  Make sure you calibrate your own code!
    // We threshold a reading between 40mm and 12mm.
    // The rationale being:
    // We can't trust very close readings or very far.
    // ...but feel free to investigate this.
    byte cell_read = Map.readEeprom(Pose.getX(),Pose.getY());

    if (cell_read != (byte)'O' && cell_read != (byte)'L' && cell_read != (byte)'R'){
        Map.updateMapFeature((byte)'V',Pose.getY(),Pose.getX());
    }


    //OBSTACLE avoidance

    float distance = DistanceSensor.readCalibrated();
    if( distance < 400 && distance > 100 ) {
        // We know the romi has the sensor mounted
        // to the front of the robot.  Therefore, the
        // sensor faces along Pose.Theta.
        // We also add on the distance of the
        // sensor away from the centre of the robot.
        distance += 80;


        // Here we calculate the actual position of the obstacle we have detected
        float projected_x = Pose.getX() + ( distance * cos( Pose.getThetaRadians() ) );
        float projected_y = Pose.getY() + ( distance * sin( Pose.getThetaRadians() ) );
        Map.updateMapFeature( (byte)'O', projected_y, projected_x );
    }

    // Check RFID scanner.
    // Look inside RF_interface.h for more info.
    if( checkForRFID() ) {

        // Add card to map encoding.
        Map.updateMapFeature( (byte)'R', Pose.getY(), Pose.getX() );

        // you can check the position reference and
        // bearing information of the RFID Card in
        // the following way:
        // serialToBearing( rfid.serNum[0] );
        // serialToXPos( rfid.serNum[0] );
        // serialToYPos( rfid.serNum[0] );
        //
        // Note, that, you will need to set the x,y
        // and bearing information in rfid.h for your
        // experiment setup.  For the experiment days,
        // we will tell you the serial number and x y
        // bearing information for the cards in use.

    }

    // Basic uncalibrated check for a line.
    // Students can do better than this after CW1 ;)
    // Condition will depend on calibration method, the one below worked for my Romi using static calibration
    if( (LineCentre.readCalibrated() + LineLeft.readCalibrated() + LineRight.readCalibrated()) > LINE_CONFIDENCE  ) {
        Map.updateMapFeature( (byte)'L', Pose.getY(), Pose.getX() );
    }
}
