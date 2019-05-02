
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
#define TIME_LIMIT  1000000
#define LINE_CONFIDENCE 70
#define VMAX    3
//#define USE_MAGNETOMETER    1     //To use magnetometer uncomment this line

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

SharpIR       DistanceFront(SHARP_IR_FRONT_PIN); //Distance sensor front
SharpIR       DistanceLeft(SHARP_IR_LEFT_PIN); //Distance sensor left
SharpIR       DistanceRight(SHARP_IR_RIGHT_PIN); //Distance sensor right

Imu           imu;

#ifdef USE_MAGNETOMETER
Magnetometer  Mag; // Class for the magnetometer
#endif

Motor         LeftMotor(MOTOR_PWM_L, MOTOR_DIR_L);
Motor         RightMotor(MOTOR_PWM_R, MOTOR_DIR_R);

//These work for our Romi - We strongly suggest you perform your own tuning
PID           LeftSpeedControl( 10, 0.1, 1 );
PID           RightSpeedControl( 10, 0.1, 1 );
PID           HeadingControl( 4, 0, 1 );
PID           TurningControl( 0.7, 0, 0.6 );

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
 const Point_t points[] = {{1764, 900}, {900, 1764}, {36, 900}, {900, 36}};
 int point_index = 0;

//Use these variables to set the demand of the speed controller
 float left_speed_demand = 0;
 float right_speed_demand = 0;;

//Mapping variables
unsigned long count_mapping = 0;
bool stop_mapping = false;
enum SensorPosition_t {
    SENSOR_LEFT,
    SENSOR_FRONT,
    SENSOR_RIGHT,
    SENSOR_UNKNOWN
};
const float sensors_offset[] = {0.383972, 0, -0.383972};

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

  Wire.begin();
  Serial.println("Calibrating IMU");
  imu.init();
  imu.calibrate();

#ifdef USE_MAGNETOMETER
  Serial.println("Initialising Magnetometer");
  Mag.init();
  Serial.println("Press button to calibrate Magnetometer");
  ButtonB.waitForButton();
  LeftMotor.setPower(30);
  RightMotor.setPower(-30);
  Mag.calibrate();
  LeftMotor.setPower(0);
  RightMotor.setPower(0);
#endif

  // Set the random seed for the random number generator
  // from A0, which should itself be quite random.
  randomSeed(analogRead(A0));

  //// Watch for second button press, then begin autonomous mode.
  Serial.println("Press button to begin autonomous mode");
  ButtonB.waitForButton();

  // Your extra setup code is best placed here:
  // ...
  Map.resetMap();
#ifdef USE_MAGNETOMETER
  Mag.set_zero();
#endif
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
#ifdef USE_MAGNETOMETER
    Mag.read();
#endif
    imu.getFiltered();

    Pose.predictAngularVelocity();
    Pose.updateAngularVelocity(deg2rad(imu.gz));
    Pose.predictOrientation();
#ifdef USE_MAGNETOMETER
    //Pose.updateOrientation(Mag.headingFiltered());
#endif
    Pose.updatePosition();
}

void SensorsTask() {
    // The aim of this task is to perform all sensor readings and only return
    // the real value when needed instead of read everytime, this reduces
    // latency and speeds up the program execution

    DistanceLeft.read();
    DistanceFront.read();
    DistanceRight.read();
    LineCentre.read();
    LineLeft.read();
    LineRight.read();
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
    Serial.print(DistanceLeft.readCalibrated());
    Serial.print(", ");
    Serial.print(DistanceFront.readCalibrated());
    Serial.print(", ");
    Serial.print(DistanceRight.readCalibrated());
#ifdef USE_MAGNETOMETER
    Serial.print(", ");
    Serial.print(Mag.headingFiltered());
#endif
    Serial.print("] (");
    Serial.print(x_goal);
    Serial.print(", ");
    Serial.print(y_goal);
    Serial.println(")");
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

    if(position_error > 10) {
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

    //OBSTACLE mapping
    float distance;
    Point_t coordinate;
    const int distance_resolution = 18;
    const int min_confidence = 0;
    const int max_confidence = 150;

    distance = DistanceLeft.readCalibrated();
    for(int i = min_confidence; (i < distance) && (i < max_confidence); i += distance_resolution) {
        coordinate = getObstacleCoordinates(i, sensors_offset[SENSOR_LEFT]);
        Map.updateMapFeature(Map.EXPLORED, coordinate.y, coordinate.x );
    }
    if(distance < max_confidence) {
        coordinate = getObstacleCoordinates(distance, sensors_offset[SENSOR_LEFT]);
        Map.updateMapFeature(Map.OBSTACLE, coordinate.y, coordinate.x );
    }

    distance = DistanceFront.readCalibrated();
    for(int i = min_confidence; (i < distance) && (i < 2*max_confidence); i += distance_resolution) {
        coordinate = getObstacleCoordinates(i, sensors_offset[SENSOR_FRONT]);
        Map.updateMapFeature(Map.EXPLORED, coordinate.y, coordinate.x );
    }
    if(distance < 2*max_confidence) {
        coordinate = getObstacleCoordinates(distance, sensors_offset[SENSOR_FRONT]);
        Map.updateMapFeature(Map.OBSTACLE, coordinate.y, coordinate.x );
    }

    distance = DistanceRight.readCalibrated();
    for(int i = min_confidence; (i < distance) && (i < max_confidence); i += distance_resolution) {
        coordinate = getObstacleCoordinates(i, sensors_offset[SENSOR_RIGHT]);
        Map.updateMapFeature(Map.EXPLORED, coordinate.y, coordinate.x );
    }
    if(distance < max_confidence) {
        coordinate = getObstacleCoordinates(distance, sensors_offset[SENSOR_RIGHT]);
        Map.updateMapFeature(Map.OBSTACLE, coordinate.y, coordinate.x );
    }


    // Check RFID scanner.
    // Look inside RF_interface.h for more info.
    if( checkForRFID() ) {

        // Add card to map encoding.
        Map.updateMapFeature( Map.RFID, Pose.getY(), Pose.getX() );

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
        Map.updateMapFeature(Map.LINE, Pose.getY(), Pose.getX() );
    }

    Map.updateMapFeature(Map.VISITED,Pose.getY(),Pose.getX());
}

Point_t getObstacleCoordinates(float distance, float orientation_offset) {
    Point_t coordinate;

    distance += 80;
    coordinate.x = distance * cos(Pose.getThetaRadians() + orientation_offset);
    coordinate.x += Pose.getX();
    coordinate.y = distance * sin(Pose.getThetaRadians() + orientation_offset);
    coordinate.y += Pose.getY();

    return coordinate;
}
