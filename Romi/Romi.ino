
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
#include "src/robotActions.h"



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Definitions.  Other definitions exist in the .h files above.                  *
 * Also ensure you check pins.h for pin/device definitions.                      *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define BAUD_RATE 115200
#define SAMPLING_TICK_PERIOD    5
#define MAX_VELOCITY    3
#define TIME_LIMIT  1800000
#define LINE_CONFIDENCE 60
#define VMAX    3
#define USE_SENSOR_FUSION   1
//#define USE_MAGNETOMETER    1     //To use magnetometer uncomment this line
//#define USE_OBSTACLE_AVOIDANCE  1

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Class Instances.                                                              *
 * This list is complete for all devices supported in this code.                 *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
Kinematics    Pose; //Kinematics class to store position and heading

LineSensor    LineLeft(LINE_LEFT_PIN); //Left line sensor
//LineSensor    LineCentre(LINE_CENTRE_PIN); //Centre line sensor
LineSensor    LineRight(LINE_RIGHT_PIN); //Right line sensor

SharpIR       DistanceFront(SHARP_IR_FRONT_PIN); //Distance sensor front
SharpIR       DistanceLeft(SHARP_IR_LEFT_PIN); //Distance sensor left
SharpIR       DistanceRight(SHARP_IR_RIGHT_PIN); //Distance sensor right
SharpIR       DistanceFront2(SHARP_IR_FRONT2_PIN); //Distance sensor front

Imu           imu;

#ifdef USE_MAGNETOMETER
Magnetometer  Mag; // Class for the magnetometer
#endif

Motor         LeftMotor(MOTOR_PWM_L, MOTOR_DIR_L);
Motor         RightMotor(MOTOR_PWM_R, MOTOR_DIR_R);

//These work for our Romi - We strongly suggest you perform your own tuning
PID           LeftSpeedControl( 10, 0.1, 1 );
PID           RightSpeedControl( 10, 0.1, 1 );
PID           HeadingControl( 9, 0, 9 );
PID           TurningControl( 1, 0, 0.7 );

Mapper        Map; //Class for representing the map

Pushbutton    ButtonB( BUTTON_B, DEFAULT_STATE_HIGH);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Global variables.                                                             *
 * These global variables are not mandatory, but are used for the example loop() *
 * routine below.                                                                *
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

 // Variables of the position goal
 Point_t goal;
 float x_error;
 float y_error;
 float orientation_error;
 float position_error;
 float last_distance_error;
 int stucked_counter = 0;
 float Ks = PI/4;
 bool obstacle_avoidance_flag = false;

 // Planning Variables
 volatile bool goal_reached = false;

 // Obstacle avoidance Variables


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
    SENSOR_FRONT2,
    SENSOR_UNKNOWN
};
const float sensors_offset[] = {PI/4, 0.20944, -PI/4, -0.20944};

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
  goal.x = Pose.getX();
  goal.y = Pose.getY();

  //Setup RFID card
  //setupRFID();

  // Calibration code
  Serial.println("Calibrating line sensors");
  //LineCentre.calibrate();
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

  for(int i = 0; i < 5; i++) {
      digitalWrite(YELLOW_LED, HIGH);
      digitalWrite(GREEN_LED, HIGH);
      delay(200);
      digitalWrite(YELLOW_LED, LOW);
      digitalWrite(GREEN_LED, LOW);
      delay(800);
  }

  // Because code flow has been blocked, we need to reset the
  // last_time variable of the PIDs, otherwise we update the
  // PID with a large time elapsed since the class was
  // initialised, which will cause a big intergral term.
  // If you don't do this, you'll see the Romi accelerate away
  // very fast!
    HeadingControl.setMax(VMAX);
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
    createTask(SensorsTask, 50);
    createTask(MappingTask, 50);
    createTask(PlanningTask, 100);
    createTask(PrintTask, 500);
    createTask(PrintMapTask, 5000);
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
#ifdef USE_SENSOR_FUSION
    Pose.updateAngularVelocity(deg2rad(imu.gz));
#endif
    Pose.predictOrientation();
#ifdef USE_SENSOR_FUSION
#ifdef USE_MAGNETOMETER
    Pose.updateOrientation(Mag.headingFiltered());
#endif
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
    DistanceFront2.read();
    //LineCentre.read();
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
    Serial.print(LineLeft.readCalibrated());
    Serial.print(", ");
    Serial.print(LineRight.readCalibrated());
    Serial.print(", ");
    Serial.print(DistanceFront2.readCalibrated());
    Serial.print(", ");
    Serial.print(DistanceRight.readCalibrated());
#ifdef USE_MAGNETOMETER
    Serial.print(", ");
    Serial.print(Mag.headingFiltered());
#endif
    Serial.print("] (");
    Serial.print(goal.x);
    Serial.print(", ");
    Serial.print(goal.y);
    Serial.print(", ");
	Serial.print(goal_reached);
    Serial.print(", ");
	Serial.print(stucked_counter);
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
    float x_error = goal.x - Pose.getX();
    float y_error = goal.y - Pose.getY();
    Point_t direction;
    float distance_error;
    float turning;
    float ahead;

    direction = obstacleAvoidanceSensors (goal.x, goal.y);
    digitalWrite(RED_LED, obstacle_avoidance_flag);

    last_distance_error = distance_error;
    if(obstacle_avoidance_flag) {
        position_error = direction.x;
        distance_error = sqrt(x_error*x_error + y_error*y_error);
        orientation_error = direction.y - Pose.getThetaRadians();
        Ks = PI;
    } else {
        position_error = sqrt(x_error*x_error + y_error*y_error);
        distance_error = position_error;
        if(position_error > 1) {
            position_error = 1;
        }
        orientation_error = atan2(y_error, x_error) - Pose.getThetaRadians();
        Ks = PI/4;
    }

    if(orientation_error < -PI ){
        orientation_error += (2 * PI);
    }
    if(orientation_error > PI){
        orientation_error -= (2 * PI);
    }

    if(last_distance_error - distance_error < 10) {
        stucked_counter++;
    }

    if((distance_error > 20) && (stucked_counter < 200)) {
        sat = min(Ks, max(-Ks, orientation_error));

        turning = TurningControl.update(orientation_error, 0);
        ahead = HeadingControl.update(position_error, 0);
        ahead *= (1 - (abs(sat) / Ks));

        left_speed_demand = ahead - turning;
        right_speed_demand = ahead + turning;
    } else {
        goal_reached = true;
        stucked_counter = 0;

        left_speed_demand = decreaseSpeed(left_speed_demand, 0.1);
        right_speed_demand = decreaseSpeed(right_speed_demand, 0.1);
    }
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
* This function iterates over a list of points or coordetates to change the
* goal position and make Romi explore the map
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void PlanningTask() {
    //int size = sizeof(points)/sizeof(Point_t);

    // Changes the goal when the current goal has reached
    if(goal_reached) {
		int x_index = Pose.getX() / (MAP_X / MAP_RESOLUTION);
		int y_index = Pose.getY() / (MAP_Y / MAP_RESOLUTION);;

        Point_t goals = move(x_index,y_index, Pose.getThetaRadians(), Map);

        goal.x = goals.x*(MAP_X / MAP_RESOLUTION) +36 ;
        goal.y = goals.y*(MAP_Y / MAP_RESOLUTION) + 36;

        if(goals.heading == 255) {
#ifdef USE_OBSTACLE_AVOIDANCE
            obstacle_avoidance_flag = true;
#endif
        } else {
            obstacle_avoidance_flag = false;
        }

        goal_reached = false;

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

    // Sensor left
    distance = DistanceLeft.readCalibrated();
    for(int i = min_confidence; (i < distance - 72) && (i < max_confidence); i += distance_resolution) {
        coordinate = getObstacleCoordinates(i, sensors_offset[SENSOR_LEFT], true);
        Map.updateMapFeature(Map.EXPLORED, coordinate.y, coordinate.x );
    }
    if(distance < max_confidence) {
        coordinate = getObstacleCoordinates(distance, sensors_offset[SENSOR_LEFT], true);
        Map.updateMapFeature(Map.OBSTACLE, coordinate.y, coordinate.x );
    }

    // Sensor front
    distance = DistanceFront.readCalibrated();
    for(int i = min_confidence; (i < distance - 72) && (i < 2*max_confidence); i += distance_resolution) {
        coordinate = getObstacleCoordinates(i, sensors_offset[SENSOR_FRONT], true);
        Map.updateMapFeature(Map.EXPLORED, coordinate.y, coordinate.x );
    }
    if(distance < 2*max_confidence) {
        coordinate = getObstacleCoordinates(distance, sensors_offset[SENSOR_FRONT], true);
        Map.updateMapFeature(Map.OBSTACLE, coordinate.y, coordinate.x );
    }

    // Sensor front 2
    distance = DistanceFront2.readCalibrated();
    for(int i = min_confidence; (i < distance) && (i < 2*max_confidence); i += distance_resolution) {
        coordinate = getObstacleCoordinates(i, sensors_offset[SENSOR_FRONT2], true);
        Map.updateMapFeature(Map.EXPLORED, coordinate.y, coordinate.x );
    }
    if(distance < 2*max_confidence) {
        coordinate = getObstacleCoordinates(distance, sensors_offset[SENSOR_FRONT2], true);
        Map.updateMapFeature(Map.OBSTACLE, coordinate.y, coordinate.x );
    }

    // Right sensor
    distance = DistanceRight.readCalibrated();
    for(int i = min_confidence; (i < distance - 72) && (i < max_confidence); i += distance_resolution) {
        coordinate = getObstacleCoordinates(i, sensors_offset[SENSOR_RIGHT], true);
        Map.updateMapFeature(Map.EXPLORED, coordinate.y, coordinate.x );
    }
    if(distance < max_confidence) {
        coordinate = getObstacleCoordinates(distance, sensors_offset[SENSOR_RIGHT], true);
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
    if((LineLeft.readCalibrated() > LINE_CONFIDENCE) || (LineRight.readCalibrated() > LINE_CONFIDENCE)) {
        Map.updateMapFeature(Map.LINE, Pose.getY(), Pose.getX() );
    }

    Map.updateMapFeature(Map.VISITED,Pose.getY(),Pose.getX());
}

Point_t getObstacleCoordinates(float distance, float orientation_offset, bool mapp) {

    Point_t coordinate;
    distance += 80;
    coordinate.x = distance * cos(Pose.getThetaRadians() + orientation_offset);
    coordinate.y = distance * sin(Pose.getThetaRadians() + orientation_offset);

    if (mapp) {
      coordinate.x += Pose.getX();
      coordinate.y += Pose.getY();
    }

    return coordinate;
}



Point_t obstacleAvoidanceSensors(float x_goal, float y_goal){
    Point_t result;
    const float Kg = 0.01; //This can be updated to achieve good obstacle avoidance response
    const float Ko = 10000; //This can be updated to achieve good obstacle avoidance response

    //Initialise total resultant forces
    float Fx_total = 0;
    float Fy_total = 0;
    float Fx_temp = 0;
    float Fy_temp = 0;

    //Calculate attractive force from the goal
    float x_error = x_goal - Pose.getX();
    float y_error = y_goal - Pose.getY();
    float Fgoal = Kg*sqrt(sq(x_error)+sq(y_error));
    float alpha_goal = atan2(y_error,x_error);
    Fx_temp = Fgoal*cos(alpha_goal);
    Fy_temp = Fgoal*sin(alpha_goal);

    Fx_total += Fx_temp;
    Fy_total += Fy_temp;

    //Calculate repulsive force from the obstacles
    float Fres_obs_x = 0; //Resultant force due to obstacles in x
    float Fres_obs_y = 0; //Resultant force due to obstacles in y
    //Read distance sensors
    float dfront = DistanceFront.readCalibrated();
    float dfront2 = DistanceFront2.readCalibrated();
    float dleft = DistanceLeft.readCalibrated();
    float dright = DistanceRight.readCalibrated();

    Point_t  obs_coord;
    float dist_x;
    float dist_y;
    float Fobs;
    float alpha_obs;
    float Fx_obs;
    float Fy_obs;

    //Checks if obstacle is too close. Triggered by front sensor only
    if (dfront< 120){
      //Force due to Front sensor
      obs_coord = getObstacleCoordinates(dfront, sensors_offset[SENSOR_FRONT],false);
      dist_x = - obs_coord.x;
      dist_y = - obs_coord.y;
      Fobs = Ko/sqrt(sq(dist_x)+sq(dist_y));
      alpha_obs = atan2(dist_y,dist_x);
      Fx_obs = Fobs*cos(alpha_obs);
      Fy_obs = Fobs*sin(alpha_obs);
      Fres_obs_x += Fx_obs;
      Fres_obs_y += Fy_obs;
    }

    //Checks if obstacle is too close. Triggered by front sensor only
    if (dfront2< 120){
      //Force due to Front sensor
      obs_coord = getObstacleCoordinates(dfront, sensors_offset[SENSOR_FRONT2],false);
      dist_x = - obs_coord.x;
      dist_y = - obs_coord.y;
      Fobs = Ko/sqrt(sq(dist_x)+sq(dist_y));
      alpha_obs = atan2(dist_y,dist_x);
      Fx_obs = Fobs*cos(alpha_obs);
      Fy_obs = Fobs*sin(alpha_obs);
      Fres_obs_x += Fx_obs;
      Fres_obs_y += Fy_obs;
    }

    //obs_x = obs_coord.x;
    //obs_y = obs_coord.y;

    if (dright < 150){
      //Force due to right sensor
        obs_coord = getObstacleCoordinates(dright, sensors_offset[SENSOR_RIGHT],false);
        dist_x = - obs_coord.x;
        dist_y = - obs_coord.y;
        Fobs = Ko/sqrt(sq(dist_x)+sq(dist_y));
        alpha_obs = atan2(dist_y,dist_x);
        Fx_obs = Fobs*cos(alpha_obs);
        Fy_obs = Fobs*sin(alpha_obs);
        Fres_obs_x += Fx_obs;
        Fres_obs_y += Fy_obs;
    }

    if (dleft < 150){
      //Force due to left sensor
      obs_coord = getObstacleCoordinates(dleft, sensors_offset[SENSOR_LEFT],false);
      dist_x = - obs_coord.x;
      dist_y = - obs_coord.y;
      Fobs = Ko/sqrt(sq(dist_x)+sq(dist_y));
      alpha_obs = atan2(dist_y,dist_x);
      Fx_obs = Fobs*cos(alpha_obs);
      Fy_obs = Fobs*sin(alpha_obs);
      Fres_obs_x += Fx_obs;
      Fres_obs_y += Fy_obs;

    }

    //Calculate Resultant Force applied on the robot
    Fx_total += Fres_obs_x;
    Fy_total += Fres_obs_y;

    //Calculate POLAR COORDINATES
    float angle = atan2(Fy_total,Fx_total);
    float F = sqrt(sq(Fx_total)+sq(Fy_total));
    if(F > 1) {
        F = 1;
    }

    result.x = F;
    result.y = angle;

    return result;
}

float decreaseSpeed(float value, float amount) {
    float sign = 0;

    if(left_speed_demand > 0) {
        sign = 1;
    }
    if(left_speed_demand < 0) {
        sign = -1;
    }

    value = abs(value) - amount;
    if(value < 0) {
        value = 0;
    }
    return value * sign;
}

void PrintMapTask() {
    Map.printMap();
}
