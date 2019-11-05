#ifndef CONFIGURATION_H
#define CONFIGURATION_H

//RAMPS_V_1_4

#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
   

// This determines the communication speed of the printer
#define BAUDRATE 115200

const float Pi = 3.14159;

//SCARA Robot Variables

 // Calculate the motor steps required

//****************************** Currently 8 stepping with a belt driven Z axis
float x_steps_per_degree = 55; //(((200*8)/360)*(40/8)); // formula: ((steps per rev * stepping mode of motor)/360) * Gear ratio
float y_steps_per_degree = 55; //(((200*8)/360)*(40/8)); // formula: ((steps per rev * stepping mode of motor)/360) * Gear Ratio
float z_steps_per_mm = 40; // (200*8)/(8*5); formula: (steps per rev * stepping mode of motor) / (number of teeth on pulley * belt pitch)

const float PRIM_ARM_LENGTH = 100;      //The length of the primary arm in mm
const float SEC_ARM_LENGTH = 100;       //The length of the secondary arm in mm
const float HOME_POS_OFFSET_X = 100;      //The distance in X from the primary arm pivot to zero in cartesian coordinates (Usually same as SEC_ARM_LENGTH)
const float HOME_POS_OFFSET_Y = 100;    //The distance in Y from the primary arm pivot to zero in cartesian coordinates (Usually same as PRIM_ARM_LENGTH)

const float Y_AXIS_HOME_ANGLE = -20;    //The number of degrees when axis hits end stop during datuming  
const float X_AXIS_HOME_ANGLE = 0;      //The number of degrees when axis hits end stop during datuming
const float Z_AXIS_HOME_POS = 0;        //The number of mm when axis hits end stop during datuming

const float X_MAX = 125;                     //The primary arm can go straight out only
const float X_MIN = X_AXIS_HOME_ANGLE;       //The primary arm can bend in the same angle as the end stop
const float Y_MAX = 180;                     //The secondary arm can only go out until the end stop angle
const float Y_MIN = Y_AXIS_HOME_ANGLE;       //The minimuam angle is symmetrical to the end stop


float DistB;                     //The distance between the primary pivot and the end effector
float Theta;                     //Angle Theta
float Phi;                        //Angle Phi

// Inverting axis direction
const bool INVERT_X_DIR = true;
const bool INVERT_Y_DIR = true;
const bool INVERT_Z_DIR = true;


//// MOVEMENT SETTINGS
const int NUM_AXIS = 4; // The axis order in all axis related arrays is X, Y, Z, E

#endif
