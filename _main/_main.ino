



#include "fastio.h"        // map pin for mega uno and ramps 1.4
#include "Configuration.h"
#include "Arduino.h"       //Needed for Ramps 1.4
#include "fastio.h"
#include "math.h"

#include <AccelStepper.h>
#include <MultiStepper.h>

extern "C" void __cxa_pure_virtual();


//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G28 - Home all Axis



//Stepper Movement Variables

char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};

float start_cart[NUM_AXIS] = {0, 0, 0, 0};    //Holds the X Y Z cartesian coordinates of the current position of the tool head
float predicted_cart[NUM_AXIS] = {0, 0, 0, 0};           //Holds the predicted X Y Z cartesian coordinates of the tool head at any point in time
float destination_cart[NUM_AXIS] = {0, 0, 0, 0};         //Holds the X Y Z cartesian coordinates of the destination position of the tool head

long start_degree[NUM_AXIS] = {0, 0, 0, 0};                //Holds the start position for the SCARA MOVE in stepper steps (4th value not yet used)
long move_degree[NUM_AXIS] = {0, 0, 0, 0};           //Holds the predicted X Y Z SCARA coordinates of the tool head at any point in time
long destination_degree[NUM_AXIS] = {0, 0, 0, 0};               //Holds the finish position for the SCARA MOVE in stepper steps (4th value not yet used)

long gcode_N, gcode_LastN;
bool relative_mode = false;  //Determines Absolute or Relative Coordinates
bool has_homed = false;      //Whether the robot has been homed. No moves allowed until it has
int endstopstate;            //used when checking endstops


// comm variables
#define MAX_CMD_SIZE 96
#define BUFSIZE 8
char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
//bool fromsd[BUFSIZE];
int bufindr = 0;
int bufindw = 0;
int buflen = 0;
int i = 0;
char serial_char;
int serial_count = 0;
boolean comment_mode = false;
char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

AccelStepper stepperX(AccelStepper::DRIVER,X_STEP_PIN,X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER,Y_STEP_PIN,Y_DIR_PIN);
MultiStepper steppers;

void setup()
{ 
  stepperX.setMaxSpeed(800);
  stepperY.setMaxSpeed(800);

  steppers.addStepper(stepperX);
  steppers.addStepper(stepperY);

  Serial.begin(BAUDRATE);
  Serial.println("start");
//  for(int i = 0; i < BUFSIZE; i++){
//      fromsd[i] = false;
//  }
  
//Initialize Dir Pins
  SET_OUTPUT(X_DIR_PIN);
  SET_OUTPUT(Y_DIR_PIN);
  SET_OUTPUT(Z_DIR_PIN);

  
//Initialize Enable Pins - steppers default to disabled.
  SET_OUTPUT(X_ENABLE_PIN);
  WRITE(X_ENABLE_PIN,HIGH);
  SET_OUTPUT(Y_ENABLE_PIN);
  WRITE(Y_ENABLE_PIN,HIGH);
  SET_OUTPUT(Z_ENABLE_PIN);
  WRITE(Z_ENABLE_PIN,HIGH);

 
//endstops and pullup resistors

    SET_INPUT(X_MIN_PIN); 
    WRITE(X_MIN_PIN,HIGH);
    SET_INPUT(Y_MIN_PIN); 
    WRITE(Y_MIN_PIN,HIGH);
    SET_INPUT(Z_MIN_PIN); 
    WRITE(Z_MIN_PIN,HIGH);
 
    SET_INPUT(X_MIN_PIN); 
    SET_INPUT(Y_MIN_PIN); 
    SET_INPUT(Z_MIN_PIN); 

  
//Initialize Step Pins
  SET_OUTPUT(X_STEP_PIN);
  SET_OUTPUT(Y_STEP_PIN);
  SET_OUTPUT(Z_STEP_PIN);
 
    
}


void loop()
{
  if(buflen<3)
	get_command();
  
  if(buflen){


    process_commands();

    buflen = (buflen-1);
    bufindr = (bufindr + 1)%BUFSIZE;
    }

  }


  inline void get_command() 
  { 
    while( Serial.available() > 0  && buflen < BUFSIZE) {
      serial_char = Serial.read();
      if(serial_char == '\n' || serial_char == '\r' || serial_char == ':' || serial_char == ';'|| serial_count >= (MAX_CMD_SIZE - 1) ) 
      {
        if(!serial_count) return; //if empty line
        cmdbuffer[bufindw][serial_count] = 0; //terminate string
        if(!comment_mode){
  //    fromsd[bufindw] = false;
    if(strstr(cmdbuffer[bufindw], "N") != NULL)
    {
      strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
      gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
      if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer[bufindw], "M110") == NULL) ) {
        Serial.print("Serial Error: Line Number is not Last Line Number+1, Last Line:");
        Serial.println(gcode_LastN);
        //Serial.println(gcode_N);
        FlushSerialRequestResend();
        serial_count = 0;
        return;
      }
      
      if(strstr(cmdbuffer[bufindw], "*") != NULL)
      {
        byte checksum = 0;
        byte count = 0;
        while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
        strchr_pointer = strchr(cmdbuffer[bufindw], '*');
    
        if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
          Serial.print("Error: checksum mismatch, Last Line:");
          Serial.println(gcode_LastN);
          FlushSerialRequestResend();
          serial_count = 0;
          return;
        }
        //if no errors, continue parsing
      }
      else 
      {
        Serial.print("Error: No Checksum with line number, Last Line:");
        Serial.println(gcode_LastN);
        FlushSerialRequestResend();
        serial_count = 0;
        return;
      }
      
      gcode_LastN = gcode_N;
      //if no errors, continue parsing
    }
    else  // if we don't receive 'N' but still see '*'
    {
      if((strstr(cmdbuffer[bufindw], "*") != NULL))
      {
        Serial.print("Error: No Line Number with checksum, Last Line:");
        Serial.println(gcode_LastN);
        serial_count = 0;
        return;
      }
    }
    if((strstr(cmdbuffer[bufindw], "G") != NULL)){
      strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
      switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))){
      case 0:
      case 1:

          Serial.println("ok"); 
          break;
        default:
        break;
      }

    }
          bufindw = (bufindw + 1)%BUFSIZE;
          buflen += 1;
          
        }
        comment_mode = false; //for new command
        serial_count = 0; //clear buffer
      }
      else
      {
        if(serial_char == ';') comment_mode = true;
        if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
      }
    }


}


inline float code_value() { return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL)); }
inline long code_value_long() { return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10)); }
inline bool code_seen(char code_string[]) { return (strstr(cmdbuffer[bufindr], code_string) != NULL); }  //Return True if the string was found

inline bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

inline void process_commands()
{
  unsigned long codenum; //throw away variable
  char *starpos = NULL;

  if(code_seen('G'))
  {
    switch((int)code_value())
    {
//---------------------------------------------------------------------------------------------------------------------------------------------      
      case 0: // G0 -> G1
//---------------------------------------------------------------------------------------------------------------------------------------------      
      case 1: // G1 Linear move
        get_coordinates(); // For X Y Z E F
        scara_move();
        return;
        break;     
//---------------------------------------------------------------------------------------------------------------------------------------------
      case 5://endstop reportin
        Serial.print("Y min endstop: ");
        Serial.println(READ(Y_MIN_PIN));

        break;       
//---------------------------------------------------------------------------------------------------------------------------------------------
      case 6:
        	long position[2];
	        position[0] = -100;
          position[1]	= 100;

          WRITE(X_ENABLE_PIN,LOW);            //Enable Y axis motor
          WRITE(Y_ENABLE_PIN,LOW);            //Enable Y axis motor

          // Serial.print("Position[0]: "); Serial.println(stepperX.currentPosition());
          // Serial.print("Position[1]: "); Serial.println(position[1]);


          stepperX.setCurrentPosition(0);
          stepperY.setCurrentPosition(0);
          steppers.moveTo(position);
          steppers.runSpeedToPosition();  

          // for (int i = 0; i < 50; i= i +2)
          // {
          //   position[0] = position[0] + i;
          //   position[1] = position[1] + i;
          //   stepperX.setCurrentPosition(0);
          //   stepperY.setCurrentPosition(0);
           
          //   Serial.println("_________________________________________________________");
          //   Serial.print("Position[0]: "); Serial.println(stepperX.currentPosition());
          //   Serial.print("Position[1]: "); Serial.println(stepperY.currentPosition());
          //   Serial.print("i: ");Serial.println(i);
          //   Serial.println("_________________________________________________________");

          //   steppers.moveTo(position);
  	      //   steppers.runSpeedToPosition();  
          // }
        break;       
//---------------------------------------------------------------------------------------------------------------------------------------------        
      case 28: //G28 Home all Axis one at a time
        Serial.println("G28 - Homing Axes ");
        int lp;
        long count1;
        float Calculation;
        float prim_move;                   //reports back how much we have moved. This can be used for setting.
        float sec_move;
        
        //Home Primary Arm
        count1 = 0;
        prim_move = 0;
        endstopstate = 0;        
        WRITE(Y_ENABLE_PIN,LOW);            //Enable Y axis motor
        WRITE(Y_DIR_PIN, LOW);             //Direction of travel is towards minus end       
        do
        {
        
          WRITE(Y_STEP_PIN, HIGH);
          delayMicroseconds(100);
          WRITE(Y_STEP_PIN, LOW);
          delayMicroseconds(100); 
          endstopstate = (READ(Y_MIN_PIN));          //Read the endstop condition
          prim_move++; 
        } while (endstopstate == 1);                 //Stop moving if endstop is reached      

        //Now move back to 90 degrees straight out
        WRITE(Y_DIR_PIN, HIGH);             //Direction of travel is towards positive end
        Calculation = ( abs(Y_AXIS_HOME_ANGLE) )*y_steps_per_degree;

        count1 = (long) Calculation;  //We now have the number of steps we want to move;
        while(count1 > 0){
          WRITE(Y_STEP_PIN, HIGH);
          delayMicroseconds(100);
          WRITE(Y_STEP_PIN, LOW);
          delayMicroseconds(100);
          count1--;  
        }
      
        // Home Secondary Arm
        count1 = 0;
        sec_move = 0;
        endstopstate = 0;
        WRITE(X_ENABLE_PIN,LOW);           //Enable X axis motor
        WRITE(X_DIR_PIN, LOW);             //Direction of travel is towards minus end       
        do
        {
          WRITE(X_STEP_PIN, HIGH);
          delayMicroseconds(100);
          WRITE(X_STEP_PIN, LOW);
          delayMicroseconds(100); 
          endstopstate = (READ(X_MIN_PIN));          //Read the endstop condition
          sec_move++;
        } while (endstopstate == 1);                 //Stop moving if endstop is reached
  
        //Now move back to a 90 degree elbow
        WRITE(X_DIR_PIN, HIGH);             //Direction of travel is towards negative end
        Calculation = ( 90 )*x_steps_per_degree;
        count1 = (long) Calculation;  //We now have the number of steps we want to move;
        while(count1 > 0 ){
          WRITE(X_STEP_PIN, HIGH);
          delayMicroseconds(100);
          WRITE(X_STEP_PIN, LOW);
          delayMicroseconds(100);
          count1--;  
        }

        stepperX.setCurrentPosition(0);
        stepperY.setCurrentPosition(0);

        //Home Z axis 
        // endstopstate = 0;      
        // WRITE(Z_ENABLE_PIN,LOW);           //Enable Z axis motor
        // WRITE(Z_DIR_PIN, HIGH);            //Direction of travel is towards minus end       
        // do
        // {
        //   WRITE(Z_STEP_PIN, HIGH);
        //   delayMicroseconds(1000);
        //   WRITE(Z_STEP_PIN, LOW);
        //   delayMicroseconds(1000); 
        //   endstopstate = (READ(Z_MIN_PIN));          //Read the endstop condition 
        // } while (endstopstate != 1);                 //Stop moving if endstop is reached      

   
        //With the Primary arm pointing straight out and the secondary arm bent exactly 90 degrees we know where the end effector is
        //Set Datum position
        
        start_cart[0] = 0;       //X Cartesian coordinate is set to zero
        start_cart[1] = 0;       //Y Cartesian coordinate is set to zero
        start_cart[2] = 0;       //Z Cartesian coordinate is set to zero
        
        has_homed = true;                                //Set the flag so moves are now allowed
        Serial.println("G28 - Homing Axes Complete");
        Serial.print("Primary Arm Moved "); Serial.print(prim_move); Serial.println(" Steps ");
        Serial.print("Secondary Arm Moved "); Serial.print(sec_move); Serial.println(" Steps ");
      
        
        break;
//---------------------------------------------------------------------------------------------------------------------------------------------              
    }
  }
//---------------------------------------------------------------------------------------------------------------------------------------------
  
  
  else{
      Serial.println("Unknown command:");
      Serial.println(cmdbuffer[bufindr]);
  }
  
  ClearToSend();
      
}

void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
  Serial.flush();
  Serial.print("Resend:");
  Serial.println(gcode_LastN + 1);
  ClearToSend();
}

void ClearToSend()
{
//  previous_millis_cmd = millis();

  Serial.println("ok"); 
}

inline void get_coordinates()
{
  Serial.println("in get coordinate");                                //set the feedrate to the default feedrate in case the command hasn't a feedrate
  for(int i=0; i < NUM_AXIS; i++) {
    if(code_seen(axis_codes[i])) {
      destination_cart[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*start_cart[i];
    }
    else destination_cart[i] = start_cart[i];                                                       //Are these else lines really needed?
  }
}
