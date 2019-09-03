  //Variables that hold the number of stepper steps
    
  //Variables to hold the returned data from the Delta Transform routine

  long step[3] = {0.0,0.0,0.0}; //[0]: X motor - forearm ; [1]:Y motor -arm;[3] Z motor
  long arm_degree;
  long forearm_degree;
  long max_error;         //The maximum motor step error from true position

  float linear_distance_mm;         //The linear distance to be travelled by each move 
  
  unsigned long current_time, start_time, end_time, elapsed_time;
  float time_for_move;
  float min_time_for_move;  //used to check for overspeed of the z axis


void scara_move()
{
	start_cart[0] = start_cart[0] + HOME_POS_OFFSET_X;       //Offset the cartesian coordinates as the zero of the arm is actually under the first pivot
	start_cart[1] = start_cart[1] + HOME_POS_OFFSET_Y;
	Serial.print("X: "); Serial.println(start_cart[0]);
	Serial.print("Y: "); Serial.println(start_cart[1]);

	destination_cart[0] = destination_cart[0] + HOME_POS_OFFSET_X;       //Offset the cartesian coordinates as the zero of the arm is actually under the first pivot
	destination_cart[1] = destination_cart[1] + HOME_POS_OFFSET_Y;
	Serial.print("X_d: "); Serial.println(destination_cart[0]);
	Serial.print("Y_d: "); Serial.println(destination_cart[1]);


	Cartesian_to_Scara(start_cart[0], start_cart[1]);
  
	start_degree[0] = forearm_degree;
	start_degree[1] = arm_degree;

	Serial.print("X_s_degree: "); Serial.println(start_degree[0]);
	Serial.print("Y_s_degree: "); Serial.println(start_degree[1]);

	Cartesian_to_Scara(destination_cart[0],destination_cart[1]);

	destination_degree[0] = forearm_degree;
	destination_degree[1] = arm_degree;

	
	Serial.print("X_d_degree: "); Serial.println(destination_degree[0]);
	Serial.print("Y_d_degree: "); Serial.println(destination_degree[1]);

	Serial.print("arm: "); Serial.println(arm_degree);   
	Serial.print("forearm: "); Serial.println(forearm_degree);
	
	
	degree_to_steps();			// convert steps need to be executed from start_degree to destination_degree;
	run_motor();					//move order : arm -> forarm;

	// Reset the current position values (these are the global position variables)
	start_cart[0] = destination_cart[0] - HOME_POS_OFFSET_X;
  	start_cart[1] = destination_cart[1] - HOME_POS_OFFSET_Y;

}
void set_motor_direction(){
	if( start_degree[0] < destination_degree[0]){WRITE(X_DIR_PIN, LOW);} else{WRITE(X_DIR_PIN, HIGH);}        //Set motor direction X
   	if( start_degree[1] < destination_degree[1]) {WRITE(Y_DIR_PIN, LOW);} else{WRITE(Y_DIR_PIN, HIGH);} 
}
void run_motor(){

	set_motor_direction();
	WRITE(X_ENABLE_PIN,LOW);            //Enable Y axis motor
	WRITE(Y_ENABLE_PIN,LOW);            //Enable Y axis motor

	for(int i = step[1] ; i > 0 ; i --){
		WRITE(Y_STEP_PIN, HIGH);
		delayMicroseconds(1000);
		WRITE(Y_STEP_PIN, LOW);
		delayMicroseconds(1000); 
	}


	for(int i = step[0] ; i > 0 ; i --){
		WRITE(X_STEP_PIN, HIGH);
		delayMicroseconds(1000);
		WRITE(X_STEP_PIN, LOW);
		delayMicroseconds(1000); 
	}

	

}

void Cartesian_to_Scara(float x_cartesian, float y_cartesian) //Converts XYZ cartesian coordinates into Scara stepper motor steps
{
   float tophalf;    //to calculate the top half of the cosine rule equation
   float bottomhalf; //to calculate the bottom half of the cosine rule equation
   
    DistB =sqrt(pow(x_cartesian,2)+ pow(y_cartesian,2));
    Theta = (atan2(y_cartesian,x_cartesian))*180/Pi;
    Phi = (acos( ( pow(DistB,2) + pow(PRIM_ARM_LENGTH,2) - pow(SEC_ARM_LENGTH,2) ) / (2 * DistB * PRIM_ARM_LENGTH) ) ) *180/Pi;

	//  Serial.print("X_car: ");Serial.println(x_cartesian);
	//  Serial.print("Y_car: ");Serial.println(y_cartesian);
    Serial.print("DistB: "); Serial.println(DistB);
    Serial.print("Theta: "); Serial.println(Theta);
    Serial.print("Phi: "); Serial.println(Phi);

    //Calculate the cosine rule equation for the secondary arm angle
    tophalf = pow(SEC_ARM_LENGTH,2) + pow(PRIM_ARM_LENGTH,2)- pow(DistB,2); 
    bottomhalf = 2 * SEC_ARM_LENGTH * PRIM_ARM_LENGTH;

    
    forearm_degree = (long)(((acos( tophalf / bottomhalf)) * 180 / Pi)); //forearm
    arm_degree = (long)((Theta - Phi)); // arm 

    
}

void degree_to_steps(){
	//offset for parallel arm
	start_degree[0] = start_degree[0] + (destination_degree[1] - start_degree[1]);


	long x_lag_degree = destination_degree[0] - start_degree[0];
	long y_lag_degree = destination_degree[1] - start_degree[1];

	
	step[0] = abs( x_lag_degree ) * x_steps_per_degree;
	step[1] = abs( y_lag_degree ) * y_steps_per_degree;
	
	Serial.println((destination_degree[0] - start_degree[0]));
	Serial.println((destination_degree[1] - start_degree[1]));

	Serial.print("X -forearm  steps: ");Serial.println(step[0]);
	Serial.print("Y - arm steps: ");Serial.println(step[1]);
}

void Report_Info()         //Feeds back to the host PC information about the move
{
        Serial.println("-----------------------------------------------------------------");
        Serial.print("Linear Distance = "); Serial.print(linear_distance_mm); Serial.print(" mm   ");
        Serial.print("Time for Move = "); Serial.print(time_for_move/1000000); Serial.println(" sec");
        Serial.print("Feedrate = "); Serial.print(move_feedrate); Serial.println(" mm/sec");        
        Serial.println("");      
        
        Serial.print("Start Position");
        Serial.print("   X: "); Serial.print(start_cart[0] - HOME_POS_OFFSET_X);
        Serial.print("   Y: "); Serial.print(start_cart[1] - HOME_POS_OFFSET_Y);
	Serial.print("   Z: "); Serial.println(start_cart[2]);
/*        
        Serial.print("   Sec Angle: "); Serial.print(start_steps[0] / x_steps_per_degree);
        Serial.print("   Prim Angle: "); Serial.print(start_steps[1] / y_steps_per_degree);
	Serial.print("   Z Pos: "); Serial.println(start_steps[2] / z_steps_per_mm);
        Serial.println("");
*/        
        Serial.print("End Position "); 
        Serial.print("   X: "); Serial.print(destination_cart[0] - HOME_POS_OFFSET_X);
        Serial.print("   Y: "); Serial.print(destination_cart[1] - HOME_POS_OFFSET_Y);
	Serial.print("   Z: "); Serial.println(destination_cart[2]);
 
 /*       
        Serial.print("   Sec Angle: "); Serial.print(finish_steps[0] / x_steps_per_degree);
        Serial.print("   Prim Angle: "); Serial.print(finish_steps[1] / y_steps_per_degree);
	Serial.print("   Z Pos: "); Serial.println(finish_steps[2] / z_steps_per_mm);
        Serial.println("");
*/        
        Serial.print("Maximum Stepper error: "); Serial.print(max_error); Serial.println(" step(s) ");
        Serial.println("-----------------------------------------------------------------");
}
