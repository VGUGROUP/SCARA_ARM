//Variables that hold the number of stepper steps

//Variables to hold the returned data from the Delta Transform routine

long step[2] = {0, 0}; //[0]: X motor - forearm ; [1]:Y motor -arm;[3] Z motor
int _i = 0;
int A = 0;
int B = 0;
long test_step[51][2];
float arm_degree;
float forearm_degree;
long radius = PRIM_ARM_LENGTH + SEC_ARM_LENGTH;

bool isCoordinateValid(float Cartersian_coordinate[])
{
	long result = sqrt(pow(Cartersian_coordinate[0], 2) + pow(Cartersian_coordinate[1], 2));

	if (result < radius)
		return true;
	return false;
}

void scara_move()
{
	//Offset the cartesian coordinates as the zero of the arm is actually under the first pivot
	start_cart[0] = (start_cart[0] + HOME_POS_OFFSET_X);
	start_cart[1] = (start_cart[1] + HOME_POS_OFFSET_Y);
	;

	//Offset the cartesian coordinates as the zero of the arm is actually under the first pivot
	destination_cart[0] = (destination_cart[0] + HOME_POS_OFFSET_X);
	destination_cart[1] = (destination_cart[1] + HOME_POS_OFFSET_Y);

	Cartesian_to_Scara(start_cart[0], start_cart[1]);

	start_degree[0] = forearm_degree;
	start_degree[1] = arm_degree;

	Cartesian_to_Scara(destination_cart[0], destination_cart[1]);

	destination_degree[0] = forearm_degree;
	destination_degree[1] = arm_degree;

	degree_to_steps();
	step[0] = step[0] + A;
	step[1] = step[1] + B;

	A += abs(step[0]);
	B += abs(step[1]);

	set_motor_direction();
	run_motor();

	// Reset the current position values (these are the global position variables)
	start_cart[0] = destination_cart[0] - HOME_POS_OFFSET_X;
	start_cart[1] = destination_cart[1] - HOME_POS_OFFSET_Y;
}

void set_motor_direction()
{
	/*
	forearm and arm have opposite direction of movement.
	When increasing angle, arm move CW , forearm move CCW, and vice versa for the case
	of decreasing angle
	*/

	// FOREARMDIRECTION
	if (start_degree[0] < destination_degree[0])
	{
		// WRITE(X_DIR_PIN, LOW);
		step[0] = -step[0];
	}
	else
	{
		// WRITE(X_DIR_PIN, HIGH);
	}
	// ARM DIRECTION
	if (start_degree[1] < destination_degree[1])
	{
		// WRITE(Y_DIR_PIN, HIGH);
	}
	else
	{
		step[1] = -step[1];
		// WRITE(Y_DIR_PIN, LOW);
	}
}

void run_motor()
{

	WRITE(X_ENABLE_PIN, LOW); //Enable Y axis motor
	WRITE(Y_ENABLE_PIN, LOW); //Enable Y axis motor

	// stepperX.setCurrentPosition(0);
	// stepperY.setCurrentPosition(0);

	steppers.moveTo(step);
	steppers.runSpeedToPosition();
	// steppers.run();
	// long move[2] = {0, 0};
	// long moves[100][2];
	// for (int i = 0; i < 100; i++)
	// {
	// 	moves[i][0] = 55;
	// 	moves[i][1] = 55;
	// }

	// for (int i = 0; i < 100; i++)
	// {
	// 	stepperX.setCurrentPosition(0);
	// 	stepperY.setCurrentPosition(0);
	// 	steppers.moveTo(moves[i]);
	// 	steppers.runSpeedToPosition();
	// 	Serial.println(i);
	// }

	Report_Info();
}

void Cartesian_to_Scara(float x_cartesian, float y_cartesian) //Converts XYZ cartesian coordinates into Scara stepper motor steps
{
	float tophalf;	//to calculate the top half of the cosine rule equation
	float bottomhalf; //to calculate the bottom half of the cosine rule equation

	DistB = sqrt(pow(x_cartesian, 2) + pow(y_cartesian, 2));
	Theta = (atan2(y_cartesian, x_cartesian)) * 180 / Pi;
	Phi = (acos((pow(DistB, 2) + pow(PRIM_ARM_LENGTH, 2) - pow(SEC_ARM_LENGTH, 2)) / (2 * DistB * PRIM_ARM_LENGTH))) * 180 / Pi;

	//Calculate the cosine rule equation for the secondary arm angle
	tophalf = pow(SEC_ARM_LENGTH, 2) + pow(PRIM_ARM_LENGTH, 2) - pow(DistB, 2);
	bottomhalf = 2 * SEC_ARM_LENGTH * PRIM_ARM_LENGTH;

	forearm_degree = (long)(((acos(tophalf / bottomhalf)) * 180 / Pi)); //forearm
	arm_degree = (long)((Theta - Phi));									// arm
}

void degree_to_steps()
{
	//offset for parallel arm
	start_degree[0] = start_degree[0] + (destination_degree[1] - start_degree[1]);

	float x_lag_degree = destination_degree[0] - start_degree[0];
	float y_lag_degree = destination_degree[1] - start_degree[1];

	//forearm
	step[0] = abs(x_lag_degree) * x_steps_per_degree;
	//primary arm
	step[1] = abs(y_lag_degree) * y_steps_per_degree;

	// Serial.println((destination_degree[0] - start_degree[0]));
	// Serial.println((destination_degree[1] - start_degree[1]));

	// Serial.print("X -forearm  steps: ");Serial.println(step[0]);
	// Serial.print("Y - arm steps: ");Serial.println(step[1]);
}

void Report_Info() //Feeds back to the host PC information about the move
{
	// Serial.print("X: ");
	// Serial.println(start_cart[0]);
	// Serial.print("Y: ");
	// Serial.println(start_cart[1]);

	// Serial.print("X_d: ");
	// Serial.println(destination_cart[0]);
	// Serial.print("Y_d: ");
	// Serial.println(destination_cart[1]);

	// Serial.print("X_s_degree: ");
	// Serial.println(start_degree[0]);
	// Serial.print("Y_s_degree: ");
	// Serial.println(start_degree[1]);

	// Serial.print("X_d_degree: ");
	// Serial.println(destination_degree[0]);
	// Serial.print("Y_d_degree: ");
	// Serial.println(destination_degree[1]);
	// Serial.println(step[0]);
	// Serial.println(step[1]);

	Serial.println("Done");
}
