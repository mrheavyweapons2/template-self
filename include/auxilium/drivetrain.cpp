//includes from pros or other necessary libraries
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include <cmath>

//header file
#include "auxilium/drivetrain.hpp"

//implementation of drivetrain methods

//constructor for the class
drivetrain::drivetrain(pros::MotorGroup& leftMotorgroup, pros::MotorGroup& rightMotorgroup, //motor groups
					   double DkP, double DkI, double DkD, //PID values
					   double driverCurve, double driverOffset) //driver exponential curve values
	//matching the constructor values to the private variables
	: leftMG(leftMotorgroup), rightMG(rightMotorgroup),
	kP(DkP), kI(DkI), kD(DkD),
	driveCurve(driverCurve), driveOffset(driverOffset),

	//initializing the rest of the variables
	leftStick(0), rightStick(0),
	previousError(0), integralSec(0), integralLimit(50)
{}

// PID function that takes the setpoint and returns a desired value to set the motors to
double drivetrain::PID(double target, double current) {	
	double error = target - current;

	// proportional equation
	double proportional = error * kP;

	// integral equation
	integralSec += error;
	// if integral error gets too large, change it to the max (prevents windup)
	if (integralSec > integralLimit) integralSec = integralLimit;
	if (integralSec < -integralLimit) integralSec = -integralLimit;
	double integral = integralSec * kI;

	// derivative equation
	double derivative = (error - previousError) * kD;
	previousError = error;

	// combine the numbers and return the output
	double output = proportional + integral + derivative;

	return output;
}

// Small function for resetting the loop for reusability
void drivetrain::resetvariables() {
	previousError = 0;
	integralSec = 0;
}

// Function that takes the turn and forward values and sets the motor speeds accordingly
void drivetrain::setVelocity(int forward, int turn) {
	// couple of if statements to hold each of the values within the -127 to 127 range
	if (forward > 127) forward = 127;
	if (forward < -127) forward = -127;
	if (turn > 127) turn = 127;
	if (turn < -127) turn = -127;
	// set the motor values
	leftMG.move(forward - turn);
	rightMG.move(forward + turn);
}

// Driver control function that runs a tank drive scheme
void drivetrain::driverControl(pros::Controller controller) {
	// dih drive
	leftStick = pow((controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)), driveCurve) / driveOffset;
	if ((controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) < 0) && (leftStick > 0)) leftStick = -leftStick;
	rightStick = pow((controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)), driveCurve) / driveOffset;
	if ((controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) < 0) && (rightStick > 0)) rightStick = -rightStick;
	leftMG.move(leftStick);
	rightMG.move(rightStick);
}