//includes from pros or other necessary libraries
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include <cmath>

//header file
#include "auxilium/drivetrainService.hpp"

//implementation of drivetrain methods



// PID function that takes the setpoint and returns a desired value to set the motors to
double driveFrame::PID(double kP, double kI, double kD, double target, double current) {	
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
void driveFrame::resetvariables() {
	previousError = 0;
	integralSec = 0;
}

//constructor for tankDrivetrain
tankDrivetrain::tankDrivetrain(pros::MotorGroup& leftMotorgroup, pros::MotorGroup& rightMotorgroup, //declare the motorgroups
                               double MkP, double MkI, double MkD, //movement PID values
                               double TkP, double TkI, double TkD, //turning PID values
                               int driverCurve, int driverOffset, //driver exponential curve values
                               double* x, double* y, double* theta) //pointers that relay the robots position
    : driveFrame(MkP, MkI, MkD, TkP, TkI, TkD, driverCurve, driverOffset, x, y, theta),
      leftMG(leftMotorgroup), rightMG(rightMotorgroup) {} //initialize the motor groups

// Function that takes the turn and forward values and sets the motor speeds accordingly
void tankDrivetrain::setVelocity(int forward, int turn) {
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
void tankDrivetrain::driverControl(pros::Controller controller) {
	// dih drive
	leftStick = pow((controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)), driveCurve) / driveOffset;
	if ((controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) < 0) && (leftStick > 0)) leftStick = -leftStick;
	rightStick = pow((controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)), driveCurve) / driveOffset;
	if ((controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) < 0) && (rightStick > 0)) rightStick = -rightStick;
	leftMG.move(leftStick);
	rightMG.move(rightStick);
}

//class that holds x drive code
class xDrivetrain {

	private:
		//declare the motorgroups for the initialization
		pros::MotorGroup& frontLeft;
		pros::MotorGroup& frontRight;
		pros::MotorGroup& backLeft;
		pros::MotorGroup& backRight;

		//declare pid values for both driving in a direciton and turning
		double driveKp, driveKi, driveKd; //moving around
		double turnKp, turnKi, turnKd; //turning

		//declare drivecurve values
		double driveCurve, driveDivider;

		//function to control the motor velocities
		void setDrivetrainVelocity(double forwardVel, double strafeVel, double turnVel) {
			//calculate the motor speeds based on how an x drive should properly work
			double frontLeftSpeed = forwardVel + strafeVel + turnVel;
			double frontRightSpeed = forwardVel - strafeVel - turnVel;
			double backLeftSpeed = forwardVel - strafeVel + turnVel;
			double backRightSpeed = forwardVel + strafeVel - turnVel;

			//set the motor speeds
			frontLeft.move(frontLeftSpeed);
			frontRight.move(frontRightSpeed);
			backLeft.move(backLeftSpeed);
			backRight.move(backRightSpeed);
		}

	public:
		//constructor
		drivetrain(pros::MotorGroup& frontLeftMotorgroup, pros::MotorGroup& frontRightMotorgroup,
				  pros::MotorGroup& backLeftMotorgroup, pros::MotorGroup& backRightMotorgroup,
				  double DkP, double DkI, double DkD,
				  double TkP, double TkI, double TkD,
				  double driveCurve = 2, double driveOffset = 127)
			: frontLeft(frontLeftMotorgroup), frontRight(frontRightMotorgroup),
			  backLeft(backLeftMotorgroup), backRight(backRightMotorgroup),
			  driveKp(DkP), driveKi(DkI), driveKd(DkD),
			  turnKp(TkP), turnKi(TkI), turnKd(TkD),
			  driveCurve(driveCurve), driveDivider(driveOffset) {}

		//driver control function
		void driveControl(pros::Controller& controller) {
			//take the drive code and the driveCurve and driveDivider into account
			//invert the value if the joystick reads negative
			double forward = pow(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), driveCurve)/driveDivider;
			if (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) < 0 && (forward > 0)) forward = -forward;
			double strafe = pow(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), driveCurve)/driveDivider;
			if (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) < 0 && (strafe > 0)) strafe = -strafe;
			double turn = pow(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), driveCurve)/driveDivider;
			if (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) < 0 && (turn > 0)) turn = -turn;
			//set the drive velocity
			setDrivetrainVelocity(forward, strafe, turn);
		}