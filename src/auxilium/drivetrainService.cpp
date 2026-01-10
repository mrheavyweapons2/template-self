//includes from pros or other necessary libraries
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "math.h"
#include "pros/motors.h"

//header file
#include "auxilium/drivetrainService.hpp"

//magic numbers
#define MOTOR_CUTOFF 1

//implementation of drivetrain methods

driveFrame::driveFrame(double MkP, double MkI, double MkD,
                       double TkP, double TkI, double TkD,
                       int driverCurve, int driverOffset,
                       double* x, double* y, double* theta, double* totalDistance)
    : MkP(MkP), MkI(MkI), MkD(MkD),
      TkP(TkP), TkI(TkI), TkD(TkD),
	  x(x), y(y), theta(theta), totalDistance(totalDistance),
      driveCurve(driverCurve), driveOffset(driverOffset),
      integralLimit(0)
{
    // Optionally store x, y, theta pointers if you need them as members
    // Otherwise, remove them from the parameter list if not used
}

// PID function that takes the setpoint and returns a desired value to set the motors to
double driveFrame::PID(double kP, double kI, double kD, double target, double current, 
					   double previousError, double integralSec) {	
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



//constructor for tankDrivetrain
tankDrivetrain::tankDrivetrain(pros::MotorGroup& leftMotorgroup, pros::MotorGroup& rightMotorgroup, //declare the motorgroups
                               double MkP, double MkI, double MkD, //movement PID values
                               double TkP, double TkI, double TkD, //turning PID values
							   double autoTurnMin, double autoDriveMin, //minimum distance needed for function cutoffs
                               int driverCurve, int driverOffset, //driver exponential curve values
                               double* x, double* y, double* theta, double* totalDistance) //pointers that relay the robots position
    : driveFrame(MkP, MkI, MkD, TkP, TkI, TkD, driverCurve, driverOffset, x, y, theta, totalDistance),
      leftMG(leftMotorgroup), rightMG(rightMotorgroup), //initialize the motor groups
	  autoTurnMin(autoTurnMin), autoDriveMin(autoDriveMin) {} //initialize the minimum auto cutoffs

// Function that takes the turn and forward values and sets the motor speeds accordingly
void tankDrivetrain::setVelocity(int forward, int turn) {
	// couple of if statements to hold each of the values within the -127 to 127 range
	if (forward > 127) forward = 127;
	if (forward < -127) forward = -127;
	if (turn > 127) turn = 127;
	if (turn < -127) turn = -127;
	// set the motor values
	leftMG.move(forward + turn);
	rightMG.move(forward - turn);
}

//driver control function that runs a tank drive scheme
void tankDrivetrain::driverControlTank(pros::v5::Controller& controller) {
	// dih drive
	leftStick = pow((controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)), driveCurve) / driveOffset;
	if ((controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) < 0) && (leftStick > 0)) leftStick = -leftStick;
	rightStick = pow((controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)), driveCurve) / driveOffset;
	if ((controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y) < 0) && (rightStick > 0)) rightStick = -rightStick;
	leftMG.move(leftStick);
	rightMG.move(rightStick);
}

//driver control function that runs an arcade drive scheme
void tankDrivetrain::driverControlArcade(pros::v5::Controller& controller) {
	// arcade drive
	double forward = pow(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), driveCurve)/driveOffset;
	if (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) < 0 && (forward > 0)) forward = -forward;
	double turn = pow(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), driveCurve)/driveOffset;
	if (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) < 0 && (turn > 0)) turn = -turn;
	// set the drive velocity
	setVelocity(forward, turn);
}

void tankDrivetrain::driverControlArcadeNoET(pros::v5::Controller& controller) {
	setVelocity(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)*0.8);
}

//autonomous function that returns true/false based on if the robot is moving
bool tankDrivetrain::isStopped() {
	//check if the motors are moving
	if (fabs(leftMG.get_actual_velocity()) <= MOTOR_CUTOFF && fabs(rightMG.get_actual_velocity()) <= MOTOR_CUTOFF) {
		return true;
	} else {
		return false;
	}
}

//autonomous function that stops the robot using brake mode
void tankDrivetrain::autoStopBrake() {
	leftMG.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	rightMG.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	leftMG.move(0);
	rightMG.move(0);
}
//autonomous function that drives the robot forward a set distance
void tankDrivetrain::autoDriveDistance(double distance, double maxSpeed, bool lockHeading) {
	//get the starting values
	double distanceOffset = *totalDistance;
	double headingTarget = *theta;
	//reset the pid
	double drivePrevError = 0;
	double driveIntegralSec = 0;
	double turnPrevError = 0;
	double turnIntegralSec = 0;
	//main loop
	while (true) {
		//calculate the forward speed from the PID
		double forwardSpeed = PID(MkP, MkI, MkD, distance + distanceOffset, *totalDistance,
			 drivePrevError, driveIntegralSec);
		drivePrevError = distance + distanceOffset - *totalDistance;
		driveIntegralSec += drivePrevError;
		//cap the forward speed to the max speed
		if (forwardSpeed > maxSpeed) forwardSpeed = maxSpeed;
		if (forwardSpeed < -maxSpeed) forwardSpeed = -maxSpeed;
		//calculate the turn speed from the PID if locking heading
		double turnSpeed = 0;
		if (lockHeading) {
			if (fabs(headingTarget - *theta) > 180) {
				if (headingTarget > *theta) {
					headingTarget -= 360;
				} else {
					headingTarget += 360;
				}
			}
			//get the turn speed from the PID
			turnSpeed = PID(TkP, TkI, TkD, headingTarget, *theta, 
				turnPrevError, turnIntegralSec);
			//update previous error and integral section
			turnPrevError = headingTarget - *theta;
			turnIntegralSec += turnPrevError;
		}
		//if the distance is within the minimum threshold, break the loop
		if ((fabs((distance + distanceOffset) - *totalDistance) < autoDriveMin) && isStopped()) break;
		//set the motor speeds
		setVelocity(forwardSpeed, turnSpeed);
		//delay for loop
		pros::delay(20);
	}
	//stop the robot
	autoStopBrake();
 }

//autonomous function that turns the robot to a set angle
void tankDrivetrain::autoTurnToHeading(double angle, double maxSpeed) {
	//reset the pid
	double prevError = 0;
	double integralSec = 0;
	//main loop
	while (true) {
		//see if crossing the 0 degree point is shorter
		if (fabs(angle - *theta) > 180) {
			if (angle > *theta) {
				angle -= 360;
			} else {
				angle += 360;
			}
		}
		//calculate the turn speed from the PID
		double turnSpeed = PID(TkP, TkI, TkD, angle, *theta,
			 prevError, integralSec);
		//update previous error and integral section
		prevError = angle - *theta;
		integralSec += prevError;
		//cap the turn speed to the max speed
		if (turnSpeed > maxSpeed) turnSpeed = maxSpeed;
		if (turnSpeed < -maxSpeed) turnSpeed = -maxSpeed;
		//if the turn speed is within the minimum threshold, break the loop
		if ((fabs(angle - *theta) < autoTurnMin) && isStopped()) break;
		//set the motor speeds
		setVelocity(0, turnSpeed);
		//delay for loop
		pros::delay(20);
	}
	//stop the robot
	autoStopBrake();
}

//autonomous function that turns the robot to face a point
void tankDrivetrain::autoTurntoPoint(double targetX, double targetY, double maxSpeed) {
	//calculate the target angle
	double deltaX = targetX - *x;
	double deltaY = targetY - *y;
	double targetAngle = atan2(deltaY, deltaX) * (180.0 / M_PI);
	//call the turn to heading function
	autoTurnToHeading(targetAngle, maxSpeed);
}

//autonomous function that drives the robot to a point
void tankDrivetrain::autoDriveToPoint(double targetX, double targetY, double maxSpeed, 
									  bool turnFirst, double turnModifier) {
	//calculate the target angle and distance
	double deltaX = targetX - *x;
	double deltaY = targetY - *y;
	double targetAngle = atan2(deltaY, deltaX) * (180.0 / M_PI);
	double distanceOffset = *totalDistance;
	double targetDistance = sqrt((deltaX * deltaX) + (deltaY * deltaY));
	//if turnFirst is true, turn to the target angle first
	if (turnFirst) {
		autoTurnToHeading(targetAngle, maxSpeed);
	}
	//reset the pid
	double drivePrevError = 0;
	double driveIntegralSec = 0;
	double turnPrevError = 0;
	double turnIntegralSec = 0;
	//main loop
	while (true) {
		//calculate the forward speed from the PID
		double forwardSpeed = PID(MkP, MkI, MkD, targetDistance + distanceOffset, *totalDistance,
			 drivePrevError, driveIntegralSec);
		drivePrevError = (targetDistance + distanceOffset) - *totalDistance;
		driveIntegralSec += drivePrevError;
		//cap the forward speed to the max speed
		if (forwardSpeed > maxSpeed) forwardSpeed = maxSpeed;
		if (forwardSpeed < -maxSpeed) forwardSpeed = -maxSpeed;
		//calculate the turn speed from the PID
		if (fabs(targetAngle - *theta) > 180) {
			if (targetAngle > *theta) {
				targetAngle -= 360;
			} else {
				targetAngle += 360;
			}
		}
		double turnSpeed = PID(TkP, TkI, TkD, targetAngle, *theta, 
			turnPrevError, turnIntegralSec) * turnModifier;
		//update previous error and integral section
		turnPrevError = targetAngle - *theta;
		turnIntegralSec += turnPrevError;
		//update the delta X an Y
		deltaX = targetX - *x;
		deltaY = targetY - *y; 
		//if the distance is within the minimum threshold, break the loop
		if ((fabs(deltaX ) <= autoDriveMin) && (fabs(deltaY) <= autoDriveMin) && isStopped()) break;
		//set the motor speeds
		setVelocity(forwardSpeed, turnSpeed);
		//delay for loop
		pros::delay(20);
	}
	//stop the robot
	autoStopBrake();
}
//constructor for xDrivetrain here
omniDrivetrain::omniDrivetrain(pros::MotorGroup& frontLeftMotorgroup, pros::MotorGroup& frontRightMotorgroup, //front motor groups
                         pros::MotorGroup& backLeftMotorgroup, pros::MotorGroup& backRightMotorgroup, //rear motor groups
                         double MkP, double MkI, double MkD, //movement PID values
                         double TkP, double TkI, double TkD, //turning PID values
                         int driverCurve, int driverOffset, //driver exponential curve values
                         double* x, double* y, double* theta, double* totalDistance) //pointers that relay the robots position
    : driveFrame(MkP, MkI, MkD, TkP, TkI, TkD, driverCurve, driverOffset, x, y, theta, totalDistance),
      frontLeftMG(frontLeftMotorgroup), //initialize the motor groups
      frontRightMG(frontRightMotorgroup),
      backLeftMG(backLeftMotorgroup),
      backRightMG(backRightMotorgroup) {}

//function to control the motor velocities
void omniDrivetrain::setVelocity(double forwardVel, double strafeVel, double turnVel) {
	//if statements to keep the values within the -127 to 127 range
	if (forwardVel > 127) forwardVel = 127;
	if (forwardVel < -127) forwardVel = -127;
	if (strafeVel > 127) strafeVel = 127;
	if (strafeVel < -127) strafeVel = -127;
	if (turnVel > 127) turnVel = 127;
	if (turnVel < -127) turnVel = -127;

	//calculate the motor speeds based on how an x drive should properly work
	double frontLeftSpeed = forwardVel + strafeVel + turnVel;
	double frontRightSpeed = forwardVel - strafeVel - turnVel;
	double backLeftSpeed = forwardVel - strafeVel + turnVel;
	double backRightSpeed = forwardVel + strafeVel - turnVel;

	//set the motor speeds
	frontLeftMG.move(frontLeftSpeed);
	frontRightMG.move(frontRightSpeed);
	backLeftMG.move(backLeftSpeed);
	backRightMG.move(backRightSpeed);
}


//driver control function
void omniDrivetrain::driverControl(pros::v5::Controller& controller) {
	//take the drive code and the driveCurve and driveDivider into account
	//invert the value if the joystick reads negative
	double forward = pow(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), driveCurve)/driveOffset;
	if (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) < 0 && (forward > 0)) forward = -forward;
	double strafe = pow(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X), driveCurve)/driveOffset;
	if (controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) < 0 && (strafe > 0)) strafe = -strafe;
	double turn = pow(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), driveCurve)/driveOffset;
	if (controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) < 0 && (turn > 0)) turn = -turn;
	//set the drive velocity
	setVelocity(forward, strafe, turn);
}
