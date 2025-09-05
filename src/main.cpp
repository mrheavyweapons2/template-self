//main file for the robot program

//include PROS and other necessary libraries
#include "main.h"
#include "pros/motor_group.hpp"
#include <string>

//include auxilium implementation
#include "auxilium/drivetrainService.hpp"
#include "auxilium/liasonService.hpp"
#include "auxilium/odomService.hpp"


//simple settings that can be changed up here instead of looking for it in the program

#define GEARSET pros::v5::MotorGears::blue //motor gearset

//defining PID values for movement, as well as turning
#define drivetrainMKP 1 //proportional for movement
#define drivetrainMKI 0.001 //integral for movement
#define drivetrainMKD 0.1 //derivative for movement

#define drivetrainTKP 1 //proportional for turning
#define drivetrainTKI 0.001 //integral for turning
#define drivetrainTKD 0.1 //derivative for turning

#define DRIVECURVE 2 //exponential curve for driver control
#define CURVEOFFSET 127 //offset for the exponential curve

#define WHEELDIAMETER 2.75 //wheel diameter for odometry (in inches)
#define GEARRATIO 36.0/36.0 //gear ratio for odometry (output speed / input speed)

//defining position values the robots position (in X, Y, and heading)
double robotX = 0;
double robotY = 0;
double robotTheta = 0;

//declare the motor groups
pros::MotorGroup frontLeftMG({1,-2,3},GEARSET);
pros::MotorGroup backLeftMG({11, -12,13},GEARSET);
pros::MotorGroup frontRightMG({-4, 9,-10},GEARSET);
pros::MotorGroup backRightMG({-17, 15,-19},GEARSET);
//declare the drivetrain from drivetrainService.hpp
omniDrivetrain mechDrive(frontLeftMG, frontRightMG, backLeftMG, backRightMG, //pass the motor groups
				  drivetrainMKP, drivetrainMKI, drivetrainMKD, //movement PID values
				  drivetrainTKP, drivetrainTKI, drivetrainTKD, //turning PID values
				  DRIVECURVE, CURVEOFFSET, //curve values
				  &robotX, &robotY, &robotTheta); //pointers to the robots position variables

//declare 1 motor from each motorgroup to function as motor encoders
pros::Motor leftFrontEncoder(1, GEARSET);
pros::Motor rightFrontEncoder(-4, GEARSET);
pros::Motor backLeftEncoder(11, GEARSET);
pros::Motor backRightEncoder(-17, GEARSET);


//declare the mechanum odometry function as a task
struct odomSetup {
	pros::Motor& leftFrontEncoder;
	pros::Motor& rightFrontEncoder;
	pros::Motor& backLeftEncoder;
	pros::Motor& backRightEncoder;
	double& robotX;
	double& robotY;
	double& robotTheta;
	double wheelDiameter;
	double gearRatio;
};

odomSetup odomStruct = {leftFrontEncoder, rightFrontEncoder, backLeftEncoder, backRightEncoder,
						robotX, robotY, robotTheta,
						WHEELDIAMETER, GEARRATIO};

pros::Task odomTask(mechBasicOdom, (void*)&odomStruct, "Odom Task");

//declare other motors
pros::Motor intakeMotorFront(-7, GEARSET);
pros::Motor intakeMotorRear(14, GEARSET);


//SAMPLE FILE LOGGER DECLARATION
fileLogger logger("/usd/logfile.csv", "Time, X, Y, Theta");


//prebuilt function that runs as soon as the program starts
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
}

//prebuilt function that runs when the robot is disabled (people use this?)
void disabled() {}

//special initialization function that runs when the robot is connected to the competition controller
void competition_initialize() {}

//prebuilt that runs the autonomous code when either the field management system sets it as so or the robot is on autonomous skills mode
void autonomous() {}

//prebuilt function that runs by default when the robot is disconnected from the field controller or is set to driver control mode
void opcontrol() {
	pros::v5::Controller master(pros::E_CONTROLLER_MASTER);

	while (true) {
		//driver control
        mechDrive.driverControl(master);
		//if trigger R1 is pressed, run the intake motors forward
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			intakeMotorFront.move(127);
			intakeMotorRear.move(127);
		} else {
			intakeMotorFront.move(0);
			intakeMotorRear.move(0);
		}
		pros::delay(20);
	}
}