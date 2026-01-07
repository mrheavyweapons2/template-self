//main file for the robot program

//include PROS and other necessary libraries
#include "main.h"
#include "pros/motor_group.hpp"
#include <string>

//include auxilium implementation
#include "auxilium/drivetrainService.hpp"
#include "auxilium/liasonService.hpp"


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
pros::MotorGroup leftMG({1,-2,3},GEARSET);
pros::MotorGroup rightMG({-4, 9,-10},GEARSET);
//declare the drivetrain from drivetrainService.hpp
tankDrivetrain tankDrive(leftMG, rightMG, //motor groups
						   drivetrainMKP, drivetrainMKI, drivetrainMKD, //forward pid values
						   drivetrainTKP, drivetrainTKI, drivetrainTKD, //turning pid values
						   DRIVECURVE, CURVEOFFSET, //driver control curve values
						   &robotX, &robotY, &robotTheta); //pointers for robot position



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
	//declare the master controller
	pros::v5::Controller master(pros::E_CONTROLLER_MASTER);
	//main driver control loop
	while (true) {
		//driver control
        tankDrive.driverControlTank(master);		
		//print the x y and theta of the robot
		pros::lcd::set_text(2, ("X: " + std::to_string(robotX)).c_str());
		pros::lcd::set_text(3, ("Y: " + std::to_string(robotY)).c_str());
		pros::lcd::set_text(4, ("Theta: " + std::to_string(robotTheta)).c_str());
		//delay for loop
		pros::delay(20);
	}
}