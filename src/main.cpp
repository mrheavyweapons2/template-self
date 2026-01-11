//main file for the robot program

//include PROS and other necessary libraries
#include "main.h"
#include "liblvgl/llemu.hpp"
#include "pros/motor_group.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include <string>

//include auxilium implementation
#include "auxilium/drivetrainService.hpp"
#include "auxilium/liasonService.hpp"
#include "auxilium/odomService.hpp"
#include "pros/rtos.hpp"


//simple settings that can be changed up here instead of looking for it in the program

#define GEARSET pros::v5::MotorGears::blue //motor gearset

//defining PID values for movement, as well as turning
#define drivetrainMKP 0.75 //proportional for movement
#define drivetrainMKI 0.001 //integral for movement
#define drivetrainMKD 4 //derivative for movement

#define drivetrainTKP 3.2 //proportional for turning
#define drivetrainTKI 0.001 //integral for turning
#define drivetrainTKD 11.5 //derivative for turning

#define drivetrainForwardDeadzone 6 //deadzone for forward movement
#define drivetrainTurnDeadzone 7 //deadzone for turning

#define drivetrainAutoDriveMin 20 //minimum distance (in millimeters) for the auto drive function to consider itself at the target
#define drivetrainAutoTurnMin 2 //minimum angle (in degrees) for the auto turn function to consider itself at the target

//driver control curve settings
#define DRIVECURVE 2 //exponential curve for driver control
#define CURVEOFFSET 127 //offset for the exponential curve

//odometry settings
#define WHEELDIAMETER 2.75 //wheel diameter for odometry (in inches)
#define GEARRATIO 36.0/36.0 //gear ratio for odometry (output speed / input speed)

//defining position values the robots position (in X, Y, and heading)
double robotX = 0;
double robotY = 0;
double robotTheta = 0;
double totalDistance = 0; //total distance traveled by the robot (in millimeters)

//declare the motor groups
pros::MotorGroup leftMG({-1,-2,3},GEARSET);
pros::MotorGroup rightMG({-4, 5,6},GEARSET);
//declare the drivetrain from drivetrainService.hpp
tankDrivetrain tankDrive(leftMG, rightMG, //motor groups
						   drivetrainMKP, drivetrainMKI, drivetrainMKD, //forward pid values
						   drivetrainTKP, drivetrainTKI, drivetrainTKD, //turning pid values
						   drivetrainForwardDeadzone, drivetrainTurnDeadzone, //deadzone values
						   drivetrainAutoTurnMin, drivetrainAutoDriveMin, //minimum distance values for autonomous functions
						   DRIVECURVE, CURVEOFFSET, //driver control curve values
						   &robotX, &robotY, &robotTheta, &totalDistance); //pointers for robot position

//declare odometry objects
pros::Motor leftEnc(-4);
pros::Motor rightEnc(-1);
pros::Imu imuSensor(7);

//declare the odometry system
encoder2imu1ODOM odomSystem(leftEnc, rightEnc, imuSensor,
                            &robotX, &robotY, &robotTheta, &totalDistance,
							WHEELDIAMETER, GEARRATIO);

//SAMPLE FILE LOGGER DECLARATION
fileLogger logger("/usd/logfile.csv", "Time, X, Y, Theta");

//helper function to run the odom loop
void odomLoop(void* param) {
	while (true) {
		//calculate the new position
		odomSystem.calculate();
		//delay for loop
		pros::delay(20);
	}
}

//helper function to run a print to lcd loop
void lcdLoop(void* param) {
	while (true) {
		pros::v5::Controller master(pros::E_CONTROLLER_MASTER);
		//print the x y and theta of the robot
		pros::lcd::set_text(2, ("X: " + std::to_string(robotX)).c_str());
		pros::lcd::set_text(3, ("Y: " + std::to_string(robotY)).c_str());
		pros::lcd::set_text(4, ("Theta: " + std::to_string(robotTheta)).c_str());
		//delay for loop
		pros::delay(20);
	}
}

//prebuilt function that runs as soon as the program starts
void initialize() {
	//initialize the LCD
	pros::lcd::initialize();
	//increase the IMU refresh rate
	imuSensor.set_data_rate(5);
	//create a new thread and initialize the odom loop
	pros::Task odomTask(odomLoop, (void*)"PROS", "Odom Task");
	//create a new thread and initialize the lcd loop
	pros::Task lcdTask(lcdLoop, (void*)"PROS", "LCD Task");
	
}

//prebuilt function that runs when the robot is disabled (people use this?)
void disabled() {}

//special initialization function that runs when the robot is connected to the competition controller
void competition_initialize() {}

//prebuilt that runs the autonomous code when either the field management system sets it as so or the robot is on autonomous skills mode
void autonomous() {
	//test turn here
	tankDrive.autoDriveToPoint(300,-600, 75, true, 1, false);
	tankDrive.autoDriveToPoint(1500, -600, 75,true, 1, false);
	tankDrive.autoDriveToPoint(1500, 600, 75,true, 1, false);
	tankDrive.autoDriveToPoint(300, 600, 75,true, 1, false);
	tankDrive.autoDriveToPoint(0, 0, 75);
	tankDrive.autoTurntoPoint(-300,0,75);
}

//prebuilt function that runs by default when the robot is disconnected from the field controller or is set to driver control mode
void opcontrol() {
	//declare the master controller
	pros::v5::Controller master(pros::E_CONTROLLER_MASTER);
	//main driver control loop
	while (true) {
		//driver control
		tankDrive.driverControlArcadeNoET(master);
		//delay for loop
		pros::delay(20);
	}
}