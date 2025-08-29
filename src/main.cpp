//main file for the robot program

//include PROS and other necessary libraries
#include "main.h"
#include "pros/motor_group.hpp"
#include <fstream>
#include <string>

//include the drivetrain implementation
#include "auxilium/drivetrainService.hpp"


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

//defining position values the robots position (in X, Y, and heading)
double robotX = 0;
double robotY = 0;
double robotTheta = 0;


//class that handles logging data into a CSV file
class fileLogger {
	private:
		//holder for the file
		std::ofstream logFile;

	public:
		//constructor that opens the log file and adds the header,
		//filename example would be "/usd/logfile.csv" (you could literally copy and paste this in and it would work)
		fileLogger(std::string filename, std::string header) {
			logFile.open(filename);
			if (logFile.is_open()) {
				logFile << header << "\n"; //header
			}
		}
		//simple function that logs data to the file
		void logData(std::string data) {
			if (logFile.is_open()) {
				logFile << data << "\n"; //logged data
			}
		}


};

//SAMPLE TANK DECLARATION

//declare the motor groups
pros::MotorGroup leftMG({1, -2, 3,-4},GEARSET);
pros::MotorGroup rightMG({-5, 6, -7,8},GEARSET);
//declare the drivetrain from drivetrainService.hpp
tankDrivetrain tank(leftMG, rightMG, //pass the motor groups
                  drivetrainMKP, drivetrainMKI, drivetrainMKD, //movement PID values
                  drivetrainTKP, drivetrainTKI, drivetrainTKD, //turning PID values
                  DRIVECURVE, CURVEOFFSET, //curve values
                  &robotX, &robotY, &robotTheta); //pointers to the robots position variables


				  
//SAMPLE X DRIVE DECLARATION

//declare the motor groups
pros::MotorGroup frontLeftMG({1, -2},GEARSET);
pros::MotorGroup backLeftMG({3, -4},GEARSET);
pros::MotorGroup frontRightMG({-5, 6},GEARSET);
pros::MotorGroup backRightMG({-7, 8},GEARSET);
//declare the drivetrain from drivetrainService.hpp
xDrivetrain xDrive(frontLeftMG, backLeftMG, frontRightMG, backRightMG, //pass the motor groups
				  drivetrainMKP, drivetrainMKI, drivetrainMKD, //movement PID values
				  drivetrainTKP, drivetrainTKI, drivetrainTKD, //turning PID values
				  DRIVECURVE, CURVEOFFSET, //curve values
				  &robotX, &robotY, &robotTheta); //pointers to the robots position variables

//SAMPLE MECHANUM DRIVE

//(uses same motor groups as x drive)

//declare the drivetrain from drivetrainService.hpp
mechanumDrivetrain mechDrive(frontLeftMG, frontRightMG, backLeftMG, backRightMG, //pass the motor groups
				  drivetrainMKP, drivetrainMKI, drivetrainMKD, //movement PID values
				  drivetrainTKP, drivetrainTKI, drivetrainTKD, //turning PID values
				  DRIVECURVE, CURVEOFFSET, //curve values
				  &robotX, &robotY, &robotTheta); //pointers to the robots position variables

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
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	while (true) {
		//driver control
        tank.driverControl(master);
		pros::delay(20);
	}
}