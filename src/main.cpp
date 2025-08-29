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

#define drivetrainKP 0.5 //drivetrain proportional value
#define drivetrainKI 0.01 //drivetrain integral value
#define drivetrainKD 0.1 //drivetrain derivative value

#define DRIVECURVE 2 //exponential curve for driver control
#define CURVEOFFSET 127 //offset for the exponential curve


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


//declare the motor groups
pros::MotorGroup leftMG({1, -2, 3,-4},GEARSET);
pros::MotorGroup rightMG({-5, 6, -7,8},GEARSET);
//declare the drivetrain from drivetrain.hpp
tankDrivetrain dt(leftMG, rightMG,		    //declaring the motor groups
				drivetrainKP, drivetrainKI, drivetrainKD, //declaring PID values
				DRIVECURVE, CURVEOFFSET); //declaring drive curve values	


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
        dt.driverControl(master);
		pros::delay(20);
	}
}