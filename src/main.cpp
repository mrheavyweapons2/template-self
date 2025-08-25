#include "main.h"
#include "pros/motor_group.hpp"
#include <fstream>
#include <string>

//simple settings that can be changed up here instead of looking for it in the program

#define GEARSET pros::v5::MotorGears::blue //motor gearset

#define drivetrainKP 0.5 //drivetrain proportional value
#define drivetrainKI 0.01 //drivetrain integral value
#define drivetrainKD 0.1 //drivetrain derivative value

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

/**
 * basic rundown:
 * 1. create a drivetrain class, that constructs with 2 motors groups, the left and the right
 * (this is supposed to be so the drivetrain can be used on something as little as a 4 motor or as big as a 10 motor)
 * 2. create a function that takes the turn and forward values (ranging from 127 to -127) and sets the motor speeds accordingly
 * 3. insert a pid function from last years program, and integrate that into the drivetrain class
 * 4. plan more functions from there
 */
class drivetrain {
	private:
		//declare the motor groups
		pros::MotorGroup& leftMG;
		pros::MotorGroup& rightMG;

		//values for the driver control scheme
		double leftStick;
		double rightStick;

		//declare PID values
		double previousError;
		double integralSec;
		double integralLimit = 50;

		//complementary pid function from my code in 2024
		//PID values for distance (modified to be constructor based)
		double kP;
		double kI;
		double kD;

		//pid function that takes the setpoint and returns a desired value to set the motors to
 		double PID(double target, double current) {	
    		double error = target - current;

    		//proportional equation
    		double proportional = error * kP;
    
    		//integral equation
    		integralSec += error;
			//if integral error gets too large, change it to the max (prevents windup)
			if (integralSec > integralLimit) integralSec = integralLimit;
    		if (integralSec < -integralLimit) integralSec = -integralLimit;
    		double integral = integralSec * kI;
    
    		//derivative equation
    		double derivative = (error - previousError) * kD;
    		previousError = error;
    
    		//combine the numbers and return the output
			
			/*take together the proportional and the derivative then smash together those
			 *two different expressions with inertial to create and push out an added number
			 * 想像上のテクニック: 出力
			 */
    		double output = proportional + integral + derivative;
    
    		return output;
		}
		//small function for reseting the loop for reusability
		void resetvariables() {
			previousError = 0;
			integralSec = 0;
		}

		//function that takes the turn and forward values and sets the motor speeds accordingly
		void setVelocity(int forward, int turn) {
			//couple of if statements to hold each of the values within the -127 to 127 range
			if (forward > 127) forward = 127;
			if (forward < -127) forward = -127;
			if (turn > 127) turn = 127;
			if (turn < -127) turn = -127;
			//set the motor values
			leftMG.move(forward - turn);
			rightMG.move(forward + turn);
		}

	public:
		/**constructor for the drivetrain class that takes in the left and right motor groups, and the PID values.
		  *the first 2 parameters are the motor groups, the last 3 are the PID values
		  */
		drivetrain(pros::MotorGroup& leftMotorgroup, pros::MotorGroup& rightMotorgroup, //motor groups
					double DkP, double DkI, double DkD): //PID values
			leftMG(leftMotorgroup), rightMG(rightMotorgroup),
			kP(DkP), kI(DkI), kD(DkD) {}

		//driver control function that runs a tank drive scheme
		void driverControl(pros::Controller controller) {
			//dih drive
			leftStick = pow((controller.get_analog(ANALOG_LEFT_Y)),2)/127;
			if (controller.get_analog(ANALOG_LEFT_Y) < 0) leftStick = -leftStick;
			rightStick = pow((controller.get_analog(ANALOG_RIGHT_Y)),2)/127;
			if (controller.get_analog(ANALOG_RIGHT_Y) < 0) rightStick = -rightStick;
			leftMG.move(leftStick);
			rightMG.move(rightStick);
		}
			

		
		

};


//declare the motor groups
pros::MotorGroup leftMG({1, -2, 3,-4},GEARSET);
pros::MotorGroup rightMG({-5, 6, -7,8},GEARSET);
//declare the drivetrain
drivetrain dt(leftMG, rightMG,		    //declaring the motor groups
				 drivetrainKP, drivetrainKI, drivetrainKD);	//declaring PID values


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