/**
 * \file drivetrain.hpp
 * \ingroup cpp-auxilium-template
 *
 * Contains headers for a drivetrain class that can be used to simplify
 * drivetrain control in a robot program. This class is designed to be
 * flexible enough to be used on drivetrains of various sizes, motor counts,
 * and other miscellaneous preferences.
 * 
 */

#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "main.h"
#include "pros/motor_group.hpp"


class drivetrain {
    
    private:
        //declare the motor groups
		pros::MotorGroup& leftMG;
		pros::MotorGroup& rightMG;

		//values for the driver control scheme
		double leftStick;
		double rightStick;

		//drive curve values
		int driveCurve;
		int driveOffset;

		//declare PID values
		double previousError;
		double integralSec;
		double integralLimit;

		//complementary pid function from my code in 2024
		//PID values for distance (modified to be constructor based)
		double kP;
		double kI;
		double kD;

        //pid function that takes the setpoint and returns a desired value to set the motors to
        double PID(double target, double current);

        //small function for reseting the loop for reusability
		void resetvariables();

        //function that takes the turn and forward values and sets the motor speeds accordingly
		void setVelocity(int forward, int turn);

	public:
		//constructor that takes the motor groups and the PID values
		drivetrain(pros::MotorGroup& leftMotorgroup, pros::MotorGroup& rightMotorgroup,
               double DkP, double DkI, double DkD,
               double driverCurve, double driverOffset);


		//driver control function that runs a tank drive scheme
		void driverControl(pros::Controller controller);

};

#endif //DRIVETRAIN_HPP