/**
 * @file drivetrain.hpp
 * @author Jeremiah Nairn
 * @ingroup cpp-auxilium-template
 *
 * @brief Contains headers for a drivetrain class that can be used to simplify
 * drivetrain control in a robot program. This class is designed to be
 * flexible enough to be used on drivetrains of various sizes, motor counts,
 * and other miscellaneous preferences.
 * (before you fucking ask, yes i did copy and paste this below)
 * 
 */

#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

//required include for the PROS system
#include "pros/motor_group.hpp"

/**
 * @class drivetrain
 * @brief A class designed to simplify drivetrain control in a robot program.
 * This class is designed to be flexible enough to be used on drivetrains of
 * various sizes, motor counts, and other miscellaneous preferences. 
 */
class tankDrivetrain {
    
    private:
        /**
		 * Motor groups for the left and right sides of the drivetrain
		 * These should be initialized in main.cpp and passed to the drivetrain
		 * constructor when creating a drivetrain object.
		 * @code
		 * pros::MotorGroup leftMG({1, -2, 3});
		 * pros::MotorGroup rightMG({-4, 5, -6});
		 * //place the constructor after this
		 * @endcode
		 * This allows for flexibility in motor configuration and count, as well as
		 * makes it very easy to change motor locations and types.
		 * 
		 */
		pros::MotorGroup& leftMG;
		pros::MotorGroup& rightMG;

		//stick values for driver control (notice the drop in quality with the text? 
		// i just realized i dont need to to make long ass comments for private variables)
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
		tankDrivetrain(pros::MotorGroup& leftMotorgroup, pros::MotorGroup& rightMotorgroup,
               double DkP, double DkI, double DkD,
               double driverCurve, double driverOffset);


		//driver control function that runs a tank drive scheme
		void driverControl(pros::Controller controller);

};

#endif //DRIVETRAIN_HPP