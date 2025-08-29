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
 * @class driveFrame
 * @brief Superclass that handles function declarations, as well as
 * private variables that are shared between the different drivetrain
 * subclasses. This class should not be used directly, as it does not hold
 * any implementation of drivetrain control.
 */
 class driveFrame {
	protected:
		//drive curve values
		int driveCurve;
		int driveOffset;

		//declare PID values
		double previousError;
		double integralSec;
		double integralLimit;

		//complementary pid function from my code in 2024
		//PID values for distance (modified to be constructor based)
		//movement
		double MkP;
		double MkI;
		double MkD;
		//turning
		double TkP;
		double TkI;
		double TkD;

        //pid function that takes the setpoint and returns a desired value to set the motors to
        double PID(double kP, double kI, double kD, double target, double current);

        //small function for reseting the loop for reusability
		void resetvariables();

		//constructor that takes the PID values and drive curve values
		driveFrame(double MkP, double MkI, double MkD, //movement PID values
				   double TkP, double TkI, double TkD, //turning PID values
				   int driverCurve, int driverOffset, //driver exponential curve values
				   double* x, double* y, double* theta); // robot pose variables
 };


/**
 * @class tank drivetrain
 * @brief Class that handles tank drive control schemes,
 * tank drive is the most common form of drivetrain, as well as
 * the simplest to implement. This class is designed to be
 * flexible enough to be used on drivetrains of various sizes.
 * @code
 * tankDrivetrain yourDrivetrain(leftMotorGroup, rightMotorGroup, 
 *                pidMovementKP, pidMovementKI, pidMovementKD,
 *                pidTurnKP, pidTurnKI, pidTurnKD,
 *                driveCurve, curveOffset,
 *                &robotX, &robotY, &robotTheta); 
 * @endcode
 */
class tankDrivetrain : public driveFrame {
    
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

        //function that takes the turn and forward values and sets the motor speeds accordingly
		void setVelocity(int forward, int turn);

	public:
		//constructor that takes the motor groups and the PID values
		tankDrivetrain(pros::MotorGroup& leftMotorgroup, pros::MotorGroup& rightMotorgroup,
                       double MkP, double MkI, double MkD,
                       double TkP, double TkI, double TkD,
                       int driverCurve, int driverOffset,
                       double* x, double* y, double* theta); 

		/**
		 * Driver control function that takes a controller input and
		 * sets the motor speeds accordingly.
		 * @code
		 * pros::Controller master(pros::E_CONTROLLER_MASTER);
		 * myDrivetrain.driverControl(master);
		 * @endcode
		 */
		void driverControl(pros::Controller controller);

};

/**
 * @class xDrivetrain
 * @brief Class that handles X drive control schemes, which use four motors (or motor groups) 
 * mounted at 45-degree angles to allow for omnidirectional movement. This class 
 * provides flexible control for X drive robots, supporting various motor configurations
 * and PID tuning for precise movement and turning.
 * @code
 * xDrivetrain yourDrivetrain(frontLeftMG, frontRightMG, backLeftMG, backRightMG,
 *                pidMovementKP, pidMovementKI, pidMovementKD,
 *                pidTurnKP, pidTurnKI, pidTurnKD,
 *                driveCurve, curveOffset,
 *                &robotX, &robotY, &robotTheta);
 * @endcode
 */
class xDrivetrain : public driveFrame {
	private:
		//declare the motor groups
    	pros::MotorGroup& frontLeftMG;
    	pros::MotorGroup& frontRightMG;
    	pros::MotorGroup& backLeftMG;
    	pros::MotorGroup& backRightMG;

		//declare the values for the sticks
    	double leftStick;
    	double rightStick;
    	double strafeStick;

		//function that sets the velocity
    	void setVelocity(double forward, double strafe, double turn);

	public:
    	xDrivetrain(pros::MotorGroup& frontLeftMotorgroup, pros::MotorGroup& frontRightMotorgroup,
                	pros::MotorGroup& backLeftMotorgroup, pros::MotorGroup& backRightMotorgroup,
                	double MkP, double MkI, double MkD,
                	double TkP, double TkI, double TkD,
                	int driverCurve, int driverOffset,
                	double* x, double* y, double* theta);

		/**
		 * Driver control function that takes a controller input and
		 * sets the motor speeds accordingly.
		 * @code
		 * pros::Controller master(pros::E_CONTROLLER_MASTER);
		 * myDrivetrain.driverControl(master);
		 * @endcode
		 */
    	void driverControl(pros::Controller controller);
};


#endif //DRIVETRAIN_HPP