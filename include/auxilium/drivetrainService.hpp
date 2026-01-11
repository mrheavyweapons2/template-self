/**
 * @file drivetrainService.hpp
 * @author Jeremiah Nairn
 * @ingroup cpp-auxilium-template
 *
 * @brief Contains headers for drivertrain classes that can be used to simplify
 * drivetrain control in a robot program. They are designed to be
 * flexible enough to be used on drivetrains of various sizes, motor counts,
 * and other miscellaneous preferences.
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

		//pid loop variables
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

		//robot pose pointers
		double* x;
		double* y;
		double* theta;
		double* totalDistance;

        //pid function that takes the setpoint and returns a desired value to set the motors to
		//returns the value to set the motors to
        double PID(double kP, double kI, double kD, double target, double current, 
			double previousError, double integralSec);

        //small function for reseting the loop for reusability
		void resetvariables();

		//constructor that takes the PID values and drive curve values
		driveFrame(double MkP, double MkI, double MkD, //movement PID values
				   double TkP, double TkI, double TkD, //turning PID values
				   int driverCurve, int driverOffset, //driver exponential curve values
				   double* x, double* y, double* theta, double* totalDistance); // robot pose variables
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
		double leftStick;
		double rightStick;

		//variables for when to cut off autonomous functions
		double autoTurnMin;
		double autoDriveMin;

		//drivetrain deadzone variables 
		//(motor voltage below this value is unable to move the robot)
		double forwardDeadzone;
		double turnDeadzone;

        //function that takes the turn and forward values and sets the motor speeds accordingly
		void setVelocity(int forward, int turn);

	public:
		//constructor that takes the motor groups and the PID values
		tankDrivetrain(pros::MotorGroup& leftMotorgroup, pros::MotorGroup& rightMotorgroup,
                       double MkP, double MkI, double MkD,
                       double TkP, double TkI, double TkD,
					   double forwardDeadzone, double turnDeadzone,
                       double autoTurnMin, double autoDriveMin, 
					   int driverCurve, int driverOffset,
                       double* x, double* y, double* theta, double* totalDistance); 

		/**
		 * driver control function that takes a controller input and
		 * sets the motor speeds accordingly.
		 * @param controller The controller object to read driver input from.
		 */
		void driverControlTank(pros::v5::Controller& controller);

		/**
		 * the second control function that takes a controller input and
		 * sets the motor speeds accordingly (does arcade instead of tank drive).
		 * @param controller The controller object to read driver input from.
		 */
		void driverControlArcade(pros::v5::Controller& controller);

		/**
		 * the second control function that takes a controller input and
		 * sets the motor speeds accordingly (does arcade instead of tank drive).
		 * @param controller The controller object to read driver input from.
		 */
		void driverControlArcadeNoET(pros::v5::Controller& controller);

		//the autonomous functions come after this

		/**
		 * autonomous function that returns true/false based on if the robot is moving
		 * @return true if the robot is currently moving, false if it is not
		 */
		bool isStopped();

		/**
		 * autonomous function that stops the robot using brake mode
		 */
		void autoStopBrake();

		/**
		 * autonomous function that drives the robot forward a set distance
		 * @param distance The distance to drive forward in inches
		 * @param maxSpeed The maximum speed to drive at (0-127),
		 * @param lockHeading optional parameter that locks the robots heading during the drive
		 * @param precise optional parameter that enables a more precise stopping method
		 */
		void autoDriveDistance(double distance, double maxSpeed, bool lockHeading = false, bool precise = true);

		/**
		 * autonomous function that turns the robot to a set angle
		 * @param angle The angle to turn to in degrees (0-360)
		 * @param maxSpeed The maximum speed to turn at (0-127)
		 * @param precise optional parameter that enables a more precise stopping method
		 */
		void autoTurnToHeading(double angle, double maxSpeed, bool precise = true);

		/**
		 * autonomous function that turns the robot to face a point
		 * @param targetX the x coordinate of the target point
		 * @param targetY the y coordinate of the target point
		 * @param maxSpeed the maximum speed to turn at (0-127)
		 * @param precise optional parameter that enables a more precise stopping method
		 */
		 void autoTurntoPoint(double targetX, double targetY, double maxSpeed, bool precise = true);

		 /**
		  * autonomous function that drives the robot to a point
		  * @param targetX the x coordinate of the target point
		  * @param targetY the y coordinate of the target point
		  * @param maxSpeed the maximum speed to drive at (0-127)
		  * @param turnFirst whether to turn or to drive while turning
		  * (note: setting this to true will significantly increase odom drift if not accounted for)
		  * @param turnModifier the modifier to apply to turning speed while driving
		  * @param precise optional parameter that enables a more precise stopping method
		  */
		  void autoDriveToPoint(double targetX, double targetY, double maxSpeed, 
								  bool turnFirst= true, double turnModifier = 1.0, bool precise = true);


};

/**
 * @class odomDrivetrain
 * @brief Class that handles omni directional control schemes, which use four motors (or motor groups)
 * mounted on each corner of the robot to control movement. This class
 * provides flexible control for omni directional drive robots, supporting various motor configurations
 * and PID tuning for precise movement and turning.
 * @code
 * odomDrivetrain yourDrivetrain(frontLeftMG, frontRightMG, backLeftMG, backRightMG,
 *                pidMovementKP, pidMovementKI, pidMovementKD,
 *                pidTurnKP, pidTurnKI, pidTurnKD,
 *                driveCurve, curveOffset,
 *                &robotX, &robotY, &robotTheta);
 * @endcode
 */
class omniDrivetrain : public driveFrame {
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
    	omniDrivetrain(pros::MotorGroup& frontLeftMotorgroup, pros::MotorGroup& frontRightMotorgroup,
                	pros::MotorGroup& backLeftMotorgroup, pros::MotorGroup& backRightMotorgroup,
                	double MkP, double MkI, double MkD,
                	double TkP, double TkI, double TkD,
                	int driverCurve, int driverOffset,
                	double* x, double* y, double* theta, double* totalDistance);

		/**
		 * Driver control function that takes a controller input and
		 * sets the motor speeds accordingly.
		 * @code
		 * pros::Controller master(pros::E_CONTROLLER_MASTER);
		 * myDrivetrain.driverControl(master);
		 * @endcode
		 */
    	void driverControl(pros::v5::Controller& controller);
};

#endif //DRIVETRAIN_HPP