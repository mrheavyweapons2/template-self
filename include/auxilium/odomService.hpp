/**
 * @file odomService.hpp
 * @author Jeremiah Nairn
 * @ingroup cpp-auxilium-template
 *
 * @brief Contains headers for odometry classes that can be used to simplify
 * odometry control in a robot program. Embedded in this file are different
 * odometry systems and their respective algorithms.
 */

#ifndef ODOMSERVICE_HPP
#define ODOMSERVICE_HPP

//required include for the PROS system
#include "main.h"

/**
 * @struct odomSetup
 * @brief A struct that is required to build values in any Odom Control System
 * Each odom system has different requirements for what sensors, variables, and 
 * offsets that are needed for proper function. (they are defined in each functions declaration
 * @code
 * struct odomSetup {
   value valuename,
   value valuename,
   value valuename }
 */
struct odomSetup;

/**
 * @def Mechanum Drive Basic Odom
 * @brief A basic odometry system that only requires knowledge of the 4 Motors, the
 * wheel diameter and gear ratio, as well as the master values for the 3 position values.
 * A struct declaration would look like this:
 * @code 
 * struct odomSetup {
 *	pros::Motor& leftFrontEncoder;
 *	pros::Motor& rightFrontEncoder;
 *	pros::Motor& backLeftEncoder;
 *	pros::Motor& backRightEncoder;
 *	double& robotX;
 *	double& robotY;
 *	double& robotTheta;
 *	double wheelDiameter;
 *	double gearRatio;
 * };
 *
 * A Declaration looks like:
 * @code
 * pros::Task odomTask(mechBasicOdom, &odomStruct, "Odom Task");
 */
void mechBasicOdom(void* param);

#endif // ODOMSERVICE_HPP