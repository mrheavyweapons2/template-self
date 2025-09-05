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

struct odomSetup;

void mechBasicOdom( void* odomStruct);

#endif // ODOMSERVICE_HPP