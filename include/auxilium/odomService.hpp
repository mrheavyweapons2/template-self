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
//none yet


/**
 * @class odomFrame
 * @brief Superclass that handles a couple function declarations, as well as
 * private variables that are shared between the different odometry
 * subclasses. This class should not be used directly, as it does not hold
 * any implementation of odometry control.
 */
 class odomFrame {
    protected:
        //declare any shared variables or functions for odometry here

    public:
        //constructor for the superclass
        odomFrame() {}

};

class mechanumBasicOdom : public odomFrame {
    private:
        //declare any private variables or functions for mechanum odometry here

    public:
        //constructor for mechanum odometry
        mechanumBasicOdom() : odomFrame() {}

        //declare any public functions for mechanum odometry here
};


#endif // ODOMSERVICE_HPP