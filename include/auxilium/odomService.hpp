/**
 * @file odomService.hpp
 * @author Jeremiah Nairn
 * @ingroup cpp-auxilium-template
 *
 * @brief Contains headers for a variety of classes that are used to
 * create odometry control systems for a variety of different physical
 * odometry mechanisms
 */

#ifndef ODOMSERVICE_HPP
#define ODOMSERVICE_HPP

//include neccessary PROS libraries
#include "pros/imu.hpp"
#include "pros/motors.hpp"

/**
 * @class odomFoundation
 * @brief Base class for odometry services, contains common functions and variables
 * that carry over to all odometry implementations in this library
 */
class odomFoundation {
    protected:
        //robot position pointers
        double* robotX;
        double* robotY;
        double* robotTheta;

        double GEARRATIO;
        double WHEELDIAMETER;
        
        /**
         * @brief Constructor for the odomFoundation class
         * @param x Pointer to the robot's X position variable
         * @param y Pointer to the robot's Y position variable
         * @param theta Pointer to the robot's Theta (heading) position variable
         */
        odomFoundation(double* x, double* y, double* theta, 
                        double gearRatio, double wheelDiameter);

        /**
         * @brief Virtual function that updates the robot's 
         * position based on odometry calculations from the
         * odometry system that the base class is used in.
         * @param x The new X position of the robot
         * @param y The new Y position of the robot
         * @param theta The new Theta (heading) position of the robot
         */
        void updatePosition(double x, double y, double theta); 
};

/**
 * @class encoder2imu1ODOM
 * @brief Class for odometry using 2 encoders and 1 imu sensor for position tracking,
 * common odometry setup for tank drive bots that allows for somewhat accurate position tracking (its better than nothing)
 */
 class encoder2imu1ODOM : public odomFoundation {
    private:
        //declare the encoders and imu
        pros::Imu& imuSensor;
        pros::Motor& leftEncoder;
        pros::Motor& rightEncoder;

        //previous encoder values
        double previousLeft;
        double previousRight;

        //total distance traveled
        double* totalDistance;

    public:
        /**
         * @brief Constructor for the encoder2imu1ODOM class
         * @param leftMotorEnc Reference to the left encoder object
         * @param rightMotorEnc Reference to the right encoder object
         * @param imu Reference to the IMU sensor object
         * @param x Pointer to the robot's X position variable
         * @param y Pointer to the robot's Y position variable
         * @param theta Pointer to the robot's Theta (heading) position variable
         */
        encoder2imu1ODOM(pros::Motor& leftMotorEnc, pros::Motor& rightMotorEnc, pros::Imu& imu,
                         double* x, double* y, double* theta, double* totalDistance,
                         double gearRatio, double wheelDiameter);

         /**
         * @brief Function that runs the odometry calculations and updates the robot's position
         * based on encoder and imu readings. This function should be called in a loop to continuously
         * update the robot's position and ensure accuracy.
         */
        void calculate();

};


#endif // ODOMSERVICE_HPP