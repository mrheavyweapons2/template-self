//declare necessary includes
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/motors.hpp"

//declare the header
#include "auxilium/odomService.hpp"

//define some "magic" numbers needed for calculations
#define MILLIMETERS_PER_INCH 25.4

//constructor for the base odomFoundation class
odomFoundation::odomFoundation(double* x, double* y, double* theta) {
    //initialize the position pointers
    robotX = x;
    robotY = y;
    robotTheta = theta;
}

//update position function for odomFoundation class
void odomFoundation::updatePosition(double x, double y, double theta) {
    //update the robot position variables
    *robotX = x;
    *robotY = y;
    *robotTheta = theta;
}

//constructor for the encoder2imu1ODOM class
encoder2imu1ODOM::encoder2imu1ODOM(pros::Motor& leftMotor, pros::Motor& rightMotor,
                                   pros::Imu& imu,
                                   double* x, double* y, double* theta) 
                                   : odomFoundation(x, y, theta),
                                    leftEncoder(leftMotor),
                                    rightEncoder(rightMotor),
                                    imuSensor(imu) {}