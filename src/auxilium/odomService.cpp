//declare necessary includes
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "math.h"

//declare the header
#include "auxilium/odomService.hpp"

//define some "magic" numbers needed for calculations
#define MILLIMETER_ADJUSTMENT 30.48

//constructor for the base odomFoundation class
odomFoundation::odomFoundation(double* x, double* y, double* theta, double gearRatio, double wheelDiameter) {
    //initialize the position pointers
    robotX = x;
    robotY = y;
    robotTheta = theta;
    GEARRATIO = gearRatio;
    WHEELDIAMETER = wheelDiameter;
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
                                   double* x, double* y, double* theta, double* totalDistance,
                                   double gearRatio, double wheelDiameter) 
                                   : odomFoundation(x, y, theta, gearRatio, wheelDiameter),
                                    leftEncoder(leftMotor),
                                    rightEncoder(rightMotor),
                                    imuSensor(imu),
                                    totalDistance(totalDistance),
                                    previousLeft(0), previousRight(0) {}

//calculate function for encoder2imu1ODOM class
void encoder2imu1ODOM::calculate() {
    //get the current encoder values
    double currentLeft = leftEncoder.get_position();
    double currentRight = rightEncoder.get_position();
    //calculate the change in encoder values
    double deltaLeft = currentLeft - previousLeft;
    double deltaRight = currentRight - previousRight;
    //update previous encoder values for next calculation
    previousLeft = currentLeft;
    previousRight = currentRight;
    //calculate the distance traveled by each wheel (in inches)
    double leftDistance = (deltaLeft / 360.0) * (MILLIMETER_ADJUSTMENT * GEARRATIO * WHEELDIAMETER * 3.14159);
    double rightDistance = (deltaRight / 360.0) * (MILLIMETER_ADJUSTMENT * GEARRATIO * WHEELDIAMETER * 3.14159);


    //calculate the average distance traveled
    double averageDistance = (leftDistance + rightDistance) / 2.0;
    //get the current heading from the imu (in degrees)
    double headingDegrees = imuSensor.get_heading();
    double headingRadians = headingDegrees * (3.14159 / 180.0);
    //calculate the change in position
    double deltaX = averageDistance * cos(headingRadians);
    double deltaY = averageDistance * sin(headingRadians);

    //making sure X and Y are not going to nan or inf, bugging the code
	if ((std::isnan(deltaX)) || (std::isinf(deltaX)) ) deltaX = 0;
	if ((std::isnan(deltaY)) || (std::isinf(deltaY))) deltaY = 0;
    //update the robot's position
    double newX = *robotX + deltaX;
    double newY = *robotY + deltaY;
    double newTheta = headingDegrees; // update theta with imu heading

    updatePosition(newX, newY, newTheta);
    //add to total distance traveled
    *totalDistance += averageDistance;
}

//function that offsets the robot's perceived position
void encoder2imu1ODOM::setPosition(double x, double y, double theta) {
    //update the imu
    imuSensor.set_heading(theta);
    //update the robot's position directly
    updatePosition(x, y, theta);
}