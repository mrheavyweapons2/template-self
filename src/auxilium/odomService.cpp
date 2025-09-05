//declaring neccessary includes
//none yet

//declaring the header
#include "auxilium/odomService.hpp"

//defines
#define PI 3.14159

struct odomSetup {
	pros::Motor& leftFrontEncoder;
	pros::Motor& rightFrontEncoder;
	pros::Motor& backLeftEncoder;
	pros::Motor& backRightEncoder;
	double& robotX;
	double& robotY;
	double& robotTheta;
	double wheelDiameter;
	double gearRatio;
};

//implementation of mechanum basic odometry
void mechBasicOdom( void* &odomStruct) {
    //cast the void pointer to the odomSetup struct
    struct odomSetup* variables = (struct odomSetup*)odomStruct;

    //declare holders for the last values for the encoders
    double lastLeftFront = 0;
    double lastRightFront = 0;
    double lastBackLeft = 0;
    double lastBackRight = 0;
    //declare holders for the delta values for the encoders
    double dFL; //delta front left
    double dFR; //delta front right
    double dBL; //delta back left
    double dBR; //delta back right

    while (true) {
        //get the deltas for each of the encoders
        dFL = variables->leftFrontEncoder.get_position() - lastLeftFront;
        dFR = variables->rightFrontEncoder.get_position() - lastRightFront;
        dBL = variables->backLeftEncoder.get_position() - lastBackLeft;
        dBR = variables->backRightEncoder.get_position() - lastBackRight;

        //calculate the changes in position based on the encoder deltas
        double deltaX = (dFL + dFR + dBL + dBR);
        double deltaY = (-dFL + dFR + dBL - dBR);
        double deltaTheta = (-dFL + dFR - dBL + dBR);

        //add them to the robots position
        variables->robotX += deltaX;
        variables->robotY += deltaY;
        variables->robotTheta += deltaTheta;
        //check to keep theta within 0-360 degrees
        if (variables->robotTheta >= 360) variables->robotTheta -= 360;
        if (variables->robotTheta < 0) variables->robotTheta += 360;

        //update the last values for the next iteration
        lastLeftFront = variables->leftFrontEncoder.get_position();
        lastRightFront = variables->rightFrontEncoder.get_position();
        lastBackLeft = variables->backLeftEncoder.get_position();
        lastBackRight = variables->backRightEncoder.get_position();

        pros::delay(20); //delay for the loop
    }
}