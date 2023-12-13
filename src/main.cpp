#include "main.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cmath>
// use sensor so when prev is no longer pressed reset cata positions to 0 so it
// can intake more stuff


#define M_PI 3.14159265358979323846


pros::Motor leftBottom(3, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor leftBack(1, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_ROTATIONS); // Make left side stronger
pros::Motor leftTop(2, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor rightBottom(18, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor rightBack(12, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor rightTop(20, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_ROTATIONS);


pros::Motor_Group leftDrive({leftBottom, leftBack, leftTop});
pros::Motor_Group rightDrive({rightBottom, rightBack, rightTop});


pros::Motor Intake(11);
pros::Rotation Rotationsensor(19);
pros::Motor catapultMotor(8, pros::E_MOTOR_GEARSET_18, true,
                        pros::E_MOTOR_ENCODER_DEGREES);
pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut RW('A', false);
pros::ADIDigitalOut LW('H', false);
pros::Imu imu_sensor(17);




bool variableA = true;
bool variableB = true;
bool wingsOpenLeft = false;
bool rightWingsOpen = false;
bool leftstate = true;
bool leftcurrent = true;
bool leftprevious = true;
bool rightstate = true;
bool rightcurrent = true;
bool rightprevious = true;
bool cataOn = false;
bool cataprevious = true;
bool catacurrent = true;
bool catacontrol;
bool prevPressed = false;
const int REST_POSITION = 25700;
bool spam = false;


// PID constants
double KP = 7.5;
double KD = 1.4;


// turn PID variables
double turnKP = 1.0;
double turnKD = 0.5;


// Error in position for drive straight PID
double error, prevError, errorRate, velocity;




// Flag to indicate whether drive sensors should be reset || Maybe resets too quickly
bool resetDriveSensors = true;


double getPosition() {
  return 3.25*M_PI*leftBottom.get_position()*(3.0/5.0);
}




// Function to control the drive straight and turning using PID
void drivePID(double distance) {
    // reset motors
    leftDrive.tare_position();
    rightDrive.tare_position();


    error = distance;
    prevError = error;


    // Continue the loop while drive PID is enabled
    while (std::abs(error) > 0.5) {
        // Calculate average position and its derivative for drive straight PID
        error = distance - getPosition();
        errorRate = error - prevError;




        velocity = KP * error + KD * errorRate;


        leftDrive = velocity;
        rightDrive = velocity;


        leftDrive.move(velocity);
        rightDrive.move(velocity);


        pros::lcd::print(1, "error: %f", error);
     pros::lcd::print(2, "error rate: %f", errorRate);
     pros::lcd::print(3, "velocity: %f", velocity);


        prevError = error;


        pros::delay(20);
    }


    leftDrive = 0;
    rightDrive = 0;
}


void turnPID(double degrees, double scaling = 1.0) { //, double timeout = -1) {
    imu_sensor.tare_heading();
    error = degrees;
    prevError = error;


    double startTime = pros::millis();


    double averageHeading, heading1;
    while (std::abs(error) > 8 || leftBack.get_actual_velocity() > 15 || rightBack.get_actual_velocity() > 15) {
        // distance subtracted by the average of the four ground motors
        heading1 = imu_sensor.get_heading();
        if (heading1 > 180) heading1 -= 360;


        averageHeading = (heading1);
        error = std::abs(degrees - averageHeading);
        errorRate = error - prevError;
       
        // using angular velocity instead of linear velocity
        velocity = turnKP * error + turnKD * errorRate;


        // if degrees is positive, turn right
        if (degrees > 0) {
            leftDrive = velocity*scaling;
            rightDrive = -velocity*scaling;
        // if degrees is negative, turn right
        } else {
            leftDrive = -velocity*scaling;
            rightDrive = velocity*scaling;
        }
            //if (timeout != -1) {
            //if (pros::millis() > (startTime + timeout)) {
                //break;
                //}
            //}
        prevError = error;
        pros::delay(20);
    }
    leftDrive = 0;
    rightDrive = 0;
}


void autonomous()
{
     drivePID(-8);
     Intake.move(-127);
     pros::delay(250);
     drivePID(26);
     turnPID(-35, 1);
     LW.set_value(true);
     drivePID(26);
     pros::delay(60);
     LW.set_value(false);
     pros::delay(200);
     turnPID(120, 0.8);
     Intake.move(127);
     pros::delay(300);
     Intake.move(0);
     turnPID(-160, 0.7);
     leftDrive.move(127);
     rightDrive.move(127);
     pros::delay(500);
     leftDrive.move(0);
     rightDrive.move(0);

   
   
   /* drivePID(-5);
    turnPID(150, 0.8);
    drivePID(6);
    Intake.move(-127);
    pros::delay(150);
    drivePID(-5);
    turnPID(100, 1);
    drivePID(37); */
  
}




void opcontrol()
{
    int drivePower;
    int turnPower;
    pros::delay(5);


    std::cout << "hello world";




    while (true)
    {
        pros::screen::print(TEXT_MEDIUM, 1, "ANGLE: %f", Rotationsensor.get_position());
        drivePower = MasterController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        turnPower = MasterController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        leftstate = MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        rightstate = MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        catacurrent = MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        catacontrol = MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        leftBottom.move(1.5 * (drivePower + turnPower));
        leftBack.move(1.5 * (drivePower + turnPower));
        leftTop.move(1.5 * (drivePower + turnPower));
        rightBottom.move(drivePower - turnPower);
        rightTop.move(drivePower - turnPower);
        rightBack.move(drivePower - turnPower);




        // wing L
        if (leftstate == true && leftprevious == false)
        {
            wingsOpenLeft = !wingsOpenLeft;
            LW.set_value(wingsOpenLeft);
        }
        leftprevious = leftstate;




        // wing R
        if (rightstate == true && rightprevious == false)
        {
            rightWingsOpen = !rightWingsOpen;
            RW.set_value(rightWingsOpen);
        }
        rightprevious = rightstate;




        // Intake
        if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
        {
            Intake.move(127);
        }
        else if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_B))
        {
            Intake.move(-127);
        }
        else
        {
            Intake.move(0);
        }




        // Catapult
        if (spam) {
            catapultMotor.move(127);
        } else {
        if (catacurrent == true && cataprevious == false)
        {
            cataOn = !cataOn;
             
        }
        cataprevious = catacurrent;




        if (abs(REST_POSITION - Rotationsensor.get_position()) > 500 || MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            catapultMotor.move(127);
        }
        else
        {
            catapultMotor.move(0);
        }
        }




        if ( MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        if (spam) catapultMotor.move(0);
            spam = !spam;
        }
        pros::delay(10);
    }
}





