#include "main.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cmath>
// use sesnor so when prev is no longer pressed reset cata positions to 0 so it
// can intake more stuff
pros::Motor leftBottom(3, true);
pros::Motor leftBack(1, true); // Make left side stronger
pros::Motor leftTop(2);
pros::Motor rightBottom(18);
pros::Motor rightBack(12);
pros::Motor rightTop(20, true);
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


void initalize()
{
    // pros::Task cata_monitor([] {cata_thread();});
    pros::screen::set_pen(COLOR_RED);
}



// Proportional gain for drive straight PID
double KP = 0.3;

// Integral gain for drive straight PID (currently not used)
double KI = 0.0;

// Derivative gain for drive straight PID
double KD = 0.05;

// Proportional gain for turning PID (currently not used)
double turnKP = 0.0;

// Integral gain for turning PID (currently not used)
double turnKI = 0.0;

// Derivative gain for turning PID (currently not used)
double turnKD = 0.0;

// Error in position for drive straight PID
double error = 0.5;

// Previous error in position for drive straight PID
double prevError = 0.0;

// Derivative of the error for drive straight PID
double derivative = 0.0;

// Integral of the error for drive straight PID (currently not used)
double totalError = 0.0;

// Error in position for turning PID
double turnError = 0.0;

// Previous error in position for turning PID
double turnPrevError = 0.0;

// Derivative of the error for turning PID
double turnDerivative = 0.0;

// Integral of the error for turning PID (currently not used)
double turnTotalError = 0.0;

// Flag to indicate whether drive sensors should be reset || Maybe resets too quickly 
bool resetDriveSensors = true;

// Function to control the drive straight and turning using PID
double drivePID(double desiredHeading, bool enableDrivePID, bool resetDriveSensors) {
    // Ensure proper variable initialization
    double lMotorPower = 0.0;
    double turnMotorPower = 0.0;

    // Continue the loop while drive PID is enabled
    while (enableDrivePID) {
        // Check if the absolute error is less than 0.5, and disable PID if true
        if (std::abs(error) < 0.5) {
            enableDrivePID = false;
        }

        // Reset drive sensors if needed
        if (resetDriveSensors) {
            resetDriveSensors = false;
            leftBottom.tare_position();
            leftBack.tare_position();
            leftTop.tare_position();
            rightBottom.tare_position();
            rightBack.tare_position();
            rightTop.tare_position();
        }

        // Get position values from drive sensors
        double leftBottomposition = leftBottom.get_position();
        double leftBackposition = leftBack.get_position();
        double leftTopposition = leftTop.get_position();
        double rightBottomposition = rightBottom.get_position();
        double rightBackposition = rightBack.get_position();
        double rightTopposition = rightTop.get_position();

        // Calculate average position and its derivative for drive straight PID
        double averagePosition = (leftBottomposition);
        error = averagePosition - desiredHeading;
        derivative = error - prevError;

        // Calculate motor power for driving straight using PID
        lMotorPower = error * KP + derivative * KD;

        // Get heading from IMU sensor 
        double currentHeading = imu_sensor.get_heading();

        // Calculate heading error
        double headingError = desiredHeading - currentHeading;
        // Calculate error and its derivative for turning PID
        turnError = averagePosition - desiredHeading;
        turnDerivative = turnError - turnPrevError;

        // Safety check to avoid division by zero in turning PID
        if (error != 0.0) {
            turnMotorPower = headingError * turnKP + turnDerivative * turnKD;
        }

        // Motor movements based on calculated powers
        int direction = (desiredHeading < 0) ? -1 : 1;
        leftBottom.move(direction * (lMotorPower + turnMotorPower));
        leftBack.move(direction * (lMotorPower + turnMotorPower));
        leftTop.move(direction * (lMotorPower + turnMotorPower));
        rightBottom.move(direction * (lMotorPower - turnMotorPower));
        rightTop.move(direction * (lMotorPower - turnMotorPower));
        rightBack.move(direction * (lMotorPower - turnMotorPower));

        // Update previous errors for next iteration
        prevError = error;
        turnPrevError = turnError;

    }
}

void autonomous() {
    double targetDistance = 10;
    double targetRotation = 1;
    drivePID(targetDistance, true, true);  
    pros::lcd::print(1, "Error: %f", error);
    pros::lcd::print(2, "Derivative: %f", derivative);
    pros::lcd::print(3, "Motor Power: %f", lMotorPower);
    Intake.move(127);
    pros::delay(200);
    Intake.move(0);
    drivePID(targetRotation, true, true);  
    pros::lcd::print(1, "Error: %f", error);
    pros::lcd::print(2, "Derivative: %f", derivative);
    pros::lcd::print(3, "Motor Power: %f", lMotorPower);
    pros::delay(200);
}


void opcontrol()
{
    int drivePower;
    int turnPower;
    pros::delay(5);


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