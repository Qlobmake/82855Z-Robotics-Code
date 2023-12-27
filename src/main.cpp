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
pros::Rotation PIDRotationSensor(50); 
// Re code PID using IMU and inersha sensor avg. 
// Gear Ratio for base bot also changed 


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
bool killswitch = false;

double calculatePosition() {
    double imuHeading = imu_sensor.get_rotation(); // Use IMU for heading
    double rotationPosition = Rotationsensor.get_position(); // Use rotational sensor for position
    return (imuHeading + rotationPosition) / 2.0;
}




// Function to control the drive straight and turning using PID
void drivePID(double distance, int timeOut = 10000) {
    // reset motors
    leftDrive.tare_position();
    rightDrive.tare_position();

    error = distance;
    prevError = error;
    double startTime = pros::millis();

    // Continue the loop while drive PID is enabled
    while (std::abs(error) > 0.5 && ((pros::millis() - startTime) < timeOut)) {
        // Calculate average position and its derivative for drive straight PID
        error = distance - calculatePosition();
        errorRate = error - prevError;

        velocity = KP * error + KD * errorRate;

        leftDrive = velocity;
        rightDrive = velocity;

        leftDrive.move(velocity);
        rightDrive.move(velocity);

        prevError = error;

        pros::delay(20);
    }

    leftDrive = 0;
    rightDrive = 0;
}

void turnPID(double degrees, double scaling = 1.0, int timeOut = 10000) {
    // reset heading and rotational sensor position
    imu_sensor.tare_heading();
    PIDRotationSensor.set_position(0);

    error = degrees;
    prevError = error;

    double startTime = pros::millis();

    double averagePosition, position1;
    while (std::abs(error) > 8 || leftBack.get_actual_velocity() > 15 || rightBack.get_actual_velocity() > 15 && ((pros::millis() - startTime) < timeOut)) {
        // calculate position using IMU and rotational sensor
        position1 = calculatePosition();
        
        // update error based on the difference between desired angle and position
        averagePosition = (position1);
        error = std::abs(degrees - averagePosition);
        errorRate = error - prevError;

        // calculate turn velocity using PID control
        velocity = turnKP * error + turnKD * errorRate;

        // set motor velocities based on the sign of the degrees
        if (degrees > 0) {
            leftDrive = velocity * scaling;
            rightDrive = -velocity * scaling;
        } else {
            leftDrive = -velocity * scaling;
            rightDrive = velocity * scaling;
        }

        // update previous error for the next iteration
        prevError = error;

        // introduce a delay for the loop
        pros::delay(20);
    }

    // stop the motors after the turn is complete
    leftDrive = 0;
    rightDrive = 0;
}


void autonomous()
{
}

void displayStuff() { //Displays things

    bool leftBottomConnected = leftBottom.get_position() != 0;
    pros::lcd::print(1, "Left Bottom Connected: %s", leftBottomConnected ? "Yes" : "No");

    bool leftBackConnected = leftBack.get_position() != 0;
    pros::lcd::print(2, "Left Back Connected: %s", leftBottomConnected ? "Yes" : "No");

    bool leftTopConnected = leftTop.get_position() != 0;
    pros::lcd::print(3, "Left Top Connected: %s", leftBottomConnected ? "Yes" : "No");

    bool rightTopConnected = rightTop.get_position() != 0;
    pros::lcd::print(4, "Right Top Connected: %s", leftBottomConnected ? "Yes" : "No");
    
    bool rightBottomConnected = rightBottom.get_position() != 0;
    pros::lcd::print(5, "Right Bottom Connected: %s", leftBottomConnected ? "Yes" : "No");
    
    bool rightBackConnected = rightBack.get_position() != 0;
    pros::lcd::print(6, "Right Back Connected: %s", leftBottomConnected ? "Yes" : "No");
    
    pros::lcd::print(7, "Left Bottom Temp: %d", leftBottom.get_temperature());
    pros::lcd::print(8, "Left Back Temp: %d", leftBack.get_temperature());
    pros::lcd::print(9, "Left Top Temp: %d", leftTop.get_temperature());
    pros::lcd::print(10, "Right Bottom Temp: %d", rightBottom.get_temperature());
    pros::lcd::print(11, "Right Back Temp: %d", rightBack.get_temperature());
    pros::lcd::print(12, "Right Top Temp: %d", rightTop.get_temperature());
}


void opcontrol()
{
    int drivePower;
    int turnPower;
    pros::delay(5);


    while (true)
    {

       displayStuff();
      
       
     
       
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






