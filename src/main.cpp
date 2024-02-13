#include "main.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rotation.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#define M_PI 3.14159265358979323846


pros::Motor leftBottom(12, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_ROTATIONS); //middle
pros::Motor leftBack(13, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_ROTATIONS); // Make left side stronger
pros::Motor leftTop(2, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_ROTATIONS); //front
pros::Motor rightBottom(19, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor rightBack(17, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor rightTop(10, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor_Group leftDrive({leftBottom, leftBack, leftTop});
pros::Motor_Group rightDrive({rightBottom, rightBack, rightTop});
pros::Motor Intake(8);
pros::Rotation Rotationsensor(14, true);
pros::Motor catapultMotor(18, true);
pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut RW('C', false);
pros::ADIDigitalOut LW('A', false);
pros::Imu imu_sensor(11);
pros::ADIDigitalOut blockerUp('B', false);
pros::ADIDigitalOut blockerDown('D', false);

bool wingsOpenLeft = false;
bool rightWingsOpen = false;
bool leftstate = true;
bool leftcurrent = true;
bool leftprevious = true;
bool rightstate = true;
bool rightcurrent = true;
bool rightprevious = true;
bool cataState = false;
bool prevPressed = false;
bool spam = false;
bool push = false;
bool pull = false;
bool intakeToggle = false;

// PID constants
double KP = 7.5;
double KD = 1.4;

// turn PID variables
double turnKP = 1.0;
double turnKD = 0.5;

// Error in position for drive straight PID
double error, prevError, errorRate, velocity;

double getPosition()
{
  return 2.75*M_PI*leftBottom.get_position()*(36.0/48.0);
}

void drivePID(double distance) {
    // reset motors
    leftDrive.tare_position();
    rightDrive.tare_position();

    error = distance;
    prevError = error;
 
    // Continue the loop while drive PID is enabled
    while (std::abs(error) > 0.6) {
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
         
        prevError = error;
     
       
        pros::delay(20);
    }
    leftDrive = 0;
    rightDrive = 0;
}

void autonomous()
{
/* leftDrive.move(-127);
 rightDrive.move(-127);  
pros::delay(495); //might not be able to hard code cause unpredicable
leftDrive.move(0);
rightDrive.move(0);
pros::delay(2000);
drivePID(10);
pros::delay(2000);
turnPID(80, 1);
drivePID(8);
catapultMotor.move(127);
pros::delay(4000);
drivePID(8);
turnPID(90, 1);
drivePID(5);
turnPID(-25, 1);
drivePID(37);
turnPID(-10, 1);
LW.set_value(true);
RW.set_value(true);
leftDrive.move(127);
rightDrive.move(127);
pros::delay(1000); //might not be able to hard code cause unpredicable
leftDrive.move(0);
rightDrive.move(0);
drivePID(-10);
leftDrive.move(127);
rightDrive.move(127);
pros::delay(300);
leftDrive.move(0);
rightDrive.move(0);
drivePID(-10);
turnPID(-98, 1);
LW.set_value(false);
RW.set_value(false);
drivePID(20);
turnPID(40, 1);
drivePID(5);
turnPID(45, 1);
LW.set_value(true);
RW.set_value(true);
leftDrive.move(127);
rightDrive.move(127);
pros::delay(300);
leftDrive.move(0);
rightDrive.move(0);
drivePID(-10);
leftDrive.move(127);
rightDrive.move(127);
pros::delay(300);
leftDrive.move(0);
rightDrive.move(0);
*/
//////////////////////////////////////////////////Auton 3 far side)
  /*drivePID(-8);
  Intake.move(-127);
  pros::delay(150);
  drivePID(22);
  turnPID(-35, 1);
  LW.set_value(true);
  drivePID(24);
  pros::delay(50);
  LW.set_value(false);
  turnPID(-30, 1);
  drivePID(5);
  turnPID(20, 1);
  drivePID(5);
  drivePID(-5);
  turnPID(150, 0.8);
  drivePID(6);
  Intake.move(-127);
  pros::delay(150);
  drivePID(-5);
  drivePID(-8);
  turnPID(100, 1);
  drivePID(37);
  turnPID(90, 1);
  LW.set_value(true);
  RW.set_value(true);  
  drivePID(11);
  turnPID(90, 1);
  drivePID(26);
  drivePID(-27);
  turnPID(-90, 1);
  drivePID(-30);

*/
}

void opcontrol()
{
    int drivePower;
    int turnPower;

 while (true)
    {

        drivePower = MasterController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        turnPower = MasterController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        leftstate = MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        rightstate = MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        leftBottom.move(1 * (drivePower + turnPower));
        leftBack.move(1 * (drivePower + turnPower));
        leftTop.move(1 * (drivePower + turnPower));
        rightBottom.move(drivePower - turnPower);
        rightTop.move(drivePower - turnPower);
        rightBack.move(drivePower - turnPower);

       
        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            push = !push;
        }

        if (push == true) {
            blockerUp.set_value(true);
        }

        if (push == false){
            blockerUp.set_value(false);
        }

        if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            pull = !pull;
        }

        if (pull == true) {
            blockerDown.set_value(true);
            blockerUp.set_value(false);
        }

        if (pull == false) {
            blockerDown.set_value(false);
        }

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

 


  if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        intakeToggle = !intakeToggle;
    }


   if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
{
    intakeToggle = false;
    Intake.move(-127); // Move forward when B is pressed
}
else if (intakeToggle)
{
    Intake.move(127); 
}
else
{
    intakeToggle = false; // Reset intakeToggle when neither Y nor B is pressed
    Intake.move(0); // Stop when neither Y nor B is pressed
}

        if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            cataState = !cataState;
            pros::delay(200);
       
        // If L1 is toggled off, reset catapult position
    if (!cataState) {
        
        Rotationsensor.reset_position();  // Reset the rotational sensor
      
        while (Rotationsensor.get_position() < 18000) {
            pros::delay(10);
            catapultMotor.move(127);
        }
        catapultMotor.move(0); 
    }
       
        }


      if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    catapultMotor.move(100);
}    else {
    while (Rotationsensor.get_position() < 18000) { //Place right degree 
        pros::delay(10);
        catapultMotor.move(127);
    }
    catapultMotor.move(0); 
}


    if (cataState == true)
    {
        catapultMotor.move(100);
    }


    pros::delay(10);
    }
}

/* 2.75 D,450 rpm*/

