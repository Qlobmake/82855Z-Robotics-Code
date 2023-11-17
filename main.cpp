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


// Veriable setting setup
double KP = 0.3;
double KI = 0.0;
double KD = 0.05;
double turnKP = 0.0;
double turnKI = 0.0;
double turnKD = 0.0;
double error; // Positional value, the diff of what you have and what you want
double prevError; // Position of delay __ ago
double derivative;
double totalError = 0;
double turnError; // Positional value, the diff of what you have and what you want
double turnPrevError; // Position of delay __ ago
double turnDerivative;
double turnTotalError = 0;
bool resetDriveSensors = true;
// // Auton settings
// int desiredValue = 200;
// int desiredTurnValue = 0;


// bool enableDrivePID = true;


int drivePID(int desiredValue, bool enableDrivePID, bool resetDriveSensors){
   
    while(enableDrivePID){
      if(std::abs(error) < 0.5){
        enableDrivePID = false;
      }
      if(resetDriveSensors){
          resetDriveSensors = false;
          leftBottom.tare_position();
          leftBack.tare_position();
          leftTop.tare_position();
          rightBottom.tare_position();
          rightBack.tare_position();
          rightTop.tare_position();
      }
        double leftBottomposition = leftBottom.get_position();
        double leftBackposition = leftBack.get_position();
        double leftTopposition = leftTop.get_position();
        double rightBottomposition = rightBottom.get_position();
        double rightBackposition = rightBack.get_position();
        double rightTopposition = rightTop.get_position();


      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      double averagePosition = (leftBottomposition);
    // Get avg of motors
      error = averagePosition - desiredValue;


      derivative = error - prevError;


      //totalError += error;


      double lMotorPower = (error * KP + derivative * KD);
    ////////////////////////////////////////////////////////////////////////////////
     
      double turnDiff = leftBottomposition + leftBackposition + leftTopposition - rightBottomposition - rightBackposition - rightTopposition;
   
      turnError = averagePosition - desiredValue;


      turnDerivative = turnError - turnPrevError;


    // turnTotalError += turnError;


      double turnMotorPower = (turnError * turnKP + turnDerivative * turnKD);
   
    /////////////////////////////////////////////////////////////////////////


      leftBottom.move((desiredValue < 0 ? -1 : 1) * (lMotorPower + turnMotorPower));
      leftBack.move((desiredValue < 0 ? -1 : 1) * (lMotorPower + turnMotorPower));
      leftTop.move((desiredValue < 0 ? -1 : 1) * (lMotorPower + turnMotorPower));
      rightBottom.move((desiredValue < 0 ? -1 : 1) * (lMotorPower - turnMotorPower));
      rightTop.move((desiredValue < 0 ? -1 : 1) * (lMotorPower - turnMotorPower));
      rightBack.move((desiredValue < 0 ? -1 : 1) * (lMotorPower - turnMotorPower));


      prevError = error;
      turnPrevError = turnError;
     
      pros::delay(50);
   
   
    }
}


void autonomous() {
    // resetDriveSensors = true;
    // enableDrivePID = true;
    // desiredValue = -1000;  
    // double currentHeading = imu_sensor.get_rotation();  
   
    drivePID(1, true, true);  
    pros::delay(100);
    leftBottom.move(0);
    leftBack.move(0);
    leftTop.move(0);
    rightBottom.move(0);
    rightBack.move(0);
    rightTop.move(0);
    pros::delay(100);

    leftBottom.move(-127); //drives towards the net
    leftBack.move(-127);
    leftTop.move(-127);
    rightBottom.move(-127);
    rightBack.move(-127);
    rightTop.move(-127);
    pros::delay(550);

    leftBottom.move(0); //stops at the net
    leftBack.move(0);
    leftTop.move(0);
    rightBottom.move(0);
    rightBack.move(0);
    rightTop.move(0);

    Intake.move(127); // outtakes triball near goal
    pros::delay(3500);

    leftBottom.move(127); //moves slightly away from the goal
    leftBack.move(127);
    leftTop.move(127);
    rightBottom.move(127);
    rightBack.move(127);
    rightTop.move(127);
    pros::delay(225);   

    leftBottom.move(-127); // rotates so that the ramming bar is facing the net
    leftBack.move(-127);
    leftTop.move(-127);
    rightBottom.move(127);
    rightBack.move(127);
    rightTop.move(127);
    pros::delay(325);

    leftBottom.move(0); //sops moving
    leftBack.move(0);
    leftTop.move(0);
    rightBottom.move(0);
    rightBack.move(0);
    rightTop.move(0);
    pros::delay(300);

    leftBottom.move(127); //smashes ball into goal
    leftBack.move(127);
    leftTop.move(127);
    rightBottom.move(127);
    rightBack.move(127);
    rightTop.move(127);
    pros::delay(750);
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