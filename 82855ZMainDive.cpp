#include "main.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
// use sesnor so when prev is no longer pressed reset cata positions to 0 so it
// can intake more stuff


pros::Motor leftBottom(3, true);
pros::Motor leftBack(2); // Make left side stronger
pros::Motor leftTop(1, true);
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


void cata_thread()
{
    while (true)
    {
        pros::lcd::print(1, "ANGLE %f", Rotationsensor.get_position());
    }
}


void initalize()
{
    // pros::Task cata_monitor([] {cata_thread();});
    pros::screen::set_pen(COLOR_RED);
}


void autonomous() {}


void opcontrol()
{
    int drivePower;
    int turnPower;
    pros::delay(50);


    while (true)
    {
        pros::screen::print(TEXT_MEDIUM, 1, "ANGLE: %f", Rotationsensor.get_position());
        drivePower = MasterController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        turnPower = MasterController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        leftstate = MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        rightstate = MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        catacurrent = MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        catacontrol = MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1);


        leftBottom.move(drivePower + turnPower);
        leftBack.move(drivePower + turnPower);
        leftTop.move(drivePower + turnPower);
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





