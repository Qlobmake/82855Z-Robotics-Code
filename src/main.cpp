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

pros::Motor leftBottom(12, pros::E_MOTOR_GEAR_BLUE, true,
                       pros::E_MOTOR_ENCODER_ROTATIONS); // middle
pros::Motor
    leftBack(13, pros::E_MOTOR_GEAR_BLUE, true,
             pros::E_MOTOR_ENCODER_ROTATIONS); // Make left side stronger
pros::Motor leftTop(2, pros::E_MOTOR_GEAR_BLUE, true,
                    pros::E_MOTOR_ENCODER_ROTATIONS); // front
pros::Motor rightBottom(19, pros::E_MOTOR_GEAR_BLUE, false,
                        pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor rightBack(17, pros::E_MOTOR_GEAR_BLUE, false,
                      pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor rightTop(10, pros::E_MOTOR_GEAR_BLUE, false,
                     pros::E_MOTOR_ENCODER_ROTATIONS);
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
pros::ADIDigitalOut sideHang('H', false); // press one button to turn on/ toogle

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
bool secHang = false;
bool cataToggle = false; // Assuming you have these variables
bool prevCataToggle = false;
bool autoLower = false; // Assuming you have these variables
bool cataMan = false;


//Larger the scale factor the faster it moves or turn, larger number = lower scalling factor, smaller numbers = higher scalling factors
// PID constants
double KP = 7.5;
double KD = 1.4;

// turn PID variables
double turnKP = 1.0;
double turnKD = 0.5;

// Error in position for drive straight PID
double error, prevError, errorRate, velocity;

double getPosition() {
  return 2.75 * M_PI * leftBottom.get_position() * (36.0 / 48.0);
}

void drivePID(double distance, double scaling, int maxTimeMill) {
  // reset motors
  leftDrive.tare_position();
  rightDrive.tare_position();

  error = distance;
  prevError = error;
  double startTime = pros::millis();

  // Continue the loop while drive PID is enabled
  while (std::abs(error) > 0.6 && (pros::millis() - startTime) < maxTimeMill) {
    // Calculate average position and its derivative for drive straight PID
    error = distance - getPosition();
    errorRate = error - prevError;

    velocity = KP * error + KD * errorRate;

    // Multiply velocity by the scaling factor
    velocity *= scaling;

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

void turnPID(double degrees, double scaling, int maxTimeMill) {
    imu_sensor.tare_heading();
    error = degrees;
    prevError = error;

    double startTime = pros::millis();

    double averageHeading, heading1;

    while (std::abs(error) > 8 || std::abs(leftBack.get_actual_velocity()) > 1 ||
           std::abs(rightBack.get_actual_velocity()) > 1 &&
               (pros::millis() - startTime) < maxTimeMill) {
        // distance subtracted by the average of the four ground motors
        heading1 = imu_sensor.get_heading();
        if (heading1 > 180)
            heading1 -= 360;

        averageHeading = (heading1);
        error = std::abs(degrees - averageHeading);
        errorRate = error - prevError;

        // using angular velocity instead of linear velocity
        velocity = turnKP * error + turnKD * errorRate;

        // if degrees is positive, turn right
        if (degrees > 0) {
            leftDrive = velocity * scaling;
            rightDrive = -velocity * scaling;
            // if degrees is negative, turn left
        } else {
            leftDrive = -velocity * scaling;
            rightDrive = velocity * scaling;
        }

        prevError = error;

        if ((pros::millis() - startTime) > maxTimeMill) {
            // Timeout condition, exit the loop
            break;
        }

        pros::delay(20);
    }

    leftDrive = 0;
    rightDrive = 0;
}

void autonomous() {
//inital turn
 turnPID(-45, 1.5, 1000);
 drivePID(-25, 1, 1500);
 turnPID(47, 1.5, 1500);
 
 //smash + cata
 leftDrive.move(-127);
 rightDrive.move(-127);
 pros::delay(800);
 leftDrive.move(0);
 rightDrive.move(0);
 drivePID(11, 1, 900);
 turnPID(-55, 1.4, 900);
 drivePID(12.5, 1, 20000);
 turnPID(117.5, 1.2, 2000);
 drivePID(5, 1.5, 900);
 catapultMotor.move(100);
 pros::delay(1000);

while(true){
      if (
        Rotationsensor.get_position() < 4000) { // desired angle on pros - 360.
                                                // Angle messured in cetidegrees
    
       catapultMotor.move(127);
    } else {
       catapultMotor.move(0);
       break;
    }
}
  
   
   catapultMotor.move(0);
  
  //set the position after shooting. 
 //before run down middle
 drivePID(-8, 1,800);
 turnPID(90, 1.4, 800);
 drivePID(-9.25,1.5, 800);
 turnPID(-71, 1.3, 1500);
 
 //run down the middle
 drivePID(-42,1.3, 1500);
 turnPID(-5, 1.8, 1000);
 drivePID(-32,1.3, 1500);
 
 //turn before smash
 turnPID(-45, 1.2, 1100);
 drivePID(-25, 0.9, 3000);
 turnPID(-47.5, 1.3, 1100);
 
 //smash
 leftDrive.move(-127);
 rightDrive.move(-127);
 pros::delay(800);
 leftDrive.move(0);
 rightDrive.move(0);
 
 //back off
 drivePID(10, 0.9, 1000);
//turnPID(-18, 1.5, 1100);

 /*
 drivePID(10,1, 1100);
 drivePID(-15,1, 1200);
 drivePID(10,1, 1200 );
 turnPID(-98, 1, 1000);

 drivePID(20, 800);
 turnPID(40, 1, 800);
 drivePID(5, 800);
 turnPID(45, 1, 1000);
 LW.set_value(true);
 RW.set_value(true);
 drivePID(20, 800);
 drivePID(-10, 800); */
 

  //////////////////////////////////////////////////Auton 3 far side)
 /* drivePID(-8, 1, 1000);
  Intake.move(-127);
  pros::delay(150);
  drivePID(22, 1, 100);
  turnPID(-35, 1, 15000); 
  LW.set_value(true);
  drivePID(24, 1, 1300);
  pros::delay(50);
  LW.set_value(false);
  turnPID(-30, 1, 1500);
  drivePID(5, 0.8, 1700);
  turnPID(20, 1, 1600);
  drivePID(5, 0.8, 1500);
  drivePID(-5, 0.8, 1200);
  turnPID(150, 0.8, 1200);
  drivePID(6, 0.8, 1200);
  Intake.move(-127);
  pros::delay(150);
  drivePID(-5, 0.8, 1200);
  drivePID(-8, 0.8, 1200);
  turnPID(100, 1, 1200);
  drivePID(37, 1.2, 1200);
  turnPID(90, 1, 1200);
  LW.set_value(true);
  RW.set_value(true);
  drivePID(11, 0.8, 1200);
  turnPID(90, 1, 1200);
  drivePID(26, 1, 1200);
  drivePID(-27, 1, 1200);
  turnPID(-90, 1, 1200);
  drivePID(-30, 1.2, 1200);
*/

}

void initialize() {
    pros::lcd::initialize();
}

void opcontrol() {
  int drivePower;
  int turnPower;

  while (true) {

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

    if (push == false) {
      blockerUp.set_value(false);
    }

    if (MasterController.get_digital_new_press(
            pros::E_CONTROLLER_DIGITAL_DOWN)) {
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
    if (leftstate == true && leftprevious == false) {
      wingsOpenLeft = !wingsOpenLeft;
      LW.set_value(wingsOpenLeft);
    }
    leftprevious = leftstate;

    // wing R
    if (rightstate == true && rightprevious == false) {
      rightWingsOpen = !rightWingsOpen;
      RW.set_value(rightWingsOpen);
    }
    rightprevious = rightstate;

    if (MasterController.get_digital_new_press(
            pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      secHang = !secHang;
    }

    if (secHang) {
      sideHang.set_value(true);

    } else {
      sideHang.set_value(false);
    }

    

  if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        intakeToggle = !intakeToggle;
    }

   if (intakeToggle)
{
    Intake.move(127); // Move backward when Y is pressed
}
else if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
{
    intakeToggle = false;
    Intake.move(-127); // Move forward when B is pressed
}
else
{
    intakeToggle = false; // Reset intakeToggle when neither Y nor B is pressed
    Intake.move(0); // Stop when neither Y nor B is pressed
}

    bool newCataToggle =
        MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    cataMan = MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

    if (newCataToggle && !prevCataToggle) {
      cataToggle = !cataToggle;
    }

    prevCataToggle = newCataToggle;

    if (cataToggle || cataMan || autoLower) {
      catapultMotor.move(100);
    } else {
      catapultMotor.move(30);
    }

    pros::lcd::print(2, "CATA ROTATION: %d", Rotationsensor.get_position());

    if (
        Rotationsensor.get_position() < 4000) { // desired angle on pros - 360.
                                                // Angle messured in cetidegrees
       autoLower = true;
    } else {
      autoLower = false;
    }

    pros::delay(10);
  }
}

/* 2.75 D,450 rpm*/

// time pit