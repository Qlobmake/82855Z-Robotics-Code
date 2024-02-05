#include "main.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#define M_PI 3.14159265358979323846

uint32_t pidStartTime;


pros::Motor leftBottom(12, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_ROTATIONS); //middle
pros::Motor leftBack(13, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_ROTATIONS); // Make left side stronger
pros::Motor leftTop(2, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_ROTATIONS); //front
pros::Motor rightBottom(19, pros::E_MOTOR_GEAR_BLUE, false, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor rightBack(17, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_ROTATIONS);
pros::Motor rightTop(9, pros::E_MOTOR_GEAR_BLUE, true, pros::E_MOTOR_ENCODER_ROTATIONS);


pros::Motor_Group leftDrive({leftBottom, leftBack, leftTop});
pros::Motor_Group rightDrive({rightBottom, rightBack, rightTop});


pros::Motor Intake(10);
pros::Rotation Rotationsensor(19);
pros::Motor catapultMotor(18, pros::E_MOTOR_GEARSET_18, true,
                        pros::E_MOTOR_ENCODER_DEGREES);
pros::Controller MasterController(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut RW('A', false);
pros::ADIDigitalOut LW('H', false);
pros::Imu imu_sensor(17);
pros::Rotation PIDRotationSensor(50); 
pros::ADIDigitalOut blockerUp('B', false);
pros::ADIDigitalOut blockerDown('C', false);



//bool variableA = true;
//bool variableB = true;
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
const int DESIRED_SHOTS = 50;
bool spam = false;



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

    pros::delay(1000);

}


// PID constants
double KP = 7.5;
double KD = 1.4;


// turn PID variables
double turnKP = 1.0;
double turnKD = 0.5;


// Error in position for drive straight PID
double error, prevError, errorRate, velocity;


void resetTimer() {
    pidStartTime = pros::millis();
}

bool isTimeLimitExceeded(int timeLimit) {
    return (pros::millis() - pidStartTime) > timeLimit;
}



double getPosition() {
  return 3.25*M_PI*leftBottom.get_position()*(3.0/5.0);
}


void drivePID(double distance, int timeLimit = 6000) {
    // reset motors
    leftDrive.tare_position();
    rightDrive.tare_position();


    error = distance;
    prevError = error;
  
    resetTimer();

    // Continue the loop while drive PID is enabled
    while (std::abs(error) > 0.5 && !isTimeLimitExceeded(timeLimit)) {
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

       if (isTimeLimitExceeded(timeLimit)) {
            break;  // Break out of the loop if the time limit is exceeded
        }


        pros::delay(20);
    }


    leftDrive = 0;
    rightDrive = 0;
}


void turnPID(double degrees, double scaling = 1.0, int timeLimit = 6000) { //, double timeout = -1) {
    imu_sensor.tare_heading();
    error = degrees;
    prevError = error;


    double startTime = pros::millis();

    double averageHeading, heading1;
    
    resetTimer();
    
    while (std::abs(error) > 8 || leftBack.get_actual_velocity() > 15 || rightBack.get_actual_velocity() > 15 && !isTimeLimitExceeded(timeLimit)) {
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
        
        if (isTimeLimitExceeded(timeLimit)) {
            break;  // Break out of the loop if the time limit is exceeded
        }
        
        
        pros::delay(20);
    }
    leftDrive = 0;
    rightDrive = 0;
}


void autonomous()
{
displayStuff();

 leftDrive.move(127);
 rightDrive.move(127);

    
 pros::delay(1000); //might not be able to hard code cause unpredicable 

 leftDrive.move(0);
 rightDrive.move(0);

resetTimer();
drivePID(-7);
resetTimer();
turnPID(45, 1); 
resetTimer();
drivePID(-5);


int shotsFired = 0;
for (int i = 0; i < DESIRED_SHOTS; ++i) {
        // Fire the catapult
        catapultMotor.move(0.7 * 127);
        pros::delay(500);  
        
        // Stop the catapult
        catapultMotor.move(0);

    
        shotsFired++;
    }

 if (shotsFired >= DESIRED_SHOTS) {
       
        catapultMotor.move_absolute(REST_POSITION, 127);
            
        pros::delay(1000);  // Adjust this delay based on the time it takes for the catapult to reset
        
        // Stop the catapult
        catapultMotor.move(0);
    }

resetTimer();
drivePID(8);
resetTimer();
turnPID(90, 1);
resetTimer();
drivePID(5);
resetTimer();
turnPID(-25, 1);
resetTimer();
drivePID(37);
resetTimer();
turnPID(-10, 1);


LW.set_value(true);
RW.set_value(true);

leftDrive.move(127);
rightDrive.move(127);
    
pros::delay(1000); //might not be able to hard code cause unpredicable 

leftDrive.move(0);
rightDrive.move(0);
resetTimer();
drivePID(-10);


leftDrive.move(127);
rightDrive.move(127);

pros::delay(300);

leftDrive.move(0);
rightDrive.move(0);

resetTimer();
drivePID(-10);
resetTimer();
turnPID(-98, 1);

LW.set_value(false);
RW.set_value(false);

resetTimer();
drivePID(20);
resetTimer();
turnPID(40, 1);
resetTimer();
drivePID(5);
resetTimer();
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


}





void opcontrol()
{
    int drivePower;
    int turnPower;

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

        if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            // Blocker up
            blockerUp.set_value(true);
            blockerDown.set_value(false);
        } else if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            // Blocker down
            blockerUp.set_value(false);
            blockerDown.set_value(false); // Set this to true for hang 
        } else {
            // No blocker command, keep both solenoids off
            blockerUp.set_value(false);
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
            catapultMotor.move(0.7 *127);
        } else {
        if (catacurrent == true && cataprevious == false)
        {
            cataOn = !cataOn;
             
        }
        cataprevious = catacurrent;




        if (abs(REST_POSITION - Rotationsensor.get_position()) > 500 || MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            catapultMotor.move(0.7 * 127);
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






