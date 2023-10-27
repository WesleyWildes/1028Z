/*----------------------------------------------------------------------------*/
/* */
/* Module: main.cpp */
/* Author: Wesley.Wildes */
/* Created: 9/21/2023, 3:57:08 PM */
/* Description: Over Under Robot Code */
/* */
/*----------------------------------------------------------------------------*/


#include "vex.h"


using namespace vex;


// A global instance of competition
competition Competition;
// define your global instances of motors and other devices here


//drive Motors
vex::motor LF = vex::motor(vex::PORT9);//5.5 W 200 rpm cartridge
vex::motor LM = vex::motor(vex::PORT6);//5.5 W 200 rpm cartridge
vex::motor LB = vex::motor(vex::PORT7);//11 W 600 rpm cartridge
vex::motor RF = vex::motor(vex::PORT2);//11 W 600 rpm cartridge
vex::motor RM = vex::motor(vex::PORT5);//11 W 600 rpm cartridge
vex::motor RB = vex::motor(vex::PORT10);//11 W 600 rpm cartridge


//cata
vex::motor CataLeft = vex::motor(vex::PORT4);//11 W 100 rpm cartridge
vex::motor CataRight = vex::motor(vex::PORT3);//11 W 100 rpm cartridge


//Intake
vex::motor Intake = vex::motor(vex::PORT20);////11 W 600 rpm cartridge


//others
vex::controller Controller1 = vex::controller();
vex::brain Brain = vex::brain();


//sensors
vex::rotation Rotation = vex::rotation(vex::PORT3);
vex::rotation Rotation2 = vex::rotation(vex::PORT11);
vex::rotation Rotation3 = vex::rotation(vex::PORT12);


vex::inertial Inertial = vex::inertial(vex::PORT8);


//pnuematics
vex::pneumatics HangL = vex::pneumatics(Brain.ThreeWirePort.A);
vex::pneumatics HangR = vex::pneumatics(Brain.ThreeWirePort.B);
vex::pneumatics WingL = vex::pneumatics(Brain.ThreeWirePort.C);
vex::pneumatics WingR = vex::pneumatics(Brain.ThreeWirePort.D);


//Pid Functions
void turn(int degrees, int Speed) {
double P = 0.43; // tuning values for tuning the PID loop for smooth turns
double I = 0.0;
double D = 0.23;
double error = 0.0; // difference from where you want to go and where you are
double prevError = 0.0; // error one code cycle ago
double integral = 0.0; // adds one every time to make sure it doesn't take too long to get to the target
double derivative = 0.0; // difference between the current error and the previous error
double measuredValue = 0.0;
bool reachedTarget = false;


while (true) {
measuredValue = Inertial.rotation(vex::rotationUnits::deg);
double turnSpeed = (error * P) + (integral * I) + (derivative * D);
LF.spin(vex::directionType::rev, turnSpeed, vex::velocityUnits::pct);
LM.spin(vex::directionType::rev, turnSpeed, vex::velocityUnits::pct);
LB.spin(vex::directionType::rev, turnSpeed, vex::velocityUnits::pct);
RF.spin(vex::directionType::rev, turnSpeed, vex::velocityUnits::pct);
RM.spin(vex::directionType::rev, turnSpeed, vex::velocityUnits::pct);
RB.spin(vex::directionType::rev, turnSpeed, vex::velocityUnits::pct);
error = degrees - measuredValue;
// Limit the integral term to prevent wind-up
if (abs(error) < 5) {
integral += error;
} else {
integral = 0;
}
derivative = error - prevError;
prevError = error;


if (abs(error) < 1) {
reachedTarget = true; // Mark that we've reached the target
}


// If the error changes sign after reaching the target, break the loop
if (reachedTarget) {
break;
}


wait(20, msec);
}
}


#include <cmath>




double leftEncoder = 0.0;
double rightEncoder = 0.0;
double backEncoder = 0.0;
double leftOffset = 5.25;
double rightOffset = 5.25;
double backOffset = 5.25;
float botX = 0;
float botY = 0;
float botTheta = 0;
float botDegrees;
float botDelta = 0;
float lastX = 0;
float lastY = 0;


// OdomTest function
int OdomTest () {
while(true){
Rotation.resetPosition(); //reset rotation wheel vars
Rotation2.resetPosition();
Rotation3.resetPosition();
float radius = 0.0;
leftEncoder = Rotation.position(vex::rotationUnits::raw) * 3.25;//updat rotation wheel vars
rightEncoder = Rotation2.position(vex::rotationUnits::raw) * 3.25;
backEncoder = Rotation3.position(vex::rotationUnits::raw) * 3.25;
botTheta = (leftEncoder - rightEncoder) / (leftOffset + rightOffset);
botDegrees = botTheta * (180.0 / M_PI); //rad to deg
botDelta = (2 * (sin(botTheta / 2.0))); //delta math
botX = lastX += (botDelta * ((backEncoder / botTheta) + backOffset)) ; //finding x
botY = lastY += (botDelta * ((rightEncoder / botTheta) + rightOffset)); //finding y


botX = lastX;
botY = lastY;
wait(100, msec);
}
}


void move(int distance, int Speed) {
double P = 0.37; // tuning values for tuning the PID loop for smooth turns
double I = 0.0;
double D = 0.21;
double error = 0.0; // difference from where you want to go and where you are
double prevError = 0.0; // error one code cycle ago
double integral = 0.0; // adds one every time to make sure it doesn't take too long to get to the target
double derivative = 0.0; // difference between the current error and the previous error
double measuredValue = 0.0;
bool reachedTarget = false;
RF.setPosition(0, vex::rotationUnits::deg);
RM.setPosition(0, vex::rotationUnits::deg);
RB.setPosition(0, vex::rotationUnits::deg);
LF.setPosition(0, vex::rotationUnits::deg);
LM.setPosition(0, vex::rotationUnits::deg);
LB.setPosition(0, vex::rotationUnits::deg);




while (true) {
measuredValue = LF.position(vex::rotationUnits::deg) + RF.position(vex::rotationUnits::deg) + LM.position(vex::rotationUnits::deg) + RM.position(vex::rotationUnits::deg) + LB.position(vex::rotationUnits::deg) + RB.position(vex::rotationUnits::deg) / 6;
double turnSpeed = (error * P) + (integral * I) + (derivative * D);
if (turnSpeed > Speed) {
turnSpeed = Speed;
}


LF.spin(vex::directionType::fwd, turnSpeed, vex::velocityUnits::pct);
LM.spin(vex::directionType::fwd, turnSpeed, vex::velocityUnits::pct);
LB.spin(vex::directionType::fwd, turnSpeed, vex::velocityUnits::pct);
RF.spin(vex::directionType::rev, turnSpeed, vex::velocityUnits::pct);
RM.spin(vex::directionType::rev, turnSpeed, vex::velocityUnits::pct);
RB.spin(vex::directionType::rev, turnSpeed, vex::velocityUnits::pct);
error = distance - measuredValue;
Brain.Screen.newLine();
Brain.Screen.print(error);
// Limit the integral term to prevent wind-up
if (abs(error) < 5) {
integral += error;
} else {
integral = 0;
}
derivative = error - prevError;
prevError = error;


if (abs(error) < 1) {
reachedTarget = true; // Mark that we've reached the target
}


// If the error changes sign after reaching the target, break the loop
if (reachedTarget) {
break;
}


wait(20, msec);
}
}


//other auton functions
void Cata(int speed) {
double rotationPosition = 0;
CataLeft.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
CataRight.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
wait(500, msec);


while (true) {
rotationPosition = Rotation.position(vex::rotationUnits::deg);


if (rotationPosition == 30) {
CataLeft.stop(vex::brakeType::brake);
CataRight.stop(vex::brakeType::brake);
break;
}
vex::task::sleep(20);
}
}
void intakeSpin(int time, int speed) {
Intake.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
wait(time, msec);
Intake.stop(vex::brakeType::brake);
}
void slantRight(int time) {
LF.spin(vex::directionType::rev, 88, vex::velocityUnits::pct);
LM.spin(vex::directionType::rev, 88, vex::velocityUnits::pct);
LB.spin(vex::directionType::rev, 88, vex::velocityUnits::pct);
RF.spin(vex::directionType::fwd, 55, vex::velocityUnits::pct);
RM.spin(vex::directionType::fwd, 55, vex::velocityUnits::pct);
RB.spin(vex::directionType::fwd, 55, vex::velocityUnits::pct);
wait (time, msec);
LF.stop(vex::brakeType::brake);
LM.stop(vex::brakeType::brake);
LB.stop(vex::brakeType::brake);
RF.stop(vex::brakeType::brake);
RM.stop(vex::brakeType::brake);
RB.stop(vex::brakeType::brake);


}
void slantLeft(int time1, int time2) {
LF.spin(vex::directionType::rev, 20, vex::velocityUnits::pct);
LM.spin(vex::directionType::rev, 20, vex::velocityUnits::pct);
LB.spin(vex::directionType::rev, 20, vex::velocityUnits::pct);
RF.spin(vex::directionType::fwd, 88, vex::velocityUnits::pct);
RM.spin(vex::directionType::fwd, 88, vex::velocityUnits::pct);
RB.spin(vex::directionType::fwd, 88, vex::velocityUnits::pct);

wait (time1, msec);
WingL.open();
wait (time2, msec);

LF.stop(vex::brakeType::brake);
LM.stop(vex::brakeType::brake);
LB.stop(vex::brakeType::brake);
RF.stop(vex::brakeType::brake);
RM.stop(vex::brakeType::brake);
RB.stop(vex::brakeType::brake);


}
void armDown(double time) {
Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
waitUntil(Intake.position(vex::rotationUnits::deg) >= time); {
Intake.stop(vex::brakeType::brake);
}
}
void armUp(double time) {
Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
waitUntil(Intake.position(vex::rotationUnits::deg) >= time); {
Intake.stop(vex::brakeType::brake);
}
}
void moveNoPid(int time, int speed) {
LF.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
LM.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
LB.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
RF.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
RM.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
RB.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
wait(time, msec);
LB.stop(vex::brakeType::brake);
RF.stop(vex::brakeType::brake);
RM.stop(vex::brakeType::brake);
RB.stop(vex::brakeType::brake);
LF.stop(vex::brakeType::brake);
LM.stop(vex::brakeType::brake);
}
void moveRevNoPid(int time, int speed) {
LF.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
LM.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
LB.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
RF.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
RM.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
RB.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
wait(time, msec);
LB.stop(vex::brakeType::brake);
RF.stop(vex::brakeType::brake);
RM.stop(vex::brakeType::brake);
RB.stop(vex::brakeType::brake);
LF.stop(vex::brakeType::brake);
LM.stop(vex::brakeType::brake);
}

void turnNoPid(int time, int speed) {
LF.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
LM.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
LB.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
RF.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
RM.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
RB.spin(vex::directionType::rev, speed, vex::velocityUnits::pct);
wait(time, msec);
LB.stop(vex::brakeType::brake);
RF.stop(vex::brakeType::brake);
RM.stop(vex::brakeType::brake);
RB.stop(vex::brakeType::brake);
LF.stop(vex::brakeType::brake);
LM.stop(vex::brakeType::brake);
}
//Cata task for driver
int Cata() {
if (Controller1.ButtonR1.pressing()) {
double rotationPosition = 0;
CataLeft.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
CataRight.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
wait(500, msec);


while (true) {
rotationPosition = Rotation.position(vex::rotationUnits::deg);
if (rotationPosition <= 220) {
CataLeft.stop(vex::brakeType::coast);
CataRight.stop(vex::brakeType::coast);
break;
}
vex::task::sleep(20); // Sleep to avoid excessive CPU usage
}
}
return 1;
}








//auton selector
int  autonSwitchVar = 0;

bool autonSwitch = true;


int autonSelector() {


Brain.Screen.clearScreen();

Brain.Screen.setFont(vex::fontType::mono20);
Brain.Screen.clearScreen();
Brain.Screen.setFillColor(vex::color::blue);
Brain.Screen.setPenColor(vex::color::white);
Brain.Screen.drawRectangle(10, 10, 440, 260);
Brain.Screen.setCursor(2, 4);
Brain.Screen.print("Autonomous Selector");
Brain.Screen.setCursor(3, 4);
Brain.Screen.print("0: Close");
Brain.Screen.setCursor(4, 4);
Brain.Screen.print("1: Far");
Brain.Screen.setCursor(5, 4);
Brain.Screen.print("2: Close No Bar");
Brain.Screen.setCursor(6, 4);
Brain.Screen.print("3: Far No Bar");
Brain.Screen.setCursor(7, 4);
Brain.Screen.print("4: Skills");
Brain.Screen.setCursor(8, 4);
Brain.Screen.setFont(vex::fontType::mono20);
Brain.Screen.print("Current selected auton is %d", autonSwitchVar);
Brain.Screen.setFont(vex::fontType::mono60);
Brain.Screen.setCursor(4, 1);
Brain.Screen.print("1028R You Ready?");

   while (true) {
  

        if (Brain.Screen.pressing()) {
   autonSwitchVar += 1;
Brain.Screen.setFont(vex::fontType::mono20);
Brain.Screen.clearScreen();
Brain.Screen.setFillColor(vex::color::blue);
Brain.Screen.setPenColor(vex::color::white);
Brain.Screen.drawRectangle(10, 10, 440, 260);
Brain.Screen.setCursor(2, 4);
Brain.Screen.print("Autonomous Selector");
Brain.Screen.setCursor(3, 4);
Brain.Screen.print("0: Close");
Brain.Screen.setCursor(4, 4);
Brain.Screen.print("1: Far");
Brain.Screen.setCursor(5, 4);
Brain.Screen.print("2: Close No Bar");
Brain.Screen.setCursor(6, 4);
Brain.Screen.print("3: Far No Bar");
Brain.Screen.setCursor(7, 4);
Brain.Screen.print("4: Skills");
Brain.Screen.setCursor(8, 4);
Brain.Screen.setFont(vex::fontType::mono20);
Brain.Screen.print("Current selected auton is %d", autonSwitchVar);
Brain.Screen.setFont(vex::fontType::mono60);
Brain.Screen.setCursor(4, 1);
Brain.Screen.print("1028R You Ready?");

            if (autonSwitchVar > 4) {
                autonSwitchVar = 0;
            }
            wait(800, msec);
            

   
        }

        return 1;
   
}
}



//autons
void close() {

}
void far() {
    Intake.spin(vex::directionType::rev, 100,vex::velocityUnits::pct);
move(75,100);
wait(400, msec);
move(-400, 40);
slantLeft(350, 375);

moveNoPid(500, 100);

move(150, 100);

Intake.stop(vex::brakeType::brake);
WingL.close();

turn( 270, 100);
moveNoPid(600, 100);

Intake.spin(vex::directionType::rev, 100,vex::velocityUnits::pct);// first intaked triball
move(625, 100);
turn(90, 100);
move(-300, 100);
WingL.open();
turn(85, 100);


}
void closeNoBar() {
}
void farNoBar() {

Intake.spin(vex::directionType::rev, 100,vex::velocityUnits::pct);
move(75,100);
wait(400, msec);
move(-400, 40);
slantLeft(350, 375);

moveNoPid(500, 100);

move(150, 100);
Intake.stop(vex::brakeType::brake);
WingL.close();
turn(18, 100);
Intake.spin(vex::directionType::rev, 100,vex::velocityUnits::pct);// first intaked triball
move(600, 100);
turn(155, 100);
Intake.spin(vex::directionType::fwd, 100,vex::velocityUnits::pct);//scoring first intaked triball
moveRevNoPid(1000, 100);
wait(200,msec);
moveNoPid(200, 100);
turn(175, 100);
move(-375, 100);
turn(25, 100);
move(75, 100);
Intake.spin(vex::directionType::rev, 100,vex::velocityUnits::pct);
wait(300, msec);
turn(170, 100);
WingL.open();
Intake.spin(vex::directionType::fwd, 100,vex::velocityUnits::pct);
moveRevNoPid(1000, 100);
}




/*---------------------------------------------------------------------------*/
/* Pre-Autonomous Functions */
/* */
/* You may want to perform some actions before the competition starts. */
/* Do them in the following function. You must return from this function */
/* or the autonomous and usercontrol tasks will not be started. This */
/* function is only called once after the V5 has been powered on and */
/* not every time that the robot is disabled. */
/*---------------------------------------------------------------------------*/


void pre_auton(void) {
autonSelector();
}
/*---------------------------------------------------------------------------*/
/* */
/* Autonomous Task */
/* */
/* This task is used to control your robot during the autonomous phase of */
/* a VEX Competition. */
/* */
/* You must modify the code to add your own robot specific commands here. */
/*---------------------------------------------------------------------------*/


void autonomous(void) {
vex::task::stop(autonSelector);
autonSwitch = false;


Intake.spin(vex::directionType::rev, 100,vex::velocityUnits::pct);
move(75,100);
wait(400, msec);
move(-420, 40);
slantLeft(350, 375);

moveNoPid(500, 100);

move(150, 100);
Intake.spin(vex::directionType::fwd, 100,vex::velocityUnits::pct);
WingL.close();
wait(500,msec);
turn(270, 100);
WingL.open();
Intake.stop(vex::brakeType::brake);
moveNoPid(300, 100);

/*
Intake.spin(vex::directionType::rev, 100,vex::velocityUnits::pct);// first intaked triball
move(600, 100);
turn(155, 100);
Intake.spin(vex::directionType::fwd, 100,vex::velocityUnits::pct);//scoring first intaked triball
moveRevNoPid(1000, 100);
wait(200,msec);
moveNoPid(200, 100);
turn(175, 100);
move(-375, 100);
turn(25, 100);
move(75, 100);
Intake.spin(vex::directionType::rev, 100,vex::velocityUnits::pct);
wait(300, msec);
turn(170, 100);
WingL.open();
Intake.spin(vex::directionType::fwd, 100,vex::velocityUnits::pct);
moveRevNoPid(1000, 100);*/

/**]
switch(autonSwitchVar) {
case 1:
close();
break;
case 2:
far();
break;
case 3:
closeNoBar();
break;
case 4:
farNoBar();
break;

}*/
}
/*---------------------------------------------------------------------------*/
/* */
/* User Control Task */
/* */
/* This task is used to control your robot during the user control phase of */
/* a VEX Competition. */
/* */
/* You must modify the code to add your own robot specific commands here. */
/*---------------------------------------------------------------------------*/


void usercontrol(void) {
// User control code here, inside the loop
autonSwitch = false;
vex::task::stop(autonSelector);


Brain.Screen.setFont(vex::fontType::mono20);
Brain.Screen.clearScreen();
Brain.Screen.setFillColor(vex::color::blue);
Brain.Screen.setPenColor(vex::color::white);
Brain.Screen.drawRectangle(10, 10, 440, 260);
Brain.Screen.setCursor(2, 4);
Brain.Screen.print("Autonomous Selector");
Brain.Screen.setCursor(3, 4);
Brain.Screen.print("0: Close");
Brain.Screen.setCursor(4, 4);
Brain.Screen.print("1: Far");
Brain.Screen.setCursor(5, 4);
Brain.Screen.print("2: Close No Bar");
Brain.Screen.setCursor(6, 4);
Brain.Screen.print("3: Far No Bar");
Brain.Screen.setCursor(7, 4);
Brain.Screen.print("4: Skills");
Brain.Screen.setCursor(8, 4);
Brain.Screen.setFont(vex::fontType::mono20);
Brain.Screen.print("Current selected auton is %d", autonSwitchVar);
Brain.Screen.setFont(vex::fontType::mono60);
Brain.Screen.setCursor(4, 1);
Brain.Screen.print("1028R You Ready?");
while (1) {


// arcade chassis code takes the value of the left joystick and adds / subtracts it to the value of the right joystick
//Cata();


LF.spin(vex::directionType::fwd, (Controller1.Axis3.value() - Controller1.Axis1.value()), vex::velocityUnits::pct);
LM.spin(vex::directionType::fwd, (Controller1.Axis3.value() - Controller1.Axis1.value()), vex::velocityUnits::pct);
LB.spin(vex::directionType::fwd, (Controller1.Axis3.value() - Controller1.Axis1.value()), vex::velocityUnits::pct);
RF.spin(vex::directionType::rev, (Controller1.Axis3.value() + Controller1.Axis1.value()), vex::velocityUnits::pct);
RM.spin(vex::directionType::rev, (Controller1.Axis3.value() + Controller1.Axis1.value()), vex::velocityUnits::pct);
RB.spin(vex::directionType::rev, (Controller1.Axis3.value() + Controller1.Axis1.value()), vex::velocityUnits::pct);


if (Controller1.ButtonB.pressing()) {
CataLeft.spin(vex::directionType::fwd, 30,vex::velocityUnits::pct);
CataRight.spin(vex::directionType::rev, 30, vex::velocityUnits::pct);

}
if (Controller1.ButtonA.pressing()) {
HangL.open();
HangR.open();

}
if (Controller1.ButtonX.pressing()) {
HangL.close();
HangR.close();

}

if (Controller1.ButtonR2.pressing()) {
CataLeft.spin(vex::directionType::fwd, 50,vex::velocityUnits::pct);
CataRight.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
}else {
CataLeft.stop(vex::brakeType::brake);
CataRight.stop(vex::brakeType::brake);
}

if (Controller1.ButtonB.pressing()) {
    WingL.open();
    WingR.open();
}
if (Controller1.ButtonY.pressing()) {
    WingL.close();
    WingR.close();
}


if (Controller1.ButtonL1.pressing()) {
Intake.spin(vex::directionType::fwd, 100,vex::velocityUnits::pct);
} else {


if (Controller1.ButtonL2.pressing()) {
Intake.spin(vex::directionType::rev, 100,vex::velocityUnits::pct);
}else {
Intake.stop(vex::brakeType::brake);
}
// This is the main execution loop for the user control program.
// Each time through the loop your program should update motor + servo
// values based on feedback from the joysticks.


// ........................................................................
// Insert user code here. This is where you use the joystick values to
// update your motors, etc.
// ........................................................................


wait(20, msec); // Sleep the task for a short amount of time to
// prevent wasted resources.
}
}
}



//
// Main will set up the competition functions and callbacks.
//
int main() {
// Set up callbacks for autonomous and driver control periods.
Competition.autonomous(autonomous);
Competition.drivercontrol(usercontrol);


// Run the pre-autonomous function.
pre_auton();


// Prevent main from exiting with an infinite loop.
while (true) {
wait(100, msec);
}
return 1;
}
