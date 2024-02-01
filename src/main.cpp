
#include "vex.h"

using namespace vex;
competition Competition;
vex::brain Brain = vex::brain(); 
vex::inertial Inertial = vex::inertial(vex::PORT9);
vex::motor FrontLeft (vex::PORT11, vex::gearSetting::ratio18_1, true);
vex::motor FrontRight (vex::PORT20, vex::gearSetting::ratio18_1, false);
vex::motor BackLeft (vex::PORT13, vex::gearSetting::ratio18_1, true);
vex::motor BackRight (vex::PORT17, vex::gearSetting::ratio18_1, false);
vex::motor MiddleLeft (vex::PORT12, vex::gearSetting::ratio18_1, true);
vex::motor MiddleRight (vex::PORT19, vex::gearSetting::ratio18_1, false);
vex::motor Intake (vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::controller Controller1 (vex::controllerType::primary);
vex::pneumatics wingsLeft = vex::pneumatics(Brain.ThreeWirePort.F);
vex::pneumatics wingsRight = vex::pneumatics(Brain.ThreeWirePort.H);
vex::pneumatics wingsVertical = vex::pneumatics(Brain.ThreeWirePort.E);
vex::pneumatics hang = vex::pneumatics(Brain.ThreeWirePort.G);
vex::motor flywheel (vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor kicker (vex::PORT10, vex::gearSetting::ratio18_1, false);

vex::distance KickerDistance = vex::distance(vex::PORT6);
vex::rotation Rotation = vex::rotation(vex::PORT2);

void resetDrive(){
  FrontLeft.resetPosition();
  BackLeft.resetPosition();
  MiddleLeft.resetPosition();
  BackRight.resetPosition();
  MiddleRight.resetPosition();
  FrontRight.resetPosition();
}



void turn(int degrees, int Speed) {
 double P = 0.345; // tuning values for tuning the PID loop for smooth turns
  double I = 0.08; // need to adjust
  double D = 0.15;
  double error = 0.0; // difference from where you want to go and where you are
  double prevError = 0.0; // error one code cycle ago
  double integral = 0.0; // adds one every time to make sure it doesn't take too long to get to the target
  double derivative = 0.0; // difference between the current error and the previous error
  double measuredValue = 0.0;
  bool reachedTarget = false;

  while (true) {
    measuredValue = Inertial.rotation(vex::rotationUnits::deg);

    double turnSpeed = (error * P) + (integral * I) + (derivative * D);
     
    FrontLeft.spin(vex::directionType::rev, turnSpeed, vex::velocityUnits::pct);
    MiddleLeft.spin(vex::directionType::rev, turnSpeed, vex::velocityUnits::pct);
    BackLeft.spin(vex::directionType::rev, turnSpeed, vex::velocityUnits::pct);
    FrontRight.spin(vex::directionType::fwd, turnSpeed, vex::velocityUnits::pct);
    MiddleRight.spin(vex::directionType::fwd, turnSpeed, vex::velocityUnits::pct);
    BackRight.spin(vex::directionType::fwd, turnSpeed, vex::velocityUnits::pct);
    
    error = -1*(degrees - measuredValue);

   
   
    if (abs(error) < 5) {
      integral += error;
    } else {
      integral = 0;
    }
    
    derivative = error - prevError;
    prevError = error;

    if (abs(error) < 1.5) {
      reachedTarget = true; 
    }
    if (reachedTarget) {
      FrontLeft.stop();
      MiddleLeft.stop();
      BackLeft.stop();
      FrontRight.stop();
      MiddleRight.stop();
      BackRight.stop();
      break;
    }

    wait(20, msec);
  }
}


void move(int distance, int Speed) {
  double P = 0.31; // tuning values for tuning the PID loop for smooth turns
  double I = 0.0;
  double D = 0.15;
  double error = 0.0; // difference from where you want to go and where you are
  double prevError = 0.0; // error one code cycle ago
  double integral = 0.0; // adds one every time to make sure it doesn't take too long to get to the target
  double derivative = 0.0; // difference between the current error and the previous error
  double measuredValue = 0.0;
  bool reachedTarget = false;
FrontRight.setPosition(0, vex::rotationUnits::deg);
MiddleRight.setPosition(0, vex::rotationUnits::deg);
BackRight.setPosition(0, vex::rotationUnits::deg);
FrontLeft.setPosition(0, vex::rotationUnits::deg);
MiddleLeft.setPosition(0, vex::rotationUnits::deg);
BackLeft.setPosition(0, vex::rotationUnits::deg);


  while (true) {
    
    measuredValue = (FrontLeft.position(vex::rotationUnits::deg) + FrontRight.position(vex::rotationUnits::deg) + MiddleLeft.position(vex::rotationUnits::deg) + MiddleRight.position(vex::rotationUnits::deg) + BackLeft.position(vex::rotationUnits::deg) + BackRight.position(vex::rotationUnits::deg)) / 6;
   
    double turnSpeed = (error * P) + (integral * I) + (derivative * D);
    
    FrontLeft.spin(vex::directionType::fwd, turnSpeed, vex::velocityUnits::pct);
    MiddleLeft.spin(vex::directionType::fwd, turnSpeed, vex::velocityUnits::pct);
    BackLeft.spin(vex::directionType::fwd, turnSpeed, vex::velocityUnits::pct);
    FrontRight.spin(vex::directionType::fwd, turnSpeed, vex::velocityUnits::pct);
    MiddleRight.spin(vex::directionType::fwd, turnSpeed, vex::velocityUnits::pct);
    BackRight.spin(vex::directionType::fwd, turnSpeed, vex::velocityUnits::pct);
    
    error = distance - measuredValue;
    Brain.Screen.newLine();
    Brain.Screen.print(error);
   
    // Limit the integral teMiddleRight to prevent wind-up
    if (abs(error) < 5) {
      integral += error;
    } else {
      integral = 0;
    }
    
    derivative = error - prevError;
    prevError = error;

    if (abs(error) < 3) {
      reachedTarget = true; // Mark that we've reached the target
    }

    // If the error changes sign after reaching the target, break the loop
    if (reachedTarget) {

      break;
    }

    wait(20, msec);
  }
}
void moveSlow(int distance, int Speed) {
  double P = 0.31; // tuning values for tuning the PID loop for smooth turns
  double I = 0.0;
  double D = 0.15;
  double error = 0.0; // difference from where you want to go and where you are
  double prevError = 0.0; // error one code cycle ago
  double integral = 0.0; // adds one every time to make sure it doesn't take too long to get to the target
  double derivative = 0.0; // difference between the current error and the previous error
  double measuredValue = 0.0;
  bool reachedTarget = false;
FrontRight.setPosition(0, vex::rotationUnits::deg);
MiddleRight.setPosition(0, vex::rotationUnits::deg);
BackRight.setPosition(0, vex::rotationUnits::deg);
FrontLeft.setPosition(0, vex::rotationUnits::deg);
MiddleLeft.setPosition(0, vex::rotationUnits::deg);
BackLeft.setPosition(0, vex::rotationUnits::deg);


  while (true) {
    
    measuredValue = (FrontLeft.position(vex::rotationUnits::deg) + FrontRight.position(vex::rotationUnits::deg) + MiddleLeft.position(vex::rotationUnits::deg) + MiddleRight.position(vex::rotationUnits::deg) + BackLeft.position(vex::rotationUnits::deg) + BackRight.position(vex::rotationUnits::deg)) / 6;
   
    double turnSpeed = (error * P) + (integral * I) + (derivative * D);
    
    FrontLeft.spin(vex::directionType::fwd, turnSpeed * 0.25, vex::velocityUnits::pct);
    MiddleLeft.spin(vex::directionType::fwd, turnSpeed * 0.25, vex::velocityUnits::pct);
    BackLeft.spin(vex::directionType::fwd, turnSpeed * 0.25, vex::velocityUnits::pct);
    FrontRight.spin(vex::directionType::fwd, turnSpeed * 0.25, vex::velocityUnits::pct);
    MiddleRight.spin(vex::directionType::fwd, turnSpeed * 0.25, vex::velocityUnits::pct);
    BackRight.spin(vex::directionType::fwd, turnSpeed * 0.25, vex::velocityUnits::pct);
    
    error = distance - measuredValue;
    Brain.Screen.newLine();
    Brain.Screen.print(error);
   
    // Limit the integral teMiddleRight to prevent wind-up
    if (abs(error) < 5) {
      integral += error;
    } else {
      integral = 0;
    }
    
    derivative = error - prevError;
    prevError = error;

    if (abs(error) < 3) {
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

double leftEncoderRaw = 0.0;
double rightEncoderRaw = 0.0;
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
//Rotation.resetPosition(); //reset rotation wheel vars
//Rotation2.resetPosition();
//Rotation3.resetPosition();
FrontLeft.resetPosition();
BackLeft.resetPosition();
MiddleLeft.resetPosition();
FrontRight.resetPosition();
BackRight.resetPosition();
MiddleRight.resetPosition();

float radius = 0.0;
leftEncoderRaw = FrontLeft.position(vex::rotationUnits::raw) + MiddleLeft.position(vex::rotationUnits::raw) + BackLeft.position(vex::rotationUnits::raw) / 3;
rightEncoderRaw = FrontRight.position(vex::rotationUnits::raw) + MiddleRight.position(vex::rotationUnits::raw) + BackRight.position(vex::rotationUnits::raw) / 3;
leftEncoder = leftEncoderRaw / 6 / (3/4) * 3.25 * M_PI;//updat rotation wheel vars
rightEncoder = rightEncoderRaw / 6 / (3/4) * 3.25 * M_PI;
//leftEncoder = Rotation.position(vex::rotationUnits::raw) * 3.25;//updat rotation wheel vars
//rightEncoder = Rotation2.position(vex::rotationUnits::raw) * 3.25;
//backEncoder = Rotation3.position(vex::rotationUnits::raw) * 3.25;
//(leftEncoder - rightEncoder) / (leftOffset + rightOffset);
botTheta = Inertial.rotation(vex::rotationUnits::deg);
botDegrees = botTheta * (180.0 / M_PI); //rad to deg
botDelta = (2 * (sin(botTheta / 2.0))); //delta math
botX = lastX += (botDelta * ((backEncoder / botTheta) + backOffset)) ; //finding x
botY = lastY += (botDelta * ((rightEncoder / botTheta) + rightOffset)); //finding y


botX = lastX;
botY = lastY;
wait(200, msec);
}
}
  double rotationThresh = 320;
int KickerAuto () {

  double distancePos = 0.0;
  double distanceThresh = 38;
  double rotationThresh = 320;
 int count = 0;
while(true){


 
if(KickerDistance.objectDistance(vex::distanceUnits::mm) < distanceThresh){
  wait(50,msec);
kicker.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
}
if(!(KickerDistance.objectDistance(vex::distanceUnits::mm) < distanceThresh)){
  wait(125, msec);
  kicker.stop();
}

}
return 0;
}



void MoveIntake (int time, double spd){
  FrontLeft.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
 
  wait(time,msec);
  FrontLeft.stop(vex::brakeType::brake);
  BackLeft.stop(vex::brakeType::brake);
  MiddleLeft.stop(vex::brakeType::brake);
  FrontRight.stop(vex::brakeType::brake);
  BackRight.stop(vex::brakeType::brake);
  MiddleRight.stop(vex::brakeType::brake);
  Intake.stop(vex::brakeType::brake);
}

void MoveForward (int time, double spd){
  FrontLeft.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  wait(time,msec);
  FrontLeft.stop(vex::brakeType::brake);
  BackLeft.stop(vex::brakeType::brake);
  MiddleLeft.stop(vex::brakeType::brake);
  FrontRight.stop(vex::brakeType::brake);
  BackRight.stop(vex::brakeType::brake);
  MiddleRight.stop(vex::brakeType::brake);
}
void TurnLeft (int time, double spd){
  FrontLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  wait(time,msec);
  FrontLeft.stop(vex::brakeType::brake);
  BackLeft.stop(vex::brakeType::brake);
  MiddleLeft.stop(vex::brakeType::brake);
  FrontRight.stop(vex::brakeType::brake);
  BackRight.stop(vex::brakeType::brake);
  MiddleRight.stop(vex::brakeType::brake);
}
//////////// custom move stuff testing

void flying(int time,int speed){
  flywheel.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
  wait(time, msec);
  flywheel.stop(vex::brakeType::brake);
}
void kickerCheck(){
if(Rotation.position(vex::rotationUnits::deg) > rotationThresh){
  kicker.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  waitUntil((Rotation.position(vex::rotationUnits::deg) < rotationThresh));
  kicker.stop();
}

}


void expell(int time, int speed) {
  Intake.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
  wait(time, msec);
  Intake.stop(vex::brakeType::brake);
}

void slantLeft(int time,int spd){
  FrontLeft.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::fwd, spd*3, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::fwd, spd*3, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::fwd, spd*3, vex::velocityUnits::pct);
  wait(time,msec);
  FrontLeft.stop(vex::brakeType::brake);
  BackLeft.stop(vex::brakeType::brake);
  MiddleLeft.stop(vex::brakeType::brake);
  FrontRight.stop(vex::brakeType::brake);
  BackRight.stop(vex::brakeType::brake);
  MiddleRight.stop(vex::brakeType::brake);
}

void slantRevLeft(int time,int spd){
  FrontLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::rev, spd*3, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::rev, spd*3, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::rev, spd*3, vex::velocityUnits::pct);
  wait(time,msec);
  FrontLeft.stop(vex::brakeType::brake);
  BackLeft.stop(vex::brakeType::brake);
  MiddleLeft.stop(vex::brakeType::brake);
  FrontRight.stop(vex::brakeType::brake);
  BackRight.stop(vex::brakeType::brake);
  MiddleRight.stop(vex::brakeType::brake);
}

void turnEncoder(double dist, double spd){
  FrontLeft.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  waitUntil(Inertial.rotation(vex::rotationUnits::deg) >= dist);
  FrontLeft.stop(vex::brakeType::brake);
  BackLeft.stop(vex::brakeType::brake);
  MiddleLeft.stop(vex::brakeType::brake);
  FrontRight.stop(vex::brakeType::brake);
  BackRight.stop(vex::brakeType::brake);
  MiddleRight.stop(vex::brakeType::brake);
  }
void slantRight(int time,int spd){
  FrontLeft.spin(vex::directionType::fwd, spd*4.3, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::fwd, spd*4.3, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::fwd, spd*4.3, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  wait(time,msec);
  FrontLeft.stop(vex::brakeType::brake);
  BackLeft.stop(vex::brakeType::brake);
  MiddleLeft.stop(vex::brakeType::brake);
  FrontRight.stop(vex::brakeType::brake);
  BackRight.stop(vex::brakeType::brake);
  MiddleRight.stop(vex::brakeType::brake);
}
void slantRevRight(int time,int spd){
  FrontLeft.spin(vex::directionType::rev, spd*4.3, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::rev, spd*4.3, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::rev, spd*4.3, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  wait(time,msec);
  FrontLeft.stop(vex::brakeType::brake);
  BackLeft.stop(vex::brakeType::brake);
  MiddleLeft.stop(vex::brakeType::brake);
  FrontRight.stop(vex::brakeType::brake);
  BackRight.stop(vex::brakeType::brake);
  MiddleRight.stop(vex::brakeType::brake);
}
void slantRightLess(int time,int spd){
  FrontLeft.spin(vex::directionType::fwd, spd*3, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::fwd, spd*3, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::fwd, spd*3, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::fwd, spd, vex::velocityUnits::pct);
  wait(time,msec);
  FrontLeft.stop(vex::brakeType::brake);
  BackLeft.stop(vex::brakeType::brake);
  MiddleLeft.stop(vex::brakeType::brake);
  FrontRight.stop(vex::brakeType::brake);
  BackRight.stop(vex::brakeType::brake);
  MiddleRight.stop(vex::brakeType::brake);
}

void moveBack(int time,int spd){
  FrontLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  wait(time,msec);
  FrontLeft.stop(vex::brakeType::brake);
  BackLeft.stop(vex::brakeType::brake);
  MiddleLeft.stop(vex::brakeType::brake);
  FrontRight.stop(vex::brakeType::brake);
  BackRight.stop(vex::brakeType::brake);
  MiddleRight.stop(vex::brakeType::brake);
}
//////////////////////////////////////////////////////////////////////////////////////////////
void FarRush1(){
  
  wingsRight.open();
  wait(50, msec);
 Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
 move(610, 100);
  wingsLeft.close();
  wingsRight.close();
  
 slantLeft(270, 30);
  MoveForward(400, 50);
 moveBack(300, 10);
 turn(115, 100);
  wingsLeft.open();
  wingsRight.open();
  
 Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
 MoveForward(500, 100);
 Intake.stop(vex::brakeType::coast);
 moveBack(100, 100);
   wingsLeft.close();
  wingsRight.close();
  
 turn(-90, 100);
  Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  move(450, 100);
  move(-300, 100);
   turn(115, 100);
    wingsLeft.open();
  wingsRight.open();
 Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
 MoveForward(700, 100);
 Intake.stop(vex::brakeType::coast);
 move(-450, 100);
       wingsLeft.close();
  wingsRight.close();
 turn(180, 100);
 move(750, 100);
 turn(90, 100);
   
  wingsRight.open();
 slantLeft(550, 28);
  MoveForward(200, 100);
 wingsLeft.close();
  wingsRight.close();
 Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  MoveForward(600, 100);
 moveBack(300, 100);
turn(-110, 100);
    moveBack(600, 100);
  move(200, 100);

}

void FarSafe2(){
 Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
MoveForward(1000, 7);
 wait(700, msec);
 move(-500, 100);
 turn(170, 100);
   wingsLeft.open();
  wingsRight.open();
 slantLeft(550, 28);
 MoveForward(200, 100);
   wingsLeft.close();
  wingsRight.close();
  
 


  Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  MoveForward(600, 100);
  moveBack(300, 100);
turn(-70, 100);
    moveBack(600, 100);
  move(200, 100);
  turn(14, 100);
   Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
move(770, 100);
wait(150, msec);
moveBack(200, 100);
turn(150, 100);
Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
MoveForward(350, 100);
turn(40, 100);
 Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
move(420, 100);
moveBack(20, 100);
turn(177, 100);
  wingsLeft.open();
  wingsRight.open();
Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
MoveForward(800, 100);
wait(50, msec);
  wingsLeft.close();
  wingsRight.close();
  
slantRevRight(575, 30);





 
}


void FarBar3(){
   Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
 move(800, 100);
 move(-675, 100);
 turn(40, 100);
 Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
}
void CloseAWP4(){
moveBack(3000, 16);
moveBack(500, 11);
  turnEncoder(25, 4);
  moveBack(800, 10);
  kicker.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
  wait(300, msec);
  waitUntil(Rotation.position(vex::rotationUnits::deg) < 305);
kicker.stop();
turn(15, 100);
move(670, 100);
turn(90, 100);
wingsVertical.open();
turn(65, 100);
wingsVertical.close();
turn(240, 100);
wingsRight.open();
move(400, 100);
wingsRight.close();
turn(235, 100);
Intake.spin(vex::directionType::rev, 100,vex::velocityUnits::pct);
move(240, 100);

  
}

void CloseRush5(){
  move(100, 100);
wingsVertical.open();
turn(-35, 100);
wingsVertical.close();
move(-100, 100);
turn(135, 100);

move(400, 100);

 wait(15, sec);
}


void Skills6(){
  //KickerAuto();
  //flywheel.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  kicker.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    wingsLeft.open();
  wingsRight.open();
  turn(155,100);
  MoveForward(395,8);

  wait(27, sec);
  flywheel.stop();
    wingsLeft.close();
  wingsRight.close();
  
  turn(55,100);
  move(-200,100);
  turn(69,100);///???
  moveBack(500,100);
  move(200,100);
  turn(-135,100);
  move(-400,100);
  turn(-160,100);

  move(-1100,100);
  turn(0,100);
    wingsLeft.open();
  wingsRight.open();
  move(50,100);
  slantLeft(720,35);
  wingsLeft.close();
  wingsRight.close();
  
  Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  turn(-60,100);
  MoveForward(300,100);
  move(-100,100);
  MoveForward(100,20);
  MoveForward(700,100);
  move(-200,100);
  turn(-65,100);
  MoveForward(700,100);
  move(-100,100);
  turn(180,100);
    Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  move(725,100);
  turn(-90,100);
  move(100,100);
    wingsLeft.open();
  wingsRight.open();
  slantRight(600,35); // first slanting right middle goal score
    Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  MoveForward(800,100);
        wingsLeft.close();
  wingsRight.close();
  move(-400,100);
  turn(80,100);
    Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  move(460, 100);
  turn(-35, 100);// turn to second push
    wingsLeft.open();
  wingsRight.open();
  Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  MoveForward(800,100);
  moveBack(500,100);
  wingsLeft.close();
  wingsRight.close();
  turn(58, 100);
  move(-450,100);
  turn(-48,100); 
    wingsLeft.open();
  wingsRight.open();
    MoveForward(400,100);
    slantRightLess(500,25);
    turn(82,100);  
    wingsLeft.close();
  wingsRight.close();
    move(-200,100);
    turn(10, 100);
     wingsLeft.open();
  wingsRight.open();
      slantRight(720,35);
    MoveForward(1000,100);
    wingsLeft.close();
  wingsRight.close();
    moveBack(100, 100);
     
     
    MoveForward(800, 100);
    

    // potentially change all code below this this to just slantRevRight(400, 20); if it runs out of time

   slantRevRight(400, 20);
    wait(10, sec);
  
    
    

    
 
  //////

  /*
  expand.close();
  Intake.stop();
  turn(60,100);
  move(950,100);
  turn(110,100);
  moveBack(800,100);
  MoveForward(800, 50);
  */

  

}
/////////////////////////////////////////////////////////////////////////////////////////////////////

//auton switch code
int autonVar = 1;

void autonSelector(){
    Brain.Screen.setFont(vex::fontType::mono20);
Brain.Screen.clearScreen();
Brain.Screen.setFillColor(vex::color::blue);
Brain.Screen.setPenColor(vex::color::white);
Brain.Screen.drawRectangle(10, 10, 440, 260);
Brain.Screen.setCursor(2, 4);
Brain.Screen.print("Autonomous Selector");
Brain.Screen.setCursor(3, 4);
Brain.Screen.print("1: FarRush");
Brain.Screen.setCursor(4, 4);
Brain.Screen.print("2: FarSafe");
Brain.Screen.setCursor(5, 4);
Brain.Screen.print("3: FarBar");
Brain.Screen.setCursor(6, 4);
Brain.Screen.print("4: CloseAWP");
Brain.Screen.setCursor(7, 4);
Brain.Screen.print("5: CloseRush");
Brain.Screen.setCursor(8, 4);
Brain.Screen.print("6: Skills");
Brain.Screen.setCursor(9, 4);
Brain.Screen.setFont(vex::fontType::mono20);
Brain.Screen.print("Current selected auton is %d", autonVar);

while(true){

if (Controller1.ButtonB.pressing()){
  switch (autonVar){
  
  case 1:
  Controller1.Screen.print("1: FarRush WILL BE RUN");
  case 2:
  Controller1.Screen.print("2: FarSafe WILL BE RUN");
  case 3:
  Controller1.Screen.print("3: FarBar WILL BE RUN");
  case 4:
  Controller1.Screen.print("4: CloseAWP WILL BE RUN");
  case 5:
  Controller1.Screen.print("5: CloseRush WILL BE RUN");
  case 6:
  Controller1.Screen.print("6: Skills WILL BE RUN");
  default:
  Controller1.Screen.print("Code Bug all Wesley's Fault");
  }



  break;
}

 if ((Brain.Screen.pressing()) or (Controller1.ButtonA.pressing())) {
autonVar +=1;
if(autonVar > 6){
  autonVar = 1;
}
Brain.Screen.setCursor(9, 4);
Brain.Screen.clearLine();
Brain.Screen.print("Current selected auton is %d", autonVar);
Controller1.Screen.clearLine();
switch (autonVar){
  
  case 1:
  Controller1.Screen.print("1: FarRush");
  case 2:
  Controller1.Screen.print("2: FarSafe");
  case 3:
  Controller1.Screen.print("3: FarBar");
  case 4:
  Controller1.Screen.print("4: CloseAWP");
  case 5:
  Controller1.Screen.print("5: CloseRush");
  case 6:
  Controller1.Screen.print("6: Skills");
  default:
  Controller1.Screen.print("Code Bug");
  }
wait(700, msec);
}




}
}
void pre_auton(void) {
 autonSelector();
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  //autonVar = ;
 switch (autonVar){
  //uncomment and change next line while testing autons to not have to use comp switch
  
  //remember to reccoment it once autons are done testing.


  case 1:
  FarRush1();
  break;
  case 2:
  FarSafe2();
  break;
  case 3:
  FarBar3();
  break;
  case 4:
  CloseAWP4();
  break;
  case 5:
  CloseRush5();
  break;
  case 6:
  Skills6();
  break;
  default:
  break;

  }

}
  




//AutonRightWCOLOR
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

int usercontrol(void) {
  // User control code here, inside the loop
  BackLeft.setBrake(vex::brakeType::brake);
  BackRight.setBrake(vex::brakeType::brake);
  FrontLeft.setBrake(vex::brakeType::brake);
  FrontRight.setBrake(vex::brakeType::brake);
  MiddleLeft.setBrake(vex::brakeType::brake);
  MiddleRight.setBrake(vex::brakeType::brake);
  while (1) {
 


    BackLeft.spin(vex::directionType::fwd, ((Controller1.Axis1.value()) + (Controller1.Axis3.value())), vex::velocityUnits::pct);
    BackRight.spin(vex::directionType::rev, (( Controller1.Axis1.value()) - (Controller1.Axis3.value())), vex::velocityUnits::pct);
    FrontLeft.spin(vex::directionType::fwd, ((Controller1.Axis1.value()) + (Controller1.Axis3.value())), vex::velocityUnits::pct);
    FrontRight.spin(vex::directionType::rev, ((Controller1.Axis1.value()) - (Controller1.Axis3.value())), vex::velocityUnits::pct);
    MiddleLeft.spin(vex::directionType::fwd, ((Controller1.Axis1.value()) + (Controller1.Axis3.value())), vex::velocityUnits::pct);
    MiddleRight.spin(vex::directionType::rev, ((Controller1.Axis1.value()) - (Controller1.Axis3.value())), vex::velocityUnits::pct);
    
    if(Controller1.ButtonL2.pressing()){
      Intake.spin(vex::directionType::rev, 100,vex::velocityUnits::pct);
    }else{ if(Controller1.ButtonL1.pressing()){
      Intake.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
    }else{
      Intake.stop(vex::brakeType::brake);
    }
    }
  

    if (Controller1.ButtonY.pressing()){
      wingsLeft.open();
    }
     if(Controller1.ButtonA.pressing()){
      wingsRight.open();
    }
     if (Controller1.ButtonX.pressing()){
      wingsRight.open();
      wingsLeft.open();
    }
     if (Controller1.ButtonB.pressing()){
      wingsRight.close();
      wingsLeft.close();
    }
   if (Controller1.ButtonUp.pressing()){
      wingsVertical.open();
    }

    if (Controller1.ButtonDown.pressing()){
      wingsVertical.close();
    }
    if (Controller1.ButtonRight.pressing()){
      kickerCheck();
    }



    wait(5, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
  return 1;
}

void driverRunner()
{
  vex::task userc(usercontrol);
  vex::task kickera(KickerAuto);
  while(Competition.isDriverControl()){}
  userc.stop();
  kickera.stop();


}

int main() {
  // Set up caBackLeftackLeftacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(driverRunner);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}




