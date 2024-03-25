
#include "vex.h"

using namespace vex;
competition Competition;
vex::brain Brain = vex::brain(); 
vex::inertial Inertial = vex::inertial(vex::PORT8);
vex::motor FrontLeft (vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor FrontRight (vex::PORT10, vex::gearSetting::ratio18_1, false);
vex::motor BackLeft (vex::PORT11, vex::gearSetting::ratio18_1, true);
vex::motor BackRight (vex::PORT20, vex::gearSetting::ratio18_1, false);
vex::motor MiddleLeft (vex::PORT2, vex::gearSetting::ratio18_1, true);
vex::motor MiddleRight (vex::PORT9, vex::gearSetting::ratio18_1, false);
vex::motor Intake (vex::PORT3, vex::gearSetting::ratio18_1, false);
vex::controller Controller1 (vex::controllerType::primary);
vex::pneumatics wingsLeft = vex::pneumatics(Brain.ThreeWirePort.A);
vex::pneumatics wingsRight = vex::pneumatics(Brain.ThreeWirePort.B);
vex::pneumatics vertLeft = vex::pneumatics(Brain.ThreeWirePort.C);
vex::pneumatics vertRight = vex::pneumatics(Brain.ThreeWirePort.D);
vex::pneumatics hang = vex::pneumatics(Brain.ThreeWirePort.G);
vex::motor kicker (vex::PORT12, vex::gearSetting::ratio18_1, false);

vex::distance KickerDistance = vex::distance(vex::PORT13);
vex::rotation Rotation = vex::rotation(vex::PORT19);

void resetDrive(){
  FrontLeft.resetPosition();
  BackLeft.resetPosition();
  MiddleLeft.resetPosition();
  BackRight.resetPosition();
  MiddleRight.resetPosition();
  FrontRight.resetPosition();
}



void turn(int degrees, int Speed) {
 double P = 0.34; // tuning values for tuning the PID loop for smooth turns
  double I = 0.08; // need to adjust
  double D = 0.18;
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
      if ((turnSpeed) < 10){
      reachedTarget = true; 
      }
    }

    if (reachedTarget) {
      FrontLeft.stop(vex::brakeType::hold);
      MiddleLeft.stop(vex::brakeType::hold);
      BackLeft.stop(vex::brakeType::hold);
      FrontRight.stop(vex::brakeType::hold);
      MiddleRight.stop(vex::brakeType::hold);
      BackRight.stop(vex::brakeType::hold);
      break;
    }

    wait(20, msec);
  }
}


void move(int distance, int Speed) {
  double P = 0.34; // tuning values for tuning the PID loop for smooth turns
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
double rotationThresh = 300;
int KickerAuto () {

  double distancePos = 0.0;
  double distanceThresh = 30;
  double rotationThresh = 300;
 int count = 0;
while(true){
if((Rotation.position(vex::rotationUnits::deg)) > 350 ){
 
 kicker.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
}
 
if(KickerDistance.objectDistance(vex::distanceUnits::mm) < distanceThresh){
  //wait(50,msec);
kicker.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
wait(25, msec);
}
if((Rotation.position(vex::rotationUnits::deg)) < rotationThresh){
 
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
  BackRight.spin(vex::directionType::fwd, spd*2.5, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::fwd, spd*2.5, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::fwd, spd*2.5, vex::velocityUnits::pct);
  wait(time,msec);
  FrontLeft.stop(vex::brakeType::brake);
  BackLeft.stop(vex::brakeType::brake);
  MiddleLeft.stop(vex::brakeType::brake);
  FrontRight.stop(vex::brakeType::brake);
  BackRight.stop(vex::brakeType::brake);
  MiddleRight.stop(vex::brakeType::brake);
}
void pointLeft(int time,int spd){
  FrontLeft.stop(vex::brakeType::hold);
  BackLeft.stop(vex::brakeType::hold);
  MiddleLeft.stop(vex::brakeType::hold);
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

void slantRevLeft(int time,int spd){
  FrontLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::rev, spd*5, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::rev, spd*5, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::rev, spd*5, vex::velocityUnits::pct);
  wait(time,msec);
  FrontLeft.stop(vex::brakeType::brake);
  BackLeft.stop(vex::brakeType::brake);
  MiddleLeft.stop(vex::brakeType::brake);
  FrontRight.stop(vex::brakeType::brake);
  BackRight.stop(vex::brakeType::brake);
  MiddleRight.stop(vex::brakeType::brake);
}
void pointRevLeft2(int time,int spd){
 FrontLeft.stop(vex::brakeType::hold);
  BackLeft.stop(vex::brakeType::hold);
  MiddleLeft.stop(vex::brakeType::hold);
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
void pointRevLeft(int time,int spd){
 FrontLeft.stop(vex::brakeType::hold);
  BackLeft.stop(vex::brakeType::hold);
  MiddleLeft.stop(vex::brakeType::hold);
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
void pointRevRight(int time,int spd){
 FrontRight.stop(vex::brakeType::hold);
  BackRight.stop(vex::brakeType::hold);
  MiddleRight.stop(vex::brakeType::hold);
  BackLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
  FrontLeft.spin(vex::directionType::rev, spd, vex::velocityUnits::pct);
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
  FrontLeft.spin(vex::directionType::fwd, spd*2.5, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::fwd, spd*2.5, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::fwd, spd*2.5, vex::velocityUnits::pct);
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
void driveStop(){
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
 wait(75,msec);
 wingsRight.close();
 Intake.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
 move(760,100);
turn(90, 100);
 Intake.spin(vex::directionType::rev,100,vex::velocityUnits::pct);
 wait(200, msec);
 turn(183,100);
 move(705,100);
 turn(-99,100);
move(240, 100);
 Intake.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
  turn(-74, 100);
 move(320,90);
 wait(5,msec);
 move(-520,100);
 turn(-100,100);
vertLeft.open();
 move(-200,100);
 turn(-230,100);
 vertLeft.close();
turn(45, 100);

 Intake.spin(vex::directionType::rev,100,vex::velocityUnits::pct);
 MoveForward(800,100);
 move(-300, 100);
 turn(-46, 100);
 Intake.spin(vex::directionType::fwd,100,vex::velocityUnits::pct); // ball intake
 move(725, 100);
 move(-75, 100);
 turn(72, 100);
 wingsLeft.open();
 MoveForward(200, 100);
 Intake.spin(vex::directionType::rev,100,vex::velocityUnits::pct);
 slantRight(300, 30);
 MoveForward(380, 100);

 move(-100, 100);
 wingsLeft.close();



}

void FarSafe2(){
  
 Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
 move(1000, 100); // move to first ball
 wingsLeft.close();
 wingsRight.close();
 moveBack(300, 10);
 turn(128, 100);
 wingsLeft.open();
 wingsRight.open();
 Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
 MoveForward(400, 100);
 Intake.stop(vex::brakeType::coast);
 MoveForward(300, 100); // push both balls in
 moveBack(100, 100);
 wingsLeft.close();
 wingsRight.close();
 move(-350,100);
 turn(250, 100);
 Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
 move(350, 100);// intake third ball
 move(-150, 100);
 turn(-2, 100);
 move(-650, 100);
 turn(75,100);
 Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
 move(100, 100); // extake third
 move(-375, 100);
 turn(-55, 100);
 move(-150, 100);
 turn(-80,100);
vertLeft.open();
vertRight.open();
 move(-250, 100);
 turn(-160, 100);
 vertLeft.close();
vertRight.close();
 turn(95, 100);
 Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
 MoveForward(200, 100);
 slantLeft(300, 30);
 MoveForward(600, 100);
 move(-100,100);






 
}


void FarBar3(){
 Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
 move(800, 100);
 move(-800, 100);
 Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
 turn(300, 100);

}
void CloseAWP4(){
  turn(45, 100);
 vertRight.open();
turn(0, 100);
vertRight.close();
turn(25, 100);
move(300, 100);
turn(0, 100);
move(190, 100);
 Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
MoveForward(1000, 5);
driveStop();

}

void CloseRush5(){
 MoveForward(75,100);
 slantRight(500, 60);
 slantLeft(250, 60);
 Intake.spin(vex::directionType::fwd, 100,vex::velocityUnits::pct);
 move(145, 100);
 move(-50, 100);
 turn(219, 100);
 wingsLeft.close();
 move(740, 100);
 turn(-35, 100);
vertLeft.open();
vertRight.open();
 slantRevRight(400, 30);
 vertLeft.close();
vertRight.close();
 slantRevRight(100, 30);
 turn(-254, 100);
 slantLeft(100, 30);
 Intake.spin(vex::directionType::rev, 100,vex::velocityUnits::pct);
 move(420, 100);
 wingsLeft.open();
 wait(100, msec);
 wingsLeft.close();
 
 turn(-270, 100);
 move(-300, 100);
 turn(-245, 100);
 move(-340, 100);
 turn(-220, 100);
 vertLeft.open();
vertRight.open();
 
 driveStop();
}


void CloseRushBoth6(){
MoveForward(75,100);
 slantRight(500, 60);
 slantLeft(250, 60);
 Intake.spin(vex::directionType::fwd, 100,vex::velocityUnits::pct);
 move(145, 100);
 move(-50, 100);
 turn(90, 100);
 wingsLeft.open();
 move(250, 100);
 move(-50, 100); 
wingsLeft.close();
 turn(224, 100);
 move(765, 100);
 turn(-24, 100);
vertLeft.open();
vertRight.open();
 slantRevRight(400, 30);
 vertLeft.close();
vertRight.close();
 slantRevRight(100, 30);
 turn(-254, 100);
 slantLeft(120, 30);
 Intake.spin(vex::directionType::rev, 100,vex::velocityUnits::pct);
 move(450, 100);
 wingsLeft.open();
 wait(100, msec);
 wingsLeft.close();
 turn(-270, 100);
 move(-300, 100);
 turn(-245, 100);
 move(-400, 100);
 turn(-220, 100);
vertLeft.open();
vertRight.open();
 driveStop();




 
  

}
  /////////////////////////////////////////////////////////////////////////////////////////////////////

  //auton switch code
  int autonVar = 1;



  int autonSelector(){
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
      Brain.Screen.print("6: CloseRushBoth");
    while(true){
    
      Brain.Screen.setCursor(9, 4);
      Brain.Screen.setFont(vex::fontType::mono20);
      Brain.Screen.print("Current selected auton is %d", autonVar);

  if ((Brain.Screen.pressing()) or (Controller1.ButtonA.pressing())) {
    autonVar +=1;

  if(autonVar > 6){
    autonVar = 1;
  }
 
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
        Controller1.Screen.print("6: CloseRushBoth");
    default:
        Controller1.Screen.print("Code Bug");
    }
  wait(700, msec);
  }

  wait(50 ,msec);

  }
  }

vex::task autons(autonSelector);
void preGame(){
vex::task autons(autonSelector);
   
}

  void pre_auton(void) {
preGame();


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
//autonVar = 4;

Inertial.setRotation(0, vex::rotationUnits::deg);

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
      CloseRushBoth6();
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
  autons.stop();


    BackLeft.spin(vex::directionType::fwd, (0.12 * ((Controller1.Axis1.value()) + ( 0.85 * (Controller1.Axis3.value())))), vex::voltageUnits::volt);
    BackRight.spin(vex::directionType::rev, (0.12 *((Controller1.Axis1.value()) - (0.85 * (Controller1.Axis3.value())))), vex::voltageUnits::volt);
    FrontLeft.spin(vex::directionType::fwd, (0.12 * ((Controller1.Axis1.value()) + (0.85 * (Controller1.Axis3.value())))), vex::voltageUnits::volt);
    FrontRight.spin(vex::directionType::rev, (0.12 *((Controller1.Axis1.value()) - (0.85 * (Controller1.Axis3.value())))), vex::voltageUnits::volt);
    MiddleLeft.spin(vex::directionType::fwd, (0.12 * ((Controller1.Axis1.value()) + (0.85 * (Controller1.Axis3.value())))), vex::voltageUnits::volt);
    MiddleRight.spin(vex::directionType::rev, (0.12 *((Controller1.Axis1.value()) - (0.85 * (Controller1.Axis3.value())))), vex::voltageUnits::volt);
    
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
   if (Controller1.ButtonR1.pressing()){
    vertRight.open();
    }

    if (Controller1.ButtonR2.pressing()){
      vertRight.close();
      vertLeft.close();
    }
   
    if (Controller1.ButtonUp.pressing()){
      hang.open();
    }
    if(Controller1.ButtonB.pressing()){
      hang.close();
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
  hang.close();
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
