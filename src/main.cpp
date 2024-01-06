
#include "vex.h"

using namespace vex;
competition Competition;
vex::brain Brain = vex::brain(); 
vex::inertial Inertial = vex::inertial(vex::PORT18);
vex::motor FrontLeft (vex::PORT1, vex::gearSetting::ratio18_1, true);
vex::motor FrontRight (vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor BackLeft (vex::PORT3, vex::gearSetting::ratio18_1, true);
vex::motor BackRight (vex::PORT4, vex::gearSetting::ratio18_1, false);
vex::motor MiddleLeft (vex::PORT5, vex::gearSetting::ratio18_1, true);
vex::motor MiddleRight (vex::PORT6, vex::gearSetting::ratio18_1, false);
vex::motor Intake (vex::PORT12, vex::gearSetting::ratio18_1, false);
vex::controller Controller1 (vex::controllerType::primary);
vex::pneumatics expand = vex::pneumatics(Brain.ThreeWirePort.H);
vex::pneumatics hang = vex::pneumatics(Brain.ThreeWirePort.G);
vex::motor flywheel (vex::PORT19, vex::gearSetting::ratio18_1, true);


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

   
    // Limit the integral term to prevent wind-up
    if (abs(error) < 5) {
      integral += error;
    } else {
      integral = 0;
    }
    
    derivative = error - prevError;
    prevError = error;

    if (abs(error) < 1.5) {
      reachedTarget = true; // Mark that we've reached the target
    }

    // If the error changes sign after reaching the target, break the loop
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
void turnMove(int move1, int degrees1, int move2, int degrees2, int move3, int spdFaster, int spdSlower) { 
  FrontRight.setPosition(0, vex::rotationUnits::deg);
  MiddleRight.setPosition(0, vex::rotationUnits::deg);
  BackRight.setPosition(0, vex::rotationUnits::deg);
  FrontLeft.setPosition(0, vex::rotationUnits::deg);
  MiddleLeft.setPosition(0, vex::rotationUnits::deg);
  BackLeft.setPosition(0, vex::rotationUnits::deg);
  //resets position for first move
  FrontLeft.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  waitUntil((FrontLeft.position(vex::rotationUnits::deg) + FrontRight.position(vex::rotationUnits::deg) + 
  MiddleLeft.position(vex::rotationUnits::deg) + MiddleRight.position(vex::rotationUnits::deg) + 
  BackLeft.position(vex::rotationUnits::deg) + BackRight.position(vex::rotationUnits::deg)) / 6 >= move1);
  // waits until motors have travelled farther or same as move 1
    FrontLeft.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::fwd, spdSlower, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::fwd, spdSlower, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::fwd, spdSlower, vex::velocityUnits::pct);
  //waints until inertial is turned farther or same as degrees1
  waitUntil(Inertial.rotation(vex::rotationUnits::deg) >= degrees1);
  FrontLeft.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  FrontRight.setPosition(0, vex::rotationUnits::deg);
  MiddleRight.setPosition(0, vex::rotationUnits::deg);
  BackRight.setPosition(0, vex::rotationUnits::deg);
  FrontLeft.setPosition(0, vex::rotationUnits::deg);
  MiddleLeft.setPosition(0, vex::rotationUnits::deg);
  BackLeft.setPosition(0, vex::rotationUnits::deg);
  waitUntil((FrontLeft.position(vex::rotationUnits::deg) + FrontRight.position(vex::rotationUnits::deg) + 
  MiddleLeft.position(vex::rotationUnits::deg) + MiddleRight.position(vex::rotationUnits::deg) + 
  BackLeft.position(vex::rotationUnits::deg) + BackRight.position(vex::rotationUnits::deg)) / 6 >= move2);
  // waits until motors have travelled farther or same as move 2
   FrontLeft.spin(vex::directionType::fwd, spdSlower, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::fwd, spdSlower, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::fwd, spdSlower, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
    //waints until inertial is less than or same as degrees2
  waitUntil(Inertial.rotation(vex::rotationUnits::deg) <= degrees2);
    FrontLeft.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  BackLeft.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  MiddleLeft.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  BackRight.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  MiddleRight.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  FrontRight.spin(vex::directionType::fwd, spdFaster, vex::velocityUnits::pct);
  FrontRight.setPosition(0, vex::rotationUnits::deg);
  MiddleRight.setPosition(0, vex::rotationUnits::deg);
  BackRight.setPosition(0, vex::rotationUnits::deg);
  FrontLeft.setPosition(0, vex::rotationUnits::deg);
  MiddleLeft.setPosition(0, vex::rotationUnits::deg);
  BackLeft.setPosition(0, vex::rotationUnits::deg);
  waitUntil((FrontLeft.position(vex::rotationUnits::deg) + FrontRight.position(vex::rotationUnits::deg) + 
  MiddleLeft.position(vex::rotationUnits::deg) + MiddleRight.position(vex::rotationUnits::deg) + 
  BackLeft.position(vex::rotationUnits::deg) + BackRight.position(vex::rotationUnits::deg)) / 6 >= move3);
  // waits until motors have travelled farther or same as move 3
  BackLeft.stop(vex::brakeType::hold);
  MiddleLeft.stop(vex::brakeType::hold);
  FrontLeft.stop(vex::brakeType::hold);
  BackRight.stop(vex::brakeType::hold);
  MiddleRight.stop(vex::brakeType::hold);
  FrontRight.stop(vex::brakeType::hold);
  
}
void flying(int time,int speed){
  flywheel.spin(vex::directionType::fwd, speed, vex::velocityUnits::pct);
  wait(time, msec);
  flywheel.stop(vex::brakeType::brake);
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


void frontExpand(int time,int spd){
  expand.open();
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
  expand.close();
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
void backExpand(int time,int spd){
  expand.open();
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
  expand.close();
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
  expand.open();
  wait(50, msec);
 Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
 move(610, 100);
 expand.close();
 slantLeft(270, 30);
  MoveForward(400, 50);
 moveBack(300, 10);
 turn(115, 100);
 expand.open();
 Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
 MoveForward(500, 100);
 Intake.stop(vex::brakeType::coast);
 moveBack(100, 100);
 expand.close();
 turn(-90, 100);
  Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  move(490, 100);
  move(-300, 100);
   turn(115, 100);
   expand.open();
 Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
 MoveForward(700, 100);
 Intake.stop(vex::brakeType::coast);
 move(-300, 100);
 expand.close();
 turn(180, 100);
 move(750, 100);
 turn(90, 100);
 expand.open();
 slantLeft(550, 28);
  MoveForward(200, 100);
 expand.close();
 Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
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
 expand.open();
 slantLeft(550, 28);
 MoveForward(200, 100);
 expand.close();
 


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
expand.open();
Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
MoveForward(800, 100);
wait(50, msec);
expand.close();
slantRevRight(575, 30);





 
}


void FarBar3(){
  expand.open();
  wait(100,msec);
  MoveForward(200,100);
  wait(100,msec);
  expand.close();
  Intake.spin(vex::directionType::fwd);
  slantLeft(400,20);
  MoveForward (300,100);
  Intake.stop();
  MoveForward(400,100);
  move(-215,100);
  expand.close();
  turn(-117, 100);
  Intake.spin(vex::directionType::rev);
  move(760,100);
  move(50,80);
  Intake.stop();

  turn(-30,100);
  expand.open();
  slantRight(500,20);
  expell(300,100);
  MoveForward(700,100); 
  expand.close();

  turn(-120,100);
  Intake.spin(vex::directionType::rev);
  move(460,100);
  move(-145,100);

  turn(35,100);
  expand.open();
  expell(300,100);
  MoveForward(600,100);
  wait(25,msec);
  expand.close();
  moveBack(300,100);
}
void CloseAWP4(){
  move(780,90);
  turn(-90,100);
  expell(500,100);
  wait(100,msec);
  MoveForward(270,100);
  move(-200,100);
  expand.open();
  turn(150,100);
  expand.close();
  turn(205,100);
  move(725,100);
  expand.open();

  turn(120,100);
  expand.close();
  wait(35, msec);
  move(200,100);
  turn(90,100);
  Intake.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
  move(400,100);
  move(-5,100);
}

void CloseRush5(){
 Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
move(740, 100);
turn(0, 100);
 moveBack(300, 30);
turn(80, 100);
 Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct); //ball across bar
 move(300, 100);
 moveBack(200, 30);
 expand.open();
 turn(-145, 100);
 expand.close();
   Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
 move(720, 100);
  turn(-35, 100);
   Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct); 
   MoveForward(350, 100);
   turn(128, 100);
   moveBack(200, 100);
   MoveForward(150, 100);
   turn(165, 100);
    expand.open();
    MoveForward(200, 100);
   slantLeft(800, 33);
   expand.close();
   MoveForward(200, 100);
   turn(-100, 100);
   move(-210, 100);
   move(350, 100);
 wait(15, sec);
}


void Skills6(){
  flywheel.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  expand.open();
  turn(155,100);
  MoveForward(395,8);

  wait(27, sec);
  flywheel.stop();
    expand.close();
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
  expand.open();
  move(50,100);
  slantLeft(720,35);
  expand.close();
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
  expand.open();
  slantRight(600,35); // first slanting right middle goal score
    Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  MoveForward(800,100);
  expand.close();
  move(-400,100);
  turn(80,100);
    Intake.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
  move(460, 100);
  turn(-35, 100);// turn to second push
  expand.open();
  Intake.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  MoveForward(800,100);
  moveBack(500,100);
  expand.close();
  turn(58, 100);
  move(-450,100);
  turn(-48,100); 
    expand.open();
    MoveForward(400,100);
    slantRightLess(500,25);
    turn(82,100);  
    expand.close();
    move(-200,100);
    turn(10, 100);
    expand.open();
      slantRight(720,35);
    MoveForward(1000,100);
    expand.close();
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
  //autonVar = 5;
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

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    BackLeft.spin(vex::directionType::rev, ((-0.7 * Controller1.Axis1.value()) - (0.5 * Controller1.Axis3.value())), vex::velocityUnits::pct);
    BackRight.spin(vex::directionType::fwd, ((-0.7 * Controller1.Axis1.value()) + (0.5 * Controller1.Axis3.value())), vex::velocityUnits::pct);
    FrontLeft.spin(vex::directionType::rev, ((-0.7 * Controller1.Axis1.value()) - (0.5 * Controller1.Axis3.value())), vex::velocityUnits::pct);
    FrontRight.spin(vex::directionType::fwd, ((-0.7 * Controller1.Axis1.value()) + (0.5 * Controller1.Axis3.value())), vex::velocityUnits::pct);
    MiddleLeft.spin(vex::directionType::rev, ((-0.7 * Controller1.Axis1.value()) - (0.5 * Controller1.Axis3.value())), vex::velocityUnits::pct);
    MiddleRight.spin(vex::directionType::fwd, ((-0.7 * Controller1.Axis1.value()) + (0.5 * Controller1.Axis3.value())), vex::velocityUnits::pct);

    if(Controller1.ButtonL2.pressing()){
      Intake.spin(vex::directionType::rev, 100,vex::velocityUnits::pct);
    }else{ if(Controller1.ButtonL1.pressing()){
      Intake.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
    }else{
      Intake.stop(vex::brakeType::brake);
    }
    }
    if(Controller1.ButtonR1.pressing()){
       flywheel.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    }else if(Controller1.ButtonR2.pressing()){
       flywheel.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    }else if (Controller1.ButtonX.pressing()){
        flywheel.stop(vex::brakeType::brake);
    }

    if (Controller1.ButtonY.pressing()){
      expand.open();
    }else if(Controller1.ButtonB.pressing()){
      expand.close();
    }
     if (Controller1.ButtonUp.pressing()){
      expand.open();
      wait(50, msec);
      hang.open();
      expand.close();
    }
    if (Controller1.ButtonDown.pressing()){
        flywheel.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
        expand.open();
        turn(161,100);
        MoveForward(140,50);
        expand.close();
        wait(32, sec);
        flywheel.stop();
        turn(55,100);
        move(-200,100);
        turn(75,100);///???
        moveBack(500,100);
    }

    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

int main() {
  // Set up caBackLeftackLeftacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}




