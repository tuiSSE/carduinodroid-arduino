/*
 * car.c
 * 
 * author   : Till Max Schwikal
 * date     : 15.12.2015
 * url      : https://github.com/tuiSSE/carduinodroid-wiki/wiki/
 */

#include "car.h"

Car::Car(byte _motorEnablePin, byte _motor14Pin, byte _motor23Pin, byte _steeringServoPin, byte _frontLightPin) :
	motorEnablePin(_motorEnablePin),
	motor14Pin(_motor14Pin),
	motor23Pin(_motor23Pin),
	frontLightPin(_frontLightPin),
	speedDir(SPEED_DIR_DEFAULT),
	speedStep(SPEED_STEP_DEFAULT),
	steerStep(STEER_STEP_DEFAULT),
	
	speedStepLimitedFront(STEERING_RESOLUTION),
	speedStepLimitedBack(STEERING_RESOLUTION),
	speedPerStepLimitedFront(MOTOR_SPEED_PER_STEP),
	speedPerStepLimitedBack(MOTOR_SPEED_PER_STEP),
	
	frontLight(FRONT_LIGHTS_DEFAULT),
	failsafeStop(FAILSAFE_STOP_DEFAULT){

  //set MotorPinouts
  pinMode(motor14Pin,OUTPUT);
  pinMode(motor23Pin,OUTPUT);
  pinMode(motorEnablePin,OUTPUT);
  
  //set LightPinouts
  pinMode(frontLightPin,OUTPUT);
  
  //set SteeringServo
  steeringServo = new Servo;
  steeringServo->attach(_steeringServoPin);  
  steeringServo->write(STEERING_SERVO_MIDDLE);
  
  stop();
  
}

volatile Car::~Car(){
  if(steeringServo) delete steeringServo;
}

void Car::update(byte _speedDir, char _speedStep, char _steer, boolean _frontLight, boolean _failsafeStop){
  steerStep    = _steer;
  speedDir     = _speedDir;
  speedStep    = _speedStep;
  frontLight   = _frontLight;
  failsafeStop = _failsafeStop;
  drive();
  steer();
  setFrontLight();
}

void Car::stop(){
  digitalWrite(motor14Pin,LOW);
  digitalWrite(motor23Pin,LOW);// stopped
  digitalWrite(motorEnablePin,LOW);
}

void Car::setSpeedLimit(byte _distFront, byte _distBack){
  if(_distFront < MOTOR_LIMIT_DIST_LOWER){
    //stop motor in front dir
    speedStepLimitedFront    = 0;
    speedPerStepLimitedFront = MOTOR_SPEED_PER_STEP;
  }
  else if(_distFront > MOTOR_LIMIT_DIST_UPPER){
    //no speed limit front
    speedStepLimitedFront    = MOTOR_RESOLUTION;
    speedPerStepLimitedFront = MOTOR_SPEED_PER_STEP;
  }
  else{
    //adapted speed limit front
    speedStepLimitedFront    = MOTOR_RESOLUTION;
    speedPerStepLimitedFront = 1.0*MOTOR_SPEED_PER_STEP * ( _distFront * MOTOR_LIMIT_SLOPE + MOTOR_LIMIT_OFFSET);
  }
  
  if(_distBack < MOTOR_LIMIT_DIST_LOWER){
    //stop motor in back dir
    speedStepLimitedBack    = 0;
    speedPerStepLimitedBack = MOTOR_SPEED_PER_STEP;
  }
  else if(_distBack > MOTOR_LIMIT_DIST_UPPER){
    //no speed limit back
    speedStepLimitedBack    = MOTOR_RESOLUTION;
    speedPerStepLimitedBack = MOTOR_SPEED_PER_STEP;
  }
  else{
    //adapted speed limit back
    speedStepLimitedBack    = MOTOR_RESOLUTION;
    speedPerStepLimitedBack = 1.0*MOTOR_SPEED_PER_STEP * ( _distBack * MOTOR_LIMIT_SLOPE + MOTOR_LIMIT_OFFSET);
  }
}

char Car::getLimitedSpeedStep(){
  char step = speedStep;
  if(failsafeStop){
    //limit speed regarding measured distance
    if(speedDir == FRONT){
      step = min(speedStep,speedStepLimitedFront);
    }
    else{
      step = min(speedStep,speedStepLimitedBack);
    }
  }
  return step;
}

float Car::getLimitedSpeedPerStep(){
  float speedPerStep = MOTOR_SPEED_PER_STEP;
  if(failsafeStop){
    //limit speed regarding measured distance
    if(speedDir == FRONT){
      speedPerStep = speedPerStepLimitedFront;
    }
    else{
      speedPerStep = speedPerStepLimitedBack;
    }
  }
  return speedPerStep;
}

void Car::drive(){
  char speedStepLimited = getLimitedSpeedStep();
  if((speedStepLimited > MOTOR_RESOLUTION) || (speedStepLimited < 1))
  {
    stop();
    return;
  }
  //set speed value
  int speed = round(MOTOR_START + speedStepLimited * getLimitedSpeedPerStep());
  if(speed > MOTOR_MAX || speed < MOTOR_START){
    stop();
    return;
  }
  DEBUG_PRINT( "speed:" ); DEBUG_PRINTLN( speed );
  //set motor pins
  if(speedDir == FRONT)
  {
    digitalWrite(motorEnablePin,LOW); //To prevent H-Bridge from short-circuit
    digitalWrite(motor23Pin,LOW);
    digitalWrite(motor14Pin,HIGH);
    analogWrite(motorEnablePin,speed);
  }
  else// if(speedDir == BACK)
  {
    digitalWrite(motorEnablePin,LOW); //To prevent H-Bridge from short-circuit
    digitalWrite(motor14Pin,LOW);
    digitalWrite(motor23Pin,HIGH);
    analogWrite(motorEnablePin,speed);
  }
}


void Car::steer(){
  if(abs(steerStep) > STEERING_RESOLUTION){
    steerStep = 0;
  }
  int angle = round(STEERING_SERVO_MIDDLE + STEERING_ANGLE_PER_STEP * steerStep);
  if(angle > STEERING_ANGLE_MAX){
    angle = STEERING_ANGLE_MAX;
  }
  if(angle < STEERING_ANGLE_MIN){
    angle = STEERING_ANGLE_MIN;
  }
  DEBUG_PRINT("angle:");
  DEBUG_PRINTLN(angle);
  steeringServo->write(angle);
}

boolean Car::getFrontLight(){
  return frontLight;
}

boolean Car::getFailsafeStop(){
  return failsafeStop;
}

void Car::setFrontLight(){
  if(frontLight){
    analogWrite(frontLightPin,FRONT_LIGHT_DIMMER);
  }
  else{
    analogWrite(frontLightPin,0);
  }
}
