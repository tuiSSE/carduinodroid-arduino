/*
 * car.h
 * 
 * author   : Till Max Schwikal
 * date     : 13.12.2015
 * url      : https://github.com/tuiSSE/carduinodroid-wiki/wiki/
 */

#ifndef CAR__H
#define CAR__H

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif
#include <Servo.h>            //Support for Servo Motor

#include "debug.h"

const byte  FRONT                    =0;
const byte  BACK                     =1;
const byte  RIGHT                    =0;
const byte  LEFT                     =1;
const boolean ON                     =true;
const boolean OFF                    =false;

const byte SPEED_DIR_DEFAULT         =FRONT;
const char SPEED_STEP_DEFAULT        =0x00;
const char STEER_STEP_DEFAULT        =0x00;
const boolean FAILSAFE_STOP_DEFAULT  =ON;
const boolean FRONT_LIGHTS_DEFAULT   =OFF;

//define steering servo
const int   STEERING_SERVO_MIDDLE    =90;
const int   STEERING_SERVO_RANGE     =40; //measured
const char  STEERING_RESOLUTION      =127;//resolution of the steering servo
const float STEERING_ANGLE_PER_STEP  =1.0*STEERING_SERVO_RANGE  / STEERING_RESOLUTION; //angle of one servo step
const int   STEERING_ANGLE_MIN       =STEERING_SERVO_MIDDLE - STEERING_SERVO_RANGE;//minimum angle
const int   STEERING_ANGLE_MAX       =STEERING_SERVO_MIDDLE + STEERING_SERVO_RANGE;//maximum angle

//define motor speed
const int   MOTOR_MAX                =255;
const byte  MOTOR_START              =90;
const byte  MOTOR_RESOLUTION         =127; //resolution of the main motor
const float MOTOR_MAX_PERCENT        =1.0;
const float MOTOR_MIN_PERCENT        =0.2;
const float MOTOR_RANGE_PERCENT      =1.0*(MOTOR_MAX_PERCENT - MOTOR_MIN_PERCENT);
const byte  MOTOR_LIMIT_DIST_UPPER   =200; //upper distance for speed limitation in cm
const byte  MOTOR_LIMIT_DIST_LOWER   =10;  //lower distance for speed limitation in cm, below the motor stops
const byte  MOTOR_LIMIT_RANGE        =MOTOR_LIMIT_DIST_UPPER - MOTOR_LIMIT_DIST_LOWER;
const float MOTOR_LIMIT_SLOPE        =1.0*MOTOR_RANGE_PERCENT/MOTOR_LIMIT_RANGE;
const float MOTOR_LIMIT_OFFSET       =MOTOR_MIN_PERCENT-(MOTOR_LIMIT_DIST_LOWER * MOTOR_RANGE_PERCENT / MOTOR_LIMIT_RANGE);
const float MOTOR_SPEED_PER_STEP     =1.0*(MOTOR_MAX-MOTOR_START)/MOTOR_RESOLUTION;

//Define front led dimmer
const int   FRONT_LIGHT_DIMMER       =1023; //Dimming of Front LEDs


class Car{	
	public:
		Car(byte _motorEnablePin, byte _motor14Pin, byte _motor23Pin, byte _steeringServoPin, byte _frontLightPin);
		volatile ~Car();
		
		void update(byte _speedDir, char _speedStep, char _steer, boolean _frontLight, boolean _failsafeStop);
		void setSpeedLimit(byte _distFront, byte _distBack);
		
		void drive();
		void steer();
		void stop();
		
                void setFrontLight();
		boolean getFrontLight();
		boolean getFailsafeStop();
	
	private:
		char  getLimitedSpeedStep();
		float getLimitedSpeedPerStep();
		
		byte motorEnablePin;
		byte motor14Pin;
		byte motor23Pin;
		byte frontLightPin;
		
		byte speedDir;
		char speedStep;
		char steerStep;
		
		char speedStepLimitedFront;
		char speedStepLimitedBack;
		float speedPerStepLimitedFront;
		float speedPerStepLimitedBack;
		
		boolean frontLight;
		boolean failsafeStop;
		
		Servo* steeringServo;
};

#endif
