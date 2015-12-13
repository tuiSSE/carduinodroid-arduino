/* 
 *  carduinodroid_arduino
 *
 *  version 2.0
 *  url     https://github.com/tuiSSE/carduinodroid-wiki/wiki/
 *
 *  Vesion History
 *  Version   Date         Feature/Fix
 *
 *  0.8       28.10.2013   New Protocol by Harald Funk
 *  0.9       03.05.2014   Emergency-Shutdown added
 *  1.0       29.05.2015   Auto-Stop + some fixes
 *  2.0       11.12.2015   Serial Protocol V2 by Till Max Schwikal
 */
 

/*                                +-----+
     +----[PWR]-------------------| USB |--+
     |                            +-----+  |
     |         GND/RST2  [ ][ ]            |
     |       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] |   C5  
     |          5V/MISO2 [ ][ ]  A4/SDA[ ] |   C4  
     |                             AREF[ ] |
     |                              GND[ ] |
     | [ ]N/C                    SCK/13[ ] |   B5  LED3_YELLOW_STATUS
     | [ ]IOREF                 MISO/12[ ] |   .   LED2_GREEN_POWER
     | [ ]RST                   MOSI/11[ ]~|   .   FRONT_LIGHT_ENABLE
     | [ ]3V3    +---+               10[ ]~|   .   LED1_RED_CONNECTION
     | [ ]5v    -| A |-               9[ ]~|   .   MOTOR_14
     | [ ]GND   -| R |-               8[ ] |   B0  ULTRASOUND_BACK_ECHO
     | [ ]GND   -| D |-                    |
     | [ ]Vin   -| U |-               7[ ] |   D7  ULTRASOUND_FRONT_ECHO
     |          -| I |-               6[ ]~|   .   ULTRASOUND_BACK_TRIGGER
     | [ ]A0    -| N |-               5[ ]~|   .   SERVO_DIRECTION
     | [ ]A1    -| O |-               4[ ] |   .   ULTRASOUND_FRONT_TRIGGER
     | [ ]A2     +---+           INT1/3[ ]~|   .   MOTOR_ENABLE
     | [ ]A3                     INT0/2[ ] |   .   MOTOR_23
SDA  | [ ]A4/SDA  RST SCK MISO     TX>1[ ] |   .   SERIAL_WRITE
SCL  | [ ]A5/SCL  [ ] [ ] [ ]      RX<0[ ] |   D0  SERIAL_READ
     |            [ ] [ ] [ ]              |
     |  UNO_R3    GND MOSI 5V  ____________/
      \_______________________/
 */

#include <Wire.h>             //Support for I2C Connection
#include <Servo.h>            //Support for Servo Motor
#include "serialProtocol.h"   //definition of the Serial Protocol
#include "DS2745.h"           //definition of the DS2745 battery chip

const byte DEBUG                     =0;

//define Light/LED Pins
const byte  LED1_RED_CONNECTION      =10;
const byte  LED2_GREEN_POWER         =12;
const byte  LED3_YELLOW_STATUS       =13;
const byte  FRONT_LIGHT              =11;
//define Motor Pins
const byte  MOTOR_ENABLE             =3;
const byte  MOTOR_14                 =9;
const byte  MOTOR_23                 =2;
//define Steering Servo Pins
const byte  STEERING_SERVO           =5;
//define Ultrasound Pins
const byte  ULTRASOUND_FRONT_TRIGGER =4;
const byte  ULTRASOUND_FRONT_ECHO    =7;
const byte  ULTRASOUND_BACK_TRIGGER  =6;
const byte  ULTRASOUND_BACK_ECHO     =8;

//define directions
const byte  FRONT                    =0;
const byte  BACK                     =1;
const byte  RIGHT                    =0;
const byte  LEFT                     =1;
const byte  TX						 =0;
const byte  RX                       =1;

//define steering servo
const int   STEERING_SERVO_MIDDLE    =90;
const int   STEERING_SERVO_RANGE     =40; //measured
const int   STEERING_RESOLUTION      =8;//127; //resolution of the steering servo
const int   STEERING_ANGLE_PER_STEP  =STEERING_SERVO_RANGE  / STEERING_RESOLUTION; //angle of one servo step
const int   STEERING_ANGLE_MIN       =STEERING_SERVO_MIDDLE - STEERING_SERVO_RANGE;//minimum angle
const int   STEERING_ANGLE_MAX       =STEERING_SERVO_MIDDLE + STEERING_SERVO_RANGE;//maximum angle

//define motor speed
const int   MOTOR_MAX                =255;
const int   MOTOR_START              =75;
const int   MOTOR_RANGE              =MOTOR_MAX - MOTOR_START;
const int   MOTOR_RESOLUTION         =9;//127;
const float MOTOR_SPEED_PER_STEP     =MOTOR_RANGE / MOTOR_RESOLUTION;

//define ultrasonic constants
const int   ULTRASOUND_MAX_RANGE     = 300; // Maximum Range of Ultrasonic Sensors to achive in cm
const int 	ULTRASOUND_MAX_VAL		 = 254;
const float ULTRASOUND_SPEED         = 29.1; // Speed of Sound in microseconds/cm
const long  ULTRASOUND_TIMEOUT       = 20000 * ULTRASOUND_MAX_RANGE / ULTRASOUND_SPEED; // max. time a sensorsignal can need to get back to the sensor in µs

//define limits for speed
const int   LIMIT_STEPS                =5; //number of limit
const int   LIMIT_SPEED[LIMIT_STEPS]   ={1 ,2 ,4  ,6  ,8};
const int   LIMIT_DISTANCE[LIMIT_STEPS]={10,50,100,200,254};

//Define front led dimmer
const int   FRONT_LIGHT_DIMMER       =1023; //Dimming of Front LEDs


//define serial constants
const int   SERIAL_EMERGENCY_TIMEOUT =400; // Timeout for emergency-stop after loss of connection btw. arduino and smartphone (in ms)
const int   SERIAL_SEND_INTERVAL     =200;
const int   SERIAL_BAUDRATE          =9600;

//define power led blink state
const int   POWER_LED_INTERVAL       =300;
const int   POWER_LED_LOW            =50;
byte        powerLedState			 =1;
unsigned long powerLedTime           =0; // time stamp for power led

//time variables
unsigned long receiveTime            =0; // time stamp for receiveing data
unsigned long sendTime               =0; // time stamp for sending data

//received bytes
byte   rxBuffer[RECEIVE_BUFFER_LENGTH];
byte   rxBufferLength                =0;
byte   rxSpeed                       =SPEED_DEFAULT;
byte   rxSteer                       =STEER_DEFAULT;
byte   rxStatus                      =STATUS_DEFAULT;
int    speedLimit                    =MOTOR_RESOLUTION;

Servo  steeringServo;    //Define steeringServo as Variable
DS2745 battery;

void setup()
{
  //set Pinmode
  pinMode(MOTOR_14,OUTPUT);
  pinMode(MOTOR_23,OUTPUT);
  pinMode(MOTOR_ENABLE,OUTPUT);
  pinMode(ULTRASOUND_FRONT_TRIGGER,OUTPUT);
  pinMode(ULTRASOUND_FRONT_ECHO,OUTPUT);
  pinMode(ULTRASOUND_BACK_TRIGGER,INPUT);
  pinMode(ULTRASOUND_BACK_ECHO,INPUT);
  pinMode(LED1_RED_CONNECTION,OUTPUT);
  pinMode(LED2_GREEN_POWER,OUTPUT);
  pinMode(LED3_YELLOW_STATUS,OUTPUT);
  pinMode(FRONT_LIGHT,OUTPUT);

  rxBuffer.reserve(RECEIVE_BUFFER_LENGTH+1);
  Serial.begin(SERIAL_BAUDRATE);
  steeringServo.attach(STEERING_SERVO);
  
  steeringServo.write(STEERING_SERVO_MIDDLE);

  battery.init();
  
  //initialize Car
  carUpdate();
}

void loop() 
{
  serialProtocolWrite();  
  emergencyCheck();
  checkPwr();
  if(DEBUG){
    delay(500);
  }
}//loop()

void serialProtocolWrite(){
  if(sendTime + SERIAL_SEND_INTERVAL < millis()){
    battery.update();
    distanceFront = measureDistance(FRONT);
    distanceBack = measureDistance(BACK);
    byte txBuffer[SEND_BUFFER_LENGTH+1];
    
    txBuffer[NUM_START]           = STARTBYTE;
    txBuffer[NUM_VERSION_LENGTH]  = SEND_VERSION_LENGTH;
    txBuffer[NUM_CURRENT]         = checkSerialProtocol(battery.getCurrent());
    txBuffer[NUM_ACC_CURRENT]     = checkSerialProtocol(battery.getAccCurrent());
    txBuffer[NUM_REL_ACC_CURRENT] = checkSerialProtocol(battery.getRelAccCurrent());
    txBuffer[NUM_VOLTAGE]         = checkSerialProtocol(battery.getVoltage());
    txBuffer[NUM_TEMPERATURE]     = checkSerialProtocol(battery.getTemperature());
    txBuffer[NUM_DISTANCE_FRONT]  = checkSerialProtocol(measureDistance(FRONT));
    txBuffer[NUM_DISTANCE_BACK]   = checkSerialProtocol(measureDistance(BACK));
    txBuffer[NUM_SEND_CHECK]      = serialProtocolCalcChecksum(txBuffer,NUM_SEND_CHECK,TX);
    Serial.write(txBuffer, sizeof(txBuffer));
    sendTime = millis();
  }
}

byte checkSerialProtocol(byte in){
	if(in == STARTBYTE){
		return in-1;
	}
	return in;
}

void serialProtocolRead(){
  receiveTime = millis();// Set timestamp emergency-stop (last received command)
  setLED(LED1_RED_CONNECTION,HIGH);
  while (Serial.available()) {
    // get the new byte:
    byte inChar = (byte) Serial.read();
    if(DEBUG){
      Serial.println(inChar,HEX);
    }
    if(inChar == STARTBYTE){
      rxBufferLength = 0;
    }
	//add char to buffer
    rxBuffer[rxBufferLength++] = inChar;
    //check if a full data packet was received
    
    if(rxBuffer[NUM_START] != STARTBYTE){
      rxBufferLength = 0;
      continue;
    }
	if(rxBufferLength >= RECEIVE_BUFFER_LENGTH){
      if(rxBuffer[NUM_VERSION_LENGTH] != RECEIVE_VERSION_LENGTH){
        rxBufferLength = 0;
        continue;
      }
      byte check = serialProtocolCalcChecksum(rxBuffer,NUM_RECEIVE_CHECK,RX);
      if(rxBuffer[NUM_RECEIVE_CHECK] != check){
        rxBufferLength = 0;
        continue;
      }
      //update values
      setLED(LED1_RED_CONNECTION,LOW);
      rxSpeed  = rxBuffer[NUM_SPEED];
      rxSteer  = rxBuffer[NUM_STEER];
      rxStatus = rxBuffer[NUM_STATUS];
      carUpdate();
    }
  }
}


/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent(){
  serialProtocolRead();
}

void emergencyCheck(){
  if (millis() - receiveTime > SERIAL_EMERGENCY_TIMEOUT){  // Stop car after emergency-timeout 
    carStop();
    setLED(LED1_RED_CONNECTION,HIGH);//Connection-Status LED on
  }else{
    setLED(LED1_RED_CONNECTION,LOW);//Connection-Status LED off
    if(getFailsafeStop()){
	//Measure Distance to Obstacles in Front or Back
      if(getDriveDir() == FRONT && getDriveStep() > 0 ){
        carLimit(measureDistance(FRONT));//Limit maximum speed to protect car from hard crashes
      }
      else 
      if(getDriveDir() == BACK && getDriveStep() > 0){
        carLimit(measureDistance(BACK)); //Limit maximum speed to protect car from hard crashes
	  }
    }
  }
}


void checkPwr(){
  if(battery.getRelAccCurrent() < POWER_LED_LOW){
    if(powerLedTime + POWER_LED_INTERVAL < millis()){
	  powerLedState ^=1; //toggle state
      setLED(LED2_GREEN_POWER,powerLedState);
    }
  }
  else{
    setLED(LED2_GREEN_POWER,HIGH);
  }
}

byte serialProtocolCalcChecksum(byte* buffer, byte length, byte dir){
  byte num = NUM_RECEIVE_CHECK;
  if(dir == TX){
	  num = NUM_SEND_CHECK;
  }
  if(length != num){
	  return STARTBYTE; //error
  }
  byte check = STARTBYTE;
  byte parity = 0;
  byte i = 0;
  for(i = 1; i < num; i++){
    check ^= buffer[i];
  }
  for(i = 0; i < PARITY_BIT; i++){
    if(((check >> i) & CHECK_MSK) == CHECK_MSK){
      parity ^= PARITY_MSK;
    }
  }
  check &= ~PARITY_MSK; //unset bit 7;
  check |=  PARITY_MSK; //set parity bit
  return check;
}

byte measureDistance(byte dir)
{
  //Send Signal
  byte trigger = ULTRASOUND_BACK_TRIGGER;
  byte echo    = ULTRASOUND_BACK_ECHO;
  if(dir == FRONT){
    trigger = ULTRASOUND_FRONT_TRIGGER;
    echo    = ULTRASOUND_FRONT_ECHO;
  }
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);

  //wait for receiving the signal (max. waittime = SensorTimeout)
  long duration = pulseIn(echo, HIGH,ULTRASOUND_TIMEOUT);
  int distance = (int) duration/2/ULTRASOUND_SPEED; // Duration until Signal / 2 / Speed of Sound im microseconds/cm
  if (distance == 0 || distance > ULTRASOUND_MAX_VAL){//Nothing in given max. Range
    distance = -1;
  }
  if(DEBUG){
    Serial.print("Distance");
    if(FRONT) Serial.print("Front:");
    else      Serial.print("Back :");
    Serial.println(distance);
  }
  return (byte) distance;
}


void carSteer()
{
  byte step = getSteerStep();
  byte dir  = getSteerDir();
  if(step <= STEERING_RESOLUTION)
  {
    if(dir == RIGHT)
      steeringServo.write(STEERING_SERVO_MIDDLE + step*STEERING_ANGLE_PER_STEP);
    else
      steeringServo.write(STEERING_SERVO_MIDDLE - step*STEERING_ANGLE_PER_STEP);  
  }
  else
    steeringServo.write(STEERING_SERVO_MIDDLE);
}


void carDrive(){
  byte step = getDriveStep();
  byte dir  = getDriveDir();
  //set step to speedLimit
  if(step > speedLimit){
    step = speedLimit;
    if(DEBUG > 0){
      Serial.print("Speed limited to ");
      Serial.println(step);
    }
  }
  //check step resolution
  if((step > MOTOR_RESOLUTION) || (step < 1))
  {
    carStop();
    return;
  }
  //set speed value
  byte speed = MOTOR_START + step * MOTOR_SPEED_PER_STEP;
  //set motor pins
  if(dir == FRONT)
  {
    digitalWrite(MOTOR_ENABLE,LOW); //To prevent H-Bridge from short-circuit
    digitalWrite(MOTOR_23,LOW);
    digitalWrite(MOTOR_14,HIGH);
    analogWrite(MOTOR_ENABLE,speed);
  }
  else// if(dir == BACK)
  {
    digitalWrite(MOTOR_ENABLE,LOW); //To prevent H-Bridge from short-circuit
    digitalWrite(MOTOR_14,LOW);
    digitalWrite(MOTOR_23,HIGH);
    analogWrite(MOTOR_ENABLE,speed);
  }
}

void carStop()
{
  digitalWrite(MOTOR_14,LOW);
  digitalWrite(MOTOR_23,LOW);// stopped
  digitalWrite(MOTOR_ENABLE,LOW);
}

void setFrontLight(byte enable, byte value){
  if(enable){
    analogWrite(FRONT_LIGHT,value);
  }
  else{
    analogWrite(FRONT_LIGHT,0);
  }
}

void setLED(byte LEDpin, byte value){
  if (value){
    digitalWrite(LEDpin,HIGH);
  }
  else{
    digitalWrite(LEDpin,LOW);
  }
}

void carLimit(byte distance){
  //set to maximal speed
  speedLimit = MOTOR_MAX;
  int i = 0;
  while(i < LIMIT_STEPS){
    if (distance < LIMIT_DISTANCE[i]){
      //decrease distance
      speedLimit = LIMIT_SPEED[i];
      break;
    }
  }
  carDrive();
}

boolean getStatusLed(){
  return bitRead(rxStatus,STATUS_LED_BIT) ? true:false;
}
boolean getFrontLight(){
  return bitRead(rxStatus,FRONT_LIGHT_BIT) ? true:false;
}
boolean getResetAccumulatedCurrent(){
  return bitRead(rxStatus,RESET_ACC_CURRENT_BIT) ? true:false;
}
boolean getFailsafeStop(){ //get Failsafe Stop
  return bitRead(rxStatus,FAILSAFE_STOP_BIT) ? true:false;
}

byte getDriveDir(){
  return bitRead(rxSpeed,SIGN_BIT) ? BACK:FRONT;
}

byte getSteerDir(){
  return bitRead(rxSteer,SIGN_BIT) ? LEFT:RIGHT;
}

byte getDriveStep(){
  return (rxSpeed & ~SIGN_MASK);
}

byte getSteerStep(){
  return (rxSteer & ~SIGN_MASK);
}

void carUpdate(){
  setLED(LED3_YELLOW_STATUS,getStatusLed());
  setFrontLight(getFrontLight(),FRONT_LIGHT_DIMMER);
  carSteer();
  carDrive();
}
