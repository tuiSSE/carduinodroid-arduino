/* 
 *  carduinodroid_arduino
 *
 *  version 2.0
 *  url     https://github.com/tuiSSE/carduinodroid-wiki/wiki/
 *          https://github.com/tuiSSE/carduinodroid-wiki/wiki/Arduino-Sketch/
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
 
//connunication between android application and arduino will be miserable in debug mode
//#define DEBUG               

#include <Servo.h>            //Support for Servo Motor
#include <Wire.h>             //Support for I2C Connection
#include "serialProtocol.h"   //definition of the Serial Protocol
#include "DS2745.h"           //definition of the DS2745 battery chip
#include "car.h"              //definition of the car actors
#include "debug.h"            //definition of debug prints

//define Light/LED Pins
const int   LED1_RED_CONNECTION      =10;
const int   LED2_GREEN_POWER         =12;
const int   LED3_YELLOW_STATUS       =13;
const int   FRONT_LIGHT              =11;
//define Motor Pins
const int   MOTOR_ENABLE             =3;
const int   MOTOR_14                 =9;
const int   MOTOR_23                 =2;
//define Steering Servo Pins
const int   STEERING_SERVO           =5;
//define Ultrasound Pins
const int   ULTRASOUND_FRONT_TRIGGER =4;
const int   ULTRASOUND_FRONT_ECHO    =7;
const int   ULTRASOUND_BACK_TRIGGER  =6;
const int   ULTRASOUND_BACK_ECHO     =8;

//define directions
const byte  TX                       =0;
const byte  RX                       =1;

//define ultrasonic constants
const int   ULTRASOUND_MAX_RANGE     = 300; // Maximum Range of Ultrasonic Sensors to achive in cm
const int   ULTRASOUND_MAX_VAL       = 255;
const float ULTRASOUND_SPEED         = 29.1; // Speed of Sound in microseconds/cm
const int   ULTRASOUND_SOUND_SPEED   = 343;  // Speed of Sound
const long  ULTRASOUND_TIMEOUT       = 2*1000000*ULTRASOUND_MAX_RANGE/100/ULTRASOUND_SOUND_SPEED; // max. time a sensorsignal can need to get back to the sensor in µs

//define serial constants
const int   SERIAL_EMERGENCY_TIMEOUT =400; // Timeout for emergency-stop after loss of connection btw. arduino and smartphone (in ms)
const int   SERIAL_SEND_INTERVAL     =200;
const int   SERIAL_BAUDRATE          =9600;

//define power led blink state
const int   POWER_LED_INTERVAL       =300;
const int   POWER_LED_LOW            =80;
byte        powerLedState            =1;
unsigned long powerLedTime           =0; // time stamp for power led

//time variables
unsigned long receiveTime            =0; // time stamp for receiveing data
unsigned long sendTime               =0; // time stamp for sending data

//serial protocol bytes
byte   txBuffer[SEND_BUFFER_LENGTH+1];
byte   rxBuffer[RECEIVE_BUFFER_LENGTH+1];
byte   rxBufferLength                =0;

DS2745* battery;
Car*    car;

void setup()
{
  //set Pinmode
  pinMode(ULTRASOUND_FRONT_TRIGGER,OUTPUT);
  pinMode(ULTRASOUND_FRONT_ECHO,INPUT);
  pinMode(ULTRASOUND_BACK_TRIGGER,OUTPUT);
  pinMode(ULTRASOUND_BACK_ECHO,INPUT);
  
  pinMode(LED1_RED_CONNECTION,OUTPUT);
  pinMode(LED2_GREEN_POWER,OUTPUT);
  pinMode(LED3_YELLOW_STATUS,OUTPUT);
  
  Serial.begin(SERIAL_BAUDRATE);
  
  battery = new DS2745();
  battery->init();
  
  car = new Car(MOTOR_ENABLE,MOTOR_14,MOTOR_23,STEERING_SERVO,FRONT_LIGHT);
}

void loop() 
{
  serialProtocolWrite();  
  emergencyCheck();
  checkPwr();
}

void serialProtocolWrite(){
  if(sendTime + SERIAL_SEND_INTERVAL < millis()){
    battery->update();
    txBuffer[NUM_START]           = STARTBYTE;
    txBuffer[NUM_VERSION_LENGTH]  = SEND_VERSION_LENGTH;
    txBuffer[NUM_CURRENT]         = checkSerialProtocolByte(battery->getCurrent());
    txBuffer[NUM_ACC_CURRENT]     = checkSerialProtocolByte(battery->getAccCurrent());
    txBuffer[NUM_REL_ACC_CURRENT] = checkSerialProtocolByte(battery->getRelAccCurrent());
    txBuffer[NUM_VOLTAGE]         = checkSerialProtocolByte(battery->getVoltage());
    txBuffer[NUM_TEMPERATURE]     = checkSerialProtocolByte(battery->getTemperature());
    txBuffer[NUM_DISTANCE_FRONT]  = checkSerialProtocolByte(measureDistance(FRONT));
    txBuffer[NUM_DISTANCE_BACK]   = checkSerialProtocolByte(measureDistance(BACK));
    txBuffer[NUM_SEND_CHECK]      = serialProtocolCalcChecksum(txBuffer,NUM_SEND_CHECK,TX);
    
    Serial.write(txBuffer, SEND_BUFFER_LENGTH);
    sendTime = millis();
  }
}

void emergencyCheck(){
  if (receiveTime + SERIAL_EMERGENCY_TIMEOUT < millis()){  // Stop car after emergency-timeout 
    car->stop();
    setLED(LED1_RED_CONNECTION,HIGH);//Connection-Status LED on
  }else{
    //Measure Distance to Obstacles in Front and Back
    car->setSpeedLimit(measureDistance(FRONT),measureDistance(BACK));
    car->drive();//set new limited driveSpeed
  }
}

byte checkSerialProtocolByte(byte in){
  if(in == STARTBYTE){
    return in - 1;
  }
  return in;
}

void serialProtocolRead(){
  setLED(LED1_RED_CONNECTION,HIGH);
  while(Serial.available()) {
    // get the new byte:
    byte inChar = (byte) Serial.read();
    DEBUG_PRINTLN_HEX(inChar);
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
      receiveTime = millis();// Set timestamp emergency-stop (last received command)
      byte rxDrive  = rxBuffer[NUM_SPEED];
      byte rxSteer  = rxBuffer[NUM_STEER];
      byte rxStatus = rxBuffer[NUM_STATUS];
      car->update(getDriveDir(rxDrive), getDriveStep(rxDrive), getSteerStep(rxSteer), getFrontLight(rxStatus), getFailsafeStop(rxStatus));
      setLED(LED3_YELLOW_STATUS,getStatusLed(rxStatus));
      if(getResetAccumulatedCurrent(rxStatus)){
        battery->resetAccuCurrent();
      }
      setLED(LED1_RED_CONNECTION,LOW);
    }
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
  for(int i = 1; i < num; i++){
    check ^= buffer[i];
  }
  
  for(int i = 0; i < PARITY_BIT; i++){
    if(((check >> i) & CHECK_MSK) == CHECK_MSK){
      parity ^= PARITY_MSK;//toggle
    }
  }
  check &= ~PARITY_MSK; //unset bit 7;
  check |=  parity; //set parity bit
  return check;
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

void checkPwr(){
  if(battery->getRelAccCurrent() < POWER_LED_LOW){
    if(powerLedTime + POWER_LED_INTERVAL < millis()){
      powerLedState ^=1; //toggle state
      setLED(LED2_GREEN_POWER,powerLedState);
    }
  }
  else{
    setLED(LED2_GREEN_POWER,HIGH);
  }
}

byte measureDistance(byte dir)
{
  int triggerPin = ULTRASOUND_BACK_TRIGGER;
  int echoPin    = ULTRASOUND_BACK_ECHO;
  if(dir == FRONT){
    triggerPin = ULTRASOUND_FRONT_TRIGGER;
    echoPin    = ULTRASOUND_FRONT_ECHO;
  }
  //Send Signal
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  //wait for receiving the signal (max. waittime = ULTRASOUND_TIMEOUT)
  long duration = pulseIn(echoPin, HIGH, ULTRASOUND_TIMEOUT);
  long distance = duration/2/ULTRASOUND_SPEED; // Duration until Signal / 2 / Speed of Sound im microseconds/cm
  
  
  if (distance == 0 || distance > ULTRASOUND_MAX_VAL){//Nothing in given max. Range
    distance = 255;
  }
  DEBUG_PRINT("Distance");
  if(dir == FRONT) 
    DEBUG_PRINT("Front:");
  else
    DEBUG_PRINT("Back :");
  DEBUG_PRINTLN(distance);
  
  return (byte) distance;
}

void setLED(byte LEDpin, byte value){
  if (value){
    digitalWrite(LEDpin,HIGH);
  }
  else{
    digitalWrite(LEDpin,LOW);
  }
}

boolean getStatusLed(byte b){
  return bitRead(b,STATUS_LED_BIT) ? true:false;
}
boolean getFrontLight(byte b){
  return bitRead(b,FRONT_LIGHT_BIT) ? true:false;
}
boolean getResetAccumulatedCurrent(byte b){
  return bitRead(b,RESET_ACC_CURRENT_BIT) ? true:false;
}
boolean getFailsafeStop(byte b){ //get Failsafe Stop
  return bitRead(b,FAILSAFE_STOP_BIT) ? true:false;
}

byte getDriveDir(byte b){
  return bitRead(b,SIGN_BIT) ? BACK:FRONT;
}

char getDriveStep(byte b){
  return abs((char)b);
}

char getSteerStep(byte b){
  return (char) b;
}
