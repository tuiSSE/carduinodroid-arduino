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

//define steering servo
const int   STEERING_SERVO_MIDDLE    =90;
const int   STEERING_SERVO_RANGE     =40; //measured
const int   STEERING_RESOLUTION      =8;//127; //resolution of the steering servo
const int   STEERING_ANGLE_PER_STEP  =STEERING_SERVO_RANGE/STEERING_RESOLUTION;//angle of one servo step
const int   STEERING_ANGLE_MIN       =STEERING_SERVO_MIDDLE - STEERING_SERVO_RANGE;//minimum angle
const int   STEERING_ANGLE_MAX       =STEERING_SERVO_MIDDLE + STEERING_SERVO_RANGE;//maximum angle

//define motor speed
const int   MOTOR_MAX                =255;
const int   MOTOR_START              =75;
const int   MOTOR_RANGE              =MOTOR_MAX-MOTOR_START;
const int   MOTOR_RESOLUTION         =9;//127;
const float MOTOR_SPEED_PER_STEP     =MOTOR_RANGE/MOTOR_RESOLUTION;

//define ultrasonic constants
const int   ULTRASONIC_MAX_RANGE     = 300; // Maximum Range of Ultrasonic Sensors to achive in cm
const float ULTRASONIC_SPEED         = 29.1; // Speed of Sound in microseconds/cm
const long  ULTRASONIC_TIMEOUT       = 2*1000000*ULTRASONIC_MAX_RANGE/100/ULTRASONIC_SPEED; // max. time a sensorsignal can need to get back to the sensor in µs

//define limits for speed
const int   LIMIT_STEPS                =5; //number of limit
const int   LIMIT_SPEED[LIMIT_STEPS]   ={1 ,2 ,4  ,6  ,8};
const int   LIMIT_DISTANCE[LIMIT_STEPS]={10,20, 50,100,200};

//Define front led dimmer
const int   FRONT_LIGHT_DIMMER       =1023; //Dimming of Front LEDs

//define battery measurement
const byte  BATT_DEBUG_READ          =0;
const byte  BATT_DEBUG_WRITE         =0;
const byte  BATT_DEBUG_VAL           =0;
const uint8_t   BATT_SLAVE_DECIVE        =72;
const byte  BATT_SET_FIRST_REG       =0x0A;
const byte  BATT_READ_NUM            =8;
const byte  BATT_PWR_ON_RST          =0x40; //power on reset bit
const byte  BATT_STATUS_REG          =0x01;
const byte  BATT_BIAS_A_REG          =0x61;
const byte  BATT_BIAS_B_REG          =0x62;
const byte  BATT_ACC_CURR_A_REG      =0x11;
const byte  BATT_ACC_CURR_B_REG      =0x10;
const byte  BATT_CURRENT_OFFSET      =0xF0; //-16 in 2-complement
const float BATT_RSNS                =0.0143; //Value of I_Battery Sense Resistor
const float BATT_CAPACITY            =1.3; //Capacity of Battery - measured
const int   BATT_ACC_CURR_START      =round(BATT_CAPACITY*BATT_RSNS/0.00000625);

//define serial constants
const int   SERIAL_EMERGENCY_TIMEOUT =400; // Timeout for emergency-stop after loss of connection btw. arduino and smartphone (in ms)
const int   SERIAL_SEND_INTERVAL     =200;
const int   SERIAL_BAUDRATE          =9600;

//time variables
unsigned long receiveTime            = 0; // time stamp for receiveing data
unsigned long sendTime               = 0; // time stamp for sending data


//ultrasonic Distances
int distanceFront                    = -1;
int distanceBack                     = -1;

int speedLimit                       = MOTOR_MAX;

//send bytes
float Temp = -1;
float MCurr = -1;
float ACurr = -1;
float Volt = -1;

//received bytes
String rxBuffer = "";
byte   rxSpeed  = SPEED_DEFAULT;
byte   rxSteer  = STEER_DEFAULT;
byte   rxStatus = STATUS_DEFAULT;

Servo steeringServo;    //Define steeringServo as Variable

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

  //Start I2C Connection to Battery Management
  Wire.begin();
  
  //initialize Car
  carUpdate();

  /*If BatteryManagement Chip gets out of Power, it expects a battery change
   and therefore resets the accumulated current to BatteryCapacity */

  byte chipRegister = getBatteryChipStatusRegister();
  if((chipRegister & BATT_PWR_ON_RST) != 0){ //If PORF (Power on Reset Flag) bit is 1
    resetAccumulatedCurrent();
    setBatteryChipStatusRegister(chipRegister & (~(BATT_PWR_ON_RST)));
  }
  setCurrentOffset(0xF0); //-16 in two complements
  
  //turn on power led
  setLED(LED2_GREEN_POWER,HIGH);
}

void loop() 
{
  serialProtocolWrite();  
  emergencyCheck();  
  if(DEBUG){
    delay(500);
  }
}//loop()

void serialProtocolWrite(){
  if(sendTime + SERIAL_SEND_INTERVAL < millis()){
    getBatteryValues();
    distanceFront = measureDistance(FRONT);
    distanceBack = measureDistance(BACK);
    byte Buffer[SEND_BUFFER_LENGTH];
    
    Buffer[NUM_START]          = STARTBYTE;
    Buffer[NUM_VERSION_LENGTH] = SEND_VERSION_LENGTH;
    Buffer[1] = MCurr* -100 ;
    Buffer[2] = ACurr* 100;
    Buffer[3] = ACurr/BATT_CAPACITY*100;
    Buffer[4] = Volt*100;
    Buffer[5] = Temp;
    Buffer[6] = distanceFront;
    Buffer[7] = distanceBack;
    
    Serial.write(Buffer, sizeof(Buffer));
    sendTime = millis();
  }
}

void serialProtocolRead(){
  receiveTime = millis();// Set timestamp emergency-stop (last received command)
  while (Serial.available()) {
    // get the new char:
    char inChar = Serial.read();
    #ifdef DEBUG
      Serial.println(inChar,HEX);
    #endif
    if(inChar == STARTBYTE){
      rxBuffer = ""; 
    }
    rxBuffer += inChar;
    checkRxBuffer();    
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
    stopCar();
    setLED(LED1_RED_CONNECTION,HIGH);//Connection-Status LED on
  }else{
    setLED(LED1_RED_CONNECTION,LOW);//Connection-Status LED off
    if(getFailsafeStop()){
        //Measure Distance to Obstacles in Front or Back
        if(getDriveDir() == FRONT && getDriveStep() > 0 ){
          distanceFront = measureDistance(FRONT);
          limitSpeed(FRONT);//Limit maximum speed to protect car from hard crashes
        }
        else 
        if (getDriveDir() == BACK && getDriveStep() > 0){
          distanceBack = measureDistance(BACK);
          limitSpeed(BACK); //Limit maximum speed to protect car from hard crashes
        }
      }
  }
}


void checkRxBuffer(){
  if(rxBuffer.length() >= RECEIVE_BUFFER_LENGTH){
    if(rxBuffer[NUM_START] != STARTBYTE){
      rxBuffer = "";
      return;
    }
    if(rxBuffer[NUM_VERSION_LENGTH] != RECEIVE_VERSION_LENGTH){
      rxBuffer = "";
      return;
    }
    byte check = calcCheck();
    if(rxBuffer[NUM_RECEIVE_CHECK] != check){
      rxBuffer = "";
      return;
    }
    //update values
    setLED(LED1_RED_CONNECTION,LOW);
    rxSpeed  = rxBuffer[NUM_SPEED];
    rxSteer  = rxBuffer[NUM_STEER];
    rxStatus = rxBuffer[NUM_STATUS];
    carUpdate();
  }
}

byte calcCheck(){
  byte check = STARTBYTE;
  byte parity = 0;
  byte i = 0;
  for(i = 1; i < NUM_RECEIVE_CHECK; i++){
    check ^= rxBuffer[i];
  }
  
  for(i = 0; i < PARITY_BIT; i++){
    if(((check >> i) & CHECK_MSK) == CHECK_MSK){
      parity ^= PARITY_MSK;
    }
  }
  check &= ~PARITY_MSK; //unset bit 7;
  check |=  PARITY_MSK;//set parity bit
  return check;
}

int measureDistance(byte dir)
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
  long duration = pulseIn(echo, HIGH,ULTRASONIC_TIMEOUT);
  long distance = (int) duration/2/ULTRASONIC_SPEED; // Duration until Signal / 2 / Speed of Sound im microseconds/cm
  if (distance == 0 || distance > ULTRASONIC_MAX_RANGE){//Nothing in given max. Range
    distance = -1;
  }
  if(DEBUG){
    Serial.print("Distance");
    if(FRONT) Serial.print("Front:");
    else      Serial.print("Back :");
    Serial.println(distance);
  }
  return distance;
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
    stopCar();
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

void stopCar()
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

void limitSpeed(byte dir){
  int distance = distanceBack;
  if(dir == FRONT){
    distance = distanceFront;
  }
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

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////BATTERY/////
//////////////////////////////////////////////////////////////////////////////////////////////////

void getBatteryValues(){
  //Start Communication
  Wire.beginTransmission(BATT_SLAVE_DECIVE); // begin transmission with slave device #72 (BATT_SLAVE_DECIVE)
  Wire.write(BATT_SET_FIRST_REG);            // Set Register to first readable Register

  //Send Request and look for error code
  int error = Wire.endTransmission();

  // Receive 8 (BATT_SLAVE_DECIVE) Byte from device #72 (BATT_SLAVE_DECIVE)
  Wire.requestFrom(BATT_SLAVE_DECIVE, BATT_READ_NUM);      
  int TempA = Wire.read();       //Get first Temperature Register
  int TempB = Wire.read();       //Get second Temperature Register
  int VoltA = Wire.read();       //Get first Voltage Register
  int VoltB = Wire.read();       //Get second Voltage Register
  int MCurrA = Wire.read();     //Get first momentary Current Register
  int MCurrB = Wire.read();     //Get second momentary Current Register
  int ACurrA = Wire.read();     //Get first accumulated Current Register
  int ACurrB = Wire.read();     //Get second accumulated Current Register
  
  if(BATT_DEBUG_READ){
      Serial.print("TempA: ");
      Serial.println(TempA,3);
      Serial.print("TempB: ");
      Serial.println(TempB,3);
      Serial.print("VoltA: ");
      Serial.println(VoltA,4);
      Serial.print("VoltB: ");
      Serial.println(VoltB,4);
      Serial.print("MCurrA: ");
      Serial.println(MCurrA,5);
      Serial.print("MCurrB: ");
      Serial.println(MCurrB,5);
      Serial.print("ACurrA: ");
      Serial.println(ACurrA,5);
      Serial.print("ACurrB: ");
      Serial.println(ACurrB,5);
  }
  //Switch Register Values to match documentation
  TempA = TempA<<3;
  TempB = TempB>>5;
  VoltA = VoltA & 0x007F<<3;
  VoltB = VoltB>>5;
  MCurrA = MCurrA <<8;
  ACurrA = ACurrA <<8;

  //Calculate absolute Values from register values
  Temp = (TempA+TempB) * 0.125;
  Volt = (VoltA+VoltB) * 0.00488*2.257; // Measured Counter * Multiplier* VoltageDivider
  MCurr = (MCurrA + MCurrB) * 0.0000015625/BATT_RSNS;
  ACurr = (ACurrA + ACurrB) * 0.00000625/BATT_RSNS;

  if(BATT_DEBUG_VAL){
    if(error>0){
      Serial.print("Error I2C: ");
      Serial.println(error);
    }
    else{
      Serial.print("Temp: ");
      Serial.println(Temp,3);
      Serial.print("Volt: ");
      Serial.println(Volt,4);
      Serial.print("MCurr: ");
      Serial.println(MCurr,5);
      Serial.print("ACurr: ");
      Serial.println(ACurr,5);
      Serial.print("Available Capacity = ");
      Serial.println(ACurr/BATT_CAPACITY*100,0); // Available Capacity in Percent
    }
  }
}

byte getBatteryChipStatusRegister(){
  return getRegister(BATT_STATUS_REG);;
}

void setBatteryChipStatusRegister(int sendbyte){
  setRegister(BATT_STATUS_REG,sendbyte);
}

void getBatteryChipBiasRegister(){
  //?!?!?
  getRegister(BATT_BIAS_A_REG);
  getRegister(BATT_BIAS_B_REG);
}

void setCurrentOffset(int offset){
  setRegister(BATT_BIAS_A_REG,offset);
}

void setRegister(int registerbyte, int sendbyte){
  Wire.beginTransmission(BATT_SLAVE_DECIVE);    // begin transmission with slave device #72
  Wire.write(registerbyte); //Set register pointer to registerbyte
  Wire.write(sendbyte); // Write value of sendbyte to register
  int error = Wire.endTransmission(); // end trandmission
  if(BATT_DEBUG_WRITE){
    Serial.print("Write To Register ");
    Serial.print(registerbyte,HEX);
    Serial.print(" Value: ");
    Serial.println(sendbyte,HEX);
    if (error){
      Serial.print("I2C Error: ");
      Serial.println(error);
    }
  }
}

byte getRegister(byte registerByte){
  Wire.beginTransmission(BATT_SLAVE_DECIVE);    // begin transmission with slave device #72
  Wire.write(registerByte);  // set register pointer to registerbyte
  int error = Wire.endTransmission();
  Wire.requestFrom((int)BATT_SLAVE_DECIVE, 1) ; // ask for 1 byte from device #72
  byte returnByte = Wire.read(); // read given byte
  if(BATT_DEBUG_READ){
    Serial.print("Read from Register: ");
    Serial.print(registerByte,HEX);
    Serial.print(" Value: ");
    Serial.println(returnByte,HEX);
    if (error){
      Serial.print("I2C Error: ");
      Serial.println(error);
    }
  }
  return returnByte;
}

void resetAccumulatedCurrent(){
  int ACurrStartRegister = BATT_ACC_CURR_START;
  byte ACurrStartRegisterA = lowByte(ACurrStartRegister);
  byte ACurrStartRegisterB = highByte(ACurrStartRegister);
  setRegister(BATT_ACC_CURR_B_REG,ACurrStartRegisterB); //Value of battery Ah in HEX (1.3Ah)
  setRegister(BATT_ACC_CURR_A_REG,ACurrStartRegisterA); //Value of battery Ah in HEX (1.3Ah)
}
