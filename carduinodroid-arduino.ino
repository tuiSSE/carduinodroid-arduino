//  Vesion History
//
//  Version   Date         Feature/Fix
//
//  0.8       28.10.2013   New Protocol by Harald Funk
//  0.9       03.05.2014   Emergency-Shutdown added
//  1.0       29.05.2015   Auto-Stop + some fixes

#include <Wire.h> //Support for I2C Connection
//#include <Usb.h> //Support for USB Connection
//#include <AndroidAccessory.h> //Support for Transfering Commands
#include <Servo.h>  //Support for Servo Motor

Servo servo_motor;    //Define servoMain as Variable
int DEBUG = 0;

//Define Constants
int maximumNumberOfBytesSent= 7; //Maximum Bytes sent by Smartphone for one protocol frame
int LightValue = 1023; //Dimming of Light LEDs
int trans_pos = 8;
int min_degree = 50;  //Min Degree 50(-90) - 90 means the wheels are aligned
int max_degree = 130; //Max Degree (90-)130 - both sides with an angle of 40 degree
float Rsns = 0.0143; //Value of I_Battery Sense Resistor
float batteryCapacity = 1.3; //Capacity of Battery
int maxSensorRange = 300; // Maximum Range of Ultrasonic Sensors to achive in cm
int sonicSpeed = 343;  // Speed of Sound
long SensorTimeout = 2*1000000*maxSensorRange/100/sonicSpeed; // max. time a sensorsignal can need to get back to the sensor in µs
int emergencyTimeout = 400; // Timeout for emergency-stop after loss of connection btw. arduino and smartphone (in ms)

unsigned long time; // timestamp in ms
unsigned long sendTime; // time stamp/interval for sending data

int step_degree; 
int stepsize_dc;
long distanceToObstacle = -1;
int speedMaximum = 9;
int controlValues[7] = {
  -1,-1,-1,-1,-1,-1,-1}; //Values received by Android-Smartphone over USB/RDS232
float Temp = -1;
float MCurr = -1;
float ACurr = -1;
float Volt = -1;
long distanceFront = -1;
long distanceBack = -1;

// Set Pins to Variables
int mInput14Pin = 9;
int mInput23Pin = 2;
int mEnablePin = 3;
int s1TriggerPin = 4;
int s2TriggerPin = 6;
int s1EchoPin = 7;
int s2EchoPin = 8;
int LED1Pin = 10;
int lEnablePin = 11;
int LED2Pin = 12;
int LED3Pin = 13;
int mDirection=-1;

int counter = 0;

int transition_dc = 9;

void setup()
{
  //set Pinmode
  pinMode(mInput14Pin,OUTPUT);
  pinMode(mInput23Pin,OUTPUT);
  pinMode(mEnablePin,OUTPUT);
  pinMode(s1TriggerPin,OUTPUT);
  pinMode(s2TriggerPin,OUTPUT);
  pinMode(s1EchoPin,INPUT);
  pinMode(s2EchoPin,INPUT);
  pinMode(LED1Pin,OUTPUT);
  pinMode(lEnablePin,OUTPUT);
  pinMode(LED2Pin,OUTPUT);
  pinMode(LED3Pin,OUTPUT);

  Serial.begin(9600);  //
  servo_motor.attach(5);   
  servo_motor.write(90);

  step_degree = (90 - min_degree)/(trans_pos);

  //Start I2C Connection to Battery Management
  Wire.begin();


  /*If BatteryManagement Chip gets out of Power, it expects a battery change
   and therefore resets the accumulated current to BatteryCapacity */

  byte chipRegister = getBatteryChipStatusRegister();
  if((chipRegister & 0x40) != 0){ //If PORF (Power on Reset Flag) bit is 1
    resetAccumulatedCurrent();
    setBatteryChipStatusRegister(chipRegister & (~(0x40)));
  }
  setCurrentOffset(0xF0); //-16 in two complements

  digitalWrite(LED2Pin,HIGH);

  stop_Car();
}

void loop() 
{
  int x;
  int i=0;
  //  float batteryValues[5];
  // motorStop(); // Motor Stop
  //Connect to Smartphone via Serial
  
  
  
  if(sendTime + 200 < millis()){
    getBatteryValues();
    distanceFront = measureDistance(s1TriggerPin,s1EchoPin);
    distanceBack = measureDistance(s2TriggerPin,s2EchoPin);
    byte protocol = 1;
    byte bytesToSend =7;
    byte Buffer[bytesToSend+1];
    Buffer[0] = 0x00 | protocol<<6 | bytesToSend;
    Buffer[1] = MCurr* -100 ;
    Buffer[2] = ACurr* 100;
    Buffer[3] = ACurr/batteryCapacity*100;
    Buffer[4] = Volt*100;
    Buffer[5] = Temp;
    Buffer[6] = distanceFront;
    Buffer[7] = distanceBack;

    Serial.write(Buffer, sizeof(Buffer));

//    Serial.print("Temp: "); 25.252
//    Serial.println(Temp,3);
//    Serial.print("Volt: "); 1.105
//    Serial.println(Volt,4);
//    Serial.print("MCurr: "); 0.600
//    Serial.println(MCurr,5);
//    Serial.print("ACurr: ");
//    Serial.println(ACurr,5);
//    Serial.print("Available Capacity = ");
//    Serial.println(ACurr/batteryCapacity*100,0); // Available Capacity in Percent
    sendTime = millis();
  }


  if(Serial.available() >= 1)
  {

    time = millis();    // Set timestamp emergency-stop (last received command)

    int length=0;
    int protocol = 0;
    int protocolHeader = 0;
    digitalWrite(LED1Pin,HIGH); //Set Connection-Status LED on
    if(DEBUG>0){
      protocolHeader = Serial.read();
      protocol = getProtocol(protocolHeader);
      Serial.print("protocol: ");
      Serial.println(protocol);
      length = getLength(protocolHeader);
      Serial.print("length: ");
      Serial.println(length);
    }
    else{
      protocolHeader = Serial.read();
      protocol = getProtocol(protocolHeader);
      length = getLength(protocolHeader);
    }
    if(length > maximumNumberOfBytesSent){
      length = maximumNumberOfBytesSent;
    }
    while (i<length) {
      if(DEBUG>0){
        controlValues[i] = Serial.read();
        if(controlValues[i] == -49){
          controlValues[i] = -1;
        }
        //Serial.print("Serial.read: ");
        //Serial.println(controlValues[i]);

      }
      else{
        controlValues[i] = Serial.read();
      }
      if(controlValues[i] != -1){
        if(DEBUG>0){
          Serial.println(controlValues[i]);
        }
        i++;
      }
    }
    if(length>=1){
      set_speed_DC(getDrivingDirection(controlValues[0]),getDrivingValue(controlValues[0]));  //direction / speed
      if(DEBUG>0){
        Serial.print("SetSpeed: ");
        Serial.println(getDrivingValue(controlValues[0]));
        Serial.print("SetDirection: ");
        Serial.println(getDrivingDirection(controlValues[0]));
      }
    }
    if (length >= 2){
      set_direction_Car(getDrivingDirection(controlValues[1]),getDrivingValue(controlValues[1])); //direction / steps
      if(DEBUG>0){
        Serial.print("SetSteps: ");
        Serial.println(getDrivingValue(controlValues[1]));
        Serial.print("SetDirection: ");
        Serial.println(getDrivingDirection(controlValues[1]));
      }
    }
    if (length >= 3){
      setLED(LED3Pin,getLEDStatus(controlValues[2]));
      if(DEBUG>0){
        Serial.print("SetLED: ");
        Serial.println(getLEDStatus(controlValues[2]));
      }
      setLight(getLightStatus(controlValues[2]),LightValue);
      if(DEBUG>0){
        Serial.print("SetLight: ");
        Serial.println(getLightStatus(controlValues[2]));
      }
      if(getRAC(controlValues[2])==1){
        resetAccumulatedCurrent();
        if(DEBUG>0){
          Serial.print("ResetRAC");
        }
      }
    }
    //    i++;
    if(length>0){
      //Send Data
      if(DEBUG>0){
        Serial.println(B01000101);
        Serial.println(round(MCurr*1000));
        Serial.println(round(ACurr*1000));
        Serial.println(round(ACurr/batteryCapacity*100));
        Serial.println(round(Volt*1000));
        Serial.println(round(Temp));
      }
      else{   
        //         byte protocol = 1;
        //         byte bytesToSend =7;
        //         byte Buffer[bytesToSend+1];
        //         Buffer[0] = 0x00 | protocol<<6 | bytesToSend;
        ////         Buffer[0] = 0x44;
        //         Buffer[1] = round(MCurr*1000);
        //         Buffer[2] = round(ACurr*1000);
        //         Buffer[3] = round(ACurr/batteryCapacity*100);
        //         Buffer[4] = round(Volt*1000);
        //         Buffer[5] = round(Temp);
        //         Buffer[6] = distanceFront;
        //         Buffer[7] = distanceBack;
        //  
        //         Serial.write(Buffer, sizeof(Buffer));


        //           if(i> 0){
        //             Serial.write(B01000101);
        //             Serial.write(10);
        //             Serial.write(13);
        //             Serial.write(round(MCurr*1000));
        //             Serial.write(10);
        //             Serial.write(13);
        //             Serial.write(round(ACurr*1000));
        //             Serial.write(10);
        //             Serial.write(13);
        //             Serial.write(round(ACurr/batteryCapacity*100));
        //             Serial.write(10);
        //             Serial.write(13);
        //             Serial.write(round(Volt*1000));
        //             Serial.write(10);
        //             Serial.write(13);
        //             Serial.write(round(Temp));
        //             Serial.write(10);
        //             Serial.write(13); 
        //             i=0;
        //           }
      }
    }


  } 

//  set_speed_DC(1,2); // just for current mesurement
//  setLight(true, 1023);
  if (millis()-time > emergencyTimeout){  // Stop car after emergency-timeout 
    stop_Car();  
  }else{
    digitalWrite(LED1Pin,LOW);//Set Connection-Status LED off

        //Measure Distance to Obstacles in Front or Back
        if(getDrivingDirection(controlValues[0]) == 1 && getDrivingValue(controlValues[0]) > 0 ){
          distanceFront = measureDistance(s1TriggerPin,s1EchoPin);
          
          if (getASE(controlValues[2]) > 0 ){
              limitSpeed(distanceFront); //Limit maximum speed to protect car from hard crashes
          }
        }
        else if (getDrivingDirection(controlValues[0]) == 0 && getDrivingValue(controlValues[0]) > 0){
          distanceBack = measureDistance(s2TriggerPin,s2EchoPin);
          if (getASE(controlValues[2]) > 0 ){
              limitSpeed(distanceBack); //Limit maximum speed to protect car from hard crashes
          }    
        }

  }

  //Ask for Battery Status
  // getBatteryValues(batteryValues);



  if(DEBUG>0){
    delay(500);
  }

}


//void getBatteryValues(float values[]){
void getBatteryValues(){
  //Start Communication
  Wire.beginTransmission(72);    // begin transmission with slave device #72
  Wire.write(0x0A);              // Set Register to first readable Register

  //Send Request and look for error code
  int error = Wire.endTransmission();

  // Receive 8 Byte from device #72
  Wire.requestFrom(72, 8) ;      
  int TempA = Wire.read();       //Get first Temperature Register
  int TempB = Wire.read();       //Get second Temperature Register
  int VoltA = Wire.read();       //Get first Voltage Register
  int VoltB = Wire.read();       //Get second Voltage Register
  int MCurrA = Wire.read();     //Get first momentary Current Register
  int MCurrB = Wire.read();     //Get second momentary Current Register
  int ACurrA = Wire.read();     //Get first accumulated Current Register
  int ACurrB = Wire.read();     //Get second accumulated Current Register
  
//      Serial.print("TempA: ");
//      Serial.println(TempA,3);
//      Serial.print("TempB: ");
//      Serial.println(TempB,3);
//      Serial.print("VoltA: ");
//      Serial.println(VoltA,4);
//      Serial.print("VoltB: ");
//      Serial.println(VoltB,4);
//      Serial.print("MCurrA: ");
//      Serial.println(MCurrA,5);
//      Serial.print("MCurrB: ");
//      Serial.println(MCurrB,5);
//      Serial.print("ACurrA: ");
//      Serial.println(ACurrA,5);
//      Serial.print("ACurrB: ");
//      Serial.println(ACurrB,5);

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
  MCurr = (MCurrA + MCurrB) * 0.0000015625/Rsns;
  ACurr = (ACurrA + ACurrB) * 0.00000625/Rsns;
//      Serial.print("Temp: ");
//      Serial.println(Temp,3);
//      Serial.print("Volt: ");
//      Serial.println(Volt,4);
//      Serial.print("MCurr: ");
//      Serial.println(MCurr,5);
//      Serial.print("ACurr: ");
//      Serial.println(ACurr,5);
  //  values[0] = Temp;
  //  values[1] = Volt;
  //  values[2] = MCurr;  
  //  values[3] = ACurr;
  //  values[4] = error;
  if(DEBUG>1){
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
      Serial.println(ACurr/batteryCapacity*100,0); // Available Capacity in Percent
    }
  }
}

long measureDistance(int TriggerPin, int EchoPin)
{
  //Send Signal
  digitalWrite(TriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TriggerPin, LOW);

  //wait for receiving the signal (max. waittime = Sensor-Timeout)
  long duration = pulseIn(EchoPin, HIGH,SensorTimeout);
  long distance = (duration/2) / 29.1; // Duration until Signal / 2 / Speed of Sound im microseconds/cm
  if (distance == 0 || distance > maxSensorRange){//Nothing in given max. Range
    distance = -1; 
  }
  if(DEBUG>0){
    Serial.print("Distance: ");
    Serial.println(distance);
  }
  return distance;
}



void set_direction_Car(int direction, int steps)
{
  if(abs(steps)<=8)
  {
    if(direction == 1)
      servo_motor.write(90 + steps*step_degree);
    else
      servo_motor.write(90 - steps*step_degree);  
  }
  else
    servo_motor.write(90);
}


void set_speed_DC(int direction, int speed){
  if((speed > 9) || (speed < 1))
  {
    stop_Car();
    return;
  }
  if(speed > speedMaximum){
    speed = speedMaximum;
    if(DEBUG > 0){
      Serial.print("Speed limited to ");
      Serial.println(speed);
    }
  }

  if(direction == 1)
  {
    digitalWrite(mEnablePin,LOW); //To prevent H-Bridge from short-circuit
    digitalWrite(mInput23Pin,LOW);
    digitalWrite(mInput14Pin,HIGH);
    analogWrite(mEnablePin,75+speed*20);
  }
  else if(direction == 0)
  {
    digitalWrite(mEnablePin,LOW); //To prevent H-Bridge from short-circuit
    digitalWrite(mInput14Pin,LOW);
    digitalWrite(mInput23Pin,HIGH);
    analogWrite(mEnablePin,75+speed*20);
  } 
  else {
    stop_Car();
  }
}


void stop_Car()
{
  digitalWrite(mInput14Pin,LOW);
  digitalWrite(mInput23Pin,LOW);     // stopped
  digitalWrite(mEnablePin,LOW);
}

void setLight(boolean state, int value){
  if(state == true){
    analogWrite(lEnablePin,value);
  }
  else{
    analogWrite(lEnablePin,0);
  }
}

byte getBatteryChipStatusRegister(){
  byte stat = getRegister(0x01);
  return(stat);
}

void setBatteryChipStatusRegister(int sendbyte){
  setRegister(0x1,sendbyte);
}

void getBatteryChipBiasRegister(){
  getRegister(0x61);
  getRegister(0x62);
}

void setCurrentOffset(int offset){
  setRegister(0x61,offset);
}

void setRegister(int registerbyte, int sendbyte){
  Wire.beginTransmission(72);    // begin transmission with slave device #72
  Wire.write(registerbyte); //Set register pointer to registerbyte
  Wire.write(sendbyte); // Write value of sendbyte to register
  int error = Wire.endTransmission(); // end trandmission
  if(DEBUG>0){
    Serial.print("Write To Register ");
    Serial.print(registerbyte,HEX);
    Serial.print(" Value: ");
    Serial.println(sendbyte,HEX);
    if (error!=0){
      Serial.print("I2C Error: ");
      Serial.println(error);
    }
  }
}
byte getRegister(byte registerByte){
  Wire.beginTransmission(72);    // begin transmission with slave device #72
  Wire.write(registerByte);  // set register pointer to registerbyte
  int error = Wire.endTransmission();
  Wire.requestFrom(72, 1) ; // ask for 1 byte from device #72
  byte returnByte = Wire.read(); // read given byte
  if(DEBUG>0){
    Serial.print("Read from Register: ");
    Serial.print(registerByte,HEX);
    Serial.print(" Value: ");
    Serial.println(returnByte,HEX);
    if (error!=0){
      Serial.print("I2C Error: ");
      Serial.println(error);
    }
  }
  return returnByte; 
}

void resetAccumulatedCurrent(){
  int ACurrStartRegister = round(batteryCapacity*Rsns/0.00000625); // Calculating 
  byte ACurrStartRegisterA = lowByte(ACurrStartRegister);
  byte ACurrStartRegisterB = highByte(ACurrStartRegister);
  setRegister(0x10,ACurrStartRegisterB); //Value of battery Ah in HEX (1.3Ah)
  setRegister(0x11,ACurrStartRegisterA); //Value of battery Ah in HEX (1.3Ah)
}

void setLED(int LEDpin, int value){
  if (value == 1){
    digitalWrite(LEDpin,HIGH);
  }
  else{
    digitalWrite(LEDpin,LOW);
  }
}

void limitSpeed(long distance){
  if (distance < 200){
    speedMaximum = 8;
  }
  if (distance < 100){
    speedMaximum = 5;
  }
  if (distance < 50){
    speedMaximum = 3;
  }
  if (distance < 10){
    speedMaximum = 1;
  }
  if (distance == -1){
    speedMaximum = 9;
  }
  set_speed_DC(getDrivingDirection(controlValues[0]),getDrivingValue(controlValues[0]));
}

int getProtocol(int value){
  //Serial.println(value,BIN);
  int protocol = 0;
  int i;
  for( i=6; i<7;i++){
    if(bitRead(value,i) == 1){
      bitSet(protocol,i-6);
    }
  }
  return protocol;
}

int getLength(int value){
  int length = 0;
  int i;
  for( i=0; i<4;i++){
    if(bitRead(value,i) == 1){
      bitSet(length,i);
    }
  }
  return length;
}

int getDrivingValue(int value){
  int drivingV = 0;
  int i;
  for( i=4; i<8;i++){
    if(bitRead(value,i) == 1){
      bitSet(drivingV,i-4);
    }
  }
  return drivingV;
}

int getDrivingDirection(int value){
  return bitRead(value,3);
}

int getLEDStatus(int value){
  return bitRead(value,7);
}
int getLightStatus(int value){
  return bitRead(value,6);
}
int getRAC(int value){
  return bitRead(value,3);
}

int getASE(int value){       // get Auto-Stop Enable
  return bitRead(value,5);
}

