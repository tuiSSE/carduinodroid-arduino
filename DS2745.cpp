
/*
 * DS27455.c
 * 
 * author   : Till Max Schwikal
 * date     : 13.12.2015
 * url      : https://github.com/tuiSSE/carduinodroid-wiki/wiki/
 */

#include "DS2745.h"
 
DS2745::DS2745(){
	temperature = -1;
	current = -1;
	accumulatedCurrent = -1;
	voltage = -1;
}

DS2745::~DS2745(){
}

byte DS2745::getTemperature(){
	return (byte) (temperature);
}
byte DS2745::getCurrent(){
	return (byte) (current* -100);
}
byte DS2745::getAccCurrent(){
	return (byte) (accumulatedCurrent* 100);
}
byte DS2745::getRelAccCurrent(){
	return (byte) (accumulatedCurrent/DS2745_CAPACITY*100);
}
byte DS2745::getVoltage(){
	return (byte) (voltage*100);
}


void DS2745::init(){
  //Start I2C Connection to Battery Management
  Wire.begin();
  /*If BatteryManagement Chip gets out of Power, it expects a battery change 
    and therefore resets the accumulated current to BatteryCapacity */

  byte chipRegister = getStatusRegister();
  if((chipRegister & DS2745_PWR_ON_RST) != 0){ //If PORF (Power on Reset Flag) bit is 1
    resetAccumulatedCurrent();
    setStatusRegister(chipRegister & (~(DS2745_PWR_ON_RST)));
  }
}


void DS2745::update(){
  //Start Communication
  Wire.beginTransmission(DS2745_SLAVE_DECIVE); // begin transmission with slave device #72 (DS2745_SLAVE_DECIVE)
  Wire.write(DS2745_SET_FIRST_REG);            // Set Register to first readable Register

  //Send Request and look for error code
  int error = Wire.endTransmission();

  // Receive 8 (DS2745_SLAVE_DECIVE) Byte from device #72 (DS2745_SLAVE_DECIVE)
  Wire.requestFrom(DS2745_SLAVE_DECIVE, DS2745_READ_NUM);      
  int temperatureA = Wire.read();        //Get first temperature Register
  int temperatureB = Wire.read();        //Get second temperature Register
  int voltageA = Wire.read();            //Get first voltage Register
  int voltageB = Wire.read();            //Get second voltage Register
  int currentA = Wire.read();            //Get first momentary Current Register
  int currentB = Wire.read();            //Get second momentary Current Register
  int accumulatedCurrentA = Wire.read(); //Get first accumulated Current Register
  int accumulatedCurrentB = Wire.read(); //Get second accumulated Current Register
  
  if(DS2745_DEBUG_READ){
      Serial.print("temperatureA: ");
      Serial.println(temperatureA,3);
      Serial.print("temperatureB: ");
      Serial.println(temperatureB,3);
      Serial.print("voltageA: ");
      Serial.println(voltageA,4);
      Serial.print("voltageB: ");
      Serial.println(voltageB,4);
      Serial.print("currentA: ");
      Serial.println(currentA,5);
      Serial.print("currentB: ");
      Serial.println(currentB,5);
      Serial.print("accumulatedCurrentA: ");
      Serial.println(accumulatedCurrentA,5);
      Serial.print("accumulatedCurrentB: ");
      Serial.println(accumulatedCurrentB,5);
  }
  //Switch Register Values to match documentation
  temperatureA = temperatureA<<3;
  temperatureB = temperatureB>>5;
  voltageA = voltageA & 0x007F<<3;
  voltageB = voltageB>>5;
  currentA = currentA <<8;
  accumulatedCurrentA = accumulatedCurrentA <<8;

  //Calculate absolute Values from register values
  temperature = (temperatureA+temperatureB) * DS2745_TEMPERATURE_MUL;
  voltage = (voltageA+voltageB) * DS2745_VOLTAGE_MUL*DS2745_VOLTAGE_DIVIDER; // Measured Counter * Multiplier* voltageageDivider
  current = (currentA + currentB) * DS2745_CURRENT_MUL/DS2745_RSNS;
  accumulatedCurrent = (accumulatedCurrentA + accumulatedCurrentB) * DS2745_CAPACITY_MUL/DS2745_RSNS;

  if(DS2745_DEBUG_VAL){
    if(error>0){
      Serial.print("Error I2C: ");
      Serial.println(error);
    }
    else{
      Serial.print("temperature: ");
      Serial.println(temperature,3);
      Serial.print("voltage: ");
      Serial.println(voltage,4);
      Serial.print("current: ");
      Serial.println(current,5);
      Serial.print("accumulatedCurrent: ");
      Serial.println(accumulatedCurrent,5);
      Serial.print("Available Capacity = ");
      Serial.println(accumulatedCurrent/DS2745_CAPACITY*100,0); // Available Capacity in Percent
    }
  }
}


void DS2745::resetAccumulatedCurrent(){
  int accCurrStartRegister = DS2745_ACC_CURR_START;
  byte accCurrStartRegisterLsb = lowByte(accCurrStartRegister);
  byte accCurrStartRegisterMsb = highByte(accCurrStartRegister);
  setRegister(DS2745_ACC_CURR_MSB_REG,accCurrStartRegisterMsb); //Value of battery Ah in HEX (1.3Ah)
  setRegister(DS2745_ACC_CURR_LSB_REG,accCurrStartRegisterLsb); //Value of battery Ah in HEX (1.3Ah)
  setCurrentOffset(DS2745_CURRENT_OFFSET); //-16 in two complements
}

byte DS2745::getStatusRegister(){
  return getRegister(DS2745_STATUS_REG);
}

void DS2745::setStatusRegister(int sendbyte){
  setRegister(DS2745_STATUS_REG,sendbyte);
}

void DS2745::setCurrentOffset(int offset){
  setRegister(DS2745_BIAS_A_REG,offset);
}

void DS2745::setRegister(int registerbyte, int sendbyte){
  Wire.beginTransmission(DS2745_SLAVE_DECIVE);    // begin transmission with slave device #72
  Wire.write(registerbyte); //Set register pointer to registerbyte
  Wire.write(sendbyte); // Write value of sendbyte to register
  int error = Wire.endTransmission(); // end trandmission
  if(DS2745_DEBUG_WRITE){
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

byte DS2745::getRegister(int registerByte){
  Wire.beginTransmission(DS2745_SLAVE_DECIVE);    // begin transmission with slave device #72
  Wire.write(registerByte);  // set register pointer to registerbyte
  int error = Wire.endTransmission();
  Wire.requestFrom((int)DS2745_SLAVE_DECIVE, 1); // ask for 1 byte from device #72
  byte returnByte = Wire.read(); // read given byte
  if(DS2745_DEBUG_READ){
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

