/*
 * DS27455.h
 * 
 * author   : Till Max Schwikal
 * date     : 13.12.2015
 * url      : https://github.com/tuiSSE/carduinodroid-wiki/wiki/
 */

#ifndef DS2745.h
#define DS2745.h

#include <Wire.h>

class DS2745{
	public:
		DS2745();
		~DS2745();
		
		byte getTemperature();
		byte getCurrent();
		byte getAccCurrent();
		byte getRelAccCurrent();
		byte getVoltage();
	
		void resetAccumulatedCurrent();
		void init();
		void update();
	private:
		byte getStatusRegister();
		void setStatusRegister(int sendbyte);
		void setCurrentOffset(int offset);
		byte getRegister(byte registerByte);
		void setRegister(int registerbyte, int sendbyte);
		float temperature;
		float current;
		float accumulatedCurrent;
		float voltage;
	
	
		const byte  DS2745_DEBUG_READ          =0;
		const byte  DS2745_DEBUG_WRITE         =0;
		const byte  DS2745_DEBUG_VAL           =0;
		
		//register
		const byte  DS2745_SLAVE_DECIVE        =72;
		const byte  DS2745_SET_FIRST_REG       =0x0A;
		const byte  DS2745_READ_NUM            =8;
		const byte  DS2745_PWR_ON_RST          =0x40; //power on reset mask
		const byte  DS2745_STATUS_REG          =0x01;
		const byte  DS2745_BIAS_A_REG          =0x61;
		const byte  DS2745_BIAS_B_REG          =0x62;
		const byte  DS2745_ACC_CURR_MSB_REG    =0x11;
		const byte  DS2745_ACC_CURR_LSB_REG      =0x10;
		
		//values
		const byte  DS2745_CURRENT_OFFSET      =0xF0; //-16 in 2-complement
		const float DS2745_RSNS                =0.0143; //Value of I_Battery Sense Resistor
		const float DS2745_CAPACITY            =1.3; //Capacity of Battery - measured
		const float DS2745_TEMPERATURE_MUL     =0.125;
		const float DS2745_VOLTAGE_MUL         =0.00488;
		const float DS2745_VOLTAGE_DIVIDER     =2.257;
		const float DS2745_CURRENT_MUL         =0.0000015625;
		const float DS2745_CAPACITY_MUL        =0.00000625;
		const int   DS2745_ACC_CURR_START      =round(DS2745_CAPACITY*DS2745_RSNS/DS2745_CAPACITY_MUL);
	
}

#endif