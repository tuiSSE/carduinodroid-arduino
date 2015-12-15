/*
 * debug.h
 * 
 * author   : Till Max Schwikal
 * date     : 15.12.2015
 * url      : https://github.com/tuiSSE/carduinodroid-wiki/wiki/
 */


#ifdef DEBUG
  #define DEBUG_PRINT(x)        Serial.print (x)
  #define DEBUG_PRINT_HEX(x)    Serial.print (x, HEX)
  #define DEBUG_PRINTLN(x)      Serial.println (x)
  #define DEBUG_PRINTLN_HEX(x)  Serial.print (x, HEX)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTDEC(x)
  #define DEBUG_PRINTLN(x) 
  #define DEBUG_PRINTLN_HEX(x)
#endif

