///////////////////////////////////////////////////////////////////////////////
//
//  ROBOKONG 2493
//  FRC 2015 - RECYCLNG RUSH
//
//  CUSTOM ELECTRONICS
//
//  ARDUINO-BASED SENSOR ARRAY
//
///////////////////////////////////////////////////////////////////////////////

/*!
  @file Baro.h
  
  @brief BMP085 Barometer sensor class
  
  @details This class is a child of the sensor class and describes the 
   BMP085 Barometer
  
*/

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

#ifndef Baro_h
#define Baro_h

#include "Arduino.h"
#include "Sensor.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINES
///////////////////////////////////////////////////////////////////////////////

//////////////////////////
// BMP085 Barometer
//////////////////////////

// I2C Address
#define  B_IC_ADDR    (0xEF >> 1)          // Wire class uses 7-bit address
#define  B_READ       0xEF
#define  B_WRITE      0xEE

// Register addresses
#define  B_EEPROM     0xAA                 // EEPROM coefficients
#define  B_EE_END     0xBF

// Convert
#define  B_CONV_CNT   0xF4

// Command Opcodes
#define  B_TEMP_RD    0x2E
#define  B_PRESS_RD   0x34

// Output data
#define  B_DATA_MSB   0xF6
#define  B_DATA_LSB   0xF7



///////////////////////////////////////////////////////////////////////////////
// CLASS AND METHOD DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

class Baro : public Sensor
{
    private:
        unsigned char _i2cAddress;
        unsigned int  _eeCoefficients[11];
        signed short _bPressure;
        signed short _bTemperature;
    public:
        Baro();                             // Constructor
        ~Baro();                            // Destructor
        boolean readID();                   // Device ID (verify bus)
        int begin();                        // Specialized initialization
        int ReadPressure();
        int ReadTemperature();
        int ProcessPressureData();          // measure and report the
                                            // atmospheric pressure
        int ProcessTemperatureData();       // measure and report the temperature
        signed short getPressure();         // accessors
        signed short getTemperature();
};

// Default instantiation
extern  Baro    imuBarometer;

#endif
