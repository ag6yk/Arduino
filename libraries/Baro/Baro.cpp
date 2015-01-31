///////////////////////////////////////////////////////////////////////////////
//
//  ROBOKONG 2493
//  FRC 2015 - RECYCLING RUSH
//
//  CUSTOM ELECTRONICS
//
//  ARDUINO-BASED SENSOR ARRAY
//
///////////////////////////////////////////////////////////////////////////////

/*!
  @file Baro.cpp
  
  @brief BMP085 Barometer sensor class
  
  @details This class is a child of the sensor class and describes the 
   BMP085 Barometer
  
*/

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include "Baro.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINES
///////////////////////////////////////////////////////////////////////////////

//////////////////////////
// BMP085 Barometer
//////////////////////////

///////////////////////////////////////////////////////////////////////////////
// CLASS AND METHOD DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Constructor
Baro::Baro()
{
  // Placeholder
}

// Specialized initialization
int Baro::begin()
{
    // Locals
    boolean Responded = false;
    unsigned char ReadLow;
    unsigned char ReadHigh;
    unsigned char RetryCount = 0;
    int i;

    // Only one fixed address
    _i2cAddress = B_IC_ADDR;

    // Check for the device
    Responded = readID();

    if(Responded == false)
    {
        return -1;
    }

    // Device responded
    // Read out the coefficients
    Wire.beginTransmission(_i2cAddress);
    Wire.write(byte(B_EEPROM));
    Wire.endTransmission();
    Wire.requestFrom(_i2cAddress, byte(22));
    Responded = false;
    // Wait for the data with timeout (approx 500 usec)
    while(Responded == false)
    {
        // 11 16-bit values from the EEPROM
        if(Wire.available() >= 22)
        {
            Responded = true;
            break;
        }
        RetryCount++;
        if(RetryCount > 10)
        {
            break;
        }
        delayMicroseconds(50);
    }

    if(Responded == false)
    {
        return -1;
    }

    // Device responded
    for(i=0; i < 22; i++)
    {
        _eeCoefficients[i] = Wire.read();
    }

    return 0;
}

// Read device ID
// really read the EEPROM byte 0
boolean Baro::readID()
{
    // Locals
    boolean Responded = false;
    unsigned char ReadBack;
    int RetryCount = 0;

    Wire.beginTransmission(_i2cAddress);      // Address
    Wire.write(byte(B_EEPROM));               // byte 0 of EEPROM
    Wire.endTransmission();                   // send
    Wire.requestFrom(_i2cAddress, byte(1));   // get the byte
    // Wait for response with timeout - total timeout approx 500 usec
    while(Responded == false)
    {
        if(Wire.available() >= 1)
        {
            Responded = true;
            break;
        }
        RetryCount++;
        if(RetryCount > 10)
        {
            break;
        }
        delayMicroseconds(50);
    }

    // Bus responded check if the value is OK
    if(Responded == true)
    {
        ReadBack = Wire.read();                   // read ID byte

        // EEPROM should not be 0 or 0xFF
        if((ReadBack != 0x00) && (ReadBack != 0xFF))
        {
            return true;                          // good read
        }
        return false;                             // bus error
    }
    return false;
}


// Read and filter the barometric pressure
int Baro::ReadPressure()
{
  return 0;
}

// Read and filter the temperature
int Baro::ReadTemperature()
{
  return 0;
}

// Signal averaging
int Baro::AvgFilter(int *Data)
{
  return 0;
}

// Destructor
Baro::~Baro()
{
  // Do nothing
}

// Default instantiation
Baro    imuBarometer = Baro();


