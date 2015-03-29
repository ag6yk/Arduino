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
  @file Compass.cpp
  
  @brief HMC5883L Digital Compass sensor class
  
  @details This class is a child of the sensor class and describes the 
   HMC5883L Digital Compass 
  
*/

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include "Compass.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINES
///////////////////////////////////////////////////////////////////////////////
#define BENCH_DISPLAY_DEBUG	0

#if BENCH_DISPLAY_DEBUG
#define dbg_print(x)        Serial.print(x)
#define dbg_printm(x,y)     Serial.print(x,y)
#define dbg_println(x)      Serial.println(x)
#define dbg_printlnm(x,y)   Serial.println(x,y)
#else
#define dbg_print(x)
#define dbg_printm(x,y)
#define dbg_println(x)
#define dbg_printlnm(x,y)
#endif


///////////////////////////////////////////////////////////////////////////////
// CLASS AND METHOD DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Constructor
Compass::Compass()
{
    // Initialize private variables
    _i2cAddress = 0;
    _cVectorX = 0;
    _cVectorY = 0;
    _cVectorZ = 0;
    _cCompassHeading = 0;
    _cData.xLSB = 0;
    _cData.xMSB = 0;
    _cData.yLSB = 0;
    _cData.yMSB = 0;
    _cData.zLSB = 0;
    _cData.zMSB = 0;

}

// Specialized initialization
int Compass::begin(void)
{
  // Locals
  boolean Responded;
  // Read Id register A to verify
  // device activity
  Responded = readID();
  if(Responded == false)
  {
      return -1;
  }


  // Configuration register A
  // Normal measurement mode
  // Default output rate
  // Signal averaging 8
  // 0b01110000
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(C_CONFIG_A));             // Multi-byte writes by default
  Wire.write(byte(0x70));
  
  // Configuration register B
  // Use defaults (0x20)
  Wire.write(byte(0x20));                   // write so that address increments
  
  // Mode register
  // Normal speed I2C
  // Single measurement mode
  // 0b00000001
  Wire.write(byte(0x01));
  Wire.endTransmission();
  
  // NICE-TO-HAVE
  // Run self test

  return 0;
}

// Read the Device ID
boolean Compass::readID()
{
    // Locals
    unsigned char ReadBack;
    boolean       Responded = false;
    int           RetryCount = 0;

    // No alternative addressing
    _i2cAddress = C_IC_ADR;

    // Send the request for the ID
    Wire.beginTransmission(_i2cAddress);
    Wire.write(byte(C_ID_A));
    Wire.endTransmission();
    Wire.requestFrom(_i2cAddress, byte(1));

    // Wait for response w/ timeout (approx 500 usecs total)
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
            Responded = false;
            break;
        }
        delayMicroseconds(50);
    }

    // Device responded, now check for the expected value
    if(Responded == true)
    {
        ReadBack = Wire.read();
        if(ReadBack != C_ID_A_VAL)
        {
            return false;                       // bus error
        }
        return true;                            // all OK
    }
    return false;
}

// Batch read the magnetic coordinate values
// Much more efficient than separate reads
// Called after MDRDY is asserted
int Compass::ReadXYZ()
{
    // Locals
    unsigned char RetryCount = 0;
    boolean Responded = false;

    // Point to the device
    Wire.beginTransmission(_i2cAddress);
    // Select mag data
    Wire.write(byte( C_X_MSB));
    Wire.endTransmission();
    Wire.requestFrom(_i2cAddress, byte(0x06));

    // Delay for the transmission/reception time @ 100 KHz
    // (2 + 7) * 8 * 10 usec/bit = 720 usec
    // 1msec is ample
    while(Responded == false)
    {
        if(Wire.available() >= 6)
        {
            Responded = true;
            break;
        }
        RetryCount++;
        if(RetryCount > 20)
        {
            return -1;
        }
        delayMicroseconds(50);
    }

    // Load the data
    _cData.xMSB = Wire.read();
    _cData.xLSB = Wire.read();
    _cData.yMSB = Wire.read();
    _cData.yLSB = Wire.read();
    _cData.zMSB = Wire.read();
    _cData.zLSB = Wire.read();

    // Internal storage
    _cVectorX = (_cData.xMSB << 8) + _cData.xLSB;
    _cVectorY = (_cData.yMSB << 8) + _cData.yLSB;
    _cVectorZ = (_cData.zMSB << 8) + _cData.zLSB;

    return(0);
}

// Process the compass data
int Compass::ProcessCompassData(int test)
{
	// Locals
	int lStatus;

	// There is no FIFO in the compass
	lStatus = ReadXYZ();

	if(test)
	{
		dbg_println("^^^");
		dbg_print("Max X = "); dbg_printlnm(_cVectorX, HEX);
		dbg_print("Max Y = "); dbg_printlnm(_cVectorY, HEX);
		dbg_print("Max Z = "); dbg_printlnm(_cVectorZ, HEX);
		dbg_println("VVVV");
	}

	// FUTURE: compute heading from mag vector
//    return lStatus;
	return 0;
}

// Compute the compass heading from the vector data
int Compass::ComputeCompassHeading()
{
    // Placeholder
    return 0;
}

// Accessors

signed short Compass::getVectorX()
{
    return _cVectorX;
}

signed short Compass::getVectorY()
{
    return _cVectorY;
}

signed short Compass::getVectorZ()
{
    return _cVectorZ;
}

signed short Compass::getCompassHeading()
{
    return _cCompassHeading;
}

// Destructor
Compass::~Compass()
{
  // Do nothing
}

// Default instantiation
Compass imuCompass = Compass();


