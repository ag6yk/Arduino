//////////////////////////////////////////////////////////////////////////////
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
  @file sensor.h
  
  @brief Generic sensor class
  
  @details This class provides a collector for each of the various sensors
           in the 
  
*/

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////
#ifndef Sensor_h
#define Sensor_h
#include "Arduino.h"

////////////////////////////////////////////////////////////////////////////////
// DEFINES
////////////////////////////////////////////////////////////////////////////////

// Define a fixed point math type
typedef	signed long	fpInt;


// Define a numerical processing buffer for computing
// the integral of a data stream
struct NUM_BUFFER
{
	fpInt	tn1;				// Value at t(n-1)
	fpInt	tn;					// Value at t(n)
	fpInt	t0;                 // accumulated value
};

// Define the sensor status bitmap

#define RANGE_0_FAIL    (0x01 << 0)		// bit 0 - Range Sensor 0
#define RANGE_1_FAIL    (0x01 << 1)     // bit 1 - Range Sensor 1
#define RANGE_2_FAIL    (0x01 << 2)     // bit 2 - Range Sensor 2
#define RANGE_3_FAIL    (0x01 << 3)     // bit 3 - Range Sensor 3
#define ACCEL_FAIL      (0x01 << 4)     // bit 4 - Accelerometer
#define GYRO_FAIL       (0x01 << 5)     // bit 5 - Gyroscope
#define COMPASS_FAIL    (0x01 << 6)     // bit 6 - Compass
#define BARO_FAIL       (0x01 << 7)     // bit 7 - Barometer fail


class Sensor
{

  private:
    signed short _samples[8];            // average 8 samples
  
  public:
    Sensor();                           // constructor
    ~Sensor();                          // destructor
    // Returns true if device responds OK I2C only
    boolean waitForI2CResponse(byte nBytes);
    // signed shifts - allows fast integer math with signed values
    // TODO: GCC may handle this in the compiler
    // GCC-avr appears to handle these correctly

#if 0
    signed long rsh_sgn32(signed long oldVal, int nbits);
    signed long lsh_sgn32(signed long oldVal, int nbits);
    signed short rsh_sgn16(signed short oldVal, int nbits);
    signed short lsh_sgn16(signed short oldVal, int nbits);
    signed char rsh_sgn8(signed char oldVal, int nbits);
    signed char lsh_sgn8(signed char oldVal, int nbits);
#endif

    // single pole low pass filter
    signed short DigFilter(signed short* Data0, signed short* Data1);
    // 8-sample signal averaging filter
    signed short AvgFilter(signed short* Data);
    // Compute the integral over the given region using the Trapezoidal approx.
    fpInt trapIntegral(fpInt Data0, fpInt Data1);
  
};
#endif

