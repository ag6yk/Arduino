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

// 100 Hz update rate
#define	DELTATP_200HZ

// Trapezoidal integration
// Define the Horner coefficients for the intervals
// 50 Hz Update rate - delta T prime of 100 Hz
#ifdef	DELTATP_100HZ
#define INTEGRATE_FACTOR1	7
#define	INTEGRATE_FACTOR2	9
#define INTEGRATE_FACTOR3   12
#else
// 100 Hz Update rate - delta T prime of 200 Hz
#ifdef	DELTATP_200HZ
#define INTEGRATE_FACTOR1	6
#define INTEGRATE_FACTOR2	8
#define INTEGRATE_FACTOR3   11
#else
// 200 Hz Update rate - delta T prime of 400 Hz
#ifdef  DELTATP_400HZ
#define INTEGRATE_FACTOR1   9
#define INTEGRATE_FACTOR2   11
#define INTEGRATE_FACTOR3   14
#else
// 400 Hz Update rate - delta T prime of 800 Hz
#ifdef  DELTATP_800HZ
#define INTEGRATE_FACTOR1   10
#define INTEGRATE_FACTOR2   12
#define INTEGRATE_FACTOR3   15
#endif
#endif
#endif
#endif

// Define a numerical processing buffer for computing
// the integral of a data stream
struct NUM_BUFFER
{
	signed short	tn1;						// Value at t(n-1)
	signed short	tn;							// Value at t(n)
	signed long     t0;                         // accumulated value
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
    // HCC-avr appears to handle these correctly
    signed long rsh_sgn32(signed long oldVal, int nbits);
    signed long lsh_sgn32(signed long oldVal, int nbits);
    signed short rsh_sgn16(signed short oldVal, int nbits);
    signed short lsh_sgn16(signed short oldVal, int nbits);
    signed char rsh_sgn8(signed char oldVal, int nbits);
    signed char lsh_sgn8(signed char oldVal, int nbits);
    // single pole low pass filter
    signed short DigFilter(signed short* Data0, signed short* Data1);
    // 8-sample signal averaging filter
    signed short AvgFilter(signed short* Data);
    // Compute the integral over the given region using the Trapezoidal approx.
    signed short trapIntegral(signed short Data0, signed short Data1);
  
};
#endif

