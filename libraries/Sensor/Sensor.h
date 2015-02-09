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

// 200 Hz update rate
#define	UPDATE_200HZ

// Trapezoidal integration
// Define the Horner coefficients for the intervals
#ifdef	UPDATE_100HZ
#define INTEGRATE_FACTOR1	7
#define	INTEGRATE_FACTOR2	9
#else
#ifdef	UPDATE_200HZ
#define INTEGRATE_FACTOR1	6
#define INTEGRATE_FACTOR2	8
#endif
#endif

// Define a numerical processing buffer for computing
// the integral of a data stream
struct NUM_BUFFER
{
	signed short	tn2;	 					// Value at t(n-2)
	signed short	tn1;						// Value at t(n-1)
	signed short	tn;							// Value at t(n)
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
    int _samples[8];                        								// average 8 samples
  
  public:
    Sensor();
    ~Sensor();
    boolean waitForI2CResponse(byte nBytes);								// Returns true if device responds OK I2C only
    signed short rsh_sgn16(signed short oldVal, int nbits);					// signed shifts - allows fast fixed math with signed values
    signed short lsh_sgn16(signed short oldVal, int nbits);
    signed char rsh_sgn8(signed char oldVal, int nbits);
    signed char lsh_sgn8(signed char oldVal, int nbits);
    signed short DigFilter(signed short* Data0, signed short* Data1);		// single pole low pass filter
    signed short AvgFilter(signed short* Data);      						// Signal averaging
    signed short trapIntegral(signed short Data0, signed short Data1);		// for the region and fixed interval
  
};
#endif

