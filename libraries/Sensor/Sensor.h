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

// Define the sensor status bitmap

#define RANGE_0_FAIL    (0x01 << 0)      // bit 0 - Range Sensor 0
#define RANGE_1_FAIL    (0x01 << 1)      // bit 1 - Range Sensor 1
#define RANGE_2_FAIL    (0x01 << 2)      // bit 2 - Range Sensor 2
#define RANGE_3_FAIL    (0x01 << 3)      // bit 3 - Range Sensor 3
#define ACCEL_FAIL      (0x01 << 4)      // bit 4 - Accelerometer
#define GYRO_FAIL       (0x01 << 5)      // bit 5 - Gyroscope
#define COMPASS_FAIL    (0x01 << 6)      // bit 6 - Compass
#define BARO_FAIL       (0x01 << 7)      // bit 7 - Barometer fail


class Sensor
{

  private:
    int _samples[8];                        	// average 8 samples
  
  public:
    Sensor();
    ~Sensor();
    boolean waitForI2CResponse(byte nBytes);	// Returns true if device responds OK I2C only
    int AvgFilter(int* Data);      				// Signal averaging
  
};
#endif

