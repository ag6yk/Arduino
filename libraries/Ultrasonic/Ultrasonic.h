//////////////////////////////////////////////////////////////////////////////
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
  @file Ultrasonic.h
  
  @brief HC-SRF04 Ultrasonic range sensor class
  
  @details This class processes an ultrasonic sensor
  
*/

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////
#ifndef Ultrasonic_h
#define Ultrasonic_h
#include "Arduino.h"
#include "Sensor.h"

////////////////////////////////////////////////////////////////////////////////
// DEFINES
////////////////////////////////////////////////////////////////////////////////


// Base class
class Ultrasonic : public Sensor
{

  private:
      int   			_trigger;			// Pin assigned for the trigger signal
      int   			_echo;              // Pin assigned for the echo signal
      unsigned short	_range;				// latest range from the sensor
  
  public:
    Ultrasonic(int Trigger, int Echo);
    ~Ultrasonic();
    int ReadRange(bool);						// Read the range data

    // Accessor
    unsigned short	getRange();
  
};

#endif

