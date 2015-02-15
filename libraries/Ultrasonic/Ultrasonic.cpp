
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
  @file Ultrasonic.cpp
  
  @brief HC-SFR04 ultrasonic range sensor class
  
  @details This class processes an ultrasonic range sensor
  
*/

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////
#include "Arduino.h"
#include "Ultrasonic.h"

////////////////////////////////////////////////////////////////////////////////
// DEFINES
////////////////////////////////////////////////////////////////////////////////

// Constructor
Ultrasonic::Ultrasonic(int Trigger, int Echo)
{
    // Assign the pin numbers
    _trigger = Trigger;
    _echo = Echo;

    // Trigger is an output
    pinMode(_trigger, OUTPUT);
    // Echo is an input
    pinMode(_echo, INPUT);
}

// Read the range data from this sensor
// Dynamic range 2" to 120"
// Dynamic range reading: 296 to 17760
// Values for inches: T/148
// Use Q8:8 fixed point value to preserve resolution
// Therefore reported range will run from 128 to 30720
// Sensor will report 36000 for out-of-range
// Function will report 128 to 30720 for valid range
// and 65535 for out-of-range
int Ultrasonic::ReadRange()
{
	// Define locals
	unsigned long Tprop;
	unsigned short Range;

	// Pulse the Ping signal
	digitalWrite(_trigger, LOW);		// Ensure we have a high-going pulse
	delayMicroseconds(5);
	digitalWrite(_trigger, HIGH);
	// Wait 20 usec
	delayMicroseconds(20);
	digitalWrite(_trigger, LOW);
	// Now time the echo pulse high
	Tprop = pulseIn(_echo, HIGH, 1000);

	// Check for no detection
	if(Tprop > 30000)
	{
		return(0xFFFF);
	}

	// Convert to inches
	// X/148 = inches

	// Scale for fixed point
	// eince 148 is 2 * 74, modify the
	// scale to get the divide by 2 for free
	Tprop <<=7;									// x << 8 >> 1

	// Finish the divide by 148 to convert to inches
	// Use integer division to keep maximum precision
	Tprop = Tprop / 74;

	// Truncate to 16-bits
	Range = (unsigned short)Tprop;

	// Update internal buffer
	_range = Range;

    return 0;
}

// Accessor
unsigned short Ultrasonic::getRange()
{
	return _range;
}

// Destructor
Ultrasonic::~Ultrasonic()
{
  // Do nothing
}



