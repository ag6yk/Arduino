
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
// Therefore reported range will run from 2 to 120
// Sensor will return 36000 for out-of-range
// Function will return 2 to 120 for valid range
// and 255 for out-of-range
unsigned char Ultrasonic::ReadRange()
{
	// Define locals
	unsigned long Tprop;
	unsigned char Range;

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
		return(255);
	}

	// Valid range
	// Convert to inches (ref: www.robot-electronics.co.uk) - divide by 148
	// TODO - convert division to Horner polynomial
	// x/148 ~= (x>>8 + x>>9 + x>>11+  x>>13 + x>>15) error of 0.2%
	// Temp = (Tprop >> 8) + (Tprop >> 9) + (Tprop >> 11) + (Tprop >> 13) + (Tprop >> 15);
	// return (unsigned char)Temp;
	Tprop = Tprop / 2;
	Tprop = Tprop / 74;
	// Truncate to byte
	Range = (unsigned char)Tprop;

    // return result
    return Range;
}

// Destructor
Ultrasonic::~Ultrasonic()
{
  // Do nothing
}



