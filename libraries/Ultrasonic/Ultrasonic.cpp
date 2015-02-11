
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
// Dynamic range: 100 to 18000
// Sensor will return 36000 for out-of-range
// Function will return 0 to 121 inches
// and 242 inches for out-of-range
unsigned char Ultrasonic::ReadRange()
{
	// Define locals
	unsigned long Tprop;
	unsigned char Range;

	// Pulse the Ping signal
	digitalWrite(_trigger, HIGH);
	// Wait 20 usec
	delayMicroseconds(20);
	digitalWrite(_trigger, LOW);
	// Now time the echo pulse high
	Tprop = pulseIn(_echo, HIGH, 1000);
	// Convert to inches (ref: www.robot-electronics.co.uk)
	Tprop = Tprop / 148;
	// Truncate - dynamic range is 248 to 120 so byte is OK
	Range = (unsigned char)Tprop;

    // return result
    return Range;
}

// Destructor
Ultrasonic::~Ultrasonic()
{
  // Do nothing
}



