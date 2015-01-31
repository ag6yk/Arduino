
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
  @file Sensor.cpp
  
  @brief Generic I2C sensor class
  
  @details This class is a child of the TwoWire class which adds a signal averaging method
  
*/

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////
#include "Arduino.h"
#include <Wire.h>
#include "Sensor.h"

////////////////////////////////////////////////////////////////////////////////
// DEFINES
////////////////////////////////////////////////////////////////////////////////

// Constructor
Sensor::Sensor()
{
  // TODO
}

// Returns true if device responds correctly within sanity timeout
boolean Sensor::waitForI2CResponse(byte nBytes)
{
	// Locals
	int	RetryCount = nBytes + 1;
	boolean Responded = false;

	// The bus transmission has already occurred
	// Waiting on data
	while(Responded == false)
	{
		// Done?
		if(Wire.available() >= nBytes)
		{
			return true;
		}
		// No
		RetryCount--;
		if(RetryCount == 0)
		{
			// Timeout
			return false;
		}
		// Wait one I2C 100KHz byte time
		delayMicroseconds(80);
	}
	// Should never get here but
	return false;
}

// Perform signal averaging on an array of 16-bit signed integers
int Sensor::AvgFilter(int* Data)
{
  // Average 8 signals together
  int i;
  int Sum = 0;
  unsigned int SignTest;
  unsigned int AbsSum;

  for(i = 0; i < 8; i++)
  {
	  // Add the next sample to the accumulator
	  Sum = Sum + *Data++;
  }

  // These are signed values so
  // play a trick to do fast division
  SignTest = (unsigned int)Sum;
  AbsSum = SignTest;

  // Compute the absolute value of Sum
  if(SignTest & 0x8000)
  {
	  AbsSum = AbsSum - 1;
	  AbsSum = ~AbsSum;
  }

  // Quick divide by 8
  AbsSum = AbsSum >> 3;

  // Restore sign if required
  if(SignTest & 0x8000)
  {
	  AbsSum = ~AbsSum;
	  AbsSum++;
  }

  // Recast to signed
  Sum = (signed int)AbsSum;

  return Sum;
}

// Destructor
Sensor::~Sensor()
{
  // Do nothing
}


