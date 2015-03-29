
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


// Execute a single pole low-pass filter on data stream
signed short Sensor::DigFilter(signed short* Data0, signed short* Data1)
{
	// Locals
	signed short	temp1;
	signed short	temp2;

	// Digital filter takes the form
	// Xfilter(t) = Xn(t) * alpha - (1-alpha) * Xn-1(t)
	// If alpha is chosen to be 0.5, the equation reduces:
	// Xfilter(t) = (Xn(t) - Xn-1(t)) / 2
	// Which can be implemented using a right shift and subtraction
	// Perform the shift after the subtraction to 1) retain sign and 2) precision
	temp1 = *Data1 - *Data0;
	temp2 = temp1 >> 1;
	return(temp2);
}

// Perform signal averaging on an array of 16-bit signed integers
signed short Sensor::AvgFilter(int shiftValue, signed short* Data)
{
  // Average 8 signals together
  int i;
  signed long lSum = 0;
  signed long lTemp;
  signed short Sum = 0;
  signed short filteredValue;
  int k;

  k = 8;
  if(shiftValue == 5)
  {
	  k = 32;
  }


  // Sum the values together
  // Use a 32-bit value to prevent overflow
  for(i = 0; i < k; i++)
  {
      // Cast the next sample as a long
      Sum = *Data++;
      lTemp = (signed long)Sum;
	  // Add the next sample to the accumulator
      lSum = lSum + lTemp;
  }

  // Divide by using shifts
  lTemp = lSum >> shiftValue;

  // Truncate back to short
  filteredValue = (signed short)lTemp;

  return filteredValue;
}

// Compute the integral for the input region
// using the Trapezoidal approximation
fpInt Sensor::trapIntegral(fpInt Data0, fpInt Data1, int nUpdateRate)
{
	// Locals
	fpInt			newVal;
	fpInt			fpAugend;
	fpInt			fpAddend;

	// Estimate the integral over the region using the
	// trapezoidal rule:
	// I(a,b) ~= (b-a) * [f(b)+f(a)/2]
	//        ~= ((b-a)/2)) * [f(a) + f(b)]
	// where a and b are sample points on the curve, i.e. tn-1 and tn
	// Let deltaT = tn - tn-1
	// I(tn-1,t) ~= deltaT * [(f(tn) + f(tn-1))/2]

	// Convert the inputs into fixed point Q16:16
	fpAugend = Data0;
	fpAddend = Data1;

	// Compute the numerator
	fpAugend = fpAugend + fpAddend;

	// Now divide by 2 to get the average value
	// and multiply by the time interval

	// For 100 Hz this translates to fpAugend/200
	// which can be expressed as (1/4) * fpAugend/50
	// For 800 Hz this is directly fpAugend/50

	// Update rate check
	if(nUpdateRate == 100)
	{
		fpAugend = fpAugend >> 2;
	}

	newVal = fpAugend / 50;

	return(newVal);

}


// Destructor
Sensor::~Sensor()
{
  // Do nothing
}


