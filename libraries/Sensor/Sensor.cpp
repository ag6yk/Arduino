
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

// These routines are not necessary for GCC-avr
#if 0

// Perform a right shift on a signed 32-bit value
signed long Sensor::rsh_sgn32(signed long oldVal, int nbits)
{
    // Locals
    unsigned long  signTest;
    unsigned long  absVal;
    signed long    newVal;

    // Check the sign of the input value
    signTest = (unsigned long)oldVal;
    absVal = (unsigned long)oldVal;

    // Correct for sign if required
    if(signTest & 0x80000000)
    {
        absVal = absVal - 1;
        absVal = ~absVal;
    }

    // Compute the shift
    absVal >>= nbits;

    // Restore sign if required
    if(signTest & 0x80000000)
    {
        absVal = ~absVal;
        absVal++;
    }

    // Cast back to signed value
    newVal = (signed long)absVal;

    return(newVal);

}

// Perform a left shift on a signed 16-bit value
signed long Sensor::lsh_sgn32(signed long oldVal, int nbits)
{
    // Locals
    unsigned long  signTest;
    unsigned long  absVal;
    signed long    newVal;

    // Check the sign of the input value
    signTest = (unsigned long)oldVal;
    absVal = (unsigned long)oldVal;

    // Correct for sign if required
    if(signTest & 0x80000000)
    {
        absVal = absVal - 1;
        absVal = ~absVal;
    }

    // Compute the shift
    absVal <<= nbits;

    // Restore sign if required
    if(signTest & 0x80000000)
    {
        absVal = ~absVal;
        absVal++;
    }

    // Cast back to signed value
    newVal = (signed long)absVal;

    return(newVal);
}


// Perform a right shift on a signed 16-bit value
signed short Sensor::rsh_sgn16(signed short oldVal, int nbits)
{
	// Locals
	unsigned short	signTest;
	unsigned short	absVal;
	signed short	newVal;

	// Check the sign of the input value
	signTest = (unsigned short)oldVal;
	absVal = (unsigned short)oldVal;

	// Correct for sign if required
	if(signTest & 0x8000)
	{
		absVal = absVal - 1;
		absVal = ~absVal;
	}

	// Compute the shift
	absVal >>= nbits;

	// Restore sign if required
	if(signTest & 0x8000)
	{
		absVal = ~absVal;
		absVal++;
	}

	// Cast back to signed value
	newVal = (signed short)absVal;

	return(newVal);

}

// Perform a left shift on a signed 16-bit value
signed short Sensor::lsh_sgn16(signed short oldVal, int nbits)
{
	// Locals
	unsigned short	signTest;
	unsigned short	absVal;
	signed short	newVal;

	// Check the sign of the input value
	signTest = (unsigned short)oldVal;
	absVal = (unsigned short)oldVal;

	// Correct for sign if required
	if(signTest & 0x8000)
	{
		absVal = absVal - 1;
		absVal = ~absVal;
	}

	// Compute the shift
	absVal <<= nbits;

	// Restore sign if required
	if(signTest & 0x8000)
	{
		absVal = ~absVal;
		absVal++;
	}

	// Cast back to signed value
	newVal = (signed short)absVal;

	return(newVal);
}

// Perform a right shift on a signed 8-bit value
signed char Sensor::rsh_sgn8(signed char oldVal, int nbits)
{
	// Locals
	unsigned char	signTest;
	unsigned char	absVal;
	signed char		newVal;

	// Check the sign of the input value
	signTest = (unsigned char)oldVal;
	absVal = (unsigned char)oldVal;

	// Correct for sign if required
	if(signTest & 0x80)
	{
		absVal = absVal - 1;
		absVal = ~absVal;
	}

	// Compute the shift
	absVal >>= nbits;

	// Restore sign if required
	if(signTest & 0x80)
	{
		absVal = ~absVal;
		absVal++;
	}

	// Cast back to signed value
	newVal = (signed char)absVal;

	return(newVal);
}

// Perform a left shift on a signed 8-bit value
signed char Sensor::lsh_sgn8(signed char oldVal, int nbits)
{
	// Locals
	unsigned char	signTest;
	unsigned char	absVal;
	signed char		newVal;

	// Check the sign of the input value
	signTest = (unsigned char)oldVal;
	absVal = (unsigned char)oldVal;

	// Correct for sign if required
	if(signTest & 0x80)
	{
		absVal = absVal - 1;
		absVal = ~absVal;
	}

	// Compute the shift
	absVal <<= nbits;

	// Restore sign if required
	if(signTest & 0x80)
	{
		absVal = ~absVal;
		absVal++;
	}

	// Cast back to signed value
	newVal = (signed char)absVal;

	return(newVal);
}
#endif


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
//	temp2 = rsh_sgn16(temp1, 1);
	temp2 = temp1 >> 1;
	return(temp2);
}

// Perform signal averaging on an array of 16-bit signed integers
signed short Sensor::AvgFilter(signed short* Data)
{
  // Average 8 signals together
  int i;
  signed long lSum = 0;
  signed long lTemp;
  signed short Sum = 0;
  signed short filteredValue;

  unsigned short AbsSum;

  // Sum the values together
  // Use a 32-bit value to prevent overflow
  for(i = 0; i < 8; i++)
  {
      // Cast the next sample as a long
      Sum = *Data++;
      lTemp = (signed long)Sum;
	  // Add the next sample to the accumulator
      lSum = lSum + lTemp;
  }

  // Divide by 8 using shifts
//  lTemp = rsh_sgn32(lSum, 3);
  lTemp = lSum >> 3;

  // Truncate back to short
  filteredValue = (signed short)lTemp;

  return filteredValue;
}

// Compute the integral for the input region
// using the Trapezoidal approximation
// TODO: Evaluate Simpson's rule
// I(a,b) ~= (b-a)/6 * [f(a) + 4*(f(a+b/2) + f(b)]
// TODO: Evaluate Simpson's 3/8 rule:
// I(a,b) ~= (b-a)/8 * [f(a) + 3*f(2a+b/3) + 3*f(a+2b/3) + f(b)]
signed short Sensor::trapIntegral(signed short Data0, signed short Data1)
{
	// Locals
	signed short	newVal;
	signed short	temp1;
	signed short	temp2;
	signed short	temp3;
	signed short    temp4;

	// Estimate the integral over the region using the
	// trapezoidal rule:
	// I(a,b) ~= (b-a) * [f(b)+f(a)/2]
	//        ~= ((b-a)/2)) * [f(a) + f(b)]
	// where a and b are sample points on the curve, i.e. tn-1 and tn
	// Let deltaT = tn - tn-1
	// I(tn-1,t) ~= deltaT * [(f(tn) + f(tn-1))/2]
	// deltaT is fixed in this application, therefore
	// Horner's rule can be used to convert the multiplication
	// of a fraction (i.e. division) into a series of shifts and
	// additions.
	// I(tn-1, t) ~= deltaTP * (f(tn-1) + f(tn))
	// deltaTP (delta T prime) is deltaT / 2
	// Use a three-term Horner polynomial for the division:
	// x/H ~= (x >> FACTOR1) + (x >> FACTOR2) + (x >> FACTOR3)
	// Depending on the update rate parameterize FACTORx

	// Compute the accumulator term f(tn-1) + f(t)
	temp1 = Data1 + Data0;

	// Compute the delta T prime Horner polynomial on the accumulator term
#if 0
	temp2 = rsh_sgn16(temp1, INTEGRATE_FACTOR1);
	temp3 = rsh_sgn16(temp1, INTEGRATE_FACTOR2);
	temp4 = rsh_sgn16(temp1, INTEGRATE_FACTOR3);
#else
	temp2 = temp1 >> INTEGRATE_FACTOR1;
	temp3 = temp1 >> INTEGRATE_FACTOR2;
	temp4 = temp1 >> INTEGRATE_FACTOR3;
#endif

	// Compute the sum to determine the final integral
	newVal = temp2 + temp3 + temp4;

	return(newVal);

}


// Destructor
Sensor::~Sensor()
{
  // Do nothing
}


