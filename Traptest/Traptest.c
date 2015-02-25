#include <stdio.h>
#include <stdlib.h>


#define true	1
#define false	0

// Define a numerical processing buffer for computing
// the integral of a data stream
struct NUM_BUFFER
{
	signed short	tn1;						// Value at t(n-1)
	signed short	tn;							// Value at t(n)
	signed long     t0;                         // accumulated value
};

struct NUM_BUFFER	_aPComputingX;
struct NUM_BUFFER	_aPComputingY;
struct NUM_BUFFER	_aPComputingZ;
struct NUM_BUFFER	_aVComputingX;
struct NUM_BUFFER	_aVComputingY;
struct NUM_BUFFER	_aVComputingZ;


signed short	_aXvector[32];
signed short	_aYvector[32];
signed short	_aZvector[32];
signed short	_aSampleCount;
int				_aVvalid;
int				_aPvalid;
signed short	_accelerationX;
signed short	_accelerationY;
signed short	_accelerationZ;
signed short	_velocityX;
signed short	_velocityY;
signed short	_velocityZ;
signed short	_positionX;
signed short	_positionY;
signed short	_positionZ;


// Compute the integral for the input region
// using the Trapezoidal approximation
signed short trapIntegral(signed short Data0, signed short Data1)
{
	// Locals
	signed short	newVal;
	signed long		fpAugend;
	signed long		fpAddend;
	signed long		fptemp;
	signed long		fpIntegral;

	// Estimate the integral over the region using the
	// trapezoidal rule:
	// I(a,b) ~= (b-a) * [f(b)+f(a)/2]
	//        ~= ((b-a)/2)) * [f(a) + f(b)]
	// where a and b are sample points on the curve, i.e. tn-1 and tn
	// Let deltaT = tn - tn-1
	// I(tn-1,t) ~= deltaT * [(f(tn) + f(tn-1))/2]
	// deltaT is fixed in this application, therefore

	// Convert the inputs into fixed point Q16:16
	fpAugend = (Data0 << 16);
	fpAddend = (Data1 << 16);

	// Compute the numerator
	fpAugend = fpAugend + fpAddend;

	// Multiply by the time interval divide by 2
	// For 100 Hz update rate, this translates to x/200
	// This can be computed as x/4 * x/50
	// Divide by 4 first
	fpAugend >>= 2;
	// Divide by 50
	fpAugend >>= 1;

	// Multiply by the time interval 1/100
	fptemp = fpAugend / 50;

	// Convert the result back to integer
	fptemp >>= 16;
	newVal = (signed short)fptemp;

	return(newVal);

}


// Scale the acceleration to the nearest 16th inch (tick)
// Accelerometer scale is +/- 4mg
// Lsb is 24.709632 ticks/sec^2
// Use Q22:10 fixed point math
signed short scaleAcceleration(signed short input)
{
	// Locals
	signed long fpMultiplicand = 25302;			// 24.709632 in fixed point
	signed long fpInput;
	signed short newValue;

	// Perform math in 32-bit values
	fpInput = input;
	fpInput = fpInput * fpMultiplicand;
	// Convert back to integer
	fpInput >>= 10;
	newValue = (signed short)fpInput;
	return(newValue);
}

// Compute the velocity from the sampled acceleration data
// using numerical integration
int ComputeVoft(struct NUM_BUFFER *n, signed short* computedValue)
{
    // Locals
    signed short 	newVelocity;

    // Compute the new integral
    newVelocity = trapIntegral(n->tn, n->tn1);
    // Add to the accumulator
    n->t0 = n->t0 + newVelocity;
    // Return the new value
    *computedValue = (signed short)n->t0;
    return 0;

}

// Compute the position from the sampled acceleration data
// using numerical integration
int ComputeXoft(struct NUM_BUFFER *n, signed short* computedValue)
{
    // Locals
    signed short newPosition;

    // Compute the new integral
    newPosition = trapIntegral(n->tn, n->tn1);
    // Add to the accumulator
    n->t0 = n->t0 + newPosition;
    // TODO: Scale the value to 1/16th inch units

    // Return the new value
    *computedValue = (signed short)n->t0;
    return 0;
}

// Perform signal averaging on an array of 16-bit signed integers
signed short AvgFilter(signed short* Data)
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


// Process X and Y axes and load the output buffer
int ProcessAccelData(int test)
{
	// Locals
	int fCount;
	int failSafe;
	int lStatus;
	int i;
	int displayCount;
	signed short temp;

	// FIFO is ready, read 8 samples of data
	for(i = 0; i < 8; i++)
	{
		_aXvector[i] = 1;
		_aYvector[i] = 1;
		_aXvector[i] = 1;
	}

	// Increment the sample count
	_aSampleCount++;
	// Update the validity flags
	if(_aSampleCount > 2)
	{
	    _aVvalid = true;
	}
	if(_aSampleCount > 3)
	{
	    _aPvalid = true;
	}

	// Samples are arranged in time-descending order,
	// i.e. 0th element is the most recent
	// Perform signal averaging on the 8 most recent values
    // Skip Z axis for now
	temp = AvgFilter(_aXvector);
	_accelerationX = scaleAcceleration(temp);
	temp = AvgFilter(_aYvector);
	_accelerationY = scaleAcceleration(temp);

	// Update the velocity numerical buffers
	_aVComputingX.tn1 = _aVComputingX.tn;
	_aVComputingX.tn = _accelerationX;

    _aVComputingY.tn1 = _aVComputingY.tn;
    _aVComputingY.tn = _accelerationY;

	// Compute the velocity from the acceleration data
    if(_aVvalid)
    {
        lStatus = ComputeVoft(&_aVComputingX, &_velocityX);
        if(lStatus == 0)
        {
            // Update the position numerical buffers
            _aPComputingX.tn1 = _aPComputingX.tn;
            _aPComputingX.tn = _velocityX;
        }
        lStatus = ComputeVoft(&_aVComputingY, &_velocityY);
        if(lStatus == 0)
        {
            // Update the position numerical buffers
            _aPComputingY.tn1 = _aPComputingY.tn;
            _aPComputingY.tn = _velocityY;
        }
    }

	// Compute the position from the acceleration data
    if(_aPvalid)
    {
        lStatus = ComputeXoft(&_aPComputingX, &_positionX);
        lStatus = ComputeXoft(&_aPComputingY, &_positionY);
    }

	return 0;
}


void main(void)
{
	int i;

	for(i= 0; i < 100; i++)
	{
		ProcessAccelData(0);
	}
}
