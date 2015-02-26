#include <stdio.h>
#include <stdlib.h>


#define true	1
#define false	0

#define TIME_INTERVAL       100

typedef signed long fpInt;

// Define a numerical processing buffer for computing
// the integral of a data stream
struct NUM_BUFFER
{
    fpInt   tn1;        // Value at t(n-1)
    fpInt   tn;		// Value at t(n)
    fpInt   t0;         // accumulated value
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
int		_aVvalid;
int		_aPvalid;
fpInt	        _accelerationX;
fpInt	        _accelerationY;
fpInt	        _accelerationZ;
fpInt	        _velocityX;
fpInt	        _velocityY;
fpInt	        _velocityZ;
fpInt	        _positionX;
fpInt	        _positionY;
fpInt	        _positionZ;

signed short    accelOutX;
signed short    accelOutY;
signed short    accelOutZ;
signed short    velocityOutX;
signed short    velocityOutY;
signed short    velocityOutZ;
signed short    positionOutX;
signed short    positionOutY;
signed short    positionOutZ;


// Convert from fpInt to signed short
signed short fpToI(fpInt input)
{
    fpInt temp;
    signed short out;

    temp = input;
    temp >>= 10;
    out = (signed short)temp;
    return(out);
}

// Compute the integral for the input region
// using the Trapezoidal approximation
// Use Q22:10 fixed point math
fpInt trapIntegral(fpInt Data0, fpInt Data1)
{
	// Locals
	fpInt       newVal;
	fpInt       fpAugend;
	fpInt       fpAddend;
	fpInt       fpDenominator;

	// Estimate the integral over the region using the
	// trapezoidal rule:
	// I(a,b) ~= (b-a) * [f(b)+f(a)/2]
	//        ~= ((b-a)/2)) * [f(a) + f(b)]
	// where a and b are sample points on the curve, i.e. tn-1 and tn
	// Let deltaT = tn - tn-1
	// I(tn-1,t) ~= deltaT * [(f(tn) + f(tn-1))/2]

	// Read the input values;
	fpAugend = Data0;
	fpAddend = Data1;

	// Compute the numerator - scaling is OK
	fpAugend = fpAugend + fpAddend;

        // Now divide by 2 to get the average value
        // and multiply by the time interval
        fpAugend = fpAugend >> 2;
         newVal = fpAugend / 50;

	// Multiply by the time interval and divide by 2
        // These two operations can be combined
	// For 100 Hz update rate, this translates to x/200
	// This can be computed as x/4 * x/50
	// Divide by 4 first then divide by 50
//        fpAugend >>= 2;

	// Divide by 50 for the final answer
//	newVal = fpAugend / 50;

	return(newVal);

}


// Scale the acceleration to the nearest 16th inch (tick)
// Accelerometer scale is +/- 4mg
// Lsb is 24.709632 ticks/sec^2
// Use Q22:10 fixed point math
fpInt scaleAcceleration(fpInt input)
{
	// Locals
	fpInt fpMultiplicand = 25303;			// 24.709632 in fixed point
	fpInt fpInput;
	fpInt newValue;

	// Perform math in 32-bit values
	fpInput = input;
	fpInput = fpInput * fpMultiplicand;
	fpInput >>= 10;

	newValue = fpInput;
	return(newValue);
}

// Compute the velocity from the sampled acceleration data
// using numerical integration
int ComputeVoft(struct NUM_BUFFER *n, fpInt* computedValue)
{
    // Locals
    fpInt   newVelocity;

    // Compute the new integral
    newVelocity = trapIntegral(n->tn, n->tn1);
    // Add to the accumulator
    n->t0 = n->t0 + newVelocity;
    // Return the new value
    *computedValue = n->t0;
    return 0;

}

// Compute the position from the sampled acceleration data
// using numerical integration
int ComputeXoft(struct NUM_BUFFER *n, fpInt* computedValue)
{
    // Locals
    fpInt newPosition;

    // Compute the new integral
    newPosition = trapIntegral(n->tn, n->tn1);
    // Add to the accumulator
    n->t0 = n->t0 + newPosition;

    // Return the new value
    *computedValue = n->t0;
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
        fpInt fpTemp;

	// FIFO is ready, read 8 samples of data
        // Test version, low constant acceleration
        // Test version, maximum constant accelration
	for(i = 0; i < 8; i++)
	{
		_aXvector[i] = -32768;
		_aYvector[i] = -32748;
		_aXvector[i] = -32768;
	}

	// Increment the sample count
	_aSampleCount++;
	// Update the validity flags
        // Velocity: valid with two acceleration samples
	if(_aSampleCount > 1)
	{
	    _aVvalid = true;
	}
        // Position: valid with four acceleration samples
	if(_aSampleCount > 2)
	{
	    _aPvalid = true;
	}

	// Samples are arranged in time-descending order,
	// i.e. 0th element is the most recent
	// Perform signal averaging on the 8 most recent values
    // Skip Z axis for now
	temp = AvgFilter(_aXvector);
        fpTemp = (fpInt)temp;
        fpTemp <<=10;
	_accelerationX = scaleAcceleration(fpTemp);
	temp = AvgFilter(_aYvector);
        fpTemp = (fpInt)temp;
        fpTemp <<=10;
	_accelerationY = scaleAcceleration(fpTemp);

	// Update the velocity numerical buffers
        // Fixed point math
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

    _accelerationX = 0;
    _accelerationY = 0;
    _accelerationZ = 0;
    _velocityX = 0;
    _velocityY = 0;
    _velocityZ = 0;
    _positionX = 0;
    _positionY = 0;
    _positionZ = 0;

    accelOutX = 0;
    accelOutY = 0;
    accelOutZ = 0;
    velocityOutX = 0;
    velocityOutY = 0;
    velocityOutZ = 0;
    positionOutX = 0;
    positionOutY = 0;
    positionOutZ = 0;


    printf("Update\tVelocity\tPosition\n");
    // Simulate 15 seconds starting from rest at 0,0
    for(i= 0; i < 1500; i++)
    {
        ProcessAccelData(0);
        printf("%d\t%d\t%d\t0x%08x\t0x%08x\n", i, _velocityX, _positionX, _velocityX, _positionX);
    }
}
