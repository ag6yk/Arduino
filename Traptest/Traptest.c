#include <stdio.h>
#include <stdlib.h>
#include <time.h>

typedef signed long fpInt;
typedef unsigned char byte;

#define true		1
#define false		0
#define FPSHIFT		10
#define ACCEL_LSB	25303

#define INTTOFP(x)	(x << FPSHIFT)
#define FPTOINT(x)	(x >> FPSHIFT)

// Define a numerical processing buffer for computing
// the integral of a data stream
struct NUM_BUFFER
{
	fpInt	tn1;						// Value at t(n-1)
	fpInt	tn;							// Value at t(n)
	fpInt   t0;                         // accumulated value
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
signed short	_aXOffset;
signed short	_aYOffset;
signed short	_aZOffset;
signed short	_aXThreshold;
signed short	_aYThreshold;
signed short	_aZThreshold;

signed short	_aSampleCount;
int				_aVvalid;
int				_aPvalid;
fpInt			_accelerationX;
fpInt			_accelerationY;
fpInt			_accelerationZ;
fpInt			_velocityX;
fpInt			_velocityY;
fpInt			_velocityZ;
fpInt			_positionX;
fpInt			_positionY;
fpInt			_positionZ;

// Simulate reading up to 32 words of data from the accelerometer fifos
// Add random noise and offset
void ReadXYZ(int count, int base, int offset, int noise)
{
	// Simulated read using base as the
	// true value (constant over one buffer)
	// offset is a constant added to the data
	// noise is a maximum value for noise

	// Locals
	int i;
	int new;

	if(count < 0)
	{
		count = 0;
	}

	if(count > 32)
	{
		count = 32;
	}

	if(base < 0)
	{
		base = 0;
	}

	if(noise < 0)
	{
		noise = 0;
	}

	for(i = 0; i < count; i++)
	{
		new = rand() % noise;
		// random number is between 0 and noise. Normalize and
		// translate by the offset
		new = new - (noise/2);
		new = new + offset;
		new = new + base;
		_aXvector[i] = new;

		// Do the same for Y and Z
		new = rand() % noise;
		// random number is between 0 and noise. Normalize and
		// translate by the offset
		new = new - (noise/2);
		new = new + offset;
		new = new + base;
		_aYvector[i] = new;

		new = rand() % noise;
		// random number is between 0 and noise. Normalize and
		// translate by the offset
		new = new - (noise/2);
		new = new + offset;
		new = new + base;
		_aZvector[i] = new;
	}

}


// Compute the integral for the input region
// using the Trapezoidal approximation
fpInt trapIntegral(fpInt Data0, fpInt Data1)
{
	// Locals
	fpInt	newVal;
	fpInt	fpAugend;
	fpInt	fpAddend;

	// Estimate the integral over the region using the
	// trapezoidal rule:
	// I(a,b) ~= (b-a) * [f(b)+f(a)/2]
	//        ~= ((b-a)/2)) * [f(a) + f(b)]
	// where a and b are sample points on the curve, i.e. tn-1 and tn
	// Let deltaT = tn - tn-1
	// I(tn-1,t) ~= deltaT * [(f(tn) + f(tn-1))/2]
	// deltaT is fixed in this application, therefore

	// Convert the inputs into fixed point Q10:22
	fpAugend = Data0;
	fpAddend = Data1;

	// Compute the numerator
	fpAugend = fpAugend + fpAddend;

	// Multiply by the time interval divide by 2
	// For 100 Hz update rate, this translates to x/200
	// This can be computed as x/8 * x/25
	// Divide by 4 first
	fpAugend >>= 3;
	// Divide by 25 directly
	newVal = fpAugend / 25;

	return(newVal);

}


// Scale the acceleration to the nearest 16th inch (tick)
// Accelerometer scale is +/- 4mg
// Lsb is 24.709632 ticks/sec^2
// Use Q22:10 fixed point math
fpInt scaleAcceleration(fpInt input)
{
	// Locals
	fpInt fpMultiplicand = ACCEL_LSB;		// 24.709632 in fixed point
	fpInt fpInput;
	fpInt newValue;

	// Perform math in 32-bit values
	fpInput = input;
	fpInput = fpInput * fpMultiplicand;
	// Convert back to integer
	fpInput >>= FPSHIFT;
	newValue = fpInput;
	return(newValue);
}

// Compute the velocity from the sampled acceleration data
// using numerical integration
int ComputeVoft(struct NUM_BUFFER *n, fpInt* computedValue)
{
    // Locals
    fpInt 	newVelocity;

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
//  lTemp = rsh_sgn32(lSum, 3);
  lTemp = lSum >> 3;

  // Truncate back to short
  filteredValue = (signed short)lTemp;

  return filteredValue;
}

// Compute the offset and threshold for the acceleration axes
void ComputeOffset(void)
{
	// Locals
	fpInt			xData;
	fpInt			yData;
	fpInt			zData;
	fpInt			fTemp;
	byte			i;
	signed short	temp;
	signed short	uTemp;

	// Reset the globals
	_aXOffset = 0;
	_aYOffset = 0;
	_aZOffset = 0;
	_aXThreshold = 0;
	_aYThreshold = 0;

	// Initialize the locals
	xData = 0;
	yData = 0;
	zData = 0;


	// Simulate reading 32 words of data
	// Bias is set to 10 and noise to 100
	ReadXYZ(32, 0, 10, 100);

	// Compute the mean and maximum magnitude of the 32 word sample
	for(i = 0; i < 32; i++)
	{
		// X axis

		// Check if this value is a candidate to be the noise threshold
		uTemp = abs(_aXvector[i]);
		if(uTemp > _aXThreshold)
		{
			_aXThreshold = uTemp;
		}

		// Read and add this value to the mean calculation
		fTemp = _aXvector[i];
		// Conver to fixed point
		fTemp = INTTOFP(fTemp);
		xData = xData + fTemp;

		// Y axis
		uTemp = abs(_aYvector[i]);
		if(uTemp > _aYThreshold)
		{
			_aYThreshold = uTemp;
		}

		// Read and add this value to the mean calculation
		fTemp = _aYvector[i];
		// Convert to fixed point
		fTemp = INTTOFP(fTemp);
		yData = yData + fTemp;

		// Z axis
		uTemp = abs(_aZvector[i]);
		if(uTemp > _aZThreshold)
		{
			_aZThreshold = uTemp;
		}

		// Read and add this value to the mean calculation
		fTemp = _aZvector[i];
		// Convert to fixed point
		fTemp = INTTOFP(fTemp);
		zData = zData + fTemp;
	}

	// Compute the offsets and convert back to integer
	_aXOffset = xData >> 5;
	_aXOffset = FPTOINT(_aZOffset);
	_aYOffset = xData >> 5;
	_aYOffset = FPTOINT(_aZOffset);
	_aZOffset = xData >> 5;
	_aZOffset = FPTOINT(_aZOffset);
}


// Process X and Y axes and load the output buffer
int ProcessAccelData(int test)
{
	// Locals
	int fCount;
	int failSafe;
	int lStatus;
	int i;
	signed short temp;
	fpInt		 fpTemp;

	// Simulate reading the FIFO
	ReadXYZ(8, 0, 10, 100);

	// Increment the sample count
	_aSampleCount++;
	// Update the validity flags
	if(_aSampleCount > 1)
	{
	    _aVvalid = true;
	}
	if(_aSampleCount > 2)
	{
	    _aPvalid = true;
	}

	// X-axis

	// Samples are arranged in time-descending order,
	// i.e. 0th element is the most recent
	// Perform signal averaging on the 8 most recent values
    temp = AvgFilter(_aXvector);
    // Correct for offset
    temp = temp - _aXOffset;
    // See if above noise threshold
    if(abs(temp) > _aXThreshold)
    {
    	// Convert to fixed point
    	fpTemp = (fpInt)temp;
    	fpTemp = INTTOFP(fpTemp);
    	// Scale to ticks/sec^2
    	_accelerationX = scaleAcceleration(fpTemp);
    }

    // Y-axis

	// Samples are arranged in time-descending order,
	// i.e. 0th element is the most recent
	// Perform signal averaging on the 8 most recent values
    temp = AvgFilter(_aYvector);
    // Correct for offset
    temp = temp - _aYOffset;
    // See if above noise threshold
    if(abs(temp) > _aYThreshold)
    {
    	// Convert to fixed point
    	fpTemp = (fpInt)temp;
    	fpTemp = INTTOFP(fpTemp);
    	// Scale to ticks/sec^2
    	_accelerationY = scaleAcceleration(fpTemp);
    }

	// Z-axis

	// Samples are arranged in time-descending order,
	// i.e. 0th element is the most recent
	// Perform signal averaging on the 8 most recent values
    temp = AvgFilter(_aZvector);
    // Correct for offset
    temp = temp - _aZOffset;
    // See if above noise threshold
    if(abs(temp) > _aZThreshold)
    {
    	// Convert to fixed point
    	fpTemp = (fpInt)temp;
    	fpTemp = INTTOFP(fpTemp);
    	// Scale to ticks/sec^2
    	_accelerationZ = scaleAcceleration(fpTemp);
    }

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

signed short getAccelerationX(void)
{
	// Locals
	signed short	accelOut;
	fpInt			fpTemp;

	// Read the private member
	fpTemp = _accelerationX;
	// Convert to integer
	fpTemp = FPTOINT(fpTemp);
	accelOut = (signed short)fpTemp;
	return accelOut;
}

signed short getAccelerationY(void)
{
	// Locals
	signed short	accelOut;
	fpInt			fpTemp;

	// Read the private member
	fpTemp = _accelerationY;
	// Convert to integer
	fpTemp = FPTOINT(fpTemp);
	accelOut = (signed short)fpTemp;
	return accelOut;
}

signed short getAccelerationZ(void)
{
	// Locals
	signed short	accelOut;
	fpInt			fpTemp;

	// Read the private member
	fpTemp = _accelerationZ;
	// Convert to integer
	fpTemp = FPTOINT(fpTemp);
	accelOut = (signed short)fpTemp;
	return accelOut;
}

signed short getVelocityX(void)
{
	// Locals
	signed short	velOut;
	fpInt			fpTemp;

	// Read the private member
	fpTemp = _velocityX;
	// Convert to integer
	fpTemp = FPTOINT(fpTemp);
	velOut = (signed short)fpTemp;
	return velOut;
}

signed short getVelocityY(void)
{
	// Locals
	signed short	velOut;
	fpInt			fpTemp;

	// Read the private member
	fpTemp = _velocityY;
	// Convert to integer
	fpTemp = FPTOINT(fpTemp);
	velOut = (signed short)fpTemp;
	return velOut;
}

signed short getVelocityZ(void)
{
	// Locals
	signed short	velOut;
	fpInt			fpTemp;

	// Read the private member
	fpTemp = _velocityZ;
	// Convert to integer
	fpTemp = FPTOINT(fpTemp);
	velOut = (signed short)fpTemp;
	return velOut;
}

signed short getPositionX(void)
{
	// Locals
	signed short	posOut;
	fpInt			fpTemp;

	// Read the private member
	fpTemp = _positionX;
	// Convert to integer
	fpTemp = FPTOINT(fpTemp);
	posOut = (signed short)fpTemp;
	return posOut;
}

signed short getPositionY(void)
{
	// Locals
	signed short	posOut;
	fpInt			fpTemp;

	// Read the private member
	fpTemp = _positionY;
	// Convert to integer
	fpTemp = FPTOINT(fpTemp);
	posOut = (signed short)fpTemp;
	return posOut;
}

signed short getPositionZ(void)
{
	// Locals
	signed short	posOut;
	fpInt			fpTemp;

	// Read the private member
	fpTemp = _positionZ;
	// Convert to integer
	fpTemp = FPTOINT(fpTemp);
	posOut = (signed short)fpTemp;
	return posOut;
}


void main(void)
{
	int i;

	srand(time(NULL));

	for(i= 0; i < 100; i++)
	{
		ProcessAccelData(0);
		printf("     X-axis\n");
		printf("----------------------\n");
		printf("Acceleration:  %d\n", getAccelerationX());
		printf("    Velocity:  %d\n", getVelocityX());
		printf("    Position:  %d\n", getPositionX());
		printf("\n");
		printf("     Y-axis\n");
		printf("----------------------\n");
		printf("Acceleration:  %d\n", getAccelerationY());
		printf("    Velocity:  %d\n", getVelocityY());
		printf("    Position:  %d\n", getPositionY());
		printf("\n");
		printf("     Z-axis\n");
		printf("----------------------\n");
		printf("Acceleration:  %d\n", getAccelerationZ());
		printf("    Velocity:  %d\n", getVelocityZ());
		printf("    Position:  %d\n", getPositionZ());
		printf("\n");
		printf("==========================================\n");
	}
}
