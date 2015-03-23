///////////////////////////////////////////////////////////////////////////////
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
  @file Accel.cpp
  
  @brief ADXL345 3-axis Accelerometer sensor class
  
  @details This class is a child of the sensor class and describes the 
   ADXL345 3-axis Accelerometer 
  
*/

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include "Accel.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINES
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// CLASS AND METHOD DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Constructor
Accel::Accel()
{
    // Locals
    int i;
    // Initialize the private variables
    _i2cAddress = 0;                        // i2c address

    for(i = 0; i < 32; i++)
    {
        _aXvector[i] = 0;                   // raw acceleration data
        _aYvector[i] = 0;
        _aZvector[i] = 0;
    }

    _afifoCount = 0;                        // FIFO depth
    _aVvalid = false;                       // computation enable flags
    _aPvalid = false;
    _aSampleCount = 0;                      // computation sample count
    _aVComputingX.t0 = 0;                   // computation accumulators
    _aVComputingX.tn = 0;
    _aVComputingX.tn1 = 0;
    _aPComputingX.t0 = 0;
    _aPComputingX.tn = 0;
    _aPComputingX.tn1 = 0;
    _aVComputingY.t0 = 0;
    _aVComputingY.tn = 0;
    _aVComputingY.tn1 = 0;
    _aPComputingY.t0 = 0;
    _aPComputingY.tn = 0;
    _aPComputingY.tn1 = 0;
    _accelerationX = 0;                     // processed nav data
    _velocityX = 0;
    _positionX = 0;
    _accelerationY = 0;
    _velocityY = 0;
    _positionY = 0;
    _aXOffset = 0;
    _aXThreshold = 0;
    _aYOffset = 0;
    _aYThreshold = 0;
}

// Specific initialization
int Accel::begin(void)
{
  // Local
  boolean Responded;
 unsigned char	i2cStatus;
 unsigned char  i2cFlowCount = 0;

  // Read the ID to verify data response
  // Start with address pin tied high
  _i2cAddress = ACCEL_IC_ADDRHI;
  Responded = readID();
  if(Responded == false)
  {
      // Try again with alternate address
      _i2cAddress = ACCEL_IC_ADDRLO;
      Responded = readID();
  }

  if(Responded == false)
  {
      return -1;
  }

  i2cFlowCount++;

  // Tap threshold - ignore

  // Offset register X - TBD
  
  // Offset register Y - TBD
  
  // Offset register Z - TBD
  
  // Tap Duration - ignore
  
  // Tap Latency - ignore
  
  // Tap window - ignore
  
  // Activity threshold - ignore
  
  // Inactivity threshold - ignore
  
  // Inactivity time - disable all
  // 0b00000000
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_TIME_INACT));
  Wire.write(byte(0x00));
  i2cStatus = Wire.endTransmission();
  if(i2cStatus != 0)
  {
	  goto accelBeginError;
  }
  
  i2cFlowCount++;
  // Free-fall threshold - ignore
  
  // Free-fall time - ignore
  
  // Tap enable - disable all
  // 0b00000000
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_TAP_AXES));
  Wire.write(byte(0x00));
  i2cStatus = Wire.endTransmission();
  if(i2cStatus != 0)
  {
	  goto accelBeginError;
  }
  
  i2cFlowCount++;
  // BW_RATE - Default 100Hz, normal operation
  // Update - increase BW to 200Hz initially
  // Update - increase BW to 800Hz initially
  // 0b00001101
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_BW_RATE));
  Wire.write(byte(0x0D));
  i2cStatus = Wire.endTransmission();
  if(i2cStatus != 0)
  {
	  goto accelBeginError;
  }
  i2cFlowCount++;
  
  // POWER_CTL
  // Link bit not used
  // Auto Sleep mode disabled
  // Actively measure
  // Disable sleep
  // 0b00000000
  // 0b00001000
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_PWR_CTRL));
  Wire.write(byte(0x00));
  i2cStatus = Wire.endTransmission();
  if(i2cStatus != 0)
  {
	  goto accelBeginError;
  }

  i2cFlowCount++;
  delayMicroseconds(5);
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_PWR_CTRL));
  Wire.write(byte(0x08));
  i2cStatus = Wire.endTransmission();
  if(i2cStatus != 0)
  {
	  goto accelBeginError;
  }

  i2cFlowCount++;
  // INT_ENABLE
  // 0b00000000
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_INT_ENABLE));
  Wire.write(byte(0x00));
  i2cStatus = Wire.endTransmission();
  if(i2cStatus != 0)
  {
	  goto accelBeginError;
  }

  i2cFlowCount++;
  // INT_MAP
  // DATA_READY to INT1
  // Watermark to INT1
  // Overrun to INT1
  // 0b01111100
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_INT_MAP));
  Wire.write(byte(0x7C));
  i2cStatus = Wire.endTransmission();
  if(i2cStatus != 0)
  {
	  goto accelBeginError;
  }
  
  i2cFlowCount++;
  // DATA_FORMAT
  // No self test
  // SPI dont care
  // INT_INVERT to active high
  // FULL_RES enabled
  // Justify right
  // Range +/- 2g
  // 0b00001000
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_DATA_FMT));
  Wire.write(byte(0x08));
  i2cStatus = Wire.endTransmission();
  if(i2cStatus != 0)
  {
	  goto accelBeginError;
  }

  i2cFlowCount++;
  // FIFO_CTL
  // FIFO Mode - FIFO mode
  // Trigger - INT1
  // Samples - 1/2 full - 16
  // 0b01010000
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_FIFO_CTL));
  Wire.write(byte(0x50));
  i2cStatus = Wire.endTransmission();
  if(i2cStatus != 0)
  {
	  goto accelBeginError;
  }

  // Hardware configured
  // Characterize the offset and the threshold
  ComputeOffset();

	// Successful processing
accelBeginSuccess:
	Serial.println("Accel::begin SUCCESS");
	Serial.print("I2C address = "); Serial.println(_i2cAddress, HEX);
	delay(500);
	return 0;

  // Error processing
accelBeginError:
    Serial.println("Accel::begin ERROR");
    Serial.print("Count  = "); Serial.println(i2cFlowCount);
    Serial.print("Status = "); Serial.println(i2cStatus);
    delay(500);
	return i2cStatus;

}

boolean Accel::readID()
{
    // Locals
    unsigned char ReadBack;
    unsigned char i2cStatus;
    unsigned char nBytes;
    unsigned char i2cFlowCount = 0;

    // Read the ID to verify data response
    Wire.beginTransmission(_i2cAddress);
    Wire.write(byte(A_DEVID));
    i2cStatus = Wire.endTransmission();
    if(i2cStatus)
    {
        goto accelReadIDError;
    }
    
    i2cFlowCount++;
    nBytes = Wire.requestFrom(_i2cAddress, byte(1));
    if(nBytes != 1)
    {
        i2cStatus = nBytes;
        goto accelReadIDError;
    }

    i2cFlowCount++;
    // Wait for response with timeout
	if(waitForI2CResponse(byte(1)) == false)
	{
        i2cStatus = 9;
		goto accelReadIDError;
	}

    i2cFlowCount++;
    ReadBack = Wire.read();                   // read ID byte
    if(ReadBack != A_I_BETTER_BE)
    {
    	i2cStatus = 99;
        goto accelReadIDError;                // Device is active but invalid
    }

    // Process success
accelReadIDSuccess:
	Serial.println("Accel::readID - good ID");
	delay(500);
	return true;

    // Process the error
accelReadIDError:
    Serial.println("Accel::readID");
    Serial.print("Count  = "); Serial.println(i2cFlowCount);
    Serial.print("Status = "); Serial.println(i2cStatus);
    delay(500);
    return false;
}

// Returns the fifo count or 0 for not ready
int Accel::available()
{
	// Locals
	byte	i2cStatus;
	byte	fifoCount = 0;

	// Read the fifo status register
	Wire.beginTransmission(_i2cAddress);
	Wire.write(byte(A_FIFO_STS));
	i2cStatus = Wire.endTransmission();
	if(i2cStatus)
	{
		goto accelAvailableError;
	}

	Wire.requestFrom(_i2cAddress, byte(1));
	if(waitForI2CResponse(byte(1)) == false)
	{
		// Return count of 0 for timeout
		return 0;
	}
	// Read the FIFO count
	fifoCount = Wire.read();
	fifoCount &= A_FIFO_CNT;
	// Save for use by other members
	_afifoCount = fifoCount;

	// Process the success
	return(fifoCount);

	// Process the error
accelAvailableError:
	Serial.println("Accel::available");
	Serial.print("Status = "); Serial.println(i2cStatus);
	delay(500);
	return 0;
}

// Block read the acceleration data
// Assumes data availability has been checked!
int Accel::ReadXYZ()
{
	// Locals
	int i;
	unsigned char lsbTemp;
	unsigned char msbTemp;
	unsigned short temp;

	// Compute the final index based on the fifo count
	// This will cause the 0th slement of each vector
	// array to be the most recent data, the 1st element
	// to be the next most recent, etc.
	for(i = (_afifoCount); i > 0; i--)
	{
		// Point to the X FIFO
		Wire.beginTransmission(_i2cAddress);
		Wire.write(byte(A_DATA_X0));
		Wire.endTransmission();
		// Block read the first element
		Wire.requestFrom(_i2cAddress, sizeof(ACCEL_DATA_BLOCK));
		if(waitForI2CResponse(sizeof(ACCEL_DATA_BLOCK)) == false)
		{
			return -1;
		}
		// One block of data is read from the I2C
		// format for further processing

		// Read from fifo LSB/MSB order
		lsbTemp = Wire.read();
		msbTemp = Wire.read();
		// Convert to a 16-bit value
		temp = (unsigned short)((msbTemp << 8) + lsbTemp);
		// Convert and store as a signed value
		_aXvector[i-1] = (signed short)temp;

		// Read and process Y-axis information
        lsbTemp = Wire.read();
        msbTemp = Wire.read();
        temp = (unsigned short)((msbTemp << 8) + lsbTemp);
        _aYvector[i-1] = (signed short)temp;

        // Read and process Z-axis information
        // not used but need to read since autoincrement feature put it
        // into the buffer
        lsbTemp = Wire.read();
        msbTemp = Wire.read();
	}

	// Success
	return 0;
}

// Scale the acceleration to the nearest 16th inch (tick)
// Accelerometer scale is +/- 4mg
// Lsb is 24.709632 ticks/sec^2
// Use Q22:10 fixed point math
fpInt Accel::scaleAcceleration(fpInt input)
{
	// Locals
	fpInt	fpMultiplicand = ACCEL_LSB;		// 24.709632 in fixed point
	fpInt	fpInput;
	fpInt	newValue;

	// Perform math in 32-bit values
	fpInput = input;
	fpInput = fpInput * fpMultiplicand;
	// Post-scale from the multiplication operation
	fpInput >>= 10;

	newValue = fpInput;
	return(newValue);
}

// Compute the velocity from the sampled acceleration data
// using numerical integration
int Accel::ComputeVoft(NUM_BUFFER *n, fpInt* computedValue)
{
    // Locals
    fpInt	newVelocity;

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
int Accel::ComputeXoft(NUM_BUFFER *n, fpInt* computedValue)
{
    // Locals
    fpInt	newPosition;

    // Compute the new integral
    newPosition = trapIntegral(n->tn, n->tn1);
    // Add to the accumulator
    n->t0 = n->t0 + newPosition;
    // Return the new value
    *computedValue = n->t0;
    return 0;
}

// Compute the offset and threshold for the acceleration axes
void Accel::ComputeOffset()
{
	// Locals
	fpInt			xData;
	fpInt		 	yData;
	fpInt			fTemp;
	byte			i;
	signed short	temp;
	signed short	uTemp;

	// Reset the globals
	_aXOffset = 0;
	_aYOffset = 0;
	_aXThreshold = 0;
	_aYThreshold = 0;

	// Initialize the computation
	xData = 0;
	yData = 0;

	// Read a full buffer of data
	while(available() < 32)
	{
		// NOP
	}

	ReadXYZ();

	// Compute the mean and maximum magnitude of the 32 word sample
	for(i = 0; i < 32; i++)
	{
		// X axis
		uTemp = abs(_aXvector[i]);
		if(uTemp > _aXThreshold)
		{
			_aXThreshold = uTemp;
		}

		// Read the next value
		fTemp = _aXvector[i];
		// Convert to Q10:22 fixed point
		fTemp <<= 10;
		xData = xData + fTemp;

		// Y axis
		uTemp = abs(_aYvector[i]);
		if(uTemp > _aYThreshold)
		{
			_aYThreshold = uTemp;
		}

		// Convert to Q10:22 fixed point
		fTemp = _aYvector[i];
		fTemp <<= 10;
		yData = yData + fTemp;

	}

	// Compute the offsets and convert to integer
	_aXOffset = xData >> 15;
	_aYOffset = yData >> 15;

}

// Process X and Y axes and load the output buffer
int Accel::ProcessAccelData(int test)
{
	// Locals
	int 			fCount;
	int 			failSafe;
	int 			lStatus;
	int 			i;
	signed short	temp;
	fpInt			fpTemp;

	// Read the FIFO count. Need at least 8 samples to update the math
	fCount = 0;
	failSafe = 0;
	while(fCount < 8)
	{
		failSafe++;
		fCount = available();
		// Load a timeout of at least 1 msec
		if(failSafe > 10)
		{
			return -1;
		}
	}

	// FIFO is ready, read 8 samples of data
	lStatus = ReadXYZ();

	if(0)
	{
		Serial.print("AFifo count = "); Serial.println(fCount);
    	for(i=0; i < fCount; i++)
    	{
      		Serial.print("Xddot   = "); Serial.println(_aXvector[i], DEC);
    		Serial.print(":Yddot   = "); Serial.println(_aYvector[i], DEC);
    		Serial.print(":Zddot   = "); Serial.println(_aZvector[i], DEC);
    	}
	}

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

	// Samples are arranged in time-descending order,
	// i.e. 0th element is the most recent
	// Perform signal averaging on the 8 most recent values
    // Skip Z axis for now

	// X-axis
	temp = AvgFilter(3, _aXvector);
	// Correct for offset
	temp = temp - _aXOffset;
	// See if above noise threshold. If not, set acceleration to 0
	_accelerationX = 0;
	if(abs(temp) > _aXThreshold)
	{
		// Convert to fixed point
		fpTemp = (fpInt)temp;
		fpTemp <<=10;
		// Scale to ticks/sec^2
		_accelerationX = scaleAcceleration(fpTemp);
	}

	// Y-axis
	temp = AvgFilter(3, _aYvector);
	// Correct for offset
	temp = temp - _aYOffset;
	// See if above noise threshdold. If not, set acceleration to 0
	_accelerationY = 0;
	if(abs(temp) > _aYThreshold)
	{
		// Convert to fixed point
		fpTemp = (fpInt)temp;
		fpTemp <<=10;
		// Scale to ticks/sec^2
		_accelerationY = scaleAcceleration(fpTemp);
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

    if(0)
    {
    	Serial.print("Ax= "); Serial.println(_accelerationX);
    	Serial.print("Vx= "); Serial.println(_velocityX);
    	Serial.print("Px= "); Serial.println(_positionX);
//        delay(100);
    }

	return 0;
}

// Accessors

signed short Accel::getAccelerationX()
{
	// Locals
	signed short	accelOut;
	fpInt			fpTemp;

	// Read the private member
	fpTemp = _accelerationX;
	// Convert to integer
	fpTemp >>= 10;
	accelOut = (signed short)fpTemp;
    return accelOut;
}

signed short Accel::getAccelerationY()
{
	// Locals
	signed short	accelOut;
	fpInt			fpTemp;

	// Read the private member
    fpTemp =  _accelerationY;
    // Convert to integer
    fpTemp >>= 10;
    accelOut = (signed short)fpTemp;
    return accelOut;
}

signed short Accel::getVelocityX()
{
	// Locals
	signed short	velOut;
	fpInt			fpTemp;

	// Read the private member
	fpTemp = _velocityX;
	// Convert to integer
	fpTemp >>= 10;
	velOut = (signed short)fpTemp;
	return velOut;
}

signed short Accel::getVelocityY()
{
	// Locals
	signed short	velOut;
	fpInt			fpTemp;

	// Read the private member
	fpTemp = _velocityY;
	// Convert to integer
	fpTemp >>= 10;
	velOut = (signed short)fpTemp;
	return velOut;
}

signed short Accel::getPositionX()
{
	// Locals
	signed short	posOut;
	fpInt			fpTemp;

	// Read the private member
	fpTemp = _positionX;
	// Convert to integer
	fpTemp >>= 10;
	posOut = (signed short)fpTemp;
	return posOut;
}

signed short Accel::getPositionY()
{
	// Locals
	signed short	posOut;
	fpInt			fpTemp;

	// Read the private member
	fpTemp = _positionY;
	// Convert to integer
	fpTemp >>= 10;
	posOut = (signed short)fpTemp;
	return posOut;
}

// Flush the accelerometer FIFOs
int Accel::flush()
{
    // locals
    int lStatus;

    // Set up the fifo count
    lStatus = available();
    // Read the contents of the fifo
    lStatus = ReadXYZ();

    return 0;
}

// Reset the current processing
int Accel::setOrigin()
{
    // Reset the velocity and position accumulators
    _aVComputingX.t0 = 0;
    _aVComputingX.tn = 0;
    _aVComputingX.tn1 = 0;
    _aPComputingX.t0 = 0;
    _aPComputingX.tn = 0;
    _aPComputingX.tn1 = 0;
    _aVComputingY.t0 = 0;
    _aVComputingY.tn = 0;
    _aVComputingY.tn1 = 0;
    _aPComputingY.t0 = 0;
    _aPComputingY.tn = 0;
    _aPComputingY.tn1 = 0;
    _velocityX = 0;
    _positionX = 0;
    _velocityY = 0;
    _positionY = 0;
    // Force a complete refresh of the computing buffers
    _aVvalid = false;
    _aPvalid = false;
    _aSampleCount = 0;
    // Refresh the fifos
    flush();

    // Refresh the offsets and thresholds
    ComputeOffset();

    return 0;
}

// Update the accumulator for stopped motion
// Reset the current processing
int Accel::stopMotion(int nFlag)
{
	// Bit map
	// Bit 0: X-axis
	// Bit 1: Y-axis

	// Check for X-axis
	if(nFlag & 0x01)
	{
		// Reset the velociy accumulator
	    _aVComputingX.t0 = 0;
	    _aVComputingX.tn = 0;
	    _aVComputingX.tn1 = 0;
	    _velocityX = 0;
	    // Reset the accelerator accumulator
	    _accelerationX = 0;
	}

	// Check the Y-axis
	if(nFlag & 0x02)
	{
		// Reset the velocity accumulator
		_aVComputingY.t0 = 0;
		_aVComputingY.tn = 0;
		_aVComputingY.tn1 = 0;
		_velocityY = 0;
		// Reset the accerlerator accumulator
		_accelerationY = 0;
	}
    // Force a complete refresh of the velocity computing buffers
	// This is safe for all cases
    _aVvalid = false;
    _aPvalid = false;
    _aSampleCount = 0;
    // Refresh the fifos
    flush();

    return 0;
}


// Destructor
Accel::~Accel()
{
  // Do nothing
}

// Default instantiation
Accel   imuAccel = Accel();

