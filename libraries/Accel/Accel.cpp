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
  if(i2cStatus == 0)
  {
	  goto accelBeginSuccess;
  }

  // Error processing
accelBeginError:
    Serial.println("Accel::begin");
    Serial.print("Count  = "); Serial.println(i2cFlowCount);
    Serial.print("Status = "); Serial.println(i2cStatus);
    delay(500);
	return i2cStatus;

	// Successful processing
accelBeginSuccess:
	Serial.println("Accel::begin");
	Serial.print("I2C address = "); Serial.println(_i2cAddress, HEX);
	delay(500);
  return 0;
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

// Compute the velocity from the sampled acceleration data
// using numerical integration
int Accel::ComputeVoft(NUM_BUFFER *n, signed short* computedValue)
{
    // Locals
    signed short newVelocity;

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
int Accel::ComputeXoft(NUM_BUFFER *n, signed short* computedValue)
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

// Process X and Y axes and load the output buffer
int Accel::ProcessAccelData(int test)
{
	// Locals
	int fCount;
	int failSafe;
	int lStatus;
	int i;
	int displayCount;

	// Read the FIFO count. If the processor throughput
	// is what we believe, there should be 8 samples in each of
	// the FIFOs for the accelerometer. We need 8 samples or
	// the math doesn't work
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
		Serial.println("Accel FIFO data...");
		displayCount = 0;
    	for(i = 0; i < fCount; i++)
    	{
    		Serial.print(":Xddot   = "); Serial.print(_aXvector[i], DEC);
    		displayCount++;
    		if(displayCount > 4)
    		{
    			displayCount = 0;
    			Serial.println();
    		}
    	}
    	Serial.println();
    	displayCount = 0;
    	for(i=0; i < fCount; i++)
    	{
    		Serial.print(":Yddot   = "); Serial.print(_aYvector[i], DEC);
    		displayCount++;
    		if(displayCount > 4)
    		{
    			displayCount = 0;
    			Serial.println();
    		}
    	}
    	Serial.println();
    	displayCount = 0;
    	for(i=0; i < fCount; i++)
    	{
    		Serial.print(":Zddot   = "); Serial.print(_aZvector[i], DEC);
    		displayCount ++;
    		if(displayCount > 4)
    		{
    			displayCount = 0;
    			Serial.println();
    		}
    	}
    	Serial.println();

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
	_accelerationX = AvgFilter(_aXvector);
	_accelerationY = AvgFilter(_aYvector);

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

// Accessors

signed short Accel::getAccelerationX()
{
    return _accelerationX;
}

signed short Accel::getAccelerationY()
{
    return _accelerationY;
}

signed short Accel::getVelocityX()
{
    return _velocityX;
}

signed short Accel::getVelocityY()
{
    return _velocityY;
}

signed short Accel::getPositionX()
{
    return _positionX;
}

signed short Accel::getPositionY()
{
    return _positionY;
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

    return 0;
}

// Destructor
Accel::~Accel()
{
  // Do nothing
}

// Default instantiation
Accel   imuAccel = Accel();

