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
  // Placeholder 
}

// Specific initialization
int Accel::begin(void)
{
  // Local
  boolean Responded;

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
  Wire.endTransmission();
  
  // Free-fall threshold - ignore
  
  // Free-fall time - ignore
  
  // Tap enable - disable all
  // 0b00000000
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_TAP_AXES));
  Wire.write(byte(0x00));
  Wire.endTransmission();
  
  // BW_RATE - Default 100Hz, normal operation
  // 0b00001010
  
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
  Wire.endTransmission();
  delayMicroseconds(5);
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_PWR_CTRL));
  Wire.write(byte(0x08));
  Wire.endTransmission();

  // INT_ENABLE
  // Enable DATA_READY
  // Enable Watermark
  // Enable Overrun
  // 0b10000011
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_INT_ENABLE));
  Wire.write(byte(0x83));
  Wire.endTransmission();

  // INT_MAP
  // DATA_READY to INT1
  // Watermark to INT1
  // Overrun to INT1
  // 0b01111100
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_INT_MAP));
  Wire.write(byte(0x7C));
  Wire.endTransmission();
  
  // DATA_FORMAT
  // No self test
  // SPI dont care
  // INT_INVERT to active high
  // FULL_RES enabled
  // Justify Left
  // Range +/- 8g
  // 0b00010110
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_DATA_FMT));
  Wire.write(byte(0x16));
  Wire.endTransmission();

  // FIFO_CTL
  // FIFO Mode - FIFO mode
  // Trigger - INT1
  // Samples - 1/2 full - 16
  // 0b01010000
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(A_FIFO_CTL));
  Wire.write(byte(0x50));
  Wire.endTransmission();
  
  // NICE-TO-HAVE
  // Run self-test

  return 0;
}

boolean Accel::readID()
{
    // Locals
    unsigned char ReadBack;

    // Read the ID to verify data response
    Wire.beginTransmission(_i2cAddress);
    Wire.write(byte(A_DEVID));
    Wire.endTransmission();
    Wire.requestFrom(_i2cAddress, byte(1));
    // Wait for response with timeout
	if(waitForI2CResponse(byte(1)) == false)
	{
		return false;
	}

    ReadBack = Wire.read();                   // read ID byte
    if(ReadBack == A_I_BETTER_BE)
    {
        return true;                         // Device is active and valid
    }
    return false;                            // Error
}

// Returns the fifo count or 0 for not ready
int Accel::available()
{
	// Locals
	byte	fifoCount = 0;

	// Read the fifo register
	Wire.beginTransmission(_i2cAddress);
	Wire.write(byte(A_FIFO_STS));
	Wire.endTransmission();
	Wire.requestFrom(_i2cAddress, byte(1));
	if(waitForI2CResponse(byte(1)) == false)
	{
		return 0;
	}
	fifoCount = Wire.read();
	fifoCount &= A_FIFO_CNT;
	// Save for use by other members
	_afifoCount = fifoCount;
	return(fifoCount);
}

// Block read the acceleration data
// Assumes data availability has been checked!
int Accel::ReadXYZ(ACCEL_DATA* pAcData)
{
	// Locals
	int i;
	ACCEL_DATA* lpAcData = pAcData;

	// Read the buffer
	for(i = 0; i < _afifoCount; i++)
	{
		// Point to the X FIFO
		Wire.beginTransmission(_i2cAddress);
		Wire.write(byte(A_DATA_X0));
		Wire.endTransmission();
		// Block read the first element
		Wire.requestFrom(_i2cAddress, sizeof(ACCEL_DATA));
		if(waitForI2CResponse(sizeof(ACCEL_DATA)) == false)
		{
			return -1;
		}
		// Write to member buffer
		lpAcData->xLSB = Wire.read();
		lpAcData->xMSB = Wire.read();
		lpAcData->yLSB = Wire.read();
		lpAcData->yMSB = Wire.read();
		lpAcData->zLSB = Wire.read();
		lpAcData->zMSB = Wire.read();
		// point to next element
		lpAcData++;
	}

	// Success
	return 0;
}

// Compute the velocity in the x-direction
int Accel::ComputeVx()
{
  return 0;
}

// Compute the velocity in the y-direction
int Accel::ComputeVy()
{
  return 0;
}

// Compute the velocity in the z-direction
int Accel::ComputeVz()
{
  return 0;
}

// Compute the displacement in the x-direction
int Accel::ComputePx()
{
// Px = Vx*t + 1/2 Ax*t*t
// Vx = Ax*t
// integral(f(x)dx) =~ (b-a) * [(f(a) + f(b))/2]
// Vx(t) = (period) * (Ax(t-1) + Ax(t))/2
// Px(t) = (
  return 0;
}

// Compute the displacement in the y-direction
int Accel::ComputePy()
{
  return 0;
}

// Compute the displacement in the z-direction
int Accel::ComputePz()
{
  return 0;
}

// Signal averaging
int Accel::AvgFilter(int *Data)
{
//	Digital Filter

//	y(t) = alpha * x(t) - (1-alpha) * y(t-1)
// Where y(t) = filtered output
// alpha = smoothing factor (0.5)
// x(t) = raw sampled data
// y(t-1) = previous value


  return 0;
}

// Destructor
Accel::~Accel()
{
  // Do nothing
}

// Default instantiation
Accel   imuAccel = Accel();

