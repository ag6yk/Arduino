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
  @file Gyro.cpp
  
  @brief L3G4200D MEMS Motion Sensor (Gyroscope) sensor class
  
  @details This class is a child of the sensor class and describes the 
  L3G4200D MEMS Motion Sensor gyroscope
  
*/

////////////////////////////////////////////////////////////////////////////////
// INCLUDES
////////////////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include "Gyro.h"

////////////////////////////////////////////////////////////////////////////////
// DEFINES
////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// CLASS AND METHOD DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Constructor
Gyro::Gyro(void)
{
    // Locals
    int i;
    // Initialize the private variables
    _i2cAddress = 0;

    for(i = 0; i < 32; i++)
    {
        _gBuf[i].hLSB = 0;
        _gBuf[i].hMSB = 0;
        _gBuf[i].pLSB = 0;
        _gBuf[i].pMSB = 0;
        _gBuf[i].yLSB = 0;
        _gBuf[i].yMSB = 0;
    }

    _gfifoCount = 0;
    _gdRoll = 0;
    _gdPitch = 0;
    _gdYaw = 0;
    _gHeading = 0;
    _gPitch = 0;

}

// Specific initialization
// Parsed out from the constructor
// to allow the system initialization
// order to be specified by the programmer
// Assumes Wire library is instantiated
int Gyro::begin(void)
{
  boolean Responded;
  // Initialize the hardware

  // Read the ID register to confirm 
  // communications
  _i2cAddress = GYRO_IC_ADDRHI;             // start with high address
  Responded = readID();
  if(Responded == false)
  {
      _i2cAddress = GYRO_IC_ADDRLO;         // try with the alternate
      Responded = readID();
  }

  // See if success
  if(Responded == false)
  {
      return -1;
  }


  // Gyro is responding, configure for measurement
  Wire.beginTransmission(_i2cAddress);

  // Control register 1
  // Data rate - 100 Hz
  // Cut-off 12.5
  // Normal mode (no sleep)
  // Enable X and Y axis
  // 0b00001011
  Wire.write(byte(G_CTRL_REG1 | 0x80));     // point to start of multi-byte
  Wire.write(byte(0x0B));
  
  // Control register 2
  // Normal mode, 8Hz cutoff
  // 0b00000000
  Wire.write(byte(0));
  
  // Control register 3
  // Enable INT1
  // Disable BOOT status
  // Active high signal
  // Push-pull drive
  // Data ready disable
  // FIFO watermark disable
  // FIFO overrun disable
  // FIFO Empty disable
  // 0b10000000
  Wire.write(byte(0x80));
  
  // Control register 4
  // Block data update not until MSB and LSB read
  // Little-endian
  // 250 degrees/sec
  // selt-test disabled
  // SPI dont care
  // 0b10000000
  Wire.write(byte(0x80));
  
  // Control register 5
  // Normal boot mode
  // Enable FIFO
  // Enable HPF
  // INT1 from LPF 2
  // Output from LPF 2
  // 0b01011111
  Wire.write(byte(0x2F));
  // Reference value
  // 0
  Wire.write(byte(0));
  Wire.endTransmission();
  
  // Fifo control
  // FIFO mode
  // Watermark to 16 (1/2 full)
  // 0b00110000
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(G_FIFO_C_REG));
  Wire.write(byte(0x30));
  Wire.endTransmission();
  
  // INT1 Configuration
  // AND-OR - dont care
  // Latch - dont care
  // Disable all high and low event interrupts
  // 0b00000000
  Wire.beginTransmission(_i2cAddress);
  Wire.write(byte(G_INT1_CFG));
  Wire.write(byte(0));
  Wire.endTransmission();

  // Since INT1 is not used, use default for 
  // thresholds and duration

  // NICE-TO-HAVE
  // Run self-test


  // Return successful config
  return 0;

}

// Read Device ID
boolean Gyro::readID()
{
    // Locals
    boolean Responded = false;
    unsigned char ReadBack;
    int RetryCount = 0;

    Wire.beginTransmission(_i2cAddress);      // Address Gyro
    Wire.write(byte(G_WHO_AM_I));             // read ID byte
    Wire.endTransmission();                   // send to Gyro
    Wire.requestFrom(_i2cAddress, byte(1));   // get the ID byte
    // Wait for response with timeout
    if(waitForI2CResponse(byte(1)) == false)
    {
    	return false;
    }

    // Bus responded check if the value is OK
    ReadBack = Wire.read();                   // read ID byte
    if(ReadBack == G_I_BETTER_BE)             // Device active and valid
    {
        return true;
    }
    return false;                             // Error
}

// Returns FIFO count or 0 for not ready
int Gyro::available()
{
	// Locals
	byte	fifoCount = 0;

	// Read the fifo register
	Wire.beginTransmission(_i2cAddress);
	Wire.write(byte(G_FIFO_S_REG));
	Wire.endTransmission();
	Wire.requestFrom(_i2cAddress, byte(1));
	if(waitForI2CResponse(byte(1)) == false)
	{
		return 0;
	}
	fifoCount = Wire.read();
	fifoCount &= G_FIFO_COUNT;
	// Save for use by other members
	_gfifoCount = fifoCount;
	return(fifoCount);
}

// Block read the gyro data
// Assumes data availability has been checked!
int Gyro::ReadGyroData(GYRO_DATA* pGData)
{
	// Locals
	int i;
	GYRO_DATA* lpGData = pGData;

	// Read the buffer
	for(i = 0; i < _gfifoCount; i++)
	{
		// Point to the Heading FIFO
		Wire.beginTransmission(_i2cAddress);
		Wire.write(byte(G_OUT_X_L));
		Wire.endTransmission();
		// Block read the first element
		Wire.requestFrom(_i2cAddress, sizeof(GYRO_DATA));
		if(waitForI2CResponse(sizeof(GYRO_DATA)) == false)
		{
			return -1;
		}
		// Write to member buffer
		lpGData->hLSB = Wire.read();
		lpGData->hMSB = Wire.read();
		lpGData->pLSB = Wire.read();
		lpGData->pMSB = Wire.read();
		lpGData->yLSB = Wire.read();
		lpGData->yMSB = Wire.read();
		// point to next element
		lpGData++;
	}

	// Success
	return 0;
}

// Process rate data
int Gyro::ProcessGyroData()
{
    // placeholder
    return 0;
}

// Compute heading from the vector data
int Gyro::ComputeHeading(GYRO_DATA *g)
{
    // Placeholder
    return 0;
}

// Compute pitch from the vector data
int Gyro::ComputePitch(GYRO_DATA *g)
{
    // Placeholder
    return 0;
}


// Accessors
signed short Gyro::getHeading()
{
    return _gHeading;
}

signed short Gyro::getPitch()
{
    return _gPitch;
}

signed short Gyro::getdRoll()
{
    return _gdRoll;
}

signed short Gyro::getdPitch()
{
    return _gdPitch;
}

signed short Gyro::getdYaw()
{
    return _gdYaw;
}

// Destructor
Gyro::~Gyro()
{
  // Does nothing
}

// Default instantiation
Gyro    imuGyro = Gyro();


