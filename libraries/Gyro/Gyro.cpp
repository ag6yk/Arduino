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
        _gOyVector[i] = 0;      // raw rotational velocity data
        _gOpVector[i] = 0;
    }

    _gfifoCount = 0;
    _gDataValid = false;
    _gSampleCount = 0;
    _pitch.t0 = 0;
    _pitch.tn = 0;
    _pitch.tn1 = 0;
    _yaw.t0 = 0;
    _yaw.tn = 0;
    _yaw.tn1 = 0;
    _roll.t0 = 0;
    _roll.tn = 0;
    _roll.tn1 = 0;
    _gHeading = 0;
    _gPitch = 0;
    _gRoll = 0;
    _gdRoll = 0;
    _gdPitch = 0;
    _gdYaw = 0;

}

// Specific initialization
// Parsed out from the constructor
// to allow the system initialization
// order to be specified by the programmer
// Assumes Wire library is instantiated
int Gyro::begin(void)
{
    boolean Responded;
    unsigned char i2cStatus;
    unsigned char i2cFlowCount = 0;

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
    // Enable X Y and Z axis
    // 0b00001111
    Wire.write(byte(G_CTRL_REG1 | 0x80));     // point to start of multi-byte
    Wire.write(byte(0x0F));
  
    // Control register 2
    // Normal mode, 8Hz cutoff
    // 0b00000000
    Wire.write(byte(0));
  
    // Control register 3
    // Disable INT1
    // Disable BOOT status
    // Active high signal
    // Push-pull drive
    // Data ready disable
    // FIFO watermark disable
    // FIFO overrun disable
    // FIFO Empty disable
    // 0b10000000
    Wire.write(byte(0x00));
  
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
    Wire.write(byte(0x5F));
    // Reference value
    // 0
    Wire.write(byte(0));
    i2cStatus = Wire.endTransmission();
    if(i2cStatus != 0)
    {
        goto gyroBeginError;
    }
    i2cFlowCount++;
  
    // Fifo control
    // FIFO mode
    // Watermark to 16 (1/2 full)
    // 0b00110000
    Wire.beginTransmission(_i2cAddress);
    Wire.write(byte(G_FIFO_C_REG));
    Wire.write(byte(0x30));
    i2cStatus = Wire.endTransmission();
    if(i2cStatus != 0)
    {
        goto gyroBeginError;
    }
    i2cFlowCount++;

    // INT1 Configuration
    // AND-OR - dont care
    // Latch - dont care
    // Disable all high and low event interrupts
    // 0b00000000
    Wire.beginTransmission(_i2cAddress);
    Wire.write(byte(G_INT1_CFG));
    Wire.write(byte(0));
    i2cStatus = Wire.endTransmission();
    if(i2cStatus == 0)
    {
        goto gyroBeginSuccess;
    }
  
    // Since INT1 is not used, use default for
    // thresholds and duration

    // Error processing
gyroBeginError:
    Serial.println("Gyro::begin");
    Serial.print("Count  = "); Serial.println(i2cFlowCount);
    Serial.print("Status = "); Serial.println(i2cStatus);
    delay(500);
    return i2cStatus;

    // Successful processing
gyroBeginSuccess:
    Serial.println("Gyro::begin");
    Serial.print("I2C address = "); Serial.println(_i2cAddress, HEX);
    delay(500);
    return 0;
}

// Read Device ID
boolean Gyro::readID()
{
    // Locals
    unsigned char ReadBack;
    unsigned char i2cStatus;
    unsigned char nBytes;
    unsigned char i2cFlowCount = 0;

    // Read the ID to verify data response
    Wire.beginTransmission(_i2cAddress);      // Address Gyro
    Wire.write(byte(G_WHO_AM_I));             // read ID byte
    i2cStatus = Wire.endTransmission();       // send to Gyro
    if(i2cStatus)
    {
        goto gyroReadIDError;
    }
    i2cFlowCount++;

    nBytes = Wire.requestFrom(_i2cAddress, byte(1));   // get the ID byte
    if(nBytes != 1)
    {
        i2cStatus = nBytes;
        goto gyroReadIDError;
    }
    i2cFlowCount++;

    // Wait for response with timeout
    if(waitForI2CResponse(byte(1)) == false)
    {
        i2cStatus = 10;
        goto gyroReadIDError;
    }
    i2cFlowCount++;

    // Bus responded check if the value is OK
    ReadBack = Wire.read();                   // read ID byte
    if(ReadBack != G_I_BETTER_BE)             // Device active and valid
    {
        i2cStatus = 109;
        goto gyroReadIDError;
    }

    // Process success
gyroReadIDSuccess:
    return true;

    // Process the error
gyroReadIDError:
    Serial.println("Gyro::readID - error!");
    Serial.print("Count  = "); Serial.println(i2cFlowCount);
    Serial.print("Status = "); Serial.println(i2cStatus);
    delay(500);
    return false;
}

// Returns FIFO count or 0 for not ready
int Gyro::available()
{
	// Locals
    byte    i2cStatus;
	byte	fifoCount = 0;

	// Read the fifo register
	Wire.beginTransmission(_i2cAddress);
	Wire.write(byte(G_FIFO_S_REG));
	i2cStatus = Wire.endTransmission();
	if(i2cStatus)
	{
	    goto gyroAvailableError;
	}

	Wire.requestFrom(_i2cAddress, byte(1));
	if(waitForI2CResponse(byte(1)) == false)
	{
	    // Return a count of 0 for timeout
		return 0;
	}

	// Read the FIFO count
	fifoCount = Wire.read();
	fifoCount &= G_FIFO_COUNT;
	// Save for use by other members
	_gfifoCount = fifoCount;

	// Process the success
	return(fifoCount);

	// Process the error
gyroAvailableError:
    Serial.println("Gyro::available");
    Serial.print("Status = "); Serial.println(i2cStatus);
    delay(500);
    return 0;
}

// Block read the gyro data
// Assumes data availability has been checked!
int Gyro::ReadGyroData()
{
	// Locals
	int i;
	unsigned char lsbTemp;
	unsigned char msbTemp;
	unsigned short temp;

	// Compute the final index based on the fifo count
	// This will cause the 0th element of each vector
	// array to be the most recent data, the 1st element
	// to be the next most recent, etc.
	for(i = (_gfifoCount); i > 0; i--)
	{
	    // Point to the X axis rotational data FIFO
        Wire.beginTransmission(_i2cAddress);
        Wire.write(byte(G_OUT_X_L));
        Wire.endTransmission();
        // Block read the first element
        Wire.requestFrom(_i2cAddress, sizeof(GYRO_DATA_BLOCK));
        if(waitForI2CResponse(sizeof(GYRO_DATA_BLOCK)) == false)
        {
            return -1;
        }

        // One block of data is read from the I2C
        // format for further processing
        // Roll - rotation around the X-axis
        // Pitch - rotation around the Y-axis
        // Yaw - rotation around the Z-axis

        // Read from fifo LSB/MSB order

        // Read the roll rate data, done do anything with it but
        // it is in the buffer due to the autoincrement feature
        lsbTemp = Wire.read();
        msbTemp = Wire.read();
        _gOrVector[i-1] = (signed short)temp;

        // Read the pitch rate data
        lsbTemp = Wire.read();
        msbTemp = Wire.read();
        // Convert to a 16-bit value
        temp = (unsigned short)((msbTemp << 8) + lsbTemp);
        // Convert and store as a signed value
        _gOpVector[i-1] = (signed short)temp;

        // Read the yaw rate data
        lsbTemp = Wire.read();
        msbTemp = Wire.read();
        // Convert to a 16-bit value
        temp = (unsigned short)((msbTemp << 8) + lsbTemp);
        // Convert and store as a signed value
        _gOyVector[i-1] = (signed short)temp;

	}

	// Success
	return 0;
}

// Compute the roll from the sampled rotational velocity data
// using numerical integration
int Gyro::ComputeRoll(NUM_BUFFER *n, signed short* computedValue)
{
	// Locals
	signed short newRoll;

	// Compute the new integral
	newRoll = trapIntegral(n->tn, n->tn1);
	// Add to the accumulator
	n->t0 = n->t0 + newRoll;
	// Return the new value
	*computedValue = (signed short)n->t0;
	return 0;
}

// Compute the pitch from the sampled rotational velocity data
// using numerical integration
int Gyro::ComputePitch(NUM_BUFFER *n, signed short* computedValue)
{
    // Locals
    signed short newPitch;

    // Compute the new integral
    newPitch = trapIntegral(n->tn, n->tn1);
    // Add to the accumulator
    n->t0 = n->t0 + newPitch;
    // Return the new value
    *computedValue = (signed short)n->t0;
    return 0;
}

// Compute the Heading from the sampled rotational velocity data
// TODO: orient to the front of the robot
int Gyro::ComputeHeading(NUM_BUFFER *n, signed short* computedValue)
{
    // Locals
    signed short newHeading;

    // Compute the new integral
    newHeading = trapIntegral(n->tn, n->tn1);
    // Add to the accumulator
    n->t0 = n->t0 + newHeading;
    // Return the new value
    *computedValue = (signed short)n->t0;
    return 0;
}

// Process rate data
int Gyro::ProcessGyroData(bool test)
{
    // Locals
    int fCount;
    int failSafe;
    int lStatus;
    int i;

    // Read the FIFO count. If the processor throughput is what
    // we believe, there should be at least one sample in each of the
    // FIFOs for the gyroscope. The hardware will be sampling at 8x the
    // output rate, so it should match the rest of the sensors.
    fCount = 0;
    failSafe = 0;
    while(fCount < 1)
    {
        failSafe++;
        fCount = available();
        // Load a timeout of at least 1 msec
        if(failSafe > 10)
        {
            return -1;
        }
    }

    if(test)
    {
    	Serial.println("Gyro FIFO ready");
    }

    // FIFO is ready, read the data
    lStatus = ReadGyroData();

    // Increment the sample count
    _gSampleCount++;

    // Update the validity flags
    if(_gSampleCount > 2)
    {
        _gDataValid = true;
    }

    // Samples are arranged in time-descending order,
    // i.e. 0th element is the most recent

    // Update the numerical buffers
    _pitch.tn1 = _pitch.tn;
    _pitch.tn = _gdPitch;

    _yaw.tn1 = _yaw.tn;
    _yaw.tn = _gdYaw;

    _roll.tn1 = _roll.tn;
    _roll.tn = _gdRoll;

    // Compute the rotational data from the omegas
    if(_gDataValid)
    {
#if PITCH_ENABLE
    	lStatus = ComputePitch(&_pitch, &_gPitch);
#endif

#if ROLL_ENABLE
    	lStatus = ComputeRoll(&_roll, &_gRoll);
#endif

#if YAW_ENABLE
    	lStatus = ComputeHeading(&_yaw, &_gHeading);
#endif

    }

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

signed short Gyro::getRoll()
{
	return _gRoll;
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

// Flush the gyroscope FIFOs
int Gyro::flush()
{
    // Locals
    int lStatus;

    // Set up the fifo count
    lStatus = available();
    // Read and dump the contents of the fifo
    lStatus = ReadGyroData();
}

// Reset the current processing
int Gyro::setOrigin()
{
    // Reset the pitch and heading accumulators
    _gDataValid = false;
    _gSampleCount = 0;
    _pitch.t0 = 0;
    _pitch.tn = 0;
    _pitch.tn1 = 0;
    _yaw.t0 = 0;
    _yaw.tn = 0;
    _yaw.tn1 = 0;
    _roll.t0 = 0;
    _roll.tn = 0;
    _roll.tn1 = 0;
    _gHeading = 0;
    _gPitch = 0;
    _gRoll = 0;
    _gdRoll = 0;
    _gdPitch = 0;
    _gdYaw = 0;

    return 0;
}

// Destructor
Gyro::~Gyro()
{
  // Does nothing
}

// Default instantiation
Gyro    imuGyro = Gyro();


