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
  @file Accel.h
  
  @brief ADXL345 3-axis Accelerometer sensor class
  
  @details This class is a child of the sensor class and describes the 
   ADXL345 3-axis Accelerometer 
  
*/

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

#ifndef Accel_h
#define Accel_h

#include "Arduino.h"
#include "Sensor.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINES
///////////////////////////////////////////////////////////////////////////////

// Define a block read element
struct ACCEL_DATA_BLOCK
{
	unsigned char	xLSB;					// least significant byte X
	unsigned char	xMSB;					// most significant byte X
	unsigned char	yLSB;					// and so on
	unsigned char	yMSB;
	unsigned char	zLSB;
	unsigned char	zMSB;
};

///////////////////////////////////
// ADXL345 3-axis Accelerometer 
///////////////////////////////////

// I2C Addressing
// CS=3.3V
#define  ACCEL_IC_ADDRHI (0x3B >> 1)        // Wire class uses 7-bit address
#define  ACCEL_READ_HI   0x3B
#define  ACCEL_WRITE_HI  0x3A
// CS=0V
#define  ACCEL_IC_ADDRLO (0xA7 >> 1)
#define  ACCEL_READ_LO   0xA7
#define  ACCEL_WRITE_LO  0xA6

// Register definitions

// Device ID register
#define  A_DEVID         0x00              
#define  A_I_BETTER_BE   0xE5              // Fixed value

// TAP threshold
#define  A_THRESH_TAP    0x1D              // TAP threshold

// Accelerometer offset adjustments
#define  A_X_OFFSET      0x1E              // X-axis offset
#define  A_Y_OFFSET      0x1F              // Y-axis offset
#define  A_Z_OFFSET      0x20              // Z-axis offset

// TAP controls
#define  A_DURATION      0x21              // Tap duration
#define  A_LATENCY       0x22              // Tap latency
#define  A_WINDOW        0x23              // Tap window

// Activity/inactivity controls
#define  A_THRESH_ACT    0x24              // Activity threshold
#define  A_THRESH_INACT  0x25              // Inactivity threshold
#define  A_TIME_INACT    0x26              // Inactivity time
#define  A_ACT_INACT_C   0x27              // Axis enable control for activity

// Free-fall controls
#define  A_THRESH_FF     0x28              // Free-fall threshold
#define  A_TIME_FF       0x29              // Free-fall time

// Single tap
#define  A_TAP_AXES      0x2A              // Axis control for single tap
#define  A_ACT_TAP_STS   0x2B              // Source for single tap

// Data control

// Data rate
#define  A_BW_RATE       0x2C              // Data rate and power mode
#define  A_DATA_RATE     0x0F              // data output rate (default 100 Hz)
#define  A_LOW_PWR       (0x01 << 4)       // Enable low power mode

// Power management
#define  A_PWR_CTRL      0x2D              // Power management
#define  A_WAKEUP        0x03              // Sampling Wakeup control
                                           // 00 - 8 Hz
                                           // 01 - 4 Hz
                                           // 10 - 2 Hz
                                           // 11 - 1 Hz
                                           
#define  A_SLEEP         (0x01 <<  2)      // Sleep mode enable
#define  A_MEASURE       (0x01 << 3)       // 1= active measurement mode
#define  A_AUTO_SLEEP    (0x01 << 4)       // Sleep on inactivity
#define  A_LINK          (0x01 << 5)       // links inactivity and activity

// Interrupt Control
#define  A_INT_ENABLE    0x2E              // Interrupt enable control
                                           // 1=enable 0=disable
#define  A_INT_OVR       0x01              // Overrun
#define  A_INT_WTM       (0x01 << 1)       // Watermark
#define  A_INT_FF        (0x01 << 2)       // Free-fall
#define  A_INT_INACT     (0x01 << 3)       // Inactivity
#define  A_INT_ACT       (0x01 << 4)       // Activity
#define  A_INT_DTAP      (0x01 << 5)       // Double-tap
#define  A_INT_STAP      (0x01 << 6)       // Single tap
#define  A_INT_DREADY    (0x01 << 7)       // Data ready

#define  A_INT_MAP       0x2F              // Interrupt mapping 
                                           // Bit map same as enable
                                           // 1=INT2 0=INT1

#define  A_INT_SRC       0x30              // Interrupt source status
                                           // Bit map same as enable

// Data Format
#define  A_DATA_FMT      0x31              // Data format control
#define  A_DATA_RNG      0x03              // g range
                                           // 00 +/- 2g
                                           // 01 +/- 4g
                                           // 10 +/- 8g
                                           // 11 +/- 16g
#define  A_DATA_JUS      (0x01 << 2)       // 1=left justified (MSB)
                                           // 0=right justified w/ sign extension
#define  A_FULL_RES      (0x01 << 3)       // 1- full resolution mode (4mg/LSB)
                                           // 0- 10-bit resolution
#define  A_INT_INVERT    (0x01 << 5)       // 1- interrupts to active low
#define  A_DATA_SPI      (0x01 << 6)       // 1- 3wire SPI 0- 4-wire SPI
#define  A_SELF_TEST     (0x01 << 7)       // Enable self testing

// Acceleration Data
// 16-bit 2's complement
#define  A_DATA_X0       0x32              // X-axis data 0 LSB
#define  A_DATA_X1       0x33              // X-axis data 1 MSB
#define  A_DATA_Y0       0x34              // y-axis data 0
#define  A_DATA_Y1       0x35              // y-axis data 1
#define  A_DATA_Z0       0x36              // z-axis data 0
#define  A_DATA_Z1       0x37              // z-axis data 1

// FIFO control
#define  A_FIFO_CTL      0x38              // FIFO control
#define  A_FIFO_THRS     0x1F              // FIFO threshold count
                                           // writing a 0 sets WTM
                                           
#define  A_FIFO_TRG      (0x01 << 5)       // 0- trigger event to INT1
#define  A_FIFO_MODE     (0x03 << 6)       // FIFO mode
                                           // 00 - bypass
                                           // 01 - FIFO
                                           // 10 - Stream
                                           // 11 - Trgger
                                           
#define  A_FIFO_STS      0x39              // FIFO status
#define  A_FIFO_CNT      0x3F              // FIFO count
#define  A_FIFO_TRIG     (0x01 << 7)       // FIFO trigger event occurred

///////////////////////////////////////////////////////////////////////////////
// CLASS AND METHOD DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

class Accel : public Sensor
{
    private:
        unsigned char 		_i2cAddress;
        signed short        _aXvector[32];
        signed short        _aYvector[32];
        signed short        _aZvector[32];
        int					_afifoCount;
        boolean             _aVvalid;
        boolean             _aPvalid;
        int                 _aSampleCount;
        NUM_BUFFER			_aVComputingX;
        NUM_BUFFER          _aPComputingX;
        NUM_BUFFER          _aVComputingY;
        NUM_BUFFER			_aPComputingY;
        signed short		_accelerationX;
        signed short		_velocityX;
        signed short		_positionX;
        signed short		_accelerationY;
        signed short		_velocityY;
        signed short		_positionY;

        int                 flush();        // flush the h/w fifos

    public:
        Accel();                            // Constructor
        ~Accel();                           // Destructor
        int begin();                        // Specific initialization
        boolean readID();                   // Read device ID (verify bus)
        int available();				    // Returns fifo count or
        									// 0 for not ready
        int ReadXYZ();                      // Block read of data

        // Compute velocity from sampled data
        int ComputeVoft(NUM_BUFFER *n, signed short* computedValue);
        // Compute position from sampled data
        int ComputeXoft(NUM_BUFFER *n, signed short* computedValue);

        // Process acceleration data from sensor
        int ProcessAccelData(int);
        signed short getAccelerationX();    // accessors
        signed short getAccelerationY();
        signed short getVelocityX();
        signed short getVelocityY();
        signed short getPositionX();
        signed short getPositionY();

        int         setOrigin();            // set the origin

};

// Default instantiation
extern Accel    imuAccel;
#endif

