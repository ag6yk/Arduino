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
  @file Gyro.h
  
  @brief L3G4200D MEMS Motion Sensor (Gyroscope) sensor class
  
  @details This class is a child of the sensor class and describes the 
  L3G4200D MEMS Motion Sensor gyroscope
  
*/

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

#ifndef Gyro_h
#define Gyro_h

#include "Arduino.h"
#include "Sensor.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINES
///////////////////////////////////////////////////////////////////////////////

// Define the requested support
#define	YAW_ENABLE		1
#define ROLL_ENABLE		0
#define PITCH_ENABLE	1

struct GYRO_DATA_BLOCK
{
	unsigned char	hLSB;						// heading
	unsigned char	hMSB;
	unsigned char	pLSB;						// pitch
	unsigned char	pMSB;
	unsigned char	yLSB;						// yaw
	unsigned char	yMSB;
};

////////////////////////////////////////////
// L3G4200D MEMS Motion Sensor (Gyroscope)
////////////////////////////////////////////

// I2C Addressing
// If SDO=+3.3
#define  GYRO_IC_ADDRHI (0xD3 >> 1)      // Wire class uses 7-bit addressing
#define  GYRO_READ_HI   0xD3
#define  GRYO_WRITE_HI  0xD2
// If SDO=0
#define  GYRO_IC_ADDRLO (0xD1 >> 1)
#define  GYRO_READ_LO   0xD1
#define  GRYO_WRITE_LO  0xD0

// Register definitions

// ID register
#define  G_WHO_AM_I       0x0F            // Address
#define  G_I_BETTER_BE    0xD3            // Fixed value

// Control register 1 - Data rate and power modes
#define  G_CTRL_REG1      0x20            // Address
#define  G_XEN            0x01            // Enable x-axis
#define  G_YEN            (0x01 << 1)     // Enable y-axis
#define  G_ZEN            (0x01 << 2)     // Enable z-axis
#define  G_PD             (0x01 << 3)     // Power down enable
#define  G_BWSEL          (0x03 << 4)     // Bandwidth selection
#define  G_DRATE          (0x03 << 6)     // Output Data rate selection

// Control register 2 - High pass filter and cutoff
#define  G_CTRL_REG2      0x21
#define  G_HPCF           0x0F            // High pass filter cut off frequency
#define  G_HPM            0x30            // High pass filter mode selection

// Control register 3 - Interrupt control
#define  G_CTRL_REG3      0x22
#define  G_FIFO_EMPTY_I2  0x01            // Set FIFO empty to INT2
#define  G_FIFO_ORUN_I2   (0x01 << 1)     // Set FIFO overrun to INT2
#define  G_WTM_I2         (0x01 << 2)     // FIFO watermark to INT2
#define  G_DRDY_I2        (0x01 << 3)     // Data ready to INT2
#define  G_PP_OD          (0x01 << 4)     // Push-pull/open drain
#define  G_H_LACTIVE      (0x01 << 5)     // Interrupt active to INT1
#define  G_I1_BOOT        (0x01 << 6)     // Boot status available to INT1
#define  G_I1_ENABLE      (0x01 << 7)     // INT1 enable

// Control register 4 - Data configuration
#define  CTRL_REG4      0x23
#define  G_SIM          0x01              // SPI mode select
#define  G_SELFTEST     (0x03 << 1)       // Self test mode enable
#define  G_FULLSCALE    (0x03 << 4)       // Full scale selection
#define  G_BIGENDIAN    (0x01 << 6)       // 1 = MSB low address
#define  G_BLOCKDATA    (0x01 << 7)       // Block data selection

// Control register 5 - Misc
#define  CTRL_REG5      0x24
#define  G_OUT_SEL      0x03              // Output selection
#define  G_INT1_SEL     (0x03 << 2)       // INT1 selection
#define  G_HPEN         (0x01 << 4)       // High pass filter enable
#define  G_FIFO_EN      (0x01 << 6)       // FIFO enable
#define  G_BOOT         (0x01 << 7)       // 1= re-boot memory contents

// Reference value for data capture - 8 bits
#define  G_REFERENC     0x25

// Temperature output - 8 bits
#define  G_OUT_TEMP     0x26

// Status register
#define  G_STATUS_REG   0x27
#define  G_XDA          0x01              // New x data available
#define  G_YDA          (0x01 << 1)       // New y data available
#define  G_ZDA          (0x01 << 2)       // New z data available
#define  G_ZXYDA        (0x01 << 3)       // New x,y and z data available
#define  G_XOR          (0x01 << 4)       // x data overrun
#define  G_YOR          (0x01 << 5)       // y data overrun
#define  G_ZOR          (0x01 << 6)       // z data overrun
#define  G_ZYXOR        (0x01 << 7)       // x,y and z data overrun

// X angular rate data - 16 bit 2's complement
#define  G_OUT_X_L      0x28
#define  G_OUT_X_H      0x29

// Y angular rate data - 16 bit 2's complement
#define  G_OUT_Y_L      0x2A
#define  G_OUT_Y_H      0x2B

// Z angular rate data - 16 bit 2's complement
#define  G_OUT_Z_L      0x2C
#define  G_OUT_Z_H      0x2D

// Fifo control register
#define  G_FIFO_C_REG   0x2E
#define  G_FIFO_WTM     0x1F              // Watermark
#define  G_FIFO_MODE    0x0E              // Fifo mode select
                                          // 000 Bypass
                                          // 001 FIFO mode
                                          // 010 Stream mode
                                          // 011 Stream-to-FIFO mode
                                          // 100 Bypass-to-stream mode

// FIFO status register
#define  G_FIFO_S_REG   0x2F
#define  G_FIFO_COUNT   0x1F              // Fifo depth
#define  G_FIFO_EMPTY   (0x01 << 5)       // Fifo empty
#define  G_FIFO_OVRN    (0x01 << 6)       // Fifo overrun
#define  G_FIFO_WTMS    (0x01 << 7)       // Fifo Watermark set

// INT1 configuration
#define  G_INT1_CFG     0x30
#define  G_I1_XLE       0x01              // Enable int1 on x low event
#define  G_I1_XHE       (0x01 << 1)       // Enable int1 on x high event
#define  G_I1_YLE       (0x01 << 2)       // Enable int1 on y low event
#define  G_I1_YHE       (0x01 << 3)       // Enable int1 on y high event
#define  G_I1_ZLE       (0x01 << 4)       // Enable int1 on z low event
#define  G_I1_ZHE       (0x01 << 5)       // Enable int1 on z high event
#define  G_LIR          (0x01 << 6)       // Latch int1 irq
#define  G_ANDOR        (0x01 << 7)       // 1=AND events 0=OR events

// Interrupt 1 source
// Reading this register clears the bit and acks the interrupt
#define  G_INT1_SRC       0x31
#define  G_XL             0x01            // x low event
#define  G_XH             (0x01 << 1)     // x high event
#define  G_YL             (0x01 << 2)     // y low event
#define  G_YH             (0x01 << 3)     // y high event
#define  G_ZL             (0x01 << 4)     // z low event
#define  G_ZH             (0x01 << 5)     // z high event
#define  G_IA             (0x01 << 6)     // interrupt active
// Bit 7 reserved = 0

// Threshold values for low and high events (limit checks)
#define  G_INT1_TSH_XH    0x32
#define  G_INT1_TSH_XL    0x33
#define  G_INT1_TSH_YH    0x34
#define  G_INT1_TSH_YL    0x35
#define  G_INT1_TSH_ZH    0x36
#define  G_INT1_TSH_ZL    0x37
// Set an optional threshold delay (allow the signal to return to normal range)
#define  G_INT1_DURATION  0x38

///////////////////////////////////////////////////////////////////////////////
// CLASS AND METHOD DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Forward reference

class Gyro : public Sensor
{
    private:
        unsigned char   	_i2cAddress;
        signed short        _gOrVector[32];
        signed short        _gOyVector[32];
        signed short        _gOpVector[32];
        int					_gfifoCount;
        boolean             _gDataValid;
        int                 _gSampleCount;
        NUM_BUFFER          _pitch;
        NUM_BUFFER          _yaw;
        NUM_BUFFER			_roll;
        signed short        _gdRoll;    // Rotation rate around the X axis
        signed short        _gdPitch;   // Rotation rate around the Y axis
        signed short        _gdYaw;     // Rotation rate around the Z axis
        signed short        _gHeading;  // Heading relative to the robot front
        signed short        _gPitch;    // Pitch relative to robot level
        signed short		_gRoll;		// Roll relative to robot level

        int                 flush();    // clear all gyroscope fifos
        int					resetFifos();

    public:
        Gyro();                     	// Constructor
        ~Gyro();                    	// Destructor
        int begin();                	// Specific initialization
        boolean readID();           	// Read device ID (verify bus)
        int available();            	// Returns FIFO count or 0 for not ready
        int ReadGyroData();	            // Block read of data

        // Compute heading (integrate omega y)
        int ComputeHeading(NUM_BUFFER *n, signed short *computedValue);
        // Compute pitch (integrate omega p)
        int ComputePitch(NUM_BUFFER *n, signed short *computedValue);
        // Comupte roll (integrate omega r)
        int ComputeRoll(NUM_BUFFER *n, signed short *computedValue);

        int ProcessGyroData(bool);      // Process pitch and yaw rates
        signed short getHeading();      // accessors
        signed short getPitch();
        signed short getRoll();
        signed short getdRoll();
        signed short getdPitch();
        signed short getdYaw();
        int          setOrigin();       // reset computation buffers;
        
};

// Default instantiation
extern Gyro imuGyro;

#endif

