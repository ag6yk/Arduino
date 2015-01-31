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
  @file Compass.h
  
  @brief HMC5883L Digital Compass sensor class
  
  @details This class is a child of the sensor class and describes the 
   HMC5883L Digital Compass 
  
*/

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

#ifndef Compass_h
#define Compass_h

#include "Arduino.h"
#include "Sensor.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINES
///////////////////////////////////////////////////////////////////////////////

/////////////////////////////
// HMC5883L Digital Compass
/////////////////////////////

// I2C Address
#define  C_IC_ADR      (0x3D >> 1)        // Wire class uses 7-bit addresses
#define  C_READ        0x3D
#define  C_WRITE       0x3C

// Register definitions

// Configuration
#define  C_CONFIG_A    0x00                // output rate measurement config
#define  C_MEASURE_M   0x03                // Measurement mode
                                           // 00 - Normal
                                           // 01 - Positive bias
                                           // 10 - Negative bias
                                           // 11 - reserved
#define  C_DATA_RATE   (0x07 << 2)         // Output data rate
                                           // 000 - 0.75 Hz
                                           // 001 - 1.5 Hz
                                           // 010 - 3 Hz
                                           // 011 - 7.5 Hz
                                           // 100 - 15 Hz (default)
                                           // 101 - 30 Hz
                                           // 110 - 75 Hz
                                           // 111 - Reserved
#define  C_CONFIG_B    0x01                // Device gain 
                                           // Fill in if needed
                                           
#define  C_MODE        0x02                // Mode register
#define  C_MODE_SEL    0x03                // Measurement modes
                                           // 00 - continuous
                                           // 01 - single 
                                           // 1x - idle
#define  C_MODE_HS     (0x01 << 7)         // 1-high speed I2C (3.4M)

// Output Data
// 16-bit 2's complement
#define  C_X_MSB       0x03
#define  C_X_LSB       0x04
#define  C_Y_MSB       0x05
#define  C_Y_LSB       0x06
#define  C_Z_MSB       0x07
#define  C_Z_LSB       0x08

// Status
#define  C_STATUS      0x09
#define  C_STS_RDY     0x01                // Data available
#define  C_STS_LOCK    (0x01 << 1)         // output registers locked


// ID
#define  C_ID_A        0x0A                // ID value = 0x48
#define  C_ID_A_VAL    0x48
#define  C_ID_B        0x0B                // ID value = 0x34
#define  C_ID_C        0x0C                // ID value = 0x33

///////////////////////////////////////////////////////////////////////////////
// DATA STRUCTURES
///////////////////////////////////////////////////////////////////////////////
struct MAG_DATA
{
    unsigned char   xMSB;                   // X-axis magnetometer data
    unsigned char   xLSB;
    unsigned char   yMSB;
    unsigned char   yLSB;
    unsigned char   zMSB;
    unsigned char   zLSB;
};


///////////////////////////////////////////////////////////////////////////////
// CLASS AND METHOD DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

class Compass : public Sensor
{
    private:
        unsigned char _i2cAddress;
    public:
        Compass();                          // Constructor
        ~Compass();                         // Destructor
        int begin();                        // specialized initialization
        boolean readID();                   // Read device ID (verify bus)
        int ReadXYZ(MAG_DATA* pMagData);    // block read the output data
};

// Default instantiation
extern  Compass imuCompass;

#endif
