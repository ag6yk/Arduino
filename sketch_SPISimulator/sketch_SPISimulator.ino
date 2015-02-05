////////////////////////////////////////////////////////////////////////////////
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
    @file   sketch_SPISimulator.ino

    @brief  This file contains the driver program to simulate the RobotRIO
    communication with the LibertySensorIF arduino microcontroller.

    @author Robert Cavanaugh, Engineering Mentor

    @version    Revision History

*/

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <SPI.h>
#include "Sensor.h"
#include "Gyro.h"
#include "Accel.h"
#include "Compass.h"
#include "Baro.h"
#include "Ultrasonic.h"

///////////////////////////////////////////////////////////////////////////////
//
// DEFINITIONS
//
///////////////////////////////////////////////////////////////////////////////

//////////////////////////
// Arduino Pin definitions
//////////////////////////


// Pins 10-13 used as SPI interface
// Pins 0-1 uses as serial debug monitor

// SPI pseudo register map
//
// Registers
#define SPI_NAV_DATA_STS    0x10                // Status request
#define SPI_NAV_DATA_DAT    0x20                // read nav data (autoincrement)
#define SPI_PERFORM_POST    0x30                // run self-test (future, reserved)

// Nav data status responses
#define STS_NAV_DATA_NOT_READY  (0x01 << 0)     // bit set indicates busy

// POST responses
#define STS_POST_PASS           0x01            // Success
#define STS_POST_FAIL_GYRO      0xF7            // Failure codes
#define STS_POST_FAIL_ACCEL     0xF8
#define STS_POST_FAIL_COMPASS   0xF9
#define STS_POST_FAIL_BARO      0xFA

///////////////////////////////////////////////////////////////////////////////
// DATA structures
///////////////////////////////////////////////////////////////////////////////


// Define the Nav Data packet status to the RobotRIO
// SPI payload
struct NAV_DATA
{
    char           FrameCount;      // Packet count
    char           Position_X_MSB;  // Most significant byte of X position data
    char           Position_X_LSB;  // Least significant byte of X position data
    char           Velocity_X_MSB;  // Most significant byte of X velocity data
    char           Velocity_X_LSB;  // Least significant byte of X velocity data
    char           Accel_X_MSB;     // Most significant byte of X acceleration data
    char           Accel_X_LSB;     // Least significant byte of X acceleration data
    char           Position_Y_MSB;  // Most significant byte of Y position data
    char           Position_Y_LSB;  // Least significant byte of Y position data
    char           Velocity_Y_MSB;  // Most significant byte of Y velocity data
    char           Velocity_Y_LSB;  // Least significant byte of Y velocity data
    char           Accel_Y_MSB;     // Most significant byte of Y acceleration data
    char           Accel_Y_LSB;     // Least significant byte of Y acceleration data
    char           Heading_MSB;     // Most significant byte of heading 
                                    // (relative to robot)
    char           Heading_LSB;     // Least significant byte of heading 
    char           Pitch_MSB;       // Most significant byte of pitch
    char           Pitch_LSB;       // Least significant byte of pitch
    unsigned char  Range_0;         // Range 0 sensor reading
    unsigned char  Range_1;         // Range 1 sensor reading
    unsigned char  Range_2;         // Range 2 sensor reading
    unsigned char  Range_3;         // Range 3 sensor reading
};


///////////////////////////////////////////////////////////////////////////////
// GLOBAL DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

// SPI input buffer
struct NAV_DAtA inBuf;
struct NAV_DATA *pBuf; = &inBuf;


///////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// ARDUINO-REQUIRED FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

// Initialization
void setup(void)
{
    // Locals
    int i;
    unsigned char *pptr;
  
    // Set up the debugger
    Serial.begin(9600);
    Serial.println("SPI simulator: Setup... ");
    delay(500);
  
    // Clear the input buffer
    pptr = (unsigned char*)(&inBuf);
    for(i=0; i < sizeof(NAV_DATA); i++)
    {
        *pptr++ = 0;
    }

    // Initialize the SPI interface as master
    SPI.begin();
 
}

// Executive loop
void loop(void)
{
    // Locals
    unsigned char   SPIData;

    // See if the data is ready
    digitalWrite(SS, LOW);
    SPI.transfer(SPI_NAV_DATA_STS);
    

  
  
  // And do it again, and again, ...
}

// End of sketch_SPISimulator.ino

