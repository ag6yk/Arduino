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
    @file   LibertySensorsIf.ino

    @brief  This file contains the driver program for the Liberty robot
            custom electronics to drive four HC-SR04 ultrasonic range sensors and
            a  VUPN6602 multi-function Inertial Measurement Unit (IMU)

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

// Define the Pins used for the range sensor data
#define PING0       4
#define ECHO0       5
#define PING1       6
#define ECHO1       7
#define PING2       8
#define ECHO2       9
#define PING3       18
#define ECHO3       19

// SPI interface commands
#define CMD_GET_NAV_DATA    0xA6
#define CMD_ABORT_TRANSFER  0xB9
#define CMD_PERFORM_POST    0xC5

// SPI interface responses
#define STS_NAV_DATA_NOT_READY  0x12
#define STS_NAV_DATA_READY      0x36
#define STS_ABORT_ACK           0x90
#define STS_POST_PASS           0xA8
#define STS_POST_FAIL_GYRO      0xF7
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
    unsigned char  SensorStatus;    // Bit mapped for each sensor
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

// Pin assignments
const int	Ping0 = PING0;
const int	Echo0 = ECHO0;
const int	Ping1 = PING1;
const int	Echo1 = ECHO1;
const int	Ping2 = PING2;
const int	Echo2 = ECHO2;
const int	Ping3 = PING3;
const int	Echo3 = ECHO3;

// Instantiate four range sensors
Ultrasonic rangeSensor0 = Ultrasonic::Ultrasonic(Ping0, Echo0);
Ultrasonic rangeSensor1 = Ultrasonic::Ultrasonic(Ping1, Echo1);
Ultrasonic rangeSensor2 = Ultrasonic::Ultrasonic(Ping2, Echo2);
Ultrasonic rangeSensor3 = Ultrasonic::Ultrasonic(Ping3, Echo3);

// SPI interface values
// NAV data buffers
struct NAV_DATA PingPong0;
struct NAV_DATA PingPong1;
// Read data directly into the buffer
volatile struct NAV_DATA* ReadPtr;
// Data is transferred over SPI in bytes
volatile unsigned char* WritePtr;
volatile boolean SPIDataReady;
volatile boolean SPINewNavData;
volatile int SPINavDataTransferState;
volatile int SPITransferCount;


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
  unsigned char *qptr;
  // Set up the debugger
  Serial.begin(115200);
  
  // Clear the NAV buffers
  pptr = (unsigned char*)(&PingPong0);
  qptr = (unsigned char*)(&PingPong1);
  for(i=0; i < sizeof(NAV_DATA); i++)
  {
    *pptr++ = 0;
    *qptr++ = 0;
  }
  
  // Initialize the I2C as a master
  Wire.begin();
  
  // Initialize the SPI as a slave
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SS, INPUT);
  pinMode(SCK, INPUT);
  
  // turn on SPI in slave mode
  // MSB transmitted first
  // SPI Mode 0
  SPCR |= _BV(SPE);
  
  // get ready for the SPI interrupt
  // Start filling ping pong 0
  SPIDataReady = false;
  SPINewNavData = false;
  SPINavDataTransferState = 0;
  SPITransferCount = 0;
  // Initialize the ping pong buffer
  ReadPtr = &PingPong0;
  WritePtr = (unsigned char*)&PingPong1;
  
  // Configure the IMU
  
  // Configure the gyroscope
  imuGyro.begin();
  
  // Configure the accelerometer
  imuAccel.begin();
 
  // Configure the compass
  imuCompass.begin();
 
  // Configure the barometer
  imuBarometer.begin();
  
  // Enable the SPI interrupt
  SPI.attachInterrupt();
 
}

// SPI interrupt service routine
ISR(SPI_STC_vect)
{
  // Locals
  byte spiData;

  // First check for any data from
  // the master
  spiData = SPDR;
  
  switch(SPINavDataTransferState)
  {
    // Waiting for new command
    case 0:
    {
      if(SPDR == CMD_GET_NAV_DATA)
      {
        // See if a new buffer is ready
        // If not inform the master not yet
        if(SPINewNavData == false)
        {
          SPDR = STS_NAV_DATA_NOT_READY;
        }
        else
        // New nav data ready to go
        {
          SPDR = STS_NAV_DATA_READY;
          SPINavDataTransferState = 1;
          SPITransferCount = 0;
        }
      }
      // ignore everything else
      break;
    }
    
    // Writing nav data out to master
    case 1:
    {
      // Make sure master is not killing transaction
      if(SPDR == CMD_ABORT_TRANSFER)
      {
        // Force completion
        SPDR = STS_ABORT_ACK;
        SPITransferCount = sizeof(NAV_DATA);
      }
      else
      {
        // Write the next byte to the bus
        SPDR = *WritePtr++;
        SPITransferCount++;
      }
      // See if buffer complete
      if(SPITransferCount >= sizeof(NAV_DATA))
      {
        // Transfer complete
        SPINavDataTransferState = 0;
        SPINewNavData = false;
        // Swap the ping pong
        if(WritePtr == (unsigned char*)&PingPong0)
        {
          WritePtr = (unsigned char*)&PingPong1;
        }
        else
        {
          WritePtr = (unsigned char*)&PingPong0;
        }
      }
      break;
    }
  }
}

// Executive loop
void loop(void)
{
  // Locals
  
  // SPI transactions handled in interrupt
 
  // Process IMU
  
  // Check if new Gyro data is available
  
  // If so, read and process the Gyro data
  
  // Check if new Accelerometer data is available
  
  // If so, read and process the accelerometer data
  
  // Check if new Compass data is available
  
  // If so, read and process the compass data
  
  // FUTURE: Check if new pressure or temperature data is available
 
  // Process Range sensors
  ReadPtr->Range_0 = rangeSensor0.ReadRange();
  ReadPtr->Range_1 = rangeSensor1.ReadRange();
  ReadPtr->Range_2 = rangeSensor2.ReadRange();
  ReadPtr->Range_3 = rangeSensor3.ReadRange();
  
  // And do it again, and again, ...
}

// End if LibertySensorIF.ino

