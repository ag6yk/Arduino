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

//////////////////////////
// Arduino Pin definitions
//////////////////////////


// Pins 10-13 used as SPI interface
// Pins 0-1 uses as serial debug monitor

// Define the Pins used for the range sensor data
#define PING0       2
#define ECHO0       3
#define PING1       4
#define ECHO1       5
#define PING2       6
#define ECHO2       7
#define PING3       8
#define ECHO3       9
#define PING4       A0
#define ECHO4       A1

// Define the Pins used to interface to the IMU
// SDA and SCL already defined for A4 and A5 respectively

// Pins 10-13 used as SPI interface
// Pins 0-1 uses as serial debug monitor

// SPI pseudo register map
//
// Registers
#define SPI_SET_ORIGIN          0x10            // set the current position to (0,0,0)
#define SPI_NAV_DATA_STS        0x20            // Status request
#define SPI_NAV_DATA_DAT        0x30            // read nav data (autoincrement)
#define SPI_PERFORM_POST        0x40            // run self-test (future, reserved)

// Nav data status responses
#define STS_NAV_DATA_READY      (0x01 << 0)     // bit set indicates ready
#define STS_NAV_DATA_NREADY     0x00            // not ready macro

// POST responses
#define STS_POST_PASS           0x01            // Success
#define STS_POST_FAIL_GYRO      0xF7            // Failure codes
#define STS_POST_FAIL_ACCEL     0xF8
#define STS_POST_FAIL_COMPASS   0xF9
#define STS_POST_FAIL_BARO      0xFA

//////////////////
// SPI Interface
//////////////////

/*!
The SPI interface will use a register-based architecture. The RobotRIO master
will write to pseudo-register addresses to command the Arduino slave to 
see if a new nav data packet is ready, to send a packet of nav data, 
to set the origin, and in future to run calibration and self-test

The Nav data will be updated every 20 milliseconds, with the Arduino 
collecting/processing the data in the first 10 millisconds and the other
10 milliseconds transferring the data. Depending on processor throughput this
update rate may be increased (e.g. 10ms, 5ms processing 5ms transferring)

The nav data will be double-buffered, with the main processing loop reading
the sensors, processing the data, and filling one buffer while the other
buffer is available to the SPI interface. The buffers will be accessed by
dedicated pointers, one for producing and one for consuming.
The producer will have control of the buffer switching. 

*/

///////////////////////
// SPI Transfer states
///////////////////////

#define SPI_XFR_IDLE    0               // Idle
#define SPI_XFR_ACTIVE  1               // Producing or consuming


///////////////////////////////////////////////////////////////////////////////
// DATA structures
///////////////////////////////////////////////////////////////////////////////


// Define the Nav Data packet status to the RobotRIO
// SPI payload
struct NAV_DATA
{
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
    unsigned char  Range_4;         // Range 4 sensor reading
};


///////////////////////////////////////////////////////////////////////////////
// GLOBAL DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

// C++ Pin assignments
const int    Ping0 = PING0;          // Range sensor 0 (front left)
const int    Echo0 = ECHO0;
const int    Ping1 = PING1;          // Range sensor 1 (front right)
const int    Echo1 = ECHO1;
const int    Ping2 = PING2;          // Range sensor 2 (right)
const int    Echo2 = ECHO2;
const int    Ping3 = PING3;          // Range sensor 3 (back)
const int    Echo3 = ECHO3;
const int    Ping4 = PING4;          // Range sensor 4 (left)
const int    Echo4 = ECHO4;

// Instantiate five range sensor objects
Ultrasonic rangeSensor0(Ping0, Echo0);
Ultrasonic rangeSensor1(Ping1, Echo1);
Ultrasonic rangeSensor2(Ping2, Echo2);
Ultrasonic rangeSensor3(Ping3, Echo3);
Ultrasonic rangeSensor4(Ping4, Echo4);


/////////////////////////////////
// NAV data buffers and pointers
/////////////////////////////////

struct NAV_DATA navBuffer0;
struct NAV_DATA navBuffer1;
struct NAV_DATA* producer = &navBuffer0;
struct NAV_DATA* consumer = &navBuffer1;

// Producer state variables
volatile int        producerCount;          // Data counter
volatile int        producerState;          // main state variable

// Consumer state variables
volatile int        consumerCount;          // Data counter
volatile int        consumerState;          // main state variable
volatile boolean    consumerLock;           // true = current buffer locked
volatile boolean    consumerEnable;         // true = consumer can update

// Inter-process variables
volatile boolean    spiSetOrigin;           // true = read current position, set as 0,0

// Device status
int             GyroStatus;                 // 0 = device OK
int             AccelStatus;
int             CompassStatus;
int             BaroStatus;



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
    Serial.begin(9600);
    Serial.println("Setup Debug Info...");
    delay(500);
  
    // Clear the NAV buffers
    pptr = (unsigned char*)(&navBuffer0);
    qptr = (unsigned char*)(&navBuffer1);
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
    
    // Initialize the producer state machine
    producer = &navBuffer0;
    producerCount = 0;
    producerState = SPI_XFR_IDLE;
    
    // Initialize the consumer state machine
    // Put on hold until the first nav data is ready 
    consumer = &navBuffer0;
    consumerCount = 0;
    consumerState = SPI_XFR_IDLE;
    consumerLock = false;
    consumerEnable = false;
    
    // Initialize the inter-process variables
    spiSetOrigin = false;
    
    // Preset the SPI buffer 
    SPDR = STS_NAV_DATA_NREADY;
    
  
    Serial.println("SPI configured");
    delay(500);
  
    // Configure the IMU
 
    // Configure the gyroscope
//  GyroStatus = imuGyro.begin();
//  Serial.print("GyroStatus = ");
//  Serial.println(GyroStatus, DEC);
//  delay(500);
  
  
    // Configure the accelerometer
    AccelStatus = imuAccel.begin();
    Serial.print("AccelStatus = ");
    Serial.println(AccelStatus, DEC);
 
  // Configure the compass
//  CompassStatus = imuCompass.begin();
//  Serial.print("CompassStatus = ");
//  Serial.println(CompassStatus, DEC);
  
  // Configure the barometer
//  BaroStatus = imuBarometer.begin();
//  Serial.print("BaroStatus = ");
//  Serial.println(BaroStatus, DEC);
  
  // Enable the SPI interrupt
//  SPI.attachInterrupt();
 
}

#if 0
// SPI interrupt service routine
ISR(SPI_STC_vect)
{
    // Locals
    byte spiData;
    
    // The nature of SPI guarantees we have some
    // data
    spiData = SPDR;
    
    // Process the interrupt based on the state and
    // the input value
    switch(consumerState)
    {
        // Idle, waiting for new command
        case SPI_XFR_IDLE:
        {
            // Process each register
            switch(spiData)
            {
                // Set the inter-process flag
                case SPI_SET_ORIGIN:
                {
                    spiSetOrigin = true;
                    break;
                }
                
                // Report whether new data is available
                case SPI_NAV_DATA_STS:
                {
                    if(consumerEnable == true)
                    {
                        spiData = STS_NAV_DATA_READY;
                        break;
                    }
                    else
                    {
                        spiData = STS_NAV_DATA_NREADY;
                        break;
                    }
                }

                // Read Nav data
                case SPI_NAV_DATA_DAT:
                {
                    // If the buffers are not locked
                    // begin transmission
                    
                    // Else set data not ready
                    break;
                }
                
                // Illegal or dummy write
                default:
            }// End switch

        
        // Actively transferring nav data
        case SPI_XFR_ACTIVE:
            // Process each register
            switch(spiData)
            {
                // Set the inter-process flag
                case SPI_SET_ORIGIN:
                {
                    // Transition to IDLE
                    // Signal the main loop to re-set the origin
                    spiSetOrigin = true;
                    break;
                }
                
                // Report whether new data is available
                case SPI_NAV_DATA_STS:
                {
                    // Transition to IDLE
                    if(consumerEnable == true)
                    {
                        spiData = STS_NAV_DATA_READY;
                        break;
                    }
                    else
                    {
                        spiData = STS_NAV_DATA_NREADY;
                        break;
                    }
                }

                // Read Nav data
                case SPI_NAV_DATA_DAT:
                {
                    // Continue processing
                    // If the buffers are not locked
                    // begin transmission
                    
                    // Else set data not ready
                    break;
                }
                
                // Dummy write to receive data
                default:
                {
                    // Continue processing nav data
                }
            }// End switch

        // Illegal state - go to IDLE
        default:
        {
        }
    }// End switch
        
}
#endif 

// Executive loop
void loop(void)
{
#if 0
  
  // Process IMU. This will occur every 10 ms
  
  // Process any new accelerometer data
  
  // Process any new Gyroscope data
  
  // Process any new Compass data
  
  // FUTURE: Process any new pressure data
  
  // FUTURE: Process any new temperature data
 
  // Process Range sensors
  producer->Range_0 = rangeSensor0.ReadRange();
  producer->Range_1 = rangeSensor1.ReadRange();
  producer->Range_2 = rangeSensor2.ReadRange();
  producer->Range_3 = rangeSensor3.ReadRange();
  producer->Range_4 = rangeSensor4.ReadRange();
  
  // Switch nav data buffers if available
  // If not wait 1 ms and try again
  
  // And do it again, and again, ...
  
#endif
}

// End if LibertySensorIF.ino

