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


///////////////////////////////////////////////////////////////////////////////
// DATA structures
///////////////////////////////////////////////////////////////////////////////


// Define the Nav Data packet status to the RobotRIO
// SPI payload
struct NAV_DATA
{
    unsigned char  SensorStatus;    // Bit mapped for each sensor
    char		   FrameCount;      // Packet count
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

// NAV data buffers
struct NAV_DATA	PingPong0;
struct NAV_DATA PingPong1;
struct NAV_DATA* ReadPtr = &PingPong0;
struct NAV_DATA* WritePtr = &PingPong1;

volatile byte pos;
volatile boolean process_it;
char buf[100];


///////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// ARDUINO-REQUIRED FUNCTIONS
///////////////////////////////////////////////////////////////////////////////

// Initialization
void setup(void)
{
  // Set up the debugger
  Serial.begin(115200);

  // Initialize the I2C as a master
  Wire.begin();
  
  // Initialize the SPI as a slave
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SS, INPUT);
  pinMode(SCK, INPUT);
  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  
  // get ready for the SPI interrupt
  pos = 0;
  process_it = false;
  
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
	
	// grab a byte from the SPI Data register
	byte c = SPDR;
	
	// add to buffer if room
	if(pos < sizeof buf)
	{
		buf[pos++] = c;
		
		// process data
		process_it = true;
	}
}

// Executive loop
void loop(void)
{
  // Locals
  
  // Check for any requests from the SPI
  if(process_it)
  {
  	// send the correct packets
  	process_it = false;
  }
  // If data available send packet
  // Process IMU
  // Process Range sensors
  WritePtr->Range_0 = rangeSensor0.ReadRange();
  WritePtr->Range_1 = rangeSensor1.ReadRange();
  WritePtr->Range_2 = rangeSensor2.ReadRange();
  WritePtr->Range_3 = rangeSensor3.ReadRange();
}

// End if LibertySensorIF.ino

