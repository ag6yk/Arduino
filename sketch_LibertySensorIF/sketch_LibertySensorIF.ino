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
            a GY-80 multi-function Inertial Measurement Unit (IMU)

    @author Robert Cavanaugh, Engineering Mentor

    @version    Revision History

*/

///////////////////////////////////////////////////////////////////////////////
// CONFIGURATION
///////////////////////////////////////////////////////////////////////////////

// Conditional compile flags for testing environments and transports

// Enable to see serial monitor output
#define BENCH_TESTING   1
#define BENCH_DISPLAY_DEBUG	1

#if BENCH_DISPLAY_DEBUG
#define dbg_print(x)        Serial.print(x)
#define dbg_println(x)      Serial.println(x)
#define dbg_printlnm(x,y)   Serial.println(x,y)
#else
#define dbg_print(x)
#define dbg_println(x)
#define dbg_printlnm(x,y)
#endif


// Enable to test NAV interface with canned data
#define NAV_DATA_TEST   0

// Enable to test the Arduino sensor interface
// without connection to the nav shield
#define NAV_SENSOR_TEST 0

// Conditionally compile support for each sensor in the IMU
#define ACCEL_ENABLE    1
#define GYRO_ENABLE     1
#define COMPASS_ENABLE  0
#define BARO_ENABLE     0
#define RANGES_ENABLE   0

// Conditionally compile support for the SPI interrupt
#define SPI_INT_ENABLE  0

// Conditionally compile support for software UART
#define SUART_ENABLE    1


///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////

// Due to a quirk in the Arduino build process,
// All Arduino library functions must be included here

// I2C support
#include <Wire.h>
// SPI transport
#include <SPI.h>
// Software UART
#include <SoftwareSerial.h>
#if SPI_INT_ENABLE
#include <NavSPI.h>
#else
#if SUART_ENABLE
#include <NavUart.h>
#else
#error "No valid transport selected. Choose SPI or Soft UART"
#endif
#endif
// Sensor superclass
#include "Sensor.h"
// Specific sensor classes
#include "Gyro.h"
#include "Accel.h"
#include "Compass.h"
#include "Baro.h"
#include "Ultrasonic.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////


//////////////////////////
// Arduino Pin definitions
//////////////////////////

// Define the Pins used to interface to the IMU
// SDA and SCL already defined for A4 and A5 respectively
// No other signals are used from the IMU

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

#if SUART_ENABLE
// Define the soft UART pins
#define SUART_RX    12
#define SUART_TX    13
// D10 and D11 are spare
// A2 and A3 are spare
#else
// Pins 10-13 are reserved for SPI interface
// A2 and A3 are spare
#endif

// Time constants
#define	DISPLAY_TIME	1000
#define UPDATE_TIME		10
#define DEBUG_RATE		115200


///////////////////////////////////////////////////////////////////////////////
// GLOBAL DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

// Ultrasonic sensor support

// C++ Pin assignments
const int    Ping0 = PING0;             // Range sensor 0 (front left)
const int    Echo0 = ECHO0;
const int    Ping1 = PING1;             // Range sensor 1 (front right)
const int    Echo1 = ECHO1;
const int    Ping2 = PING2;             // Range sensor 2 (right)
const int    Echo2 = ECHO2;
const int    Ping3 = PING3;             // Range sensor 3 (back)
const int    Echo3 = ECHO3;
const int    Ping4 = PING4;             // Range sensor 4 (left)
const int    Echo4 = ECHO4;



// Instantiate five range sensor objects
Ultrasonic rangeSensor0(Ping0, Echo0);
Ultrasonic rangeSensor1(Ping1, Echo1);
Ultrasonic rangeSensor2(Ping2, Echo2);
Ultrasonic rangeSensor3(Ping3, Echo3);
Ultrasonic rangeSensor4(Ping4, Echo4);

#if SUART_ENABLE
// C++ Pin assignments
const int   UartRx = SUART_RX;          // Software UART RX pin
const int   UartTx = SUART_TX;          // Software UART TX pin

// Instantiate the software UART
NavUart  navIf((int)UartRx, (int)UartTx, false);

#else
#if SPI_INT_ENABLE
// Instantiate the SPI transport
NavSPI  navIf(0, 0);

#endif
#endif

// Device status
int             GyroStatus;                 // 0 = device OK
int             AccelStatus;
int             CompassStatus;
int             BaroStatus;

// Sample timing variables
// Nav data update state
unsigned long   previousMillis;
unsigned long   currentMillis;
unsigned long   eTime;                      // elapsed time in ms
unsigned long   updateInterval = 1;        // 10 ms update rate
boolean         upDate = false;
unsigned long   updateCount = 0;

// Display update state
unsigned long   previousDisplayMillis;
boolean         Display = false;

// Arduino heartbeart
unsigned long   ArduinoHeartBeat;           // heart beat variable
boolean         hbUpdate = false;


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
    int Status;
    
#if BENCH_TESTING
    // Set up the debugger
    Serial.begin(DEBUG_RATE);
    Serial.println(" ");
    Serial.println("Setup Debug Info...");
    delay(500);
#endif

    // Initialize the I2C as a master
    Wire.begin();

    // Configure the IMU
 
#if GYRO_ENABLE
    // Enable the gyroscope
    navIf.setGyroscope(&imuGyro);
    // Configure the gyroscope
    GyroStatus = imuGyro.begin();
    dbg_print("GyroStatus = ");
    dbg_printlnm(GyroStatus, DEC);
#endif
 
#if ACCEL_ENABLE
    // Enable the accelerometer
    navIf.setAccelerometer(&imuAccel);
    // Configure the accelerometer
    AccelStatus = imuAccel.begin();
    dbg_print("AccelStatus = ");
    dbg_printlnm(AccelStatus, DEC);
#endif

#if COMPASS_ENABLE
    // Enable the compass
    navIf.setCompass(&imuCompass);
    // Configure the compass
    CompassStatus = imuCompass.begin();
    dbg_print("CompassStatus = ");
    dbg_printlnm(CompassStatus, DEC);
#endif

#if BARO_ENABLE
    // Enable the barometer
    navIf.setBarometer(&imuBarometer);
    // Configure the barometer
    BaroStatus = imuBarometer.begin();
    dbg_print("BaroStatus = ");
    dbg_printlnm(BaroStatus, DEC);
#endif

#if RANGES_ENABLE
    // Enable the range sensors
    navIf.setRangeSensor(&rangeSensor0, 0);
    navIf.setRangeSensor(&rangeSensor1, 1);
    navIf.setRangeSensor(&rangeSensor2, 2);
    navIf.setRangeSensor(&rangeSensor3, 3);
    navIf.setRangeSensor(&rangeSensor4, 4);
#endif

    // Snapshot the timer, initialize the update and display timers
    currentMillis = millis();
    previousMillis = currentMillis;
    previousDisplayMillis = currentMillis;
    
    // Initialize the transport
#if SPI_INT_ENABLE  
    Status = navIf.begin(BENCH_TESTING);
#else
    // For production force good data
    // TODO: conditional compile test code rather than parameterize
    Status = navIf.begin(57600, 0);
#endif
  
    dbg_println("Setup complete");
    dbg_println("==============");
    delay(400);
 
}

#if SPI_INT_ENABLE
// SPI interrupt service routine
// Arduino is always SPI slave in this application
// Arduino will be addressed as a registered device
// with the following sequence:
// Address - continue
// Arduino will respond with
// [buffer contents] value
ISR(SPI_STC_vect)
{
    navIf.SPIisr();
}
#endif 

// Executive loop
void loop(void)
{
    // Locals
    int              imuStatus;
    int              navCommand;
    int              i;
    
    // loop() gets called in an outer loop
    // so all the locals are volatile!
    
    // Initialize flags
    Display = false;
    upDate = false;
    hbUpdate = false;

    // Process any commands every call
    navCommand = navIf.getNavCommand();
    imuStatus = navIf.processNavCommand(navCommand);
        
    // Update the interval timer
    currentMillis = millis();

    // Set the flags depending on the elapsed time    
    if((currentMillis - previousDisplayMillis) >= DISPLAY_TIME)
    {
        Display = true;
        hbUpdate = true;
        previousDisplayMillis = currentMillis;
    }
    
    if((currentMillis - previousMillis) >= UPDATE_TIME)
    {
        upDate = true;
        previousMillis = currentMillis;
    }
    
    
    // Update the heartbeat on the HB interval
//    if(hbUpdate)
    if(0)
    {
        ArduinoHeartBeat++;
        dbg_print("HB = "); dbg_println(ArduinoHeartBeat);
    }
    
    if(upDate)
    {
        dbg_print("updateCount = "); dbg_println(updateCount);
    }
    
    // Update the nav data on the update interval
    if(upDate)
    {
        // Increment debug counter
        updateCount++;
       
        // Refresh the currect nav data buffer
        imuStatus = navIf.update(Display);
        
        // Switch nav data buffers
        navIf.switchBuffers();
    }
    
    if(Display)
    {
        dbg_print("Status = "); dbg_println(imuStatus);
    }
}

// End if LibertySensorIF.ino

