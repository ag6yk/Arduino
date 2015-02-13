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
#include "SPINavIf.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Uncomment to test Arduino without the RobotRIO
#define BENCH_TEST  1

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


///////////////////////////////////////////////////////////////////////////////
// GLOBAL DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

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


/////////////////////////////////
// NAV data buffers and pointers
/////////////////////////////////

struct NAV_DATA navBuffer0;
struct NAV_DATA navBuffer1;
struct NAV_DATA* producer = &navBuffer0;
struct NAV_DATA* consumer = &navBuffer1;
unsigned char*  pSPIData;

// Producer state variables

// Consumer state variables
volatile int        consumerCount;          // Data counter
volatile int        consumerBusy;           // state variable
volatile boolean    consumerEnable;         // true = consumer can update

// Inter-process variables
volatile boolean    spiSetOrigin;           // true = read current position
                                            // and set as 0,0

// Device status
int             GyroStatus;                 // 0 = device OK
int             AccelStatus;
int             CompassStatus;
int             BaroStatus;

// Sample timing variables
unsigned long   previousMillis;
unsigned long   currentMillis;
unsigned long   eTime;                      // elapsed time in ms
unsigned long   interval = 10;              // 10 ms update rate
unsigned long   debug_interval = 2000;      // 2 second display rate
unsigned long   previousDisplayMillis;
unsigned long   deTime;                     // for debug display



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
    
    // Initialize the producer
    producer = &navBuffer0;
    
    // Initialize the consumer
    // Put on hold until the first nav data is ready 
    consumer = &navBuffer1;
    consumerCount = 0;
    consumerEnable = false;
    consumerBusy = false;
    pSPIData = (unsigned char*)&consumer;
    
    // Initialize the inter-process variables
    spiSetOrigin = false;
    
    // Preset the SPI buffer 
    SPDR = STS_NAV_DATA_NREADY;

#if BENCH_TEST
    Serial.println("SPI configured");
    delay(500);
#endif
  
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


    // Snapshot the timer, initialize the update and display timers
    currentMillis = millis();
    previousMillis = currentMillis;
    previousDisplayMillis = currentMillis;
  
    // Enable the SPI interrupt
//  SPI.attachInterrupt();

 
}

// SPI interrupt service routine
ISR(SPI_STC_vect)
{
    // Locals
    byte spiData;
    
    // The nature of SPI guarantees we have some data
    spiData = SPDR;
    
    // Process the interrupt based on the the input value and state
    switch(spiData)
    {
        // In the process of or begin sending nav data
        case SPI_NAV_CONTINUE:
        case SPI_NAV_DATA_DAT:        
        {
            // Skip if locked
            if(consumerEnable == false)
            {
                SPDR = STS_NAV_DATA_NREADY;
            }
            else
            {
                // Send the next data point
                SPDR = *pSPIData++;
                
                // See if complete
                consumerCount++;
                if(consumerCount > sizeof(NAV_DATA))
                {
                    // Notify the main loop ready to switch
                    consumerBusy = false;
                    // Disable any re-sends
                    consumerEnable = false;
                }
            }
            break;
        }
        
        // Requesting to reset the nav data origin
        case SPI_SET_ORIGIN:
        {
            // Set the flag - always allowed
            spiSetOrigin = true;
            break;
        }
        
        // Report whether new data is available
        case SPI_NAV_DATA_STS:
        {
            // Check for data available
            if((consumerEnable == true)&& (consumerBusy == false))
            {
                // Data available, set up state machine
                SPDR = STS_NAV_DATA_READY;
                pSPIData = (unsigned char*)consumer;
                consumerCount = 0;
                consumerBusy = true;
            }
            else
            {
                // Data not ready yet
                SPDR = STS_NAV_DATA_NREADY;
            }
            break;
        }
        
        // Illegal command - send bad status
        default:
        {
            SPDR = SPI_ILLEGAL_REGISTER;
        }
    }// End switch    
}
 

// Executive loop
void loop(void)
{
    // Locals
    struct NAV_DATA*    pTemp;
    int imuStatus;

    // Always check if the nav data should be reset
    if(spiSetOrigin == true)
    {
        imuAccel.setOrigin();
        // imuGyro.setOrigin();
        spiSetOrigin = false;
    }
    
    // See if it is time to sample, every 10 ms
    currentMillis = millis();
    eTime = currentMillis - previousMillis;
    deTime = currentMillis - previousDisplayMillis;
    if(eTime >= interval)
    {
        // Restart interval timer
        previousMillis = currentMillis;
        
        // Process the IMU sensors
  
        // Process any new accelerometer data
        imuStatus = imuAccel.ProcessAccelData();
        // If no errors update the buffer
#if BENCH_TEST
        // Check if display update elapsed
        if(deTime >= debug_interval)
        {
            // Display debug information
            Serial.print("Acceleration X = "); Serial.println(imuAccel.getAccelerationX(), DEC);
            Serial.print("Velocity X     = "); Serial.println(imuAccel.getVelocityX(), DEC);
            Serial.print("Position X     = "); Serial.println(imuAccel.getPositionX(), DEC);
            Serial.print("Acceleration Y = "); Serial.println(imuAccel.getAccelerationY(), DEC);
            Serial.print("Velocity Y     = "); Serial.println(imuAccel.getVelocityY(), DEC);
            Serial.print("Position Y     = "); Serial.println(imuAccel.getPositionY(), DEC);
            delay(500);
        }
#else
        if(imuStatus == 0)
        {
            // Split the 16-bit values into 2 8-bit values
            producer->Position_X_MSB = highByte(imuAccel.getPositionX());
            producer->Position_X_LSB = lowByte(imuAccel.getPositionX());
            producer->Velocity_X_MSB = highByte(imuAccel.getVelocityX());
            producer->Velocity_X_LSB = lowByte(imuAccel.getVelocityX());
            producer->Accel_X_MSB = highByte(imuAccel.getAccelerationX());
            producer->Accel_X_LSB = lowByte(imuAccel.getAccelerationX());
            producer->Position_Y_MSB = highByte(imuAccel.getPositionY());
            producer->Position_Y_LSB = lowByte(imuAccel.getPositionY());
            producer->Velocity_Y_MSB = highByte(imuAccel.getVelocityY());
            producer->Velocity_Y_LSB = lowByte(imuAccel.getVelocityY());
            producer->Accel_Y_MSB = highByte(imuAccel.getAccelerationY());
            producer->Accel_Y_LSB = lowByte(imuAccel.getAccelerationY());
        }
        else
        {
            // Indicate data invalid
            producer->Position_X_MSB = 0xFF;
            producer->Position_X_LSB = 0xFF;
            producer->Velocity_X_MSB = 0xFF;
            producer->Velocity_X_LSB = 0xFF;
            producer->Accel_X_MSB = 0xFF;
            producer->Accel_X_LSB = 0xFF;
            producer->Position_Y_MSB = 0xFF;
            producer->Position_Y_LSB = 0xFF;
            producer->Velocity_Y_MSB = 0xFF;
            producer->Velocity_Y_LSB = 0xFF;
            producer->Accel_Y_MSB = 0xFF;
            producer->Accel_Y_LSB = 0xFF;
        }
#endif

#if 0
        
        // Process any new Gyroscope data
        imuStatus = imuGyro.ProcessGyroData();
        if(imuStatus == 0)
        {
            // Update the nav buffer
            producer->Heading_MSB = highByte(imuGyro.getHeading());
            producer->Heading_LSB = lowByte(imuGyro.getHeading());
            producer->Pitch_MSB = highByte(imuGyro.getPitch());
            producer->Pitch_LSB = lowByte(imuGyro.getPitch());
        }
        else
        {
            // indicate data invalid
            producer->Heading_MSB = 0xFF;
            producer->Heading_LSB = 0xFF;
            producer->Pitch_MSB = 0xFF;
            producer->Pitch_LSB = 0xFF;
        }
  
        // Process any new Compass data
        imuStatus = imuCompass.ProcessCompassData();
        if(imuStatus == 0)
        {
            // Update the nav buffer
            producer->VectorX_MSB = highByte(imuCompass.getVectorX());
            producer->VectorX_LSB = lowByte(imuCompass.getVectorX());
            producer->VectorY_MSB = highByte(imuCompass.getVectorY());
            producer->VectorY_LSB = lowByte(imuCompass.getVectorY());
        }
        else
        {
            // Update the nav buffer
            producer->VectorX_MSB = 0xFF;
            producer->VectorX_LSB = 0xFF;
            producer->VectorY_MSB = 0xFF;
            producer->VectorY_LSB = 0xFF;
        }
  
        // Barometer data will be used internally for future applications
        
        // Process any new pressure data
        imuStatus = imuBarometer.ProcessPressureData();
  
        // Process any new temperature data
        imuStatus = imuBarometer.ProcessTemperatureData();
#endif

#if BENCH_TEST
        // Check to see if display interval has elapsed
        if(deTime >= debug_interval)
        {
            Serial.print("Range 0 = "); Serial.println(rangeSensor0.ReadRange(), DEC);
            Serial.print("Range 1 = "); Serial.println(rangeSensor0.ReadRange(), DEC);
            Serial.print("Range 2 = "); Serial.println(rangeSensor0.ReadRange(), DEC);
            Serial.print("Range 3 = "); Serial.println(rangeSensor0.ReadRange(), DEC);
            Serial.print("Range 4 = "); Serial.println(rangeSensor0.ReadRange(), DEC);
            delay(500);
            
            // Reset display timer
            previousDisplayMillis = currentMillis;
        }
#else
        // Process Range sensors
        producer->Range_0 = rangeSensor0.ReadRange();
        producer->Range_1 = rangeSensor1.ReadRange();
        producer->Range_2 = rangeSensor2.ReadRange();
        producer->Range_3 = rangeSensor3.ReadRange();
        producer->Range_4 = rangeSensor4.ReadRange();
#endif
  
        // Switch nav data buffers
        while(consumerBusy)
        {
            // at 500 KHz 16 usec per byte, so wait 20 usecs each time
            delayMicroseconds(20);
        }
        // Critical section
        SPI.detachInterrupt();                  // disable SPI interrupt
        consumerEnable = false;                 // lock ISR
        pTemp = consumer;                       // swap pointers
        consumer = producer;
        pSPIData = (unsigned char*)consumer;
        producer = pTemp;
        consumerEnable = true;                  // unlock ISR
        SPI.attachInterrupt();                  // enable SPI interrupt
    }
  
    // And do it again, and again, ...
}

// End if LibertySensorIF.ino

