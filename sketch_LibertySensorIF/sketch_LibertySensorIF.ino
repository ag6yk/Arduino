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

// Conditional compile flags for testing environments

// Enable to see serial monitor output
#define BENCH_TESTING   1

// Enable to test SPI interface with canned data
#define NAV_DATA_TEST   1

// Enable to test the Arduino sensor interface
// without connection to the nav shield
#define NAV_SENSOR_TEST 0

// Conditionally compile support for each sensor in the IMU
#define ACCEL_ENABLE    1
#define GYRO_ENABLE     1
#define COMPASS_ENABLE  0
#define BARO_ENABLE     0
#define RANGES_ENABLE   1

// Conditionally compile support for the SPI interrupt
#define SPI_INT_ENABLE  1

//////////////////////////
// Arduino Pin definitions
//////////////////////////

// Define the Pins used to interface to the IMU
// SDA and SCL already defined for A4 and A5 respectively
// No other signals are used from the IMU

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

// A2 and A3 are spare



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

// Define two test buffers with canned data
const struct NAV_DATA   testBuffer0 =
{
    100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
    110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
    120, 121, 122, 123
};

const struct NAV_DATA   testBuffer1 =
{
    200, 201, 202, 203, 204, 205, 206, 207, 208, 209,
    210, 211, 212, 213, 214, 215, 216, 217, 218, 219,
    220, 221, 222, 223
};

// Define two buffers in RAM for actual data
struct NAV_DATA navBuffer0;
struct NAV_DATA navBuffer1;

// Initialize pointers to the data
struct NAV_DATA* producer = &navBuffer0;
struct NAV_DATA* consumer = &navBuffer1;
unsigned char*  pSPIData;
volatile unsigned long SPIInterruptCount;
volatile unsigned char SPICommand;

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
unsigned long   debug_interval = 1000;      // 2 second display rate
unsigned long   previousDisplayMillis;
unsigned long   deTime;                     // for debug display


// Health monitoring
unsigned long   ArduinoHeartBeat;           // heart beat variable
unsigned long   previousHBMillis;           // heart beat update 1Hz
unsigned long   hbTime;
unsigned long   hbInterval = 1000;


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
    
#if BENCH_TESTING
    // Set up the debugger
    Serial.begin(9600);
    Serial.println("Setup Debug Info...");
    delay(500);
#endif
  
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
#if NAV_DATA_TEST
    producer = (NAV_DATA*)&testBuffer0;
#else
    producer = &navBuffer0;
#endif
    
    // Initialize the consumer
    // Put the consumer on hold until the first nav data is ready
#if NAV_DATA_TEST
    consumer = (NAV_DATA *)&testBuffer1;
#else 
    consumer = &navBuffer1;
#endif
    consumerCount = 0;
    consumerEnable = false;
    consumerBusy = false;
    pSPIData = (unsigned char*)&consumer;
    
    // Initialize the inter-process variables
    spiSetOrigin = false;
    
    // Preset the SPI buffer 
    SPDR = STS_NAV_DATA_NREADY;
    SPICommand = 0;

#if BENCH_TESTING
    Serial.println("SPI configured");
    delay(500);
#endif
  
    // Configure the IMU
 
#if GRYO_ENABLE
    // Configure the gyroscope
    GyroStatus = imuGyro.begin();
    Serial.print("GyroStatus = ");
    Serial.println(GyroStatus, DEC);
    delay(500);
#endif
  
#if ACCEL_ENABLE
    // Configure the accelerometer
    AccelStatus = imuAccel.begin();
    Serial.print("AccelStatus = ");
    Serial.println(AccelStatus, DEC);
#endif

#if COMPASS_ENABLE
    // Configure the compass
    CompassStatus = imuCompass.begin();
    Serial.print("CompassStatus = ");
    Serial.println(CompassStatus, DEC);
#endif

#if BARO_ENABLE
    // Configure the barometer
    BaroStatus = imuBarometer.begin();
    Serial.print("BaroStatus = ");
    Serial.println(BaroStatus, DEC);
#endif

    // Snapshot the timer, initialize the update and display timers
    currentMillis = millis();
    previousMillis = currentMillis;
    previousDisplayMillis = currentMillis;
  
    // Enable the SPI interrupt
#if SPI_INT_ENABLE
    SPI.attachInterrupt();
#endif

 
}

volatile byte foo;

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
    // Locals
    byte spiReceivedData;                   // Data received from master
    byte spiSentData;                       // Data loaded into the SPI buffer
                                            // to be sent on the next clock
    
    // Increment the count
    SPIInterruptCount++;
    
    // The nature of SPI guarantees we have some data
    spiReceivedData = SPDR;
    
    // Process the interrupt based on the the input value 
    // and the value of the last command byte
    switch(spiReceivedData)
    {
        // Continuation command
        case SPI_NAV_CONTINUE:
        {
            // Select current action on the previous command
            switch(SPICommand)
            {
                // Continuation - In the process of sending nav data
                case SPI_NAV_CONTINUE:
                {
                    // Skip if locked
                    if(consumerEnable == false)
                    {
                        // Indicate nav data blocked
                        spiSentData = STS_NAV_DATA_NREADY;
                    }
                    else
                    {
                        // Send the next data point
                        spiSentData = *pSPIData++;
                
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

                // The first byte of nav data                
                case SPI_NAV_DATA_DAT:
                {
                    // Skip if locked
                    if(consumerEnable == false)
                    {
                        spiSentData = STS_NAV_DATA_NREADY;
                    }
                    else
                    {
                        // Send the first data point
                        spiSentData = *pSPIData++;
                
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
                    SPICommand = 0;
                    break;
                }

                // Nav data status was requested and preloaded                
                case SPI_NAV_DATA_STS:
                {
                    // Check for lock
                    if(consumerEnable == true)
                    {
                        // Data available, set up state machine
                        spiSentData = STS_NAV_DATA_READY;
                        pSPIData = (unsigned char*)consumer;
                        consumerCount = 0;
                        consumerBusy = true;
                    }
                    else
                    {
                        // Data not ready yet
                        spiSentData = STS_NAV_DATA_NREADY;
                    }
                    SPICommand = 0;
                    break;
                }
            
                default:
                {
                    // Send a known response
                    spiSentData = 60;
                }
            }
            break;
        }           
        
        // Begin sending nav data
        case SPI_NAV_DATA_DAT:        
        {
            // Capture the state
            SPICommand = spiReceivedData;
            // Send a unique response code
            spiSentData = 50;
            break;
        }
         
        
        // Requesting to reset the nav data origin
        case SPI_SET_ORIGIN:
        {
            // Set the flag - always allowed
            spiSetOrigin = true;
            spiSentData = 40;
            break;
        }
        
        // Report whether new data is available
        case SPI_NAV_DATA_STS:
        {
            // Capture the state
            SPICommand = spiReceivedData;
            spiSentData = STS_NAV_DATA_NREADY;
            if(consumerEnable == true)
            {
                spiSentData = STS_NAV_DATA_READY;
            }
            break;
        }
        
        // Illegal command - send bad status
        default:
        {
            spiSentData = SPI_ILLEGAL_REGISTER;
            SPICommand = 0;
        }
    }// End switch 
    
    // Preload the new byte to be sent on the next clock
    SPDR = spiSentData;
    
#if BENCH_TESTING
    Serial.print("Rcvd:  "); Serial.println(spiReceivedData, DEC);
    Serial.print("Sent:  "); Serial.println(spiSentData, DEC);
#endif
       
}
#endif 

// Executive loop
void loop(void)
{
    // Locals
    struct NAV_DATA*    pTemp;                  // used to format data
    int imuStatus;
    boolean Display = false;

    // Always check if the nav data should be reset
    if(spiSetOrigin == true)
    {
#if BENCH_TESTING
        Serial.println("Origin Set");
#endif

#if ACCEL_ENABLE
        imuAccel.setOrigin();
#endif

#if GYRO_ENABLE
        imuGyro.setOrigin();
#endif
        spiSetOrigin = false;
    }

    // Update the interval timers    
    currentMillis = millis();
    eTime = currentMillis - previousMillis;
    deTime = currentMillis - previousDisplayMillis;
    hbTime = currentMillis - previousHBMillis;
    
    if(deTime >= debug_interval)
    {
        Display = true;
        previousDisplayMillis = currentMillis;
    }
    
    // Update the heart beat every second
    if(hbTime >= hbInterval)
    {
        // Restart the timer
        ArduinoHeartBeat++;
        previousHBMillis = currentMillis;
#if BENCH_TESTING
        Serial.print("HB = "); Serial.println(ArduinoHeartBeat);
        delay(10);
#endif
    }

#if BENCH_TESTING
    // See if there is a new SPI interrupt
    if(Display)
    {
        Serial.print("SPI count = "); Serial.println(SPIInterruptCount);
        previousDisplayMillis = currentMillis;
    }
#endif
    
    // Update nav data every 10 ms
    if(eTime >= interval)
    {
        // Restart interval timer
        previousMillis = currentMillis;
        
        // Process the IMU sensors

#if ACCEL_ENABLE  
        // Process any new accelerometer data
        imuStatus = imuAccel.ProcessAccelData();
        
#if NAV_DATA_TEST
        // Skip this step
#else
        // If no errors update the buffer
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
#if BENCH_TESTING
        // Check if display update elapsed
        if(Display)
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
#endif
#endif
#endif

#if GYRO_ENABLE
        // Process any new Gyroscope data
        imuStatus = imuGyro.ProcessGyroData();
        
#if NAV_DATA_TEST
        // skip this step
#else
        // If no errors update the buffer
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

#if BENCH_TEST
        // Check if display update elapsed
        if(Display)
        {
            // Display debug information
            Serial.print("Heading   = "); Serial.println(imuGyro.getHeading(), DEC);
            Serial.print("Pitch     = "); Serial.println(imuGyro.getPitch(), DEC);
            delay(500);
        }
        
#endif
#endif
#endif

        // Compass: not supported for this application
       
        // Barometer: not supported for this application
        
#if RANGES_ENABLE

        // Process the range sensor data    
        rangeSensor0.ReadRange();
        rangeSensor1.ReadRange();
        rangeSensor2.ReadRange();
        rangeSensor3.ReadRange();
        rangeSensor4.ReadRange();
    
#if NAV_DATA_TEST
        // skip this step
#else
        // Update the buffer
        producer->Range_0_MSB = highbyte(rangeSensor0.getRange());
        producer->Range_0_LSB = lowbyte(rangeSensor0.getRange());
        producer->Range_1_MSB = highbyte(rangeSensor1.getRange());
        producer->Range_1_LSB = lowbyte(rangeSensor1.getRange());
        producer->Range_2_MSB = highbyte(rangeSensor2.getRange());
        producer->Range_2_LSB = lowbyte(rangeSensor2.getRange());
        producer->Range_3_MSB = highbyte(rangeSensor3.getRange());
        producer->Range_3_LSB = lowbyte(rangeSensor3.getRange());
        producer->Range_4_MSB = highbyte(rangeSensor4.getRange());
        producer->Range_4_LSB = lowbyte(rangeSensor4.getRange());

#if BENCH_TESTING
        // Check to see if display interval has elapsed
        if(Display)
        {
            Serial.print("Range 0 = "); Serial.println(rangeSensor0.getRange(), DEC);
            Serial.print("Range 1 = "); Serial.println(rangeSensor0.getRange(), DEC);
            Serial.print("Range 2 = "); Serial.println(rangeSensor0.getRange(), DEC);
            Serial.print("Range 3 = "); Serial.println(rangeSensor0.getRange(), DEC);
            Serial.print("Range 4 = "); Serial.println(rangeSensor0.getRange(), DEC);
            delay(500);
        }
#endif
#endif
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

