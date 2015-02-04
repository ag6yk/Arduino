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
    @file   sketch_I2CSimulator.ino

    @brief  This file contains the driver program to simulate the I2C
    communication between the Liberty SensorIF Arduino and a IMU
    sensor package

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
// ECHOn will be used to determine which device is being simulated
#define PING0       2
#define ECHO0       3
#define PING1       4
#define ECHO1       5
#define PING2       6
#define ECHO2       7
#define PING3       8
#define ECHO3       9

///////////////////////////////////////////////////////////////////////////////
// DATA structures
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// GLOBAL DECLARATIONS
///////////////////////////////////////////////////////////////////////////////

// C++ Pin assignments
const int    Ping0 = PING0;          // Range sensor 0 (front)
const int    Echo0 = ECHO0;
const int    Ping1 = PING1;          // Range sensor 1 (right)
const int    Echo1 = ECHO1;
const int    Ping2 = PING2;          // Range sensor 2 (back)
const int    Echo2 = ECHO2;
const int    Ping3 = PING3;          // Range sensor 3 (left)
const int    Echo3 = ECHO3;

volatile int  i2cConfig = 0;

// Accelerometer simulator
unsigned char  accelAddress;
unsigned char  accelReadState;
unsigned char  accelWriteLength;

// Barometer simulator
unsigned char  baroAddress;
unsigned char  baroReadState;
unsigned char  baroWriteLength;

// Compass simulator
unsigned char  compassAddress;
unsigned char  compassReadState;
unsigned char  compassWriteLength;

// Gyroscope simulator
unsigned char  gyroAddress;
unsigned char  gyroReadState;
unsigned char  gyroWriteLength;

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
  
    // Set up the debugger
    Serial.begin(9600);
    Serial.println("*** I2C Simulator ***");
    Serial.println("*** RoboKong 2493 ***");
    delay(500);
    
    // Initialize the configuration inputs
    pinMode(Echo0, INPUT);
    pinMode(Echo1, INPUT);
    pinMode(Echo2, INPUT);
    
    // Read the pin settings
    i2cConfig = digitalRead(Echo0);
    i2cConfig |= (digitalRead(Echo1) << 1);
    
    switch(i2cConfig)
    {
      case 0:
      {
        // Accelerometer simulator
        accelAddress = 0;
        accelReadState = 0;
        accelWriteLength = 0;
        Wire.begin(ACCEL_IC_ADDRHI);
        Wire.onReceive(accelReceiveEvent);
        Wire.onRequest(accelRequestEvent);
        break;
      }
      
      case 1:
      {
        // Barometer simulator
        baroAddress = 0;
        baroReadState = 0;
        baroWriteLength = 0;
        Wire.begin(B_IC_ADDR);
        Wire.onReceive(baroReceiveEvent);
        Wire.onRequest(baroRequestEvent);
        break;
      }
      
      case 2:
      {
        // Compass simulator
        compassAddress = 0;
        compassReadState = 0;
        compassWriteLength = 0;
        Wire.begin(C_IC_ADR);
        Wire.onReceive(compassReceiveEvent);
        Wire.onRequest(compassRequestEvent);
        break;
      }
      
      case 3:
      {
         // Gyroscope simulator
        gyroAddress = 0;
        gyroReadState = 0;
        gyroWriteLength = 0;
        Wire.begin(GYRO_IC_ADDRHI);
        Wire.onReceive(gyroReceiveEvent);
        Wire.onRequest(gyroRequestEvent);
        break;
      }
      
      default
      {
        Serial.println("!!!! ERROR: ILLEGAL ADDRESS !!!!");
      }
    }
}

// Executive loop
int LoopCount = 0;
void loop(void)
{
  // Locals
  Serial.print("Loop = ");
  Serial.print(LoopCount);
  Serial.println(" iterations");
  delay(500);
}

// Processing routines

// Accelerometer simulator
void accelReceiveEvent(int howMany)
{
  // Accelerometer is not set up for multi-byte writes
  unsigned char accelData;
  
  while(Wire.available())
  {
    // Register address
    if(accelReadState == 0)
    {
      accelAddress = Wire.read();
      accelReadState++;
    }
    else if(accelReadState == 1)
    {
      // Read data and echo it
      accelData = Wire.read();
      Serial.print("I2C Write to ");
      Serial.print(accelAddress, HEX);
      Serial.print(" Data = ");
      Serial.println(accelData, HEX);
      accelReadState = 0;
    }
    else
    {
      Serial.println("accelReceiveEvent state error");
      accelReadState = 0;
    }
}

void accelRequestEvent(void)
{
  // Process the possible requests
  switch(accelAddress)
  {
    case A_DEVID:
    case A_DATA_X0:
    case A_FIFO_STS:
    case A_FIFO_CNT:
    default:
  }
}
    
  
// Barometer simulator
void baroReceiveEvent(int HowMany)
{
}

void baroRequestEvent(void)
{
}

// Compass simulator
void compassReceiveEvent(int HowMany)
{
}

void compassRequestEvent(void)
{
}

// Gyro simulator
void gyroReceiveEvent(int HowMany)
{
}

void gyroRequestEvent(void)
{
}

// End if LibertySensorIF.ino


