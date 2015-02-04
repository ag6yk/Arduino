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
unsigned char  accelDataIndex;

// Fake accelerometer data
// Model a burst with a turn
const struct ACCEL_DATA accelSimData[32] =
{
    { 0, 0, 0, 0, 0, 0 },
    { 1, 1, 0, 0, 0, 0 },
    {10, 10, 0, 0, 0, 0},
    {20, 20, 0, 0, 0, 0},
    {30, 30, 0, 0, 0, 0},
    {40, 40, 0, 0, 0, 0},
    {50, 50, 0, 0, 0, 0},
    {0xFF, 45, 1, 1, 0, 0},
    {0xFF, 85, 2, 2, 0, 0},
    {0xFF, 0xFE, 10, 10, 0, 0},
    {0, 0, 20, 20, 0, 0},
    {0, 0, 30, 30, 0, 0},
    {0, 0, 10, 10, 0, 0}, 
    {0, 0, 10, 10, 0, 0}, 
    {0, 0, 10, 10, 0, 0}, 
    {0, 0, 10, 10, 0, 0}, 
    {0, 0, 10, 10, 0, 0}, 
    {0, 0, 10, 10, 0, 0}, 
    {0, 0, 10, 10, 0, 0}, 
    {0, 0, 10, 10, 0, 0}, 
    {0, 0, 5,  5, 0, 0},
    {0, 0, 2,  2, 0, 0},
    {0, 0, 1, 1, 0, 0},
    {0, 0, 0, 4, 0, 0},
    {0, 0, 0, 2, 0, 0},
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0}
};

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
        accelDataIndex = 0;
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
      
      default:
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
      Serial.print("accel: I2C Write to ");
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
}

void accelRequestEvent(void)
{
  // Process the possible requests
  switch(accelAddress)
  {
    // Device ID - one byte
    case A_DEVID:
    {
        Wire.write(byte(A_I_BETTER_BE));
        accelReadState = 0;
        break;
    }
    
    // Acceleration data - six bytes
    case A_DATA_X0:
    {
        Wire.write(byte(accelSimData[accelDataIndex].xLSB));
        Wire.write(byte(accelSimData[accelDataIndex].xMSB));
        Wire.write(byte(accelSimData[accelDataIndex].yLSB));
        Wire.write(byte(accelSimData[accelDataIndex].yMSB));
        Wire.write(byte(accelSimData[accelDataIndex].zLSB));
        Wire.write(byte(accelSimData[accelDataIndex].zMSB));
        accelDataIndex++;
        if(accelDataIndex > 31)
        {
            accelDataIndex = 0;
        }
        accelReadState = 0;
        break;
    }
     
    // FIFO status - one byte        
    case A_FIFO_STS:
    {
        // No event, 9 words waiting
        Wire.write(byte(0x09));
        accelReadState = 0;
        break;
    }
    
    default:
        // Let the master timeout
        accelReadState = 0;
  }
}
    
  
// Barometer simulator
void baroReceiveEvent(int HowMany)
{
  // barometer implements single byte writes by default
  unsigned char baroData;
  
  while(Wire.available())
  {
    // Register address
    if(baroReadState == 0)
    {
      baroAddress = Wire.read();
      baroReadState++;
    }
    else if(baroReadState == 1)
    {
      // Read data and echo it
      baroData = Wire.read();
      Serial.print("baro: I2C Write to ");
      Serial.print(baroAddress, HEX);
      Serial.print(" Data = ");
      Serial.println(baroData, HEX);
    }
    else
    {
      Serial.println("baroReceiveEvent state error");
      baroReadState = 0;
    }
  }
}

void baroRequestEvent(void)
{
    
    switch(baroAddress)
    {
        // EEPROM - 16 bits
        case B_EEPROM:
        case (B_EEPROM+2):
        case (B_EEPROM+4):
        case (B_EEPROM+6):
        case (B_EEPROM+8):
        case (B_EEPROM+0x0A):
        case (B_EEPROM+0x0C):
        case (B_EEPROM+0x0E):
        case (B_EEPROM+0x10):
        case (B_EEPROM+0x12):
        case (B_EEPROM+0x14):
        case (B_EEPROM+0x16):
        {
            Wire.write(byte(baroAddress));
            Wire.write(byte((baroAddress - B_EEPROM)));
            baroReadState = 0;
            break;
        }
        
        // Output data
        case B_DATA_MSB:
        {
            Wire.write(byte(0xDE));
            Wire.write(byte(0xAD));
            baroReadState = 0;
            break;
        }
        
        // Wait for the master to timeout
        default:
            baroReadState = 0;
    }
}

// Compass simulator
void compassReceiveEvent(int HowMany)
{
  // Compass implements multi-byte writes by default
  unsigned char compassData;
  boolean       compassMulti = false;
  unsigned char compassMultiCount = 0;
  
  while(Wire.available())
  {
    // Register address
    if(compassReadState == 0)
    {
      compassAddress = Wire.read();
      compassReadState++;
      
      // Check for single multi-byte instance
      if(compassAddress == C_CONFIG_A)
      {
        compassMultiCount = 3;
        compassMulti = true;
      }
      else
      {
        compassMultiCount = 0;
        compassMulti = false;
      }
      
    }
    else if(compassReadState == 1)
    {
      // Read data and echo it
      compassData = Wire.read();
      Serial.print("compass: I2C Write to ");
      Serial.print(compassAddress, HEX);
      Serial.print(" Data = ");
      Serial.println(compassData, HEX);
      
      // Handle multibyte write
      if(compassMulti)
      {
        compassMultiCount--;
        compassAddress++;
        if(compassMultiCount == 0)
        {
            compassReadState = 0;
            compassMulti = false;
        }
      }
      else
      {
        compassReadState = 0;
      }
    }
    else
    {
      Serial.println("compassReceiveEvent state error");
      compassMulti = false;
      compassMultiCount = 0;
      compassReadState = 0;
    }
  }
}

void compassRequestEvent(void)
{
    // switch on address
    switch(compassAddress)
    {
        case C_X_MSB:
        {
            // Output Data - six bytes
            Wire.write(byte(0x12));
            Wire.write(byte(0x34));
            Wire.write(byte(0x56));
            Wire.write(byte(0x78));
            Wire.write(byte(0x9A));
            Wire.write(byte(0xBC));
            compassReadState = 0;
            break;
        }

        // Status
        case C_STATUS:
        {
            // Data available, registers not locked
            Wire.write(byte(C_STS_RDY));
            compassReadState = 0;
            break;
        }
            
        // ID
        case C_ID_A:
        {
            // Fixed value
            Wire.write(byte(C_ID_A_VAL));
            compassReadState = 0;
            break;
        }

        // Let master timeout
        default:
            compassReadState = 0;
    }
}

// Gyro simulator
void gyroReceiveEvent(int HowMany)
{
  // Gyroscope can support multi-byte writes
  unsigned char gyroData;
  boolean       gyroMulti = false;
  unsigned char gyroMultiCount = 0;
  
  while(Wire.available())
  {
    // Register address
    if(gyroReadState == 0)
    {
      gyroAddress = Wire.read();
      gyroReadState++;
      if(gyroAddress & 0x80)
      {
        // Only sequenced set
        gyroMultiCount = 5;
        gyroMulti = true;
      }
      else
      {
        gyroMultiCount = 0;
        gyroMulti = false;
      }
      
    }
    else if(gyroReadState == 1)
    {
      // Read data and echo it
      gyroData = Wire.read();
      Serial.print("gyro: I2C Write to ");
      Serial.print(gyroAddress, HEX);
      Serial.print(" Data = ");
      Serial.println(gyroData, HEX);
      
      // Handle multibyte write
      if(gyroMulti)
      {
        gyroMultiCount--;
        gyroAddress++;
        if(gyroMultiCount == 0)
        {
            gyroReadState = 0;
            gyroMulti = false;
        }
      }
      else
      {
        gyroReadState = 0;
      }
    }
    else
    {
      Serial.println("gyroReceiveEvent state error");
      gyroMulti = false;
      gyroMultiCount = 0;
      gyroReadState = 0;
    }
  }
}

void gyroRequestEvent(void)
{
    // Switch one the address
    switch(gyroAddress)
    {
        // ID register
        case G_WHO_AM_I:
        {
            Wire.write(byte(G_I_BETTER_BE));
            gyroReadState = 0;
            break;
        }
        
        // Temperature output
        case G_OUT_TEMP:
        {
            Wire.write(byte(0xBC));
            gyroReadState = 0;
            break;
        }
        
        // Status register
        case G_STATUS_REG:
        {
            // Data available, no errors
            Wire.write(byte(0x0F));
            gyroReadState = 0;
            break;
        }
        
        // Angular velocities data - six bytes
        case G_OUT_X_L:
        {
            Wire.write(byte(0x10));
            Wire.write(byte(0x20));
            Wire.write(byte(0x30));
            Wire.write(byte(0x40));
            Wire.write(byte(0x50));
            Wire.write(byte(0x60));
            gyroReadState = 0;
            break;
        }

        // FIFO status register
        case G_FIFO_S_REG:
        {
            // seven words ready, no errors
            Wire.write(byte(0x07));
            gyroReadState = 0;
            break;
        }

        // Default - let the master timeout
        default:
            gyroReadState = 0;
    }
}

// End if LibertySensorIF.ino


