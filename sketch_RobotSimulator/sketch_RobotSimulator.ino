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
    @file   sketch_RobotSimulator.ino

    @brief  This file contains the driver program for the RobotRIO simulator.
    
    @author Robert Cavanaugh, Engineering Mentor

    @version    Revision History

*/

///////////////////////////////////////////////////////////////////////////////
// CONFIGURATION
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////
#include "Arduino.h"
#include <SoftwareSerial.h>

///////////////////////////////////////////////////////////////////////////////
// DATA STRUCTURES
///////////////////////////////////////////////////////////////////////////////

// Use alternating 7-bit Barker code to ensure autocorrelation by the host
#define NAV_SYNC_NOINVERT       0x27
#define NAV_SYNC_INVERT         0xD8
#define NAV_ENDOF_FRAME         0xBC

struct NAV_DATA
{
    byte    NavSync;            // Alternating Barker code sync
    byte    Position_X_MSB;     // Most significant byte of X position data
    byte    Position_X_LSB;     // Least significant byte of X position data
    byte    Position_Y_MSB;     // Most significant byte of Y position data
    byte    Position_Y_LSB;     // Least significant byte of Y position data
    byte    Heading_MSB;        // Most significant byte of heading
    byte    Heading_LSB;        // Least significant byte of heading
    byte    Pitch_MSB;          // Most significant byte of pitch
    byte    Pitch_LSB;          // Least significant byte of pitch
    byte	Mag_X_MSB;
    byte	Mag_X_LSB;
    byte	Mag_Y_MSB;
    byte	Mag_Y_LSB;
    byte    Range_0_MSB;        // Most significant byte of Range 0 sensor reading
    byte    Range_0_LSB;        // Least significant byte of Range 0 sensor reading
    byte    Range_1_MSB;        // Most significant byte of Range 0 sensor reading
    byte    Range_1_LSB;        // Least significant byte of Range 0 sensor reading
    byte    Range_2_MSB;     	// Most significant byte of Range 0 sensor reading
    byte    Range_2_LSB;     	// Least significant byte of Range 0 sensor reading
    byte    Range_3_MSB;     	// Most significant byte of Range 0 sensor reading
    byte    Range_3_LSB;     	// Least significant byte of Range 0 sensor reading
    byte    Range_4_MSB;     	// Most significant byte of Range 0 sensor reading
    byte    Range_4_LSB;     	// Least significant byte of Range 0 sensor reading
    byte    NavEoF;             // End of frame
};


///////////////////////////////////////////////////////////////////////////////
// GLOBALS
///////////////////////////////////////////////////////////////////////////////

// Instantiate the soft uart
SoftwareSerial    rUart(13, 12, false);

unsigned long     rxCount;

byte              navBuffer[48];

///////////////////////////////////////////////////////////////////////////////
// FUNCTIONS
///////////////////////////////////////////////////////////////////////////////


void setup()
{
    // Locals
    int i;
    
    // Initialize the debug serial port
    Serial.begin(115200);
    
    // Initialize the software serial port
    rUart.begin(57600);
    
    // Clear the nav buffer
    for(i=0; i < 48; i++)
    {
        navBuffer[i] = 0;
    }
    // Initialize the nav processing
    rxCount = 0;
    // Zero the sensors
    rUart.write(byte(0x58));
    delay(500);
    rUart.write(byte(0x59));
    delay(500);
    Serial.println();
    Serial.println("==Setup Complete==");
    delay(500);
}

void loop() 
{
    // locals
    int i;
    int temp;

    void processNavData();

    // Check for new nav data
    rUart.listen();
    
    // Wait for a complete buffer
    while(rxCount < 24)
    {
        rxCount = rUart.available();
    }
    
    // For now assume it is aligned
    for(i=0; i < 24; i++)
    {
        navBuffer[i] = rUart.read();
    }
    
    for(i=0; 
    Serial.println();
    delay(100);
    
    
//    processNavData();
}

void processNavData()
{
	signed short sTemp;
	unsigned short uTemp;

	// Process the data
	// Byte 0: nav Sync
        Serial.println("===");
	Serial.print("Sync = "); Serial.println(navBuffer[0], DEC);
	// Byte 1/2: X position
	uTemp = (navBuffer[1] << 8) + navBuffer[2];
	sTemp = (signed short)uTemp;
	Serial.print("   X = "); Serial.println(sTemp, DEC);
	// Byte 3/4: Y position
	uTemp = (navBuffer[3] << 8) + navBuffer[4];
	sTemp = (signed short)uTemp;
	Serial.print("   Y = "); Serial.println(sTemp, DEC);
	// Byte 5/6: Heading
        uTemp = (navBuffer[5] << 8) + navBuffer[6];
	sTemp = (signed short)uTemp;
	Serial.print("HDG  = "); Serial.println(sTemp, DEC);
	// Byte 7/8: Pitch
        uTemp = (navBuffer[7] << 8) + navBuffer[8];
	sTemp = (signed short)uTemp;
	Serial.print("PITCH= "); Serial.println(sTemp, DEC);
        // Byte 9/10 Mag Data X vector
	uTemp = (navBuffer[9] << 8) + navBuffer[10];
	sTemp = (signed short)uTemp;
        Serial.print("MAGX= "); Serial.println(sTemp, DEC);        
        // Byte 11/12 Mag Data Y vector
	uTemp = (navBuffer[11] << 8) + navBuffer[12];
	sTemp = (signed short)uTemp;
        Serial.print("MAGY= "); Serial.println(sTemp, DEC);        
	// Byte 13/14: Range sensor 0
	uTemp = (navBuffer[13] << 8) + navBuffer[14];
	sTemp = (signed short)uTemp;
	Serial.print("RNG 0= "); Serial.println(sTemp, DEC);
	// Byte 15/16: Range sensor 1
	uTemp = (navBuffer[15] << 8) + navBuffer[16];
	sTemp = (signed short)uTemp;
	Serial.print("RNG 1= "); Serial.println(sTemp, DEC);
	// Byte 17/18: Range sensor 2
	uTemp = (navBuffer[17] << 8) + navBuffer[18];
	sTemp = (signed short)uTemp;
	Serial.print("RNG 2= "); Serial.println(sTemp, DEC);
	// Byte 19/20: Range sensor 3
	uTemp = (navBuffer[19] << 8) + navBuffer[20];
	sTemp = (signed short)uTemp;
	Serial.print("RNG 3= "); Serial.println(sTemp, DEC);
	// Byte 21/22: Range sensor 4
	uTemp = (navBuffer[21] << 8) + navBuffer[22];
	sTemp = (signed short)uTemp;
	Serial.print("RNG 4= "); Serial.println(sTemp, DEC);
	// Byte 23: EOF
	sTemp = (signed short)uTemp;
	Serial.print("EOF  = "); Serial.println(navBuffer[23], DEC);
        Serial.println("+++");
}

