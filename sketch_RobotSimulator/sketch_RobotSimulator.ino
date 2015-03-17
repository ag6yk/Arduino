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

byte              navBuffer[64];
byte              *nPb;
byte              *nPfb;

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
    for(i=0; i < 64; i++)
    {
        navBuffer[i] = 0;
    }
    // Initialize the nav processing
    nPb = (byte*)navBuffer;
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

    void processNavData(byte *p);

    // Check for new nav data
    rUart.listen();
    
    // Wait for a complete buffer
    while(rxCount < 21)
    {
        rxCount = rUart.available();
    }
    
    // For now assume it is aligned
    for(i=0; i < 21; i++)
    {
        navBuffer[i] = rUart.read();
    }
    
    nPb = navBuffer;
    processNavData(nPb);
}

void processNavData(byte *p)
{
	byte*	lP = p;
	signed short sTemp;
	unsigned short uTemp;

	// Process the data
	// Byte 0: nav Sync
        Serial.println("===");
	Serial.print("Sync = "); Serial.println(*lP++, DEC);
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	// Byte 1/2: X position
	uTemp = (*lP++ << 8);
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	uTemp += *lP++;
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	sTemp = (signed short)uTemp;
	Serial.print("   X = "); Serial.println(sTemp, DEC);
	// Byte 3/4: Y position
	uTemp = (*lP++ << 8);
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	uTemp += *lP++;
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	sTemp = (signed short)uTemp;
	Serial.print("   Y = "); Serial.println(sTemp, DEC);
	// Byte 5/6: Heading
	uTemp = (*lP++ << 8);
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	uTemp += *lP++;
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	sTemp = (signed short)uTemp;
	Serial.print("HDG  = "); Serial.println(sTemp, DEC);
	// Byte 7/8: Pitch
	uTemp = (*lP++ << 8);
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	uTemp += *lP++;
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	sTemp = (signed short)uTemp;
	Serial.print("PITCH= "); Serial.println(sTemp, DEC);
	// Byte 9/10: Range sensor 0
	uTemp = (*lP++ << 8);
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	uTemp += *lP++;
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	sTemp = (signed short)uTemp;
	Serial.print("RNG 0= "); Serial.println(sTemp, DEC);
	// Byte 11/12: Range sensor 1
	uTemp = (*lP++ << 8);
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	uTemp += *lP++;
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	sTemp = (signed short)uTemp;
	Serial.print("RNG 1= "); Serial.println(sTemp, DEC);
	// Byte 13/14: Range sensor 2
	uTemp = (*lP++ << 8);
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	uTemp += *lP++;
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	sTemp = (signed short)uTemp;
	Serial.print("RNG 2= "); Serial.println(sTemp, DEC);
	// Byte 15/16: Range sensor 3
	uTemp = (*lP++ << 8);
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	uTemp += *lP++;
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	sTemp = (signed short)uTemp;
	Serial.print("RNG 3= "); Serial.println(sTemp, DEC);
	// Byte 17/18: Range sensor 4
	uTemp = (*lP++ << 8);
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	uTemp += *lP++;
	if(lP > &navBuffer[19])
	{
		lP = navBuffer;
	}
	sTemp = (signed short)uTemp;
	Serial.print("RNG 4= "); Serial.println(sTemp, DEC);
	// Byte 19: EOF
	sTemp = (signed short)uTemp;
	Serial.print("EOF  = "); Serial.println(*lP++, DEC);
        Serial.println("+++");
}

