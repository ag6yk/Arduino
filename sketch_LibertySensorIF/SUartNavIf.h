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
 * @file SUartNavIf.h
 * @brief This file contains the definitions and declarations for the soft UART
 * interface for nav data transmission to the host processor
 *
 * @author Robert Cavanaugh, Engineering Mentor
 *
 * @version Revision History
 *  Created on: February 17, 2015
 *
 */

#ifndef SUARTNAVIF_H_
#define SUARTNAVIF_H_

///////////////////////////////////////////////////////////////////////////////
// DEFINITIONS
///////////////////////////////////////////////////////////////////////////////

// Define the Nav data sync word
// Use alternating 7-bit Barker code to ensure autocorrelation by the host
#define NAV_SYNC_NOINVERT		0x27
#define NAV_SYNC_INVERT			0xD8
#define NAV_ENDOF_FRAME			0xBC

// Nav system commands
//

#define NAV_SET_ORIGIN          0x53            // 'S': set the current position
												// to (0,0,0), heading to 0
#define NAV_RUN_POST            0x54			// 'T': perform self-test

// POST responses
#define NAV_POST_PASS           0x50            // 'P': Success
// Failure codes
#define NAV_POST_FAIL_GYRO      0x47            // G: gyroscope failed
#define NAV_POST_FAIL_ACCEL     0x41            // A: accelerometer failed
#define NAV_POST_FAIL_COMPASS   0x43            // C: compass failed
#define NAV_POST_FAIL_BARO      0x42            // B: barometer failed
#define NAV_ILLEGAL_COMMAND		0x51 			// Q: Illegal command sent from host



///////////////////////////////////////////////////////////////////////////////
// DATA structures
///////////////////////////////////////////////////////////////////////////////

typedef unsigned char byte;

// Define the Nav Data packet format to the host processor
// Soft UART
struct NAV_DATA
{
	byte	NavSync;			// Alternating Barker code sync
    byte    Position_X_MSB;		// Most significant byte of X position data
    byte    Position_X_LSB;		// Least significant byte of X position data
    byte    Position_Y_MSB;		// Most significant byte of Y position data
    byte    Position_Y_LSB;		// Least significant byte of Y position data
    byte	Heading_MSB;		// Most significant byte of heading
    byte    Heading_LSB;		// Least significant byte of heading
    byte	Pitch_MSB;          // Most significant byte of pitch
    byte    Pitch_LSB;          // Least significant byte of pitch
    byte    Range_0_MSB;     	// Most significant byte of Range 0 sensor reading
    byte    Range_0_LSB;     	// Least significant byte of Range 0 sensor reading
    byte    Range_1_MSB;     	// Most significant byte of Range 0 sensor reading
    byte    Range_1_LSB;     	// Least significant byte of Range 0 sensor reading
    byte    Range_2_MSB;     	// Most significant byte of Range 0 sensor reading
    byte    Range_2_LSB;     	// Least significant byte of Range 0 sensor reading
    byte    Range_3_MSB;     	// Most significant byte of Range 0 sensor reading
    byte    Range_3_LSB;     	// Least significant byte of Range 0 sensor reading
    byte    Range_4_MSB;     	// Most significant byte of Range 0 sensor reading
    byte    Range_4_LSB;     	// Least significant byte of Range 0 sensor reading
    byte	NavEoF;				// End of frame
};


#endif /* SUARTNAVIF_H_ */
