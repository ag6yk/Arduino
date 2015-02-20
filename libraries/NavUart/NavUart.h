///////////////////////////////////////////////////////////////////////////////
//
//  ROBOKONG    2493
//  FRC 2015 - RECYCLING RUSH
//
//  CUSTOM ELECTRONICS
//
//  NAV DATA SENSOR ARRAY OVER UART TRANSPORT
//
///////////////////////////////////////////////////////////////////////////////

/*!
    @file NavUart.h
    @brief This file contains the class definition for the UART transport
        version of the Nav Data sensor interface

    @author Robert Cavanaugh, Engineering Mentor

*/

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////
#ifndef NavUart_h
#define NavUart_h
#include "Arduino.h"
#include <SoftSerial.h>

///////////////////////////////////////////////////////////////////////////////
// DEFINES
///////////////////////////////////////////////////////////////////////////////


// Define the Nav data sync word
// Use alternating 7-bit Barker code to ensure autocorrelation by the host
#define NAV_SYNC_NOINVERT       0x27
#define NAV_SYNC_INVERT         0xD8
#define NAV_ENDOF_FRAME         0xBC

// Nav system commands
#define NAV_SET_ORIGIN          0x53            // 'S': set the current position
                                                // to (0,0,0), heading to 0
#define NAV_RUN_POST            0x54            // 'T': perform self-test

// POST responses
#define NAV_POST_PASS           0x50            // 'P': Success
// Failure codes
#define NAV_POST_FAIL_GYRO      0x47            // 'G': gyroscope failed
#define NAV_POST_FAIL_ACCEL     0x41            // 'A': accelerometer failed
#define NAV_POST_FAIL_COMPASS   0x43            // 'C': compass failed
#define NAV_POST_FAIL_BARO      0x42            // 'B': barometer failed
#define NAV_ILLEGAL_COMMAND     0x51            // 'Q': Illegal command sent from host

///////////////////////////////////////////////////////////////////////////////
// DATA structures
///////////////////////////////////////////////////////////////////////////////

typedef unsigned char byte;

// Define the Nav Data packet format to the host processor
// Soft UART
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

class NavUart : public NavTransport
{
    private:
        // Define a single test buffer with canned data
        const struct NAV_DATA   testBuffer =
        {
            0x27, 101, 102, 103, 104, 105, 106, 107, 108, 109,
            110, 111, 112, 113, 114, 115, 116, 117, 118,
            0xBC
        };

    
    public:
        // Define the RAM nav buffer for actual data
        struct NAV_DATA navBuffer;

        // Initialize a byte pointer into the data
        byte*   navPtr;

        // Capture the most current command from the host
        byte    navCommand;

        // Keep track of the alternating Barker code sync byte
        byte    navBarkerCode;

        // Methods
        NavUart(int, int);                  // Constructor
        ~NavUart();                         // Destructor
        int begin(int);                     // class specific initialization
        int update(void);                   // update the data
        void debugDisplay(void);            // process serial monitor 
        void switchBuffers(void);           // Switch buffers
        void listen(void);                  // wrapper functions for a consistent API
        void write(byte);
        byte read(void);
};
#endif
// End of NavUart.h
