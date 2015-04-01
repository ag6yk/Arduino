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
#include <SoftwareSerial.h>
#include "../NavTransport/NavTransport.h"

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
#define NAV_ZERO_X              0x58            // 'X': zero x velocity
#define NAV_ZERO_Y              0x59            // 'Y': zero y velocity
#define NAV_ZERO_MOTION			0x5A			// 'Z': zero motion

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
// All data is encoded as 16-bit signed integers but transmitted
// as two unsigned bytes
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
    byte	Mag_X_MSB;			// Least significant byte of mag vector X
    byte	Mag_X_LSB;			// Most significant byte of mag vaector X
    byte	Mag_Y_MSB;			// Least significant byte of mag vector Y
    byte	Mag_Y_LSB;			// Most significatn byte of mag vector Y
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

class NavUart : public SoftwareSerial
{
    private:
        // Define a single test buffer with canned data
        const struct NAV_DATA   testBuffer =
        {
            0x27,
            101, 102, 103, 104, 105, 106, 107, 108,
            109, 110, 111, 112, 113, 114, 115, 116,
            117, 118, 119, 120, 121, 122,
            0xBC
        };

        // Pointers to sensor types. If NULL, the
		// sensor type is not supported for the specific application
		Accel*			_accelerometer;
		Gyro*			_gyroscope;
		Compass*		_compass;
		Baro*			_barometer;
		Ultrasonic*		_rangeSensor[5];
		void*			_sensor0;
		void*			_sensor1;
    
    public:

        // Define the RAM nav buffer for actual data
        struct NAV_DATA navBuffer;

        // Initialize a structure pointer
        struct NAV_DATA*	nDP;

        // Initialize a byte pointer into the data
        byte*   navPtr;

        // Capture the most current command from the host
        byte    navCommand;

        // Keep track of the alternating Barker code sync byte
        byte    navBarkerCode;

        // Current count of data received from the host
        int		rxCount;

        // Methods
        NavUart(int, int, bool);            // Constructor
       ~NavUart();                          // Destructor
        int 	begin(unsigned long, int);  // class specific initialization
        int 	update(bool);               // update the nav data
        void 	switchBuffers(void);        // wrapper function for consistent API
		int 	getNavCommand(void);        // get any new command from host
		int 	processNavCommand(int);		// process the nav command

		// Accessors
        void	setAccelerometer(Accel*);	// Assign the specific object
        void	setGyroscope(Gyro*);
        void	setCompass(Compass*);
        void	setBarometer(Baro*);
        void	setSensor0(void*);
        void	setSensor1(void*);
        void	setRangeSensor(Ultrasonic*, int);


};
#endif
// End of NavUart.h

