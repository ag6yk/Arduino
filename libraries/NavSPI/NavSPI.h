///////////////////////////////////////////////////////////////////////////////
//
//  ROBOKONG    2493
//  FRC 2015 - RECYCLING RUSH
//
//  CUSTOM ELECTRONICS
//
//  NAV DATA SENSOR ARRAY OVER SPI TRANSPORT
//
///////////////////////////////////////////////////////////////////////////////

/*!
    @file NavSPI.h
    @brief This file contains the class definition for the SPI transport
        version of the Nav Data sensor interface

    @author Robert Cavanaugh, Engineering Mentor

*/

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////
#ifndef NavSPI_h
#define NavSPI_h
#include "Arduino.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINES
///////////////////////////////////////////////////////////////////////////////

// SPI pseudo register map
//
// Registers
#define SPI_NAV_CONTINUE        0x00            // command sent as part of read
#define SPI_SET_ORIGIN          0x10            // set the current position to (0,0,0)
#define SPI_NAV_DATA_STS        0x20            // Status request
#define SPI_NAV_DATA_DAT        0x30            // read nav data (autoincrement)
#define SPI_PERFORM_POST        0x40            // run self-test (future, reserved)

// Nav data status responses
#define STS_NAV_DATA_READY      (0x01 << 0)     // bit set indicates ready
#define STS_NAV_DATA_NREADY     0x00            // not ready macro

// SPI Error messages
#define SPI_ILLEGAL_REGISTER    0xF0            // illegal address requested

// POST responses
#define STS_POST_PASS           0x01            // Success
#define STS_POST_FAIL_GYRO      0xF7            // Failure codes
#define STS_POST_FAIL_ACCEL     0xF8
#define STS_POST_FAIL_COMPASS   0xF9
#define STS_POST_FAIL_BARO      0xFA

//////////////////
// SPI Interface
//////////////////

/*!
The SPI interface will use a register-based architecture. The RobotRIO master
will write to pseudo-register addresses to command the Arduino slave to
see if a new nav data packet is ready, to send a packet of nav data,
to set the origin, and in future to run calibration and self-test

The Nav data will be updated every 20 milliseconds, with the Arduino
collecting/processing the data in the first 10 millisconds and the other
10 milliseconds transferring the data. Depending on processor throughput this
update rate may be increased (e.g. 10ms, 5ms processing 5ms transferring)

The nav data will be double-buffered, with the main processing loop reading
the sensors, processing the data, and filling one buffer while the other
buffer is available to the SPI interface. The buffers will be accessed by
dedicated pointers, one for producing and one for consuming.
The producer will have control of the buffer switching.

*/


///////////////////////////////////////////////////////////////////////////////
// DATA structures
///////////////////////////////////////////////////////////////////////////////

typedef unsigned char byte;

// Define the Nav Data packet status to the RobotRIO
// SPI payload
struct NAV_DATA
{
    byte           Position_X_MSB;  // Most significant byte of X position data
    byte           Position_X_LSB;  // Least significant byte of X position data
    byte           Velocity_X_MSB;  // Most significant byte of X velocity data
    byte           Velocity_X_LSB;  // Least significant byte of X velocity data
    byte           Accel_X_MSB;     // Most significant byte of X acceleration data
    byte           Accel_X_LSB;     // Least significant byte of X acceleration data
    byte           Position_Y_MSB;  // Most significant byte of Y position data
    byte           Position_Y_LSB;  // Least significant byte of Y position data
    byte           Velocity_Y_MSB;  // Most significant byte of Y velocity data
    byte           Velocity_Y_LSB;  // Least significant byte of Y velocity data
    byte           Accel_Y_MSB;     // Most significant byte of Y acceleration data
    byte           Accel_Y_LSB;     // Least significant byte of Y acceleration data
    byte           Heading_MSB;     // Most significant byte of heading
    byte           Heading_LSB;     // Least significant byte of heading
    byte           Range_0_MSB;     // Most significant byte of Range 0 sensor reading
    byte           Range_0_LSB;     // Least significant byte of Range 0 sensor reading
    byte           Range_1_MSB;     // Most significant byte of Range 0 sensor reading
    byte           Range_1_LSB;     // Least significant byte of Range 0 sensor reading
    byte           Range_2_MSB;     // Most significant byte of Range 0 sensor reading
    byte           Range_2_LSB;     // Least significant byte of Range 0 sensor reading
    byte           Range_3_MSB;     // Most significant byte of Range 0 sensor reading
    byte           Range_3_LSB;     // Least significant byte of Range 0 sensor reading
    byte           Range_4_MSB;     // Most significant byte of Range 0 sensor reading
    byte           Range_4_LSB;     // Least significant byte of Range 0 sensor reading
};

class NavSPI : public NavTransport
{
    private:
        struct  NAV_DATA    navBuffer0;     // Buffer 1
        struct  NAV_DATA    navBuffer1;     // Buffer 2

        // Define test buffers
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

    protected:
        // Future
    
    public:
        struct  NAV_DATA*   producer;       // buffer pointers
        struct  NAV_DATA*   comsumer;
        byte*               pSPIData;       // pointer to buffer as char array

        // Producer state variables

        // Consumer state variables
        // Will be used with the SPI interrupt
        volatile    int     consumerCount;  // Data counter
        volatile    boolean consumerBusy;   // true = consumer active
        volatile    boolean consumerEnable; // true = consumer can update

        // Inter-process variables
        volatile    boolean spiSetOrigin;   // true = read current position
                                            // and set to origin
                                            //
        // Diagnostics
        volatile    unsigned long   SPIInterruptCount;
        volatile    unsigned char   SPICommand

        // Methods
        NavSPI(int, int);                   // Constructor
        ~NavSPI();                          // Destructor
        int begin(int);                     // class specific initialization
        int update(void);                   // update the data
        void debugDisplay(void);            // process serial monitor 
        void SPIisr(void)                   // SPI interrupt service routine
        void switchBuffers(void);           // Switch buffers
        void listen(void);                  // wrapper functions for a consistent API
        void write(byte);
        byte read(void);
};
#endif
// End of NavSPI.h



