/*
 * SPINavIf.h
 *
 *  Created on: Feb 10, 2015
 *      Author: robertc
 */

#ifndef SPINAVIF_H_
#define SPINAVIF_H_

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
//+1 +1 +1 −1 −1 +1 −1
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


#endif /* SPINAVIF_H_ */
