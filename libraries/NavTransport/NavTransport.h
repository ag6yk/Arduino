///////////////////////////////////////////////////////////////////////////////
//
//  ROBOKONG    2493
//  FRC 2015 - RECYCLING RUSH
//
//  CUSTOM ELECTRONICS
//
//  NAV DATA SENSOR ARRAY TRANSPORT GENERIC CLASS
//
///////////////////////////////////////////////////////////////////////////////

/*!
 * @file NavTransport.h
 * @brief This file contains the definition of the NavTransport container
 * class for transferring navigational data from an Intertial Measurement
 * Unit (IMU) to the external host.
 *
 * @author  Robert Cavanaugh, Engineering Mentor
 *
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////
#ifndef NaVTransport_h
#define NaVTransport_h
#include "Arduino.h"

// These includes are specific to the application
#include "Accel.h"
#include "Baro.h"
#include "Compass.h"
#include "Gyro.h"
#include "Ultrasonic.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINES
///////////////////////////////////////////////////////////////////////////////

// Container class definition
class NavTransport
{
    private:
    protected:
        // Future
    public:
        NavTransport();                 	// Constructor
        ~NavTransport();                	// Destructor
        virtual int begin();            	// class specific initialization
        virtual int update();           	// Update the nav data
        virtual int send();             	// transmit data buffer to the host
};

#endif
// End of NavTransport.h

