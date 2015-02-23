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
    @brief This file contains the class source for the UART transport
        version of the Nav Data sensor interface

    @author Robert Cavanaugh, Engineering Mentor

*/

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////
#include "Arduino.h"
#include "NavUart.h"


///////////////////////////////////////////////////////////////////////////////
// DEFINES
///////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////
// DATA structures
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// METHODS
///////////////////////////////////////////////////////////////////////////////

// Constructor
NavUart::NavUart(int rx, int tx, bool lsense): SoftwareSerial(rx, tx, lsense)
{
	// Initialize the private values
	_accelerometer = NULL;
	_gyroscope = NULL;
	_compass = NULL;
	_barometer = NULL;
	_rangeSensor[0] = NULL;
	_rangeSensor[1] = NULL;
	_rangeSensor[2] = NULL;
	_rangeSensor[3] = NULL;
	_rangeSensor[4] = NULL;
	_sensor0 = 0;
	_sensor1 = 0;

	SoftwareSerial(rx, tx, lsense);

}

// Destructor
NavUart::~NavUart()
{
	// Pass through to the parent destructor
}

// Class specific initializations
int NavUart::begin(unsigned long baudRate, int test )
{
	// Pass through to the software serial
	SoftwareSerial::begin(baudRate);

	// Initialize the Barker code
	navBarkerCode = NAV_SYNC_NOINVERT;

	// Initialize the buffer pointers
	navPtr = (byte*)&navBuffer;
	nDP = &navBuffer;
	if(test)
	{
		navPtr = (byte*)&testBuffer;
		nDP = (NAV_DATA*)&testBuffer;
	}

}

// Return the current nav command or -1 if no command available
int NavUart::getNavCommand()
{
	// Locals
	int imuStatus;

	// Set up the UART for listening
	listen();

	// See if there is a new nav command
	rxCount = available();
	navCommand = -1;
	if(rxCount > 0)
	{
		navCommand = read();
	}

	return(navCommand);
}

// Process the nav commands
int NavUart::processNavCommand(int Command)
{
	// Locals

	// Currently support two commands
	switch(Command)
	{
		case NAV_SET_ORIGIN:
		{
			if(_accelerometer)
			{
				_accelerometer->setOrigin();
			}

			if(_gyroscope)
			{
				_gyroscope->setOrigin();
			}
			break;
		}

		case NAV_RUN_POST:
		{
			// Future
			break;
		}

		default:
		{
			write(NAV_ILLEGAL_COMMAND);
			break;
		}
	}

	return 0;
}

// Read the sensor data and update the nav buffer
int NavUart::update(bool test)
{
	// Locals
	int i;
	byte*	nDPb;
	int	Status = 0;
	int imuStatus;

	// Initialize the local pointer
	nDPb = (byte*)nDP;

	// Fill the buffer with invalid data
	for(i = 0; i < sizeof(NAV_DATA); i++)
	{
		*nDPb++ = 0xFF;
	}

	// Only process the sensors that have been enabled
	// by the main application
	if(_accelerometer)
	{
		imuStatus = _accelerometer->ProcessAccelData(test);

		if(imuStatus == 0)
		{
			// Update the accelerometer data
			nDP->Position_X_MSB = highByte(_accelerometer->getPositionX());
			nDP->Position_X_LSB = lowByte(_accelerometer->getPositionX());
			nDP->Position_Y_MSB = highByte(_accelerometer->getPositionY());
			nDP->Position_Y_LSB = lowByte(_accelerometer->getPositionY());
		}

		Status += imuStatus;
	}

	if(_gyroscope)
	{
		imuStatus = _gyroscope->ProcessGyroData(test);

		if(imuStatus == 0)
		{
			// Update the gyroscope data
			nDP->Heading_MSB = highByte(_gyroscope->getHeading());
			nDP->Heading_LSB = lowByte(_gyroscope->getHeading());
			nDP->Pitch_MSB = highByte(_gyroscope->getPitch());
			nDP->Pitch_LSB = lowByte(_gyroscope->getPitch());
		}
		Status += imuStatus;
	}

	if(_rangeSensor[0])
	{
		imuStatus = _rangeSensor[0]->ReadRange(test);
		if(imuStatus == 0)
		{
			// Update the range sensor data
			nDP->Range_0_MSB = highByte(_rangeSensor[0]->getRange());
			nDP->Range_0_LSB = lowByte(_rangeSensor[0]->getRange());
		}
		Status += imuStatus;
	}

	if(_rangeSensor[1])
	{
		imuStatus = _rangeSensor[1]->ReadRange(test);
		if(imuStatus == 0)
		{
			// Update the range sensor data
			nDP->Range_1_MSB = highByte(_rangeSensor[1]->getRange());
			nDP->Range_1_LSB = lowByte(_rangeSensor[1]->getRange());
		}
		Status += imuStatus;
	}

	if(_rangeSensor[2])
	{
		imuStatus = _rangeSensor[2]->ReadRange(test);
		if(imuStatus == 0)
		{
			// Update the range sensor data
			nDP->Range_2_MSB = highByte(_rangeSensor[2]->getRange());
			nDP->Range_2_LSB = lowByte(_rangeSensor[2]->getRange());
		}
		Status += imuStatus;
	}

	if(_rangeSensor[3])
	{
		imuStatus = _rangeSensor[3]->ReadRange(test);
		if(imuStatus == 0)
		{
			// Update the range sensor data
			nDP->Range_3_MSB = highByte(_rangeSensor[3]->getRange());
			nDP->Range_3_LSB = lowByte(_rangeSensor[3]->getRange());
		}
		Status += imuStatus;
	}

	if(_rangeSensor[4])
	{
		imuStatus = _rangeSensor[4]->ReadRange(test);
		if(imuStatus == 0)
		{
			// Update the range sensor data
			nDP->Range_4_MSB = highByte(_rangeSensor[4]->getRange());
			nDP->Range_4_LSB = lowByte(_rangeSensor[4]->getRange());
		}
		Status += imuStatus;
	}


	// Re-Initialize the local pointer
	nDPb = (byte*)nDP;
	nDP->NavSync = navBarkerCode;
	nDP->NavEoF = NAV_ENDOF_FRAME;

	Status = 0;
	for(i = 0; i < sizeof(NAV_DATA); i++)
	{
		if(test)
		{
			Serial.print(*nDPb, HEX);
		}
		Status++;
		this->write(*nDPb++);
	}
	if(test)
	{
		Serial.println();
	}
	Status++;

	// Compute the alternating code for the next frame
	if(navBarkerCode == NAV_SYNC_NOINVERT)
	{
		navBarkerCode = NAV_SYNC_INVERT;
	}
	else
	{
		navBarkerCode = NAV_SYNC_NOINVERT;
	}

	return(Status);

}

// Accessors

void NavUart::setAccelerometer(Accel* ac)
{
	_accelerometer = ac;
}

void NavUart::setGyroscope(Gyro* gs)
{
	_gyroscope = gs;
}

void NavUart::setCompass(Compass* cp)
{
	_compass = cp;
}

void NavUart::setBarometer(Baro* br)
{
	_barometer = br;
}

void NavUart::setSensor0(void* s0)
{
	_sensor0 = s0;
}

void NavUart::setSensor1(void* s1)
{
	_sensor1 = s1;
}

void NavUart::setRangeSensor(Ultrasonic* rs, int Index)
{
	_rangeSensor[Index] = rs;
}


// Wrapper function for consistent APIs
void NavUart::switchBuffers()
{
	// stub function
}

// End of NavUart.cpp

