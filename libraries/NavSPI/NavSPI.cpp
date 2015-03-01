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
    @brief This file contains the class source for the SPI transport
        version of the Nav Data sensor interface

    @author Robert Cavanaugh, Engineering Mentor

*/

///////////////////////////////////////////////////////////////////////////////
// INCLUDES
///////////////////////////////////////////////////////////////////////////////
#include "Arduino.h"
#include <SPI.h>
#include "NavSPI.h"

///////////////////////////////////////////////////////////////////////////////
// DEFINES
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// DATA structures
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// METHODS
///////////////////////////////////////////////////////////////////////////////


// Constructor - placeholder
NavSPI::NavSPI(int param0, int param1)
{
    // NOP
}

// Destructor - placeholder
NavSPI::~NavSPI()
{
	// NOP
}

// Class specific initialization
int NavSPI::begin(int test)
{
    // Locals
    int i;
    byte    *pptr;
    byte    *qptr;

    // Clear the nav buffers
    pptr = (byte*)&navBuffer0;
    qptr = (byte*)&navBuffer1;

    for(i=0; i < sizeof(NAV_DATA); i++)
    {
        *pptr++ = 0;
        *qptr++ = 0;
    }

    // Initialize the AVR SPI hardware as a slave
    pinMode(MISO, OUTPUT);
    pinMode(MOSI, INPUT);
    pinMode(SS, INPUT);
    pinMode(SCK, INPUT);

    // Turn on SPI in slave mode
    // MSB transmitted first
    // SPI Mode 0
    SPCR |= _BV(SPE);

    // Initialize the producer and consumer
    // Point the producer to the first buffer
    // Stall the consumer until the producer
    // reads the first full nav data buffer
    producer = &navBuffer0;
    consumer = &navBuffer1;
    if(test)
    {
        producer = (NAV_DATA*)&testBuffer0;
        consumer = (NAV_DATA*)&testBuffer1;
    }

    consumerCount = 0;
    consumerEnable = false;
    consumerBusy = false;
    pSPIData = (byte*)&consumer;

    // Initialize the inter-process variables
    spiSetOrigin = false;

    // Preset the SPI buffer
    SPDR = STS_NAV_DATA_NREADY;
    SPICommand = 0;

    // Enable the SPI interrupt
    SPI.attachInterrupt();
}

// SPI interrupt service routine
void NavSPI::SPIisr(void)
{
    // Locals
    byte spiReceivedData;
    byte spiSentData;

    // Increment the interrupt count
    SPIInterruptCount++;

    // The design of the SPI interface guarantees
    // that we will have received a byte of data
    spiReceivedData = SPDR;

    // Process the interrupt based on the input value of SPDR
    // and on the value of the last command byte:
    // Message Sequence Chart:
    // Master  Tx/Rx        Slave Tx/Rx
    //     Cmd/Junk            Junk/Cmd
    //     00/1st value        1st value/00
    //     00 is a continuation command
    //     "1st value" depends on what Cmd is
    switch(spiReceivedData)
    {
        // Continuation command - 0x00
        case SPI_NAV_CONTINUE:
        {
            // Select this action based on the previously
            // received command byte
            switch(SPICommand)
            {
                // Continuation command - 0x00
                // In the process of transmitting nav data buffer or
                // beginning the process of transmitting nav data buffer
                case SPI_NAV_CONTINUE:
                case SPI_NAV_DATA_DAT:
                {
                    // Skip if the buffer is locked by the 
                    // main processing function or by this function
                    if(consumerEnable == false)
                    {
                        // Indicate to host that nav data is blocked
                        spiSentData = STS_NAV_DATA_NREADY;
                    }
                    else
                    {
                        // Send the next byte of data
                        spiSentData = *pSPIData++;
                        consumerCount++;

                        // See if buffer is complete
                        if(consumerCount > sizeof(NAV_DATA))
                        {
                            // Notify the main loop that SPI
                            // buffer should be swapped
                            consumerBusy = false;
                            // disable resends
                            consumerEnable = false;
                        }
                        // Not complete, set up the next command
                        else
                        {
                            SPICommand = SPI_NAV_CONTINUE;
                        }
                    }
                    
                    break;
                }

                // Nav data status was requested and preloaded
                case SPI_NAV_DATA_STS:
                {
                    // Check if the buffer is locked
                    if(consumerEnable == true)
                    {
                        // Data available, set up for transfer
                        spiSentData = STS_NAV_DATA_READY;
                        pSPIData = (byte*)consumer;
                        consumerCount = 0;
                        consumerBusy = true;
                    }
                    else
                    {
                        // Data not available
                        spiSentData = STS_NAV_DATA_NREADY;
                    }
                    SPICommand = 0;
                    break;
                }

                default:
                {
                    // Send a known response
                    spiSentData = 60;
                }
            }
            break;
        }

        // Begin sending nav data
        case SPI_NAV_DATA_DAT:
        {
            // Capture the command
            SPICommand = spiReceivedData;
            // Send a unique response code
            spiSentData = 50;
            break;
        }

        // See if new nav data is available
        case SPI_NAV_DATA_STS:
        {
            // Capture the command
            SPICommand = spiReceivedData;
            spiSentData = STS_NAV_DATA_NREADY;
            if(consumerEnable == true)
            {
                spiSentData = STS_NAV_DATA_READY;
            }
            break;
        }

        // Requesting to reset the nav data origin
        case SPI_SET_ORIGIN:
        {
        	// Capture the command
        	SPICommand = spiReceivedData;
        	spiSetOrigin = true;
        	break;
        }

        // Illegal command - send bad status
        default:
        {
            spiSentData = SPI_ILLEGAL_REGISTER;
            // clear the command
            SPICommand = 0;
        }
    }// end switch

    // Preload the new byte to be sent on the next clock
    SPDR = spiSentData;

}

// Read and update the nav data
int NavSPI::update(bool test)
{
	// Locals
	int i;
	int	Status = 0;
	int imuStatus;
	byte* nDPb = (byte*)producer;

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
			producer->Position_X_MSB = highByte(_accelerometer->getPositionX());
			producer->Position_X_LSB = lowByte(_accelerometer->getPositionX());
			producer->Velocity_X_MSB = highByte(_accelerometer->getVelocityX());
			producer->Velocity_X_LSB = lowByte(_accelerometer->getVelocityX());
			producer->Accel_X_MSB = highByte(_accelerometer->getAccelerationX());
			producer->Accel_X_LSB = lowByte(_accelerometer->getAccelerationX());
			producer->Position_Y_MSB = highByte(_accelerometer->getPositionY());
			producer->Position_Y_LSB = lowByte(_accelerometer->getPositionY());
			producer->Position_Y_LSB = lowByte(_accelerometer->getPositionX());
			producer->Velocity_Y_MSB = highByte(_accelerometer->getVelocityY());
			producer->Velocity_Y_LSB = lowByte(_accelerometer->getVelocityY());
			producer->Accel_Y_MSB = highByte(_accelerometer->getAccelerationY());
			producer->Accel_Y_LSB = lowByte(_accelerometer->getAccelerationY());
		}
		Status += imuStatus;
	}

	if(_gyroscope)
	{
		imuStatus = _gyroscope->ProcessGyroData(test);

		if(imuStatus == 0)
		{
			// Update the gyroscope data
			producer->Heading_MSB = highByte(_gyroscope->getHeading());
			producer->Heading_LSB = lowByte(_gyroscope->getHeading());
			producer->Pitch_MSB = highByte(_gyroscope->getPitch());
			producer->Pitch_LSB = lowByte(_gyroscope->getPitch());
		}
		Status += imuStatus;
	}

	if(_rangeSensor[0])
	{
		imuStatus = _rangeSensor[0]->ReadRange(test);
		if(imuStatus == 0)
		{
			// Update the range sensor data
			producer->Range_0_MSB = highByte(_rangeSensor[0]->getRange());
			producer->Range_0_LSB = lowByte(_rangeSensor[0]->getRange());
		}
		Status += imuStatus;
	}

	if(_rangeSensor[1])
	{
		imuStatus = _rangeSensor[1]->ReadRange(test);
		if(imuStatus == 0)
		{
			// Update the range sensor data
			producer->Range_1_MSB = highByte(_rangeSensor[1]->getRange());
			producer->Range_1_LSB = lowByte(_rangeSensor[1]->getRange());
		}
		Status += imuStatus;
	}

	if(_rangeSensor[2])
	{
		imuStatus = _rangeSensor[2]->ReadRange(test);
		if(imuStatus == 0)
		{
			// Update the range sensor data
			producer->Range_2_MSB = highByte(_rangeSensor[2]->getRange());
			producer->Range_2_LSB = lowByte(_rangeSensor[2]->getRange());
		}
		Status += imuStatus;
	}

	if(_rangeSensor[3])
	{
		imuStatus = _rangeSensor[3]->ReadRange(test);
		if(imuStatus == 0)
		{
			// Update the range sensor data
			producer->Range_3_MSB = highByte(_rangeSensor[3]->getRange());
			producer->Range_3_LSB = lowByte(_rangeSensor[3]->getRange());
		}
		Status += imuStatus;
	}

	if(_rangeSensor[4])
	{
		imuStatus = _rangeSensor[4]->ReadRange(test);
		if(imuStatus == 0)
		{
			// Update the range sensor data
			producer->Range_4_MSB = highByte(_rangeSensor[4]->getRange());
			producer->Range_4_LSB = lowByte(_rangeSensor[4]->getRange());
		}
		Status += imuStatus;
	}

	// data is written to the host as part of the interrupt service routine

	return(Status);
}


// Switch the nav buffers
void NavSPI::switchBuffers(void)
{
	// Locals
	struct NAV_DATA* pTemp;

	// Wait for the proper synchronization
	while(consumerBusy)
	{
		// At 500 KHz, 16 usec per byte, so wait 20 usecs
		delayMicroseconds(20);
	}
	// Critical section
	SPI.detachInterrupt();				// Disable SPI interrupt
	consumerEnable = false;				// lock out ISR
	pTemp = consumer;					// swap pointers
	consumer = producer;
	pSPIData = (byte*)consumer;
	producer = pTemp;
	consumerEnable = true;				// unlock ISR
	SPI.attachInterrupt();				// enable SPI interrupt
	// End of critical section
}

// Get the current nav command
int NavSPI::getNavCommand(void)
{
	return SPICommand;
}

// Process the nav command
int NavSPI::processNavCommand(int Command)
{
	// Locals


	// Currently support two commands from the main processing loop
	switch(Command)
	{
		case SPI_SET_ORIGIN:
		{
			if(spiSetOrigin == true)
			{
				if(_accelerometer)
				{
					_accelerometer->setOrigin();
				}

				if(_gyroscope)
				{
					_gyroscope->setOrigin();
				}
				spiSetOrigin = false;
			}
			break;
		}

		case SPI_PERFORM_POST:
		{
			// Future
			break;
		}

		default:
		{
			// All other commands are handled by the ISR
		}
	}

	return 0;
}

// Accessors

void NavSPI::setAccelerometer(Accel* ac)
{
	_accelerometer = ac;
}

void NavSPI::setGyroscope(Gyro* gs)
{
	_gyroscope = gs;
}

void NavSPI::setCompass(Compass* cp)
{
	_compass = cp;
}

void NavSPI::setBarometer(Baro* br)
{
	_barometer = br;
}

void NavSPI::setSensor0(void* s0)
{
	_sensor0 = s0;
}

void NavSPI::setSensor1(void* s1)
{
	_sensor1 = s1;
}

void NavSPI::setRangeSensor(Ultrasonic* rs, int Index)
{
	_rangeSensor[Index] = rs;
}


// End of NavSPI.cpp



