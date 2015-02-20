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
NavSPI(int param0, int param1)
{
    // NOP
}

// Class specific initialization
int begin(int param0)
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
    producer = navBuffer0;
    consumer = navBuffer1;
    if(param0)
    {
        producer = testBuffer0;
        consumer = testBuffer1;
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
void SPIisr(void)
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

        // Requesting to reset the nav data origin
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

#if BENCH_TESTING
    Serial.print("Rcvd:  "); Serial.println(spiReceivedData, DEC);
    Serial.print("Sent:  "); Serial.println(spiSentData, DEC);
#endif
}

// Update the data from the sensor array



        // Inter-process variables
        volatile    boolean spiSetOrigin;   // true = read current position
                                            // and set to origin
                                            //
        int update(void);                   // update the data
        void debugDisplay(void);            // process serial monitor 
        void switchBuffers(void);           // Switch buffers
        void listen(void);                  // wrapper functions for a consistent API
        void write(byte);
        byte read(void);
};
#endif
// End of NavSPI.h



