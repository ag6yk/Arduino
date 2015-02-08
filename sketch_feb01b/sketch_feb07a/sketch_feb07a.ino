// Sample SPI slave that echos to Serial on EOL
#include "pins_arduino.h"
#include "SPI.h"

char buf [150];
volatile byte pos;
volatile boolean process_it;

void setup (void)
{
  Serial.begin (9600);   // debugging

  // setup pins
  pinMode(MISO, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(SS, INPUT);
  pinMode(SCK, INPUT);
  
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  
  // get ready for an interrupt 
  pos = 0;   // buffer empty
  process_it = false;
  
  SPDR = 0xBC;

  // now turn on interrupts
  SPI.attachInterrupt();
  
}  // end of setup

volatile unsigned char foo = 0xBC;
volatile unsigned char bar;
// SPI interrupt routine
ISR (SPI_STC_vect)
{
  bar = SPDR;  // grab byte from SPI Data Register
  
  SPDR = foo;
  foo++;
  process_it = true;
  // add to buffer if room
}  // end of interrupt routine SPI_STC_vect

// main loop - wait for flag set in interrupt routine
unsigned long oldmillis = millis();
void loop (void)
{
  if (process_it)
    {
      // terminate string
      Serial.print("SPI Sent "); Serial.println(foo, DEC); 
      Serial.print("SPI Detected! "); Serial.println (bar, DEC);
      process_it = false;
      delay(1000);
      
    }  // end of flag set
   else
   {
     Serial.println("Looping...");
     delay(1000);
   }
}  // end of loop

