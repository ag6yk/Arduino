#include <SoftwareSerial.h>

SoftwareSerial mySerial(12, 13); // RX, TX

byte testArray[24];

void setup()  
{
  int i;
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  for(i=0; i < 20; i++)
  {
     testArray[i] = 0x80+i;
  } 
  
  testArray[0] = 0x27;
  testArray[19] = 0xBC;
  
  Serial.println("Test Array:");
  for(i=0; i < 20; i++)
  {
    Serial.print(testArray[i]);
  }
  
  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(57600);
}

int interval;
int foo = 0;
void loop() // run over and over
{
  int i;
  mySerial.listen();
  interval++;
  if(interval > 100)
  {
      Serial.println("Beginning bogus nav packet...");
      interval = 9;
  }
  if(mySerial.available())
  {
    Serial.print("Received "); Serial.println(mySerial.read(), HEX);
  }
  for(i=0; i < 20; i++)
  {
    mySerial.write(testArray[i]);
  }
  delay(5);
}

