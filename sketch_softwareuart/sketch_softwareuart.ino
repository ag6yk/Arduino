#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 12); // RX, TX

byte testArray[24];

void setup()  
{
  int i;
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  for(i=0; i < 24; i++)
  {
     testArray[i] = 0x80+i;
  } 
  
  Serial.println("Test Array:");
  for(i=0; i < 24; i++)
  {
    Serial.print(testArray[i]);
  }
  
  Serial.println("Goodnight moon!");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(115200);
}

int foo = 0;
void loop() // run over and over
{
  int i;
  mySerial.listen();
  if(mySerial.available())
  {
    Serial.print("Received "); Serial.println(mySerial.read(), HEX);
    for(i=0; i < 24; i++)
    {
      mySerial.write(testArray[i]);
    }
  }
  delay(10);
}

