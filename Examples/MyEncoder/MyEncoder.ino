// Version 1.1: OO version
/*
 MyEncoder.ino - A library for reading Lady Ada's or 
 Sparkfun's rotary encoder.
 Should work for any rotary encoder with 2 pins (4 states).

Copyright 2011,2012,2013,2014 Michael Anthony Schwager

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

	Questions?  Send mail to mschwage@gmail.com

*/

#include <ByteBuffer.h>
#include <ooPinChangeInt.h> // necessary otherwise we get undefined reference errors.
#define DEBUG
#ifdef DEBUG
ByteBuffer printBuffer(200);
#endif
#include <AdaEncoder.h>

#define ENCA_a 2
#define ENCA_b 3
#define ENCB_a A0
#define ENCB_b A1

AdaEncoder encoderA = AdaEncoder('a', ENCA_a, ENCA_b);
AdaEncoder encoderB = AdaEncoder('b', ENCB_a, ENCB_b);

int8_t clicks=0;
char id=0;

void setup()
{
  Serial.begin(115200); Serial.println("---------------------------------------");
}

void loop() 
{
  char outChar;
  while ((outChar=(char)printBuffer.get()) != 0) Serial.print(outChar);
  AdaEncoder *thisEncoder=NULL;
  thisEncoder=AdaEncoder::genie();
  if (thisEncoder != NULL) {
    Serial.print(thisEncoder->getID()); Serial.print(':');
    clicks=thisEncoder->query();
    if (clicks > 0) {
      Serial.println(" CW");
    }
    if (clicks < 0) {
       Serial.println(" CCW");
    }
  }
}

