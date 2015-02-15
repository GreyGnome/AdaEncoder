// We use 4-character tabstops, so IN VIM:  <esc>:set ts=4 sw=4 sts=4<cr>
/*
 AdaEncoder.h - A library for reading Lady Ada's or 
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

/* Behavior goes like this:
 * 11 is the detent position for an encoder.  So the encoder will go through an order like this:
 *   11 <=> 01 <=> 00 <=> 10 <=> 11    OR    11 <=> 10 <=> 00 <=> 01 <=> 11
 * depending on direction.  The <=> represent a possible bounce situation.  Therefore, we keep track
 * of the * encoder behavior in a binary number called (appropriately enough) encoder.  Encoder has
 * 8 bits.
 * 
 * The process starts when we get an interrupt and read the encoder pins.
 * - Reading: 11 is either a bounce back to start, or the final reading:
 * 	- If encoder is currently xxxx0001, you have advanced CW.
 * 	- If encoder is currently xxxx0010, you have advanced CCW.
 *	- otherwise, you'd simply bounced.  No advance.
 *	- "encoder" is set to xxxx1111.
 * - Reading: 01 or 10:
 *	- If "encoder" is currently xxxx1111, set it to 1101 or 1110, appropriately.
 *	- If "encoder" is xxxx00xy, you'd either bounced or moved to the next position, which means
 *	we get closer to detent.  No effect.
 * - Reading: 00: we are in the middle.  AND encoder with xxxx0011
 */

#undef DEBUG
//#define SWINTR_DEBUG // To debug using software interrupts on your two pins, define this.
                    // Then in your sketch, set your pins as outputs.  Initialize them as you
		    // desire, attach an interrupt, then the interrupt code will be called.
		    // CAUTION: Make sure you do NOT have any switches connected to those outputs,
		    // or you may end up frying your ATmega328!

#ifndef AdaEncoder_h
#define AdaEncoder_h

#ifndef PinChangeInt_h
#define LIBCALL_OOPINCHANGEINT
#include "../ooPinChangeInt/ooPinChangeInt.h"
#endif
#include "../cbiface/cbiface.h"

#if defined(ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <pins_arduino.h>
  #include <wiring.h>
#endif

/* Class AdaEncoder.
 * Usage:
 * AdaEncoder encoderA = AdaEncoder('a', encodera_pinA, encodera_pinB);
 *
 * Constructor Arguments:
 * - id: a single char that identifies this encoder
 * - pinA: The pin on the Arduino that one side of the encoder plugs into.  Turning in this
 *   direction means we're turning clockwise.
 * - pinB: The Arduino pin connected to the encoder; this is the counterclockwise direction.
 *
 *   Pins A and B MUST be on the same PORT!
 *
 * On ATmega328-based Arduinos, the pins can be any of the digital pins 0-13, or any of the
 * analog pins A0-A5 (aka, 14-19). You can specify the analog pins as A0, A1, A2, ... etc.
 *
 * The pins must be paired in the same port.  In summary this means that the two pins should
 * together be grouped within a single port; ie, if you connect pinA to digital pin 9, you
 * must connect pinB to digital pin 8 or 10-13.  See this table for the ATmega168/328:
 * Arduino Pins     PORT
 * ------------     ----
 * Digital 0-7      D
 * Digital 8-13     B
 * Analog  0-5      C   (== digital pins 14-19)
 * 
 * ATMEGA2560 Pin Change Interrupts
    Arduino              Arduino              Arduino
      Pin*  PORT PCINT     Pin   PORT PCINT     Pin   PORT PCINT
      A8     PK0  16       10     PB4   4       SS     PB0   0
      A9     PK1  17       11     PB5   5       SCK    PB1   1
     A10     PK2  18       12     PB6   6       MOSI   PB2   2
     A11     PK3  19       13     PB7   7       MISO   PB3   3
     A12     PK4  20       14     PJ1  10
     A13     PK5  21       15     PJ0   9
     A14     PK6  22        0     PE0   8 - this one is a little odd.*
     A15     PK7  23
* Note: Arduino Pin 0 is PE0 (PCINT8), which is RX0. Also, it is the only other
pin on another port on PCI1. It is not supported by this library.
 */
class AdaEncoder : public CallBackInterface {
 public:
	AdaEncoder(char _id, uint8_t _pinA, uint8_t _pinB) {
    	/* 
     	 * Add a new encoder 
     	 * Params :
     	 * pinA			CW pin  (Arduino pin number)
     	 * pinB			CCW pin
	 	 *
     	 */
		addEncoder(_id, _pinA, _pinB);
		
	}
	int8_t query();
	int8_t getClicks();
	char getID();

	static AdaEncoder *getFirstEncoder();

	static AdaEncoder *genie();
	static AdaEncoder *genie(int8_t *clicks, char *id); // GEt Next Indicated Encoder - gets the next encoder with non-zero clicks

 private:

	void addEncoder(char _id, uint8_t _pinA, uint8_t _pinB);
	void attachInterrupt(uint8_t _pinA, uint8_t _pinB);
	void cbmethod();
	static void debugMessage();
	static void turnOffPWM(uint8_t timer);

    volatile uint8_t* port;

    uint8_t pinA, bitA;
    uint8_t pinB, bitB;
    uint8_t turning;    // Flag to keep track of turning state
    int8_t clicks;      // Counter to indicate cumulative clicks in either direction
    int8_t direction;   // indicator

    char id;

	AdaEncoder *next;
};

#endif	//AdaEncoder.h
