/*************************************************************
SoftSerialIntAP Test Program
Multi-instance Library/C++ Object for Maple/other STM32F1/Wiring
Copyright 2015 Ron Curry, InSyte Technologies
Author Ron Curry, InSyte Technologies

* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy,
* modify, merge, publish, distribute, sublicense, and/or sell copies
* of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
**************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
//#include <libmaple/libmaple.h>
#include <Arduino.h>
#include "SoftSerialIntAP.h"


// lDelay - Simple delay function independent of systick or interrupts
// Total delay is approximately 1ms per iteration @ 72mhz clock
// delay overhead is ~+1.487usec. one loop = ~137ns * 7300.
// Therefore, total delay is ~ (delay * 0.137us * 7300) + 1.487us
inline void lDelay(uint32_t delay) { 
  uint32_t i, j;

  j = delay * 7300;
  for (i = 0; i < j; i++) {
    asm volatile(
      "nop \n\t"
     ::);
  } 
}


SoftSerialIntAP SWSerial1(20, 21, 4);
SoftSerialIntAP SWSerial0(15, 16, 3);

void setup() {
  // put your setup code here, to run once:

  SWSerial1.begin(115200);
  SWSerial0.begin(115200);
  Serial.begin(230400);

//  systick_disable();
  
}

uint32_t baudrate[] = {300, 1200,2400, 4800,9600,14400,19200,28800,31250,38400,57600, 115200};
int16_t currentBaud = 11;
int receiveFlag = 0;
int silentRXFlag = 0;
int txFlag = 0;
int txTestFlag = 1;
int rxSourceFlag = 0;
int txSourceFlag = 0;
int exFlag = 0;
int exIndex = 0;
uint16_t stepSize = 1;
char testString0[] = "Testing TX Port 0 ...\n";
char testString1[] = "Testing TX Port 1 ...\n";
char testString2[] = "0123456789\n";
void loop() {
  char inChar;

  lDelay (3000);
  Serial.println("Starting now....");
  while (1) {


    if (exFlag) {

      // First sync up by clearing read buffer and write buffer
      while (SWSerial0.available() > 0)
        SWSerial0.read();

      lDelay(5000);
      
      while (1) {  
          SWSerial0.write((char)(testString2[exIndex]));
          while (1) {
            if (Serial.available())
              goto jOut;

            if (SWSerial0.available()) {
              break;
            }
          }
             
          inChar = (char)SWSerial0.read(); 
          Serial.print(inChar);

          exIndex++;
          if (exIndex > 10)
            exIndex = 0;
      }
    }
jOut:  

    
//************************************************************************
    if (txFlag) {
      if ((txSourceFlag == 1) || (txSourceFlag == 2)) {
        if (txTestFlag == 0)
          SWSerial1.write(0x55);
        else if (txTestFlag == 1) {
          SWSerial1.write(testString1, 22);
        } else
          SWSerial1.write(testString2, 10);
        lDelay(20);
      }
      if ((txSourceFlag == 0) || (txSourceFlag == 2)) {
        if (txTestFlag == 0)
          SWSerial0.write(0x55);
        else if (txTestFlag == 1) {
          SWSerial0.write(testString0, 22);
        } else
          SWSerial0.write(testString2, 10);
 
        lDelay(20);
      }
    }
 
//*************************************************************************
    if (receiveFlag) {
      if ((rxSourceFlag == 1) || (rxSourceFlag == 2)) {
        while (SWSerial1.available()) {
            if (SWSerial1.overflow()) {
              Serial.println("\nOVERFLOW Port 1");
        
              #if DEBUG_DELAY
              Serial.print("Buffer write pointer, buffer read pointer = ");
              Serial.print(SWSerial1.getOverFlowTail(), DEC);
              Serial.print(" ");
              Serial.println(SWSerial1.getOverFlowHead(), DEC);
//              receiveFlag = 0;
              #endif
            }
            else { 
              inChar = SWSerial1.read();
              if (silentRXFlag == 2) {
                Serial.write(inChar);
              }
            }
         }
      }

      if ((rxSourceFlag == 0) || (rxSourceFlag == 2)) {
        while (SWSerial0.available()) {
            if (SWSerial0.overflow()) {
              Serial.println("\nOVERFLOW Port 0");
        
              #if DEBUG_DELAY
              Serial.print("Buffer write pointer, buffer read pointer = ");
              Serial.print(SWSerial0.getOverFlowTail(), DEC);
              Serial.print(" ");
              Serial.println(SWSerial0.getOverFlowHead(), DEC);
//              receiveFlag = 0;
              #endif
            }
            else { 
              inChar = SWSerial0.read();
              if (silentRXFlag == 1) {
                Serial.write(inChar);
              }
            }
         }
      }
      
    } 
     
      
    if (Serial.available()) {
  
      inChar = Serial.read();
  
      switch (inChar) {
        case 'x':
          exFlag ^= 1;
          exIndex = 0;
          Serial.print("\nSerial Exchange Test = "); Serial.println(exFlag, DEC);
          break;
        case '?':
          Serial.println("Commands....");
          Serial.println("? - Display this help menu.");
          Serial.println("R - Toggle on/off receive test");
          Serial.println("r - Toggle receive source between port 0, 1, both");
          Serial.println("p - Toggle on/off print received bytes to console");
          Serial.println("T - Toggle on/off transmit test");
          Serial.println("t - Toggle transmit source between port 0, 1, both");
          Serial.println("5 - Toggle between sending a string or 0x55");
          Serial.println("x - Toggle on/off round robin send/receive test.");
          Serial.println("    (Requires serialtest_maple running on seperate device)");
          Serial.println("B - Bump baud rate higher");
          Serial.println("b - Bump baud rate lower");
          Serial.println("I - Bump intra-bit delay higher");
          Serial.println("i - Bump intra-bit delay lower");
          Serial.println("C - Bump bit centering later");
          Serial.println("c - Bump bit centering earlier");
          Serial.println("s = Print status of flags and other info");
          break;
        case 's':
          Serial.print("\n\nR - "); Serial.print("RX Test off(0) or on(1) = "); Serial.println(receiveFlag, DEC);
          Serial.print("r - "); Serial.print("RX Port "); Serial.println(rxSourceFlag, DEC);
          Serial.print("p - "); Serial.print("Print to console off(0) or on(1) = "); Serial.println(silentRXFlag, DEC);
          Serial.print("T - "); Serial.print("TX Test off(0) or on(1) = "); Serial.println(txFlag, DEC);
          Serial.print("5 - "); if (txTestFlag) Serial.println("Sending string"); else Serial.println("Sending 0x55");
          Serial.print("t - "); Serial.print("TX Port = "); Serial.println(txSourceFlag, DEC);
          Serial.print("Baud = "); Serial.println(baudrate[currentBaud], DEC);
          #if DEBUG_DELAY
          Serial.print("Timer Port 0 = "); Serial.println(SWSerial0.getrxtxTimer(), DEC);
          Serial.print("Timer Port 1 = "); Serial.println(SWSerial1.getrxtxTimer(), DEC);
          Serial.print("Bitperiod = "); Serial.println(SWSerial0.getBitPeriod(), DEC);
          Serial.print("Centering = "); Serial.println(SWSerial0.getBitCentering(), DEC);
          Serial.print("Stepsize = "); Serial.println(stepSize, DEC);
          Serial.print("TXRead = "); Serial.println(SWSerial0.getTXHead(), DEC);
          Serial.print("TXWrite = "); Serial.println(SWSerial0.getTXTail(), DEC);
          Serial.print("RXRead = "); Serial.println(SWSerial0.getRXHead(), DEC);
          Serial.print("RXWrite = "); Serial.println(SWSerial0.getRXTail(), DEC);
          #endif
          Serial.print("Available = "); Serial.println(SWSerial0.available(), DEC);
          break;
        case 'T':
          // do tx test
          txFlag ^= 1;
          Serial.print("\nTX Test = "); Serial.println(txFlag, DEC);
          break;
        case 't':
          txSourceFlag += 1;
          if (txSourceFlag > 2) 
            txSourceFlag = 0;
          Serial.print("\nTransmit source = "); Serial.println(txSourceFlag, DEC);
          break;
        case '5':
          txTestFlag += 1;
          if (txTestFlag >2)
            txTestFlag = 0;
          break;
        case 'R':
          receiveFlag ^= 1;
          break;
        case 'r':
          rxSourceFlag += 1;
          if (rxSourceFlag > 2) 
            rxSourceFlag = 0;
          Serial.print("\nSource = "); Serial.println(rxSourceFlag, DEC);
          break;
        case 'p':
          silentRXFlag ^= 1;
          break;
        case 'B':
          // bump up baud to next rate
          SWSerial0.end();
          SWSerial1.end();
          currentBaud++;
          if (currentBaud > 11)
              currentBaud = 0;
          SWSerial0.begin(baudrate[currentBaud]);
          SWSerial1.begin(baudrate[currentBaud]);
          Serial.println(); Serial.println(baudrate[currentBaud], DEC);
          break;
        case 'b':
          // bump up baud to next rate
          SWSerial0.end();
          currentBaud--;
          if (currentBaud < 0)
            currentBaud = 11;
          SWSerial0.begin(baudrate[currentBaud]);
          SWSerial1.begin(baudrate[currentBaud]);
          Serial.println(); Serial.println(baudrate[currentBaud], DEC);
          break;

        #if DEBUG_DELAY
        case 'I':
          // bump up rx intrabit delay value
          SWSerial0.setBitPeriod(SWSerial0.getBitPeriod() + stepSize);
          SWSerial1.setBitPeriod(SWSerial1.getBitPeriod() + stepSize);
          break;
        case 'i':
          // bump up rx intrabit delay value
          SWSerial0.setBitPeriod(SWSerial0.getBitPeriod() - stepSize);
          SWSerial1.setBitPeriod(SWSerial1.getBitPeriod() - stepSize);
          break;
        case 'C':
          // bump up rx bit centering delay value
          SWSerial0.setBitCentering(SWSerial0.getBitCentering() + stepSize);
          SWSerial1.setBitCentering(SWSerial1.getBitCentering() + stepSize);
          break;
        case 'c':
          // bump down rx centering delay value
          SWSerial0.setBitCentering(SWSerial0.getBitCentering() - stepSize);
          SWSerial1.setBitCentering(SWSerial1.getBitCentering() - stepSize);
          break;
        case '0':
          // set step size to 1
          stepSize = 1;
          break;
        case '1':
          // set step size to 10
          stepSize = 10;
          break;
        case '2':
          // set step size to 100
          stepSize = 100;
          break;
        #endif  
        default:
          break;      
      }   
    }    
  }
}
