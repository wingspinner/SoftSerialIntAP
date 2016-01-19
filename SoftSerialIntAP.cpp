/******************************************************************************
* SoftSerialIntAP.cpp
* Multi-instance Software Serial Library for STM32Duino
* Using Timers and Interrupts enabling use of any GPIO pins for RX/TX
* 
* Copyright 2015 Ron Curry, InSyte Technologies
* 
 
Features:
- Fully interrupt driven - no delay routines.
- Any GPIO pin may be used for tx or rx (hence the AP libray name suffix)
- Circular buffers on both send and receive.
- Works up to 115,200 baud.
- Member functions compatible with Hardware Serial and Arduino NewSoftSerial Libs
- Extensions for non-blocking read and transmit control
- Supports up to 4 ports (one timer used per port) without modification.
- Easily modified for more ports or different timers on chips with more timers.
- Can do full duplex under certain circumstances at lower baud rates.
- Can send/receive simultaneously with other ports/instantiatious at some baud
  rates and certain circumstances.

Notes:

Performance
- More than two ports use has not been extensively tested. High ISR latencies in the 
low-level Maple timer ISR code, C++ ISR wranglings, and the STM32 sharing interrupt 
architecture (not in that order) restrict simultaneous port use and speeds. Two ports
sending simultaniously at 115,200. As well, two ports simultansiously receiving at 57,600
simultaneously have been successfully tested. Various other situations have been 
tested. Results are dependent on various factors - you'll need to experiment.

Reliability
- Because of the way STM32 shares interrupts and the way STM32Arduino low level ISRs
processes them latencies can be very high for interrupts on a given timer. I won't go
into all the details of why but some interrupts can be essentially locked out. Causing
extremely delayed interrupt servicing. Some of this could be alleviated with more
sophisticated low-level ISRs that make sure that all shared interrupt sources get
serviced on an equal basis but that's not been done yet. 
This impacts the ability to do full duplex on a single port even at medium bit rates.
I've done some experimentation with two ports/instantiations with RX on one port
and TX on the other with good results for full-duplex. In any case, to be sure
that the serial data streams are not corrupted at higher data rates it's best to
write the application software such that rx and tx are not sending/receiving
simultaneously in each port and that other ports are not operating simultaneously
as well. This is, effectively, how the existing NewSoftSerial library on Arduino
operates so not a new concept. Again, experiment and find what works in your
application.

Improvements
No doubt the code can be improved upon. If you use the code PLEASE give back by
providing soure code for improvements or modifications you've made!
- Specific improvements that come to mind and I'd like to explore are:
  o Replacing the STM32/Maple timer interrupt handlers with something more streamlined
    and lower latency and overhead.
  o A better way to implement the high level C++ ISR's to reduce latency/overhead
  o Minor improvements that can save cycles in the C++ ISR's such as using bit-banding
  o Possibly a way to coordinate RX/TX to increase full-duplex capability.

License
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
*****************************************************************************/
 
#include <Arduino.h>
#include <HardwareTimer.h>
#include <libmaple/timer.h>
#include <ext_interrupts.h>
#include "SoftSerialIntAP.h"

/******************************************************************************
* Timer Definitions
* Change these if you wish to use different timer channels
******************************************************************************/
#define TIMER_MAX_COUNT   0xffff
#define TX_TIMER_CHANNEL  TIMER_CH3
#define TX_TIMER_MASK     TIMER_DIER_CC3IE_BIT
#define TX_TIMER_PENDING  TIMER_SR_CC3IF_BIT
#define TX_CCR            CCR3
#define RX_TIMER_CHANNEL  TIMER_CH4
#define RX_TIMER_MASK     TIMER_DIER_CC4IE_BIT
#define RX_TIMER_PENDING  TIMER_SR_CC4IF_BIT
#define RX_CCR            CCR4

/******************************************************************************
* ISR Related to Statics
* Don't modify anything here unless you know why and what
******************************************************************************/
SoftSerialIntAP           *SoftSerialIntAP::interruptObject1;
SoftSerialIntAP           *SoftSerialIntAP::interruptObject2;
SoftSerialIntAP           *SoftSerialIntAP::interruptObject3;
SoftSerialIntAP           *SoftSerialIntAP::interruptObject4;


voidFuncPtr             SoftSerialIntAP::handleRXEdgeInterruptP[4] = { 
  handleRXEdgeInterrupt1, handleRXEdgeInterrupt2, handleRXEdgeInterrupt3, handleRXEdgeInterrupt4
};

voidFuncPtr             SoftSerialIntAP::handleRXBitInterruptP[4] = {
  handleRXBitInterrupt1, handleRXBitInterrupt2, handleRXBitInterrupt3, handleRXBitInterrupt4
};

voidFuncPtr             SoftSerialIntAP::handleTXBitInterruptP[4] = {
  handleTXBitInterrupt1, handleTXBitInterrupt2, handleTXBitInterrupt3, handleTXBitInterrupt4
};


/******************************************************************************
* Convenience functions to disable/enable tx and rx interrupts
******************************************************************************/
// Mask transmit interrupt
inline void SoftSerialIntAP::noTXInterrupts() {
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, TX_TIMER_MASK) = 0;
}


// Enable transmit interrupt
// Note: Purposely does not clear pending interrupt
inline void SoftSerialIntAP::txInterrupts() {
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, TX_TIMER_MASK) = 1;
}


// Test if transmit interrupt is enabled
inline uint16_t SoftSerialIntAP::isTXInterruptEnabled() {
  return (*bb_perip(&(timerSerialDEV->regs).gen->DIER, TX_TIMER_MASK));  
}


// Clear pending interrupt and enable receive interrupt
// Note: Clears pending interrupt
inline void SoftSerialIntAP::txInterruptsClr() {
  *bb_perip(&(timerSerialDEV->regs).gen->SR, TX_TIMER_PENDING) = 0; // Clear int pending
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, TX_TIMER_MASK) = 1;
}


// Mask receive start bit interrupt
inline void SoftSerialIntAP::noRXStartInterrupts() {
    bb_peri_set_bit(&EXTI_BASE->FTSR, gpioBit, 0);
}


// Enable receive start bit interrupt
// Note: Purposely does not clear pending interrupt
inline void SoftSerialIntAP::rxStartInterrupts() {
    bb_peri_set_bit(&EXTI_BASE->FTSR, gpioBit, 1);
}


// Mask receive interrupt
inline void SoftSerialIntAP::noRXInterrupts() {
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, RX_TIMER_MASK) = 0;
}


// Enable receive interrupt
// Note: Purposely does not clear pending interrupt
inline void SoftSerialIntAP::rxInterrupts() {
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, RX_TIMER_MASK) = 1;
}


// Clear pending interrupt and enable receive interrupt
// Note: Clears pending interrupt
inline void SoftSerialIntAP::rxInterruptsClr() {
  *bb_perip(&(timerSerialDEV->regs).gen->SR, RX_TIMER_PENDING) = 0; // Clear int pending
  *bb_perip(&(timerSerialDEV->regs).gen->DIER, RX_TIMER_MASK) = 1;

}


/******************************************************************************
* Specialized functions to set interrupt priorities and assign object ointers
* These are needed due to the gyrations required to support per instance ISRs
******************************************************************************/
// Set Interrupt Priority for EXTInt line
void setEXTIntPriority(uint8_t pin, uint8_t priority) {

  switch((exti_num)(PIN_MAP[pin].gpio_bit)) {
    case EXTI0:
      nvic_irq_set_priority(NVIC_EXTI0, priority);
      break;
    case EXTI1:
      nvic_irq_set_priority(NVIC_EXTI1, priority);
      break;
    case EXTI2:
      nvic_irq_set_priority(NVIC_EXTI2, priority);
      break;
    case EXTI3:
      nvic_irq_set_priority(NVIC_EXTI3, priority);
      break;
    case EXTI4:
      nvic_irq_set_priority(NVIC_EXTI4, priority);
      break;
    case EXTI5:
    case EXTI6:
    case EXTI7:
    case EXTI8:
    case EXTI9:
      nvic_irq_set_priority(NVIC_EXTI_9_5, priority);
      break;
    case EXTI10:
    case EXTI11:
    case EXTI12:
    case EXTI13:
    case EXTI14:
    case EXTI15:
      nvic_irq_set_priority(NVIC_EXTI_15_10, priority);
      break;
  }
  
}


// Set Interrupt Priority for Timer Interrupts
void setTimerIntPriority(uint8_t timerNumber, uint8_t priority) {

  switch(timerNumber) {
    case 1:
      nvic_irq_set_priority(NVIC_TIMER1_UP, priority);
      nvic_irq_set_priority(NVIC_TIMER1_CC, priority);
      break;
    case 2:
      nvic_irq_set_priority(NVIC_TIMER2, priority);
      break;
    case 3:
      nvic_irq_set_priority(NVIC_TIMER3, priority);
      break;
    case 4:
      nvic_irq_set_priority(NVIC_TIMER4, priority);
      break;
  }
  
}


// Set the correct interruptObject for this instance
void SoftSerialIntAP::setInterruptObject(uint8_t timerNumber) {

  switch (timerNumber) {
    case 1:
      interruptObject1 = this;
      break;
    case 2:
      interruptObject2 = this;
      break;
    case 3:
      interruptObject3 = this;
      break;
    case 4:
      interruptObject4 = this;
      break;
  }
}


/******************************************************************************
* Constructor / Destructor
******************************************************************************/
// Constructor
SoftSerialIntAP::SoftSerialIntAP(int receivePinT = 15, int transmitPinT = 16, uint8_t rxtxTimerT = 1) :
  receivePin(receivePinT),
  transmitPin(transmitPinT),
  timerSerial(rxtxTimerT),
  rxtxTimer(rxtxTimerT)
{
  // Assign pointer to the hardware registers
  timerSerialDEV = timerSerial.c_dev();

  // Translate transmit pin number to external interrupt number
  gpioBit = (exti_num)(PIN_MAP[transmitPin].gpio_bit);

  // Setup ISR pointer for this instance and timer (one timer per instance)
  // This is a workaround for c++
  setInterruptObject(rxtxTimer);

}


// Destructor
SoftSerialIntAP::~SoftSerialIntAP() {
  end();
}


/******************************************************************************
* TX and RX Interrupt Service Routines
******************************************************************************/
// Transmits next bit. Called by timer ch1 compare interrupt
void SoftSerialIntAP::txNextBit(void) {

  // State 0 through 7 - receive bits
  if (txBitCount <= 7) {
    if (bitRead(transmitBuffer[transmitBufferRead], txBitCount) == 1) {
      digitalWrite(transmitPin,HIGH); 
    } else {
      digitalWrite(transmitPin,LOW);
    }

//    txTimingCount = ((uint16_t)(timerSerialDEV->regs).gen->TX_CCR) + bitPeriod;
    timerSerial.setCompare(TX_TIMER_CHANNEL, ((uint16_t)(timerSerialDEV->regs).gen->TX_CCR) + bitPeriod); 

    interrupts();
    // Bump the bit/state counter to state 8
    txBitCount++; 

    #if DEBUG_DELAY
    digitalWrite(DEBUG_PIN1,1);
    digitalWrite(DEBUG_PIN1,0);
    #endif

  // State 8 - Send the stop bit and reset state to state -1
  //          Shutdown timer interrupt if buffer empty
  } else if (txBitCount == 8) {

    // Send the stop bit
    digitalWrite(transmitPin, HIGH); 

    interrupts();

      transmitBufferRead = (transmitBufferRead == SS_MAX_TX_BUFF ) ? 0 : transmitBufferRead + 1;
//      transmitBufferRead = (transmitBufferRead + 1) % (SS_MAX_TX_BUFF - 1);

    if ((transmitBufferRead != transmitBufferWrite) && activeTX) {
      
      timerSerial.setCompare(TX_TIMER_CHANNEL, ((uint16_t)(timerSerialDEV->regs).gen->TX_CCR) + bitPeriod); 
      txBitCount = 10;
      
    } else {
      
      // Buffer empty so shutdown delay/timer until "write" puts data in
      noTXInterrupts();
    
    }

  // Send  start bit for new byte
  } else if (txBitCount == 10) {
    digitalWrite(transmitPin, 0);
    interrupts();

//    txTimingCount = ((uint16_t)(timerSerialDEV->regs).gen->TX_CCR) + bitPeriod;
    timerSerial.setCompare(TX_TIMER_CHANNEL, ((uint16_t)(timerSerialDEV->regs).gen->TX_CCR) + bitPeriod); 

    txBitCount = 0;                    
  }
  
}


// Start Bit Receive ISR
inline void SoftSerialIntAP::onRXPinChange(void){

  // Test if this is really the start bit and not a spurious edge
  if ((rxBitCount == 9) && activeRX) {  

    // Receive Timer/delay interrupt should be off now - unmask it and center the sampling time
//    rxTimingCount = ((uint16_t)(timerSerialDEV->regs).adv->CCR4 + startBitPeriod);
    timerSerial.setCompare(RX_TIMER_CHANNEL, ((uint16_t)(timerSerialDEV->regs.gen->CNT) + startBitPeriod)); 
    rxInterruptsClr();
    
    // Mask pinchange interrupt to reduce needless interrupt
    //      overhead while receiving this byte
    noRXStartInterrupts();
    interrupts();

    // Set state/bit to first bit
    rxBitCount = 0;

  } else
    interrupts();
}


// Receive next bit. Called by timer ch2 interrupt
inline void SoftSerialIntAP::rxNextBit(void) {

  if (rxBitCount < 8) {

//    rxTimingCount = ((uint16_t)(timerSerialDEV->regs).adv->CCR4 + bitPeriod);
    timerSerial.setCompare(RX_TIMER_CHANNEL, ((uint16_t)(timerSerialDEV->regs.gen->RX_CCR) + bitPeriod)); 

    receiveBuffer[receiveBufferWrite] >>= 1;  
    if (digitalRead(receivePin)) 
      receiveBuffer[receiveBufferWrite] |= 0x80;

    #if DEBUG_DELAY
    digitalWrite(DEBUG_PIN,1);
    digitalWrite(DEBUG_PIN,0);
    #endif
    
    interrupts();
      
     rxBitCount++;  

  // State 8 - Save incoming byte and update buffer
  } else if (rxBitCount == 8) {

    // Finish out stop bit while we...  
    //  Calculate location in buffer for next incoming byte
    //  Test if buffer full
    //  If the buffer isn't full update the tail pointer to point to next location
    //  Else if it is now full set the buffer overflow flag 
    // FYI - With this logic we effectively only have an (SS_MAX_RX_BUFF - 1) buffer size
    
    interrupts();
    uint8_t next = (receiveBufferWrite + 1) % SS_MAX_RX_BUFF;
    if (next != receiveBufferRead) {
      receiveBufferWrite = next;

    } else {
      bufferOverflow = true;

      
      #if DEBUG_DELAY
      overFlowTail = receiveBufferWrite;
      overFlowHead = receiveBufferRead;

      digitalWrite(DEBUG_PIN1, 1);
      digitalWrite(DEBUG_PIN1, 0);
      #endif
 
    }
    
    // Re-enable start bit detection
    rxStartInterrupts();
        
    // Shutdown nextbit timer interrupt until next start bit detected
    noRXInterrupts();

    // Set for state 9 to receive next byte
    rxBitCount = 9;
 
  } else {
    interrupts();
  }
  
}


/******************************************************************************
* Begin - Instance setup
******************************************************************************/
void SoftSerialIntAP::begin(uint32_t tBaud) {
  
  digitalWrite(transmitPin, 1);
  pinMode(receivePin, INPUT_PULLUP);
  pinMode(transmitPin, OUTPUT);

  #if DEBUG_DELAY
  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(DEBUG_PIN, 0);
  pinMode(DEBUG_PIN1, OUTPUT);
  digitalWrite(DEBUG_PIN1, 0);
  #endif

  // Initialize the timer
  noInterrupts();
  timerSerial.pause();

  if (tBaud > 2400) {
    bitPeriod = (uint16_t)((uint32_t)(CYCLES_PER_MICROSECOND * 1000000) / tBaud);
    startBitPeriod = bitPeriod + (bitPeriod / 2) - 300;
    timerSerial.setPrescaleFactor(1);
  } else if (tBaud > 300) {
    bitPeriod = (uint16_t)(((uint32_t)(CYCLES_PER_MICROSECOND * 1000000) / 16) / tBaud);
    startBitPeriod = bitPeriod + (bitPeriod / 2);
    timerSerial.setPrescaleFactor(16); 
  } else {
    bitPeriod = (uint16_t)(((uint32_t)(CYCLES_PER_MICROSECOND * 1000000) / 16) / tBaud) / 2;
    bitPeriod -= 600;
    startBitPeriod = bitPeriod + (bitPeriod / 2);
    timerSerial.setPrescaleFactor(16);     
  }

  timerSerial.setOverflow(TIMER_MAX_COUNT); 

  // Set transmit bit timer  
  // Compare value set later
  timerSerial.setMode(TX_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE);

  // State tx machine start state, attach bit interrupt, and mask it until a byte is sent
  transmitBufferRead = transmitBufferWrite = 0;
  txBitCount = 9;  
  timerSerial.attachInterrupt(TX_TIMER_CHANNEL, handleTXBitInterruptP[rxtxTimer - 1]);
  noTXInterrupts();
  
  // Set receive bit timer  
  // Compare value set later
  timerSerial.setMode(RX_TIMER_CHANNEL, TIMER_OUTPUT_COMPARE);

  // Set rx State machine start state, attach the bit interrupt and mask it until start bit is received
  receiveBufferRead = receiveBufferWrite = 0;
  rxBitCount = 9;
  timerSerial.attachInterrupt(RX_TIMER_CHANNEL, handleRXBitInterruptP[rxtxTimer - 1]);
  noRXInterrupts();
      
  // Make the timer we are using a high priority interrupt
  setTimerIntPriority(rxtxTimer, 0);

  // Load the timer values and start it
  timerSerial.refresh();     
  timerSerial.resume();

  // Set start bit interrupt and priority and leave it enabled to rx first byte
  attachInterrupt(receivePin, handleRXEdgeInterruptP[rxtxTimer - 1], FALLING);
  setEXTIntPriority(receivePin, 0);

  bufferOverflow = false;
  receiveBufferRead = receiveBufferWrite = 0;
  transmitBufferRead = transmitBufferWrite = 0;


  interrupts();
  
  listen();
  talk();

}


/******************************************************************************
* RX Related Public Methods
******************************************************************************/
// Sets current instance listening. Transmit is always enabled
// If his instance was already activeRX does nothing and returns false 
bool SoftSerialIntAP::listen() {

  // If receive not activeRX then re-init and set activeRX
  if (!activeRX) {

    // Reset receieve buffer and mark activeRX
    bufferOverflow = false;
    receiveBufferRead = receiveBufferWrite = 0;
    activeRX = true;

    // Turn the receive start bit detection on
    rxStartInterrupts();
      
    return true;
  }
  return false;
}


// Stop Listening - Shuts down only RX - Use end() to stop both rx and tx
// Returns true if was listening when called
// This instance will stop all RX interrupts after current in-process
// byte is finished receiving (if any).
// If no in-process receive byte it stops immediately
bool SoftSerialIntAP::stopListening() {

  if (activeRX) {
    
    noRXStartInterrupts();
    activeRX = false;
    return true;

  } else
    return false;
    
}


// Completely shutsdown this instance
// Not an RX related method but needs to be after stopListening
void SoftSerialIntAP::end() {

  stopListening();
  timerSerial.pause();
  detachInterrupt(receivePin);   
  timerSerial.detachInterrupt(RX_TIMER_CHANNEL);
  timerSerial.detachInterrupt(TX_TIMER_CHANNEL);
  timerSerial.setMode(TX_TIMER_CHANNEL, TIMER_DISABLED); 
  timerSerial.setMode(RX_TIMER_CHANNEL, TIMER_DISABLED); 
  digitalWrite(transmitPin, 1);
}


// Returns number of bytes in the RX buffer
int SoftSerialIntAP::available() { 
  int i;

  if (!activeRX)
    return 0;

  noRXInterrupts();
  i = (receiveBufferWrite + SS_MAX_RX_BUFF - receiveBufferRead) % SS_MAX_RX_BUFF;
  rxInterrupts();

  return i;
}

// Non-blocking read.
// Returns -1 if this instance isn't listening or the buffer is empty
int SoftSerialIntAP::readnb() { 
  
  if (!activeRX)
    return -1;
    
  if (receiveBufferRead == receiveBufferWrite)
    return -1;

  uint8_t inData = receiveBuffer[receiveBufferRead];  

  noRXInterrupts();
  receiveBufferRead = (receiveBufferRead + 1) % SS_MAX_RX_BUFF;
  rxInterrupts();
  
  return inData;
}


// Blocking read to be compatible with HardwareSerial
// Blocks until byte is available in buffer
// Returns -1 if instance is not activeRX
int SoftSerialIntAP::read() { 
  
  if (!activeRX)
    return -1;
    
  // Wait if buffer is empty
  while (receiveBufferRead == receiveBufferWrite);

  uint8_t inData = receiveBuffer[receiveBufferRead];  

  noRXInterrupts();
  receiveBufferRead = (receiveBufferRead + 1) % SS_MAX_RX_BUFF;
  rxInterrupts();
  
  return inData;
}


// Flush the receive buffer
void SoftSerialIntAP::flush() {
  
  noRXInterrupts();
  receiveBufferRead = receiveBufferWrite = 0;
  rxInterrupts();

}


// Return the next item in the receive buffer but leave in buffer
int SoftSerialIntAP::peek() {

  if (!activeRX)
    return -1;

  // If buffer is empty return false
  if (receiveBufferRead == receiveBufferWrite)
    return -1;

  // Otherwise read the byte at head of buffer but don't delete
  return receiveBuffer[receiveBufferRead];

}


/******************************************************************************
* TX Related Public Method(s)
******************************************************************************/
// Sets current instance enabled for sending
// If his instance was already activeRX does nothing and returns false 
bool SoftSerialIntAP::talk() {

  // If transmit not active then re-init and set activeTX
  if (!activeTX) {

    // Reset transmit buffer and mark active
    transmitBufferRead = transmitBufferWrite = 0;
    activeTX = true;

    // Turn transmit interrupts on
    txInterrupts();
      
    return true;
  }
  return false;
}


// Stop Sending - Shuts down only TX - Use end() to stop both rx and tx
// or "stopListening" for rx
// Returns true if sending already enabled when called
// This instance will stop sending at end of current byte immediately 
bool SoftSerialIntAP::stopTalking() {

  if (activeTX) {

    while (txBitCount < 8);
    activeTX = false;
    noTXInterrupts();
    return true;

  } else
    return false;

}


// Virtual write
// Saves tx byte in buffer and restarts transmit delay timer
// 1 bit time latency prior to transmit start if buffer was empty
size_t SoftSerialIntAP::write(uint8_t b) {

    // Check if transmit timer interrupt enabled and if not unmask it
    // transmit timer interrupt will get masked by transmit ISR when buffer becomes empty
    if (!isTXInterruptEnabled()) {
  
      // Save new data in buffer
      transmitBuffer[transmitBufferWrite] = b;
//      transmitBufferWrite = (transmitBufferWrite + 1) % (SS_MAX_TX_BUFF - 1);
      transmitBufferWrite = (transmitBufferWrite == SS_MAX_TX_BUFF) ? 0 : transmitBufferWrite + 1;
      
      // Set state to 10 (send start bit) and re-enable transmit interrupt
      txBitCount = 10;
      timerSerial.setCompare(TX_TIMER_CHANNEL, (int16_t)((timerSerialDEV->regs).bas->CNT) + 1); 
      txInterruptsClr();
      
    } else {

      // Blocks if buffer full
      bool i;
      do {
//        noTXInterrupts();
//          i = (((transmitBufferWrite + 1) == transmitBufferRead) || 
//                ((transmitBufferWrite == SS_MAX_TX_BUFF) && (transmitBufferRead == 0)));
        i = (((transmitBufferWrite + 1) % SS_MAX_TX_BUFF) == transmitBufferRead);
//        txInterrupts();
      } while (i);
  
      // Save new data in buffer and bump the write pointer
      transmitBuffer[transmitBufferWrite] = b;
//      noTXInterrupts();
//      transmitBufferWrite = (transmitBufferWrite + 1) % (SS_MAX_TX_BUFF - 1);
      transmitBufferWrite = (transmitBufferWrite == SS_MAX_TX_BUFF) ? 0 : transmitBufferWrite + 1;
//      txInterrupts();
    }
    
  return 1;
  
}

/******************************************************************************
* 
* Intermediate Level Interrupt Service Routines
* One ISR for each interrupt times 4 to support 4 instantiations of the class
* on up to 4 different timers. 
* 
* This is to work around the fact that static data and
* static member functions become part of the base class and are common to all
* instantiations and ISRs must be static in order to derive a std C type pointer to
* them which is required by the NVIC and hardware timer interrupt code. If there is 
* a better way to do this I'd welcome to learn about it.
* 
* These are at the bottom of the file just to get them out of the way.
******************************************************************************/

inline void SoftSerialIntAP::handleRXBitInterrupt1() {

//  if (interruptObject1)
//  { 
    noInterrupts(); 
    interruptObject1->rxNextBit(); 
    interrupts();
//  }
}


inline void SoftSerialIntAP::handleRXEdgeInterrupt1() {
  if (interruptObject1)
//  {
    noInterrupts(); 
    interruptObject1->onRXPinChange();
    interrupts();
//  }
}


inline void SoftSerialIntAP::handleTXBitInterrupt1() {
//  if (interruptObject1)
//  {
    noInterrupts(); 
    interruptObject1->txNextBit();
    interrupts();
//  }
}

inline void SoftSerialIntAP::handleRXBitInterrupt2() {
//  if (interruptObject2)
//  {
    noInterrupts(); 
    interruptObject2->rxNextBit();
    interrupts();
//  }
}


inline void SoftSerialIntAP::handleRXEdgeInterrupt2() {
//  if (interruptObject2)
//  {
    noInterrupts(); 
    interruptObject2->onRXPinChange();
    interrupts();
//  }
}


inline void SoftSerialIntAP::handleTXBitInterrupt2() {
//  if (interruptObject2)
//  {
    noInterrupts(); 
    interruptObject2->txNextBit();
    interrupts();
//  }
}

inline void SoftSerialIntAP::handleRXBitInterrupt3() {
//  if (interruptObject3)
//  {
    noInterrupts(); 
    interruptObject3->rxNextBit();
    interrupts();
//  }
}


inline void SoftSerialIntAP::handleRXEdgeInterrupt3() {
//  if (interruptObject3)
//  {
    noInterrupts(); 
    interruptObject3->onRXPinChange();
    interrupts();
//  }
}


inline void SoftSerialIntAP::handleTXBitInterrupt3() {
//  if (interruptObject3)
//  {
    noInterrupts(); 
    interruptObject3->txNextBit();
    interrupts();
//  }
}

inline void SoftSerialIntAP::handleRXBitInterrupt4() {
//  if (interruptObject4)
//  {
    noInterrupts(); 
    interruptObject4->rxNextBit();
    interrupts();
//  }
}


inline void SoftSerialIntAP::handleRXEdgeInterrupt4() {
//  if (interruptObject4)
//  {
    noInterrupts(); 
    interruptObject4->onRXPinChange();
    interrupts();
//  }
}


inline void SoftSerialIntAP::handleTXBitInterrupt4() {
//  if (interruptObject4)
//  {
    noInterrupts(); 
    interruptObject4->txNextBit();
    interrupts();
//  }

}

