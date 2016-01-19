# SoftSerialIntAP
Interrupt Driven Software Serial Library for STM32Duino - Use any GPIO

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
