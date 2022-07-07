#include "Arduino.h"
#include <cppQueue.h>  // Arduino Library from https://github.com/SMFSW/Queue
#include "queue.cpp"

#ifndef DShot_h
#define DShot_h

// #define DEBUG

#if defined(__AVR_ATmega328P__)
// For UNO, PortD 0-7: i.e. D0-D7
#define DSHOT_PORT1 PORTD
#define PORT1 digitalPinToPort(0)  // Arduino Port number for PORTD
#endif

#if defined(__AVR_ATmega8__)
// For UNO, PortD 0-7: i.e. D0-D7
#define DSHOT_PORT1 PORTD
#define PORT1 digitalPinToPort(0)  // Arduino Port number for PORTD
// ADDON for timers
#define TIMSK1 TIMSK
#endif

#if defined(__AVR_ATmega32U4__)
// For Leonardo, PortB 4-7: i.e. D8-D11
#define DSHOT_PORT1 PORTB
#define PORT1 digitalPinToPort(8)  // Arduino Port number for PORTB
// More pins: PortF 18-21: i.e. D18-D21
#define DSHOT_PORT2 PORTF
#define PORT2 digitalPinToPort(18)  // Arduino Port number for PORTF
#endif

class DShot {
 public:
  enum Mode { DSHOT600 = 4, DSHOT300 = 2, DSHOT150 = 1 };
  DShot(const enum Mode mode);

  // Initialises the output pin. Call once before sending anything
  void attach(uint8_t pin);

  // Send throttle value. If set_telemetry_bit is true, it will request telemetry once, and
  // afterwards sends the throttle value without telemetry request.
  uint16_t setThrottle(uint8_t pin, uint16_t throttle, bool set_telemetry_bit);

  // Send commands from List as many times as needed, waiting the specified time before sending 48
  // command (0 speed)
  uint16_t sendCommand(uint8_t pin, uint16_t command);

 private:
};

#endif
