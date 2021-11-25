#include "Arduino.h"

#ifndef DShot4_h
#define DShot4_h

#if defined(__AVR_ATmega328P__)
// For UNO, PortD 0-7: i.e. D0-D7
#define DSHOT_PORT PORTD
#endif

#if defined(__AVR_ATmega8__)
// For UNO, PortD 0-7: i.e. D0-D7
#define DSHOT_PORT PORTD
// ADDON for timers
#define TIMSK1 TIMSK
#endif

#if defined(__AVR_ATmega32U4__)
// For Leonardo, PortB 4-7: i.e. D8-D11
#define DSHOT_PORT PORTB
#endif

class DShot4 {
 public:
  enum Mode { DSHOT600, DSHOT300, DSHOT150 };
  DShot4(const enum Mode mode);
  void attach(uint8_t pin);
  uint16_t DShot4::setThrottle(uint8_t pin, uint16_t throttle,
                               uint8_t set_telemetry_bit);

 private:
  uint16_t _packet = 0;
  uint16_t _throttle = 0;
  uint8_t _pinMask = 0;
};

#endif
