#include "Arduino.h"

#ifndef DShot_h

#define DShot_h

// Bunch of NOP to pause the CPU
#define NOP asm("nop\n")
#define NOP2 NOP;NOP
#define NOP4 NOP2;NOP2
#define NOP8 NOP4;NOP4
#define NOP16 NOP8;NOP8
#define NOP32 NOP16;NOP16

#if defined(__AVR_ATmega328P__)
// We are using Digital pin 9: PORTB1
#define OUTPUT_PORT PORTB
#define OUTPUT_BIT _BV(1)
#define OUTPUT_PIN 9
#endif

#if defined(__AVR_ATmega32U4__)
// We are using Digital pin 9: PORTB5
#define OUTPUT_PORT PORTB
#define OUTPUT_BIT _BV(5)
#define OUTPUT_PIN 9
#endif


#define OUTPUT_HIGH OUTPUT_PORT |= OUTPUT_BIT
#define OUTPUT_LOW OUTPUT_PORT &= ~OUTPUT_BIT


class DShot{
  public:
    DShot(int id);
    uint16_t setThrottle(uint16_t throttle);
  private:
    uint16_t _packet = 0;
    uint16_t _throttle = 0;
    void dShot300_0();
    void dShot300_1();
};

#endif
