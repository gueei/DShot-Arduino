#include "DShot.h"
#include "Arduino.h"

// Each item contains the bit of the port
// Pins that are not attached will always be 1
// This is to offload the Off pattern calculation during bit send
static uint8_t dShotBits[16];

// Denote which pin is attached to dShot
static uint8_t dShotPins = 0;

// Mode: 600/300/150
static enum DShot::Mode dShotMode = DShot::Mode::DSHOT600;

#define NOP "NOP\n"
#define NOP2 NOP NOP
#define NOP4 NOP2 NOP2
#define NOP8 NOP4 NOP4

/*
        DSHOT600 implementation
        For 16MHz CPU,
        0: 10 cycle ON, 17 cycle OFF
        1: 20 cycle ON, 7 cycle OFF
        Total 27 cycle each bit
*/
static inline void sendData() {
  noInterrupts();
  switch (dShotMode) {
    case DShot::Mode::DSHOT600:
    default:
      asm(
          // For i = 0 to 15:
          "LDI	r23,	0\n"
          // Set High for every attached pins
          // DSHOT_PORT |= dShotPins;
          "IN	r25,	%0\n"
          "_for_loop_0:\n"
          "OR	r25,	%1\n"
          // Wait 7 cycles (7 - 6 = 1)
          "NOP\n"

          "OUT	%0,	r25\n"
          // Wait 10 cycles (10 - 4 = 6)
          NOP4 NOP2
          // Set Low for low bits only
          // DSHOT_PORT &= dShotBits[i];
          "LD	r24,	Z+\n"
          "AND	r25,	r24\n"
          "OUT	%0,	r25\n"
          // Wait 10 cycles (10 - 2 = 8)
          NOP8
          // Turn off everything
          // DSHOT_PORT &= ~dShotPins;
          "AND	r25,	%2\n"
          "OUT	%0,	r25\n"
          // Add to i (tmp_reg)
          "INC	r23\n"
          "CPI	r23,	16\n"
          "BRLO	_for_loop_0\n"
          // 7 cycles to next bit (4 to add to i and branch, 2 to turn on), no
          // wait
          :
          : "I"(_SFR_IO_ADDR(DSHOT_PORT)), "r"(dShotPins), "r"(~dShotPins),
            "z"(dShotBits)
          : "r25", "r24", "r23");
      break;
    case DShot::Mode::DSHOT300:
      asm(
          // For i = 0 to 15:
          "LDI	r23,	0\n"
          // Set High for every attached pins
          // DSHOT_PORT |= dShotPins;
          "IN	r25,	%0\n"
          "_for_loop_1:\n"
          "OR	r25,	%1\n"
          // Wait 14 cycles (14 - 6 = 8)

          // 1 + 3 * N //
          "LDI	r26,	2\n"  // 1 // set N
          "_sleep_loop_1_1:\n"
          "DEC	r26\n"              // 1 //
          "BRNE	_sleep_loop_1_1\n"  // 2 //
          "NOP\n"  // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //
          "NOP\n"

          "OUT	%0,	r25\n"
          // Wait 20 cycles (20 - 4 = 16)

          // 1 + 3 * N //
          "LDI	r26,	5\n"  // 1 // set N
          "_sleep_loop_1_2:\n"
          "DEC	r26\n"              // 1 //
          "BRNE	_sleep_loop_1_2\n"  // 2 //
          "NOP\n"  // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          // Set Low for low bits only
          // DSHOT_PORT &= dShotBits[i];
          "LD	r24,	Z+\n"  // 2 //
          "AND	r25,	r24\n"
          "OUT	%0,	r25\n"
          // Wait 20 cycles (20 - 2 = 18)

          // 1 + 3 * N //
          "LDI	r26,	5\n"  // 1 // set N
          "_sleep_loop_1_3:\n"
          "DEC	r26\n"              // 1 //
          "BRNE	_sleep_loop_1_3\n"  // 2 //
          "NOP\n"  // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          "NOP\n"
          "NOP\n"
          // Turn off everything
          // DSHOT_PORT &= ~dShotPins;
          "AND	r25,	%2\n"
          "OUT	%0,	r25\n"
          // Add to i (tmp_reg)
          "INC	r23\n"
          "CPI	r23,	16\n"
          "BRLO	_for_loop_1\n"
          // 7 cycles to next bit (4 to add to i and branch, 2 to turn on), no
          // wait
          :
          : "I"(_SFR_IO_ADDR(DSHOT_PORT)), "r"(dShotPins), "r"(~dShotPins),
            "z"(dShotBits)
          : "r25", "r24", "r23");
      break;
    case DShot::Mode::DSHOT150:
      asm(
          // For i = 0 to 15:
          "LDI	r23,	0\n"
          // Set High for every attached pins
          // DSHOT_PORT |= dShotPins;
          "IN	r25,	%0\n"

          "_for_loop_2:\n"
          "OR	r25,	%1\n"
          // Wait 28 cucles (28 - 6 = 22)

          // 1 + 3 * N //
          "LDI	r26,	7\n"  // 1 // set N
          "_sleep_loop_2_1:\n"
          "DEC	r26\n"              // 1 //
          "BRNE	_sleep_loop_2_1\n"  // 2 //
          "NOP\n"  // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          "OUT	%0,	r25\n"
          // Wait 40 cycles (40 - 4 = 36)

          // 1 + 3 * N //
          "LDI	r26,	11\n"  // 1 // set N
          "_sleep_loop_2_2:\n"
          "DEC	r26\n"              // 1 //
          "BRNE	_sleep_loop_2_2\n"  // 2 //
          "NOP\n"  // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          "NOP\n"
          "NOP\n"

          // Set Low for low bits only
          // DSHOT_PORT &= dShotBits[i];
          "LD	r24,	Z+\n"
          "AND	r25,	r24\n"
          "OUT	%0,	r25\n"
          // Wait 40 cycles (40 - 2 = 38)

          // 1 + 3 * N //
          "LDI	r26,	11\n"  // 1 // set N
          "_sleep_loop_2_3:\n"
          "DEC	r26\n"              // 1 //
          "BRNE	_sleep_loop_2_3\n"  // 2 //
          "NOP\n"  // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          "NOP\n"
          "NOP\n"
          "NOP\n"
          "NOP\n"

          // Turn off everything
          // DSHOT_PORT &= ~dShotPins;
          "AND	r25,	%2\n"
          "OUT	%0,	r25\n"
          // Add to i (tmp_reg)
          "INC	r23\n"
          "CPI	r23,	16\n"
          "BRLO	_for_loop_2\n"
          // 7 cycles to next bit (4 to add to i and branch, 2 to turn on), no
          // wait
          :
          : "I"(_SFR_IO_ADDR(DSHOT_PORT)), "r"(dShotPins), "r"(~dShotPins),
            "z"(dShotBits)
          : "r25", "r24", "r23", "r26");
      break;
  }
  interrupts();
}

static boolean timerActive = false;
/*
  Generated by:
  http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  1000 Hz Update rate
*/
static void initISR() {
  cli();       // stop interrupts
  TCCR1A = 0;  // set entire TCCR1A register to 0
  TCCR1B = 0;  // same for TCCR1B
  TCNT1 = 0;   // initialize counter value to 0
  // set compare match register for 500 Hz increments
  OCR1A = 31999;  // = 16000000 / (1 * 500) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 1 prescaler
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  timerActive = true;
  for (byte i = 0; i < 16; i++) {
    dShotBits[i] = 0;
  }
  dShotPins = 0;

  sei();  // allow interrupts
}

static boolean isTimerActive() { return timerActive; }

ISR(TIMER1_COMPA_vect) { sendData(); }

/*
  Prepare data packet, attach 0 to telemetry bit, and calculate CRC
  throttle: 11-bit data
*/
static inline uint16_t createPacket(uint16_t throttle) {
  uint8_t csum = 0;
  throttle <<= 1;
  // Indicate as command if less than 48
  if (throttle < 48 && throttle > 0) throttle |= 1;
  uint16_t csum_data = throttle;
  for (byte i = 0; i < 3; i++) {
    csum ^= csum_data;
    csum_data >>= 4;
  }
  csum &= 0xf;
  return (throttle << 4) | csum;
}

/****************** end of static functions *******************/

DShot::DShot(const enum Mode mode) { dShotMode = mode; }

void DShot::attach(uint8_t pin) {
  this->_packet = 0;
  this->_pinMask = digitalPinToBitMask(pin);
  pinMode(pin, OUTPUT);
  if (!isTimerActive()) {
    initISR();
  }
  dShotPins |= this->_pinMask;
}

/*
  Set the throttle value and prepare the data packet and store
  throttle: 11-bit data
*/
uint16_t DShot::setThrottle(uint16_t throttle) {
  this->_throttle = throttle;

  // TODO: This part can be further optimized when combine with create packet
  this->_packet = createPacket(throttle);
  uint16_t mask = 0x8000;
  for (byte i = 0; i < 16; i++) {
    if (this->_packet & mask)
      dShotBits[i] |= this->_pinMask;
    else
      dShotBits[i] &= ~(this->_pinMask);
    mask >>= 1;
  }
  return this->_packet;
}
