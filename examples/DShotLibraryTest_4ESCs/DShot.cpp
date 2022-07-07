#include "DShot.h"

// TODO: See if timing works for sequence of telemetry requests: DSHOT600: t_period = x us,
// t_telemetry_packet = x ms? Telemetry sent back over one UART --> one: first port has priority
#define MAX_TELEMETRY_REQUESTS 8
#define TELEMETRY_WAIT 0  // 4* dShotMode
                          // TELEMETRY_SEND_TIME/DSHOT150_PERIOD * (dShotMode+1) --> since the dshot
                          // commands are sent at 500 Hz rate, no wait is required.
//------------------------------------------------

#define DSHOT150_PERIOD 247      // us
#define TELEMETRY_SEND_TIME 900  // us

// Denote which pins are attached to dShot
static uint8_t dShotPins[] = {0, 0};
// Each item contains the bit of the port
// Pins that are not attached will always be 1
// This is to offload the Off pattern calculation during bit send
static uint8_t dShotBits[2][16];
// counter, how many telemetry requests have to be done. first come first served.
static uint8_t telemetryRequests[] = {0, 0};

static uint8_t tlm_req_count = 0;

static uint8_t dShotBitsTemp[16] = {0};

static Queue dShotBitsT[2];  // FIFO Queue for telemetry bits
static cppQueue telemetryRequestsPort(sizeof(uint8_t), 2 * MAX_TELEMETRY_REQUESTS,
                                      FIFO);  // FIFO Queue for telemetry port

// counter to set to the wait time in DSHOTperiod units
static uint16_t telemetryRequestWait = 0;

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

  // Logic for sending one telemetry request at a time
  uint8_t dShotBitsSend[2][16] = {0};
#ifndef DEBUG
  if (telemetryRequestWait == 0) {
    if (telemetryRequestsPort.getCount() > 0) {
      uint8_t port;
      telemetryRequestsPort.pop(&port);
      if (port == 0) {
        // if (telemetryRequests[0] > 0) {
        dShotBitsT[0].pop(dShotBitsSend[0]);
        memcpy(dShotBitsSend[1], dShotBits[1], sizeof(dShotBits[1]));
        --telemetryRequests[0];
        // } else if (telemetryRequests[1] > 0) {
      } else {
        memcpy(dShotBitsSend[0], dShotBits[0], sizeof(dShotBits[0]));
        dShotBitsT[1].pop(dShotBitsSend[1]);
        --telemetryRequests[1];
      }
      telemetryRequestWait = TELEMETRY_WAIT;  // for debug
      --tlm_req_count;
    } else {
      memcpy(dShotBitsSend[0], dShotBits[0], sizeof(dShotBits[0]));
      memcpy(dShotBitsSend[1], dShotBits[1], sizeof(dShotBits[1]));
    }
  } else {
    telemetryRequestWait--;
    memcpy(dShotBitsSend[0], dShotBits[0], sizeof(dShotBits[0]));
    memcpy(dShotBitsSend[1], dShotBits[1], sizeof(dShotBits[1]));
  }
#endif

  switch (dShotMode) {
    case DShot::Mode::DSHOT600:
    default:
      asm(
          // For i = 0 to 15:
          "LDI  r23,  0\n"
          // Set High for every attached pins
          // DSHOT_PORT |= dShotPins;
          "IN r25,  %0\n"
          "_for_loop_0:\n"
          "OR r25,  %1\n"
          // Wait 7 cycles (7 - 6 = 1)
          "NOP\n"

          "OUT  %0, r25\n"
          // Wait 10 cycles (10 - 4 = 6)
          NOP4 NOP2
          // Set Low for low bits only
          // DSHOT_PORT &= dShotBits[i];
          "LD r24,  Z+\n"
          "AND  r25,  r24\n"
          "OUT  %0, r25\n"
          // Wait 10 cycles (10 - 2 = 8)
          NOP8
          // Turn off everything
          // DSHOT_PORT &= ~dShotPins;
          "AND  r25,  %2\n"
          "OUT  %0, r25\n"
          // Add to i (tmp_reg)
          "INC  r23\n"
          "CPI  r23,  16\n"
          "BRLO _for_loop_0\n"
          // 7 cycles to next bit (4 to add to i and branch, 2 to turn on), no
          // wait
          :
          : "I"(_SFR_IO_ADDR(DSHOT_PORT1)), "r"(dShotPins[0]), "r"(~dShotPins[0]),
            "z"(dShotBitsSend[0]) 
          : "r25", "r24", "r23");

#ifdef DSHOT_PORT2
      asm(
          // For i = 0 to 15:
          "LDI  r23,  0\n"
          // Set High for every attached pins
          // DSHOT_PORT |= dShotPins;
          "IN r25,  %0\n"
          "_for_loop_02:\n"
          "OR r25,  %1\n"
          // Wait 7 cycles (7 - 6 = 1)
          "NOP\n"

          "OUT  %0, r25\n"
          // Wait 10 cycles (10 - 4 = 6)
          NOP4 NOP2
          // Set Low for low bits only
          // DSHOT_PORT &= dShotBits[i];
          "LD r24,  Z+\n"
          "AND  r25,  r24\n"
          "OUT  %0, r25\n"
          // Wait 10 cycles (10 - 2 = 8)
          NOP8
          // Turn off everything
          // DSHOT_PORT &= ~dShotPins;
          "AND  r25,  %2\n"
          "OUT  %0, r25\n"
          // Add to i (tmp_reg)
          "INC  r23\n"
          "CPI  r23,  16\n"
          "BRLO _for_loop_02\n"
          // 7 cycles to next bit (4 to add to i and branch, 2 to turn on), no
          // wait
          :
          : "I"(_SFR_IO_ADDR(DSHOT_PORT2)), "r"(dShotPins[1]), "r"(~dShotPins[1]),
            "z"(dShotBitsSend[1])  
          : "r25", "r24", "r23");
#endif
      break;
    case DShot::Mode::DSHOT300:
      asm(
          // For i = 0 to 15:
          "LDI  r23,  0\n"
          // Set High for every attached pins
          // DSHOTPORT |= dShotPins;
          "IN r25,  %0\n"
          "_for_loop_1:\n"
          "OR r25,  %1\n"
          // Wait 14 cycles (14 - 6 = 8)

          // 1 + 3 * N //
          "LDI  r26,  2\n"  // 1 // set N
          "_sleep_loop_1_1:\n"
          "DEC  r26\n"              // 1 //
          "BRNE _sleep_loop_1_1\n"  // 2 //
          "NOP\n"                   // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //
          "NOP\n"

          "OUT  %0, r25\n"
          // Wait 20 cycles (20 - 4 = 16)

          // 1 + 3 * N //
          "LDI  r26,  5\n"  // 1 // set N
          "_sleep_loop_1_2:\n"
          "DEC  r26\n"              // 1 //
          "BRNE _sleep_loop_1_2\n"  // 2 //
          "NOP\n"                   // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          // Set Low for low bits only
          // DSHOT_PORT &= dShotBits[i];
          "LD r24,  Z+\n"  // 2 //
          "AND  r25,  r24\n"
          "OUT  %0, r25\n"
          // Wait 20 cycles (20 - 2 = 18)

          // 1 + 3 * N //
          "LDI  r26,  5\n"  // 1 // set N
          "_sleep_loop_1_3:\n"
          "DEC  r26\n"              // 1 //
          "BRNE _sleep_loop_1_3\n"  // 2 //
          "NOP\n"                   // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          "NOP\n"
          "NOP\n"
          // Turn off everything
          // DSHOT_PORT &= ~dShotPins;
          "AND  r25,  %2\n"
          "OUT  %0, r25\n"
          // Add to i (tmp_reg)
          "INC  r23\n"
          "CPI  r23,  16\n"
          "BRLO _for_loop_1\n"
          // 7 cycles to next bit (4 to add to i and branch, 2 to turn on), no
          // wait
          :
          : "I"(_SFR_IO_ADDR(DSHOT_PORT1)), "r"(dShotPins[0]), "r"(~dShotPins[0]),
            "z"(dShotBitsSend[0])  
          : "r25", "r24", "r23");

#ifdef DSHOT_PORT2
      asm(
          // For i = 0 to 15:
          "LDI  r23,  0\n"
          // Set High for every attached pins
          // DSHOTPORT |= dShotPins;
          "IN r25,  %0\n"
          "_for_loop_12:\n"
          "OR r25,  %1\n"
          // Wait 14 cycles (14 - 6 = 8)

          // 1 + 3 * N //
          "LDI  r26,  2\n"  // 1 // set N
          "_sleep_loop_12_1:\n"
          "DEC  r26\n"               // 1 //
          "BRNE _sleep_loop_12_1\n"  // 2 //
          "NOP\n"                    // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //
          "NOP\n"

          "OUT  %0, r25\n"
          // Wait 20 cycles (20 - 4 = 16)

          // 1 + 3 * N //
          "LDI  r26,  5\n"  // 1 // set N
          "_sleep_loop_12_2:\n"
          "DEC  r26\n"               // 1 //
          "BRNE _sleep_loop_12_2\n"  // 2 //
          "NOP\n"                    // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          // Set Low for low bits only
          // DSHOT_PORT &= dShotBits[i];
          "LD r24,  Z+\n"  // 2 //
          "AND  r25,  r24\n"
          "OUT  %0, r25\n"
          // Wait 20 cycles (20 - 2 = 18)

          // 1 + 3 * N //
          "LDI  r26,  5\n"  // 1 // set N
          "_sleep_loop_12_3:\n"
          "DEC  r26\n"               // 1 //
          "BRNE _sleep_loop_12_3\n"  // 2 //
          "NOP\n"                    // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          "NOP\n"
          "NOP\n"
          // Turn off everything
          // DSHOT_PORT &= ~dShotPins;
          "AND  r25,  %2\n"
          "OUT  %0, r25\n"
          // Add to i (tmp_reg)
          "INC  r23\n"
          "CPI  r23,  16\n"
          "BRLO _for_loop_12\n"
          // 7 cycles to next bit (4 to add to i and branch, 2 to turn on), no
          // wait
          :
          : "I"(_SFR_IO_ADDR(DSHOT_PORT2)), "r"(dShotPins[1]), "r"(~dShotPins[1]),
            "z"(dShotBitsSend[1]) 
          : "r25", "r24", "r23");
#endif
      break;
    case DShot::Mode::DSHOT150:
      asm(
          // For i = 0 to 15:
          "LDI  r23,  0\n"
          // Set High for every attached pins
          // DSHOT_PORT |= dShotPins;
          "IN r25,  %0\n"

          "_for_loop_2:\n"
          "OR r25,  %1\n"
          // Wait 28 cucles (28 - 6 = 22)

          // 1 + 3 * N //
          "LDI  r26,  7\n"  // 1 // set N
          "_sleep_loop_2_1:\n"
          "DEC  r26\n"              // 1 //
          "BRNE _sleep_loop_2_1\n"  // 2 //
          "NOP\n"                   // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          "OUT  %0, r25\n"
          // Wait 40 cycles (40 - 4 = 36)

          // 1 + 3 * N //
          "LDI  r26,  11\n"  // 1 // set N
          "_sleep_loop_2_2:\n"
          "DEC  r26\n"              // 1 //
          "BRNE _sleep_loop_2_2\n"  // 2 //
          "NOP\n"                   // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          "NOP\n"
          "NOP\n"

          // Set Low for low bits only
          // DSHOT_PORT &= dShotBits[i];
          "LD r24,  Z+\n"
          "AND  r25,  r24\n"
          "OUT  %0, r25\n"
          // Wait 40 cycles (40 - 2 = 38)

          // 1 + 3 * N //
          "LDI  r26,  11\n"  // 1 // set N
          "_sleep_loop_2_3:\n"
          "DEC  r26\n"              // 1 //
          "BRNE _sleep_loop_2_3\n"  // 2 //
          "NOP\n"                   // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          "NOP\n"
          "NOP\n"
          "NOP\n"
          "NOP\n"

          // Turn off everything
          // DSHOT_PORT &= ~dShotPins;
          "AND  r25,  %2\n"
          "OUT  %0, r25\n"
          // Add to i (tmp_reg)
          "INC  r23\n"
          "CPI  r23,  16\n"
          "BRLO _for_loop_2\n"
          // 7 cycles to next bit (4 to add to i and branch, 2 to turn on), no
          // wait
          :
          : "I"(_SFR_IO_ADDR(DSHOT_PORT1)), "r"(dShotPins[0]), "r"(~dShotPins[0]),
            "z"(dShotBitsSend[0]) 
          : "r25", "r24", "r23", "r26");

#ifdef DSHOT_PORT2
      asm(
          // For i = 0 to 15:
          "LDI  r23,  0\n"
          // Set High for every attached pins
          // DSHOT_PORT |= dShotPins;
          "IN r25,  %0\n"

          "_for_loop_22:\n"
          "OR r25,  %1\n"
          // Wait 28 cucles (28 - 6 = 22)

          // 1 + 3 * N //
          "LDI  r26,  7\n"  // 1 // set N
          "_sleep_loop_22_1:\n"
          "DEC  r26\n"               // 1 //
          "BRNE _sleep_loop_22_1\n"  // 2 //
          "NOP\n"                    // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          "OUT  %0, r25\n"
          // Wait 40 cycles (40 - 4 = 36)

          // 1 + 3 * N //
          "LDI  r26,  11\n"  // 1 // set N
          "_sleep_loop_22_2:\n"
          "DEC  r26\n"               // 1 //
          "BRNE _sleep_loop_22_2\n"  // 2 //
          "NOP\n"                    // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          "NOP\n"
          "NOP\n"

          // Set Low for low bits only
          // DSHOT_PORT &= dShotBits[i];
          "LD r24,  Z+\n"
          "AND  r25,  r24\n"
          "OUT  %0, r25\n"
          // Wait 40 cycles (40 - 2 = 38)

          // 1 + 3 * N //
          "LDI  r26,  11\n"  // 1 // set N
          "_sleep_loop_22_3:\n"
          "DEC  r26\n"               // 1 //
          "BRNE _sleep_loop_22_3\n"  // 2 //
          "NOP\n"                    // 1 // BRNE on skip uses only 1 cycle, not 2
          // 1 + 3 * N //

          "NOP\n"
          "NOP\n"
          "NOP\n"
          "NOP\n"

          // Turn off everything
          // DSHOT_PORT &= ~dShotPins;
          "AND  r25,  %2\n"
          "OUT  %0, r25\n"
          // Add to i (tmp_reg)
          "INC  r23\n"
          "CPI  r23,  16\n"
          "BRLO _for_loop_22\n"
          // 7 cycles to next bit (4 to add to i and branch, 2 to turn on), no
          // wait
          :
          : "I"(_SFR_IO_ADDR(DSHOT_PORT2)), "r"(dShotPins[1]), "r"(~dShotPins[1]),
            "z"(dShotBitsSend[1])
          : "r25", "r24", "r23", "r26");
#endif

      break;
  }
  interrupts();
}

static boolean timerActive = false;

/*
  Generated by:
  http://www.8bit-era.cz/arduino-timer-interrupts-calculator.html
  500 Hz Update rate
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
  for (uint8_t p = 0; p < 2; p++) {
    for (uint8_t t = 0; t < MAX_TELEMETRY_REQUESTS; t++) {
      for (uint8_t i = 0; i < 16; i++) {
        // dShotBits[p][t][i] = 0;
      }
    }
    dShotPins[p] = 0;
  }

  sei();  // allow interrupts
}

static boolean isTimerActive() { return timerActive; }

ISR(TIMER1_COMPA_vect) { sendData(); }

/*
  Prepare data packet, attach 0 to telemetry bit, and calculate CRC
  throttle: 11-bit data
*/
static inline uint16_t createPacket(uint16_t throttle, bool telemetry) {
  uint8_t csum = 0;
  throttle <<= 1;
  throttle |= telemetry;
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
  uint8_t pinMask = digitalPinToBitMask(pin);
  uint8_t p = (digitalPinToPort(pin) == PORT1 ? 0 : 1);
  pinMode(pin, OUTPUT);
  if (!isTimerActive()) {
    initISR();
  }
  dShotPins[p] |= pinMask;
}

/*
  Set the throttle value and prepare the data packet and store
  pin: pin of motor
  throttle: 11-bit data
*/
uint16_t DShot::setThrottle(uint8_t pin, uint16_t throttle, bool set_telemetry_bit) {
  uint8_t bitMaskPin = digitalPinToBitMask(pin);
  uint8_t p = (digitalPinToPort(pin) == PORT1 ? 0 : 1);
  uint8_t dShotBitsTemp[16] = {0};
  memcpy(dShotBitsTemp, dShotBits[p], sizeof(dShotBitsTemp));

  uint16_t packet = createPacket(throttle, 0);
  uint16_t mask = 0x8000;
  for (byte i = 0; i < 16; i++) {
    if (packet & mask)
      dShotBitsTemp[i] |= bitMaskPin;
    else
      dShotBitsTemp[i] &= ~bitMaskPin;
    mask >>= 1;
  }
  memcpy(dShotBits[p], dShotBitsTemp, sizeof(dShotBitsTemp));

  // do not request telemetry if there are already all slots reserved
  if (set_telemetry_bit && dShotBitsT[p].getCount() < MAX_TELEMETRY_REQUESTS) {
    dShotBitsTemp[11] |= bitMaskPin;  // telemetry request bit
    dShotBitsTemp[15] ^= bitMaskPin;  // crc checksum
    ++telemetryRequests[p];
    ++tlm_req_count;

    noInterrupts();  // to block unwanted states
    telemetryRequestsPort.push(&p);
    dShotBitsT[p].push(&dShotBitsTemp);
    interrupts();
  }

#ifdef DEBUG
  DEBUG DEFINED !  // here to give compile error
      uint8_t dShotBitsSend[2][16] = {0};
  if (telemetryRequestWait == 0) {
    if (telemetryRequestsPort.getCount() > 0) {
      uint8_t port;
      telemetryRequestsPort.pop(&port);
      p = port;
      uint8_t dShotBitsTemp[16] = {0};
      if (port == 0) {
        dShotBitsT[0].pop(&dShotBitsTemp);
        memcpy(dShotBitsSend[1], dShotBits[1], sizeof(dShotBits[1]));
        --telemetryRequests[0];
        memcpy(dShotBitsSend[0], dShotBitsTemp, sizeof(dShotBitsTemp));
      } else {
        memcpy(dShotBitsSend[0], dShotBits[0], sizeof(dShotBits[0]));
        dShotBitsT[1].pop(&dShotBitsTemp);
        memcpy(dShotBitsSend[1], dShotBitsTemp, sizeof(dShotBitsTemp));
        --telemetryRequests[1];
      }
      telemetryRequestWait = TELEMETRY_WAIT;
      --tlm_req_count;
    } else {
      memcpy(dShotBitsSend[0], dShotBits[0], sizeof(dShotBits[0]));
      memcpy(dShotBitsSend[1], dShotBits[1], sizeof(dShotBits[1]));
    }
  } else {
    telemetryRequestWait--;
    memcpy(dShotBitsSend[0], dShotBits[0], sizeof(dShotBits[0]));
    memcpy(dShotBitsSend[1], dShotBits[1], sizeof(dShotBits[1]));
  }
  return dShotBitsSend[p][11] + 200;
#endif
  return 0;
  // for debugging
  return (dShotBitsT[0].getCount() * 10 + dShotBitsT[1].getCount()) * 100 +
         telemetryRequestsPort.getCount();
}

/*
  Send the ESC command immediatly, send motor speed 0 (command 48) afterwards
  pin: pin of motor
  command: 0-48, see Dshot_Digital_Cmd_Spec.txt
*/
uint16_t DShot::sendCommand(uint8_t pin, uint16_t command) {
  uint16_t sleepTime = 0;
  noInterrupts();
  switch (command) {
    case 1 ... 2:
      sleepTime = 260;
      setThrottle(pin, command, 0);
      sendData();
      break;
    case 3 ... 4:
      sleepTime = 280;
      setThrottle(pin, command, 0);
      sendData();
      break;
    case 5:
      sleepTime = 1020;
      setThrottle(pin, command, 0);
      sendData();
      break;
    case 6:
      sleepTime = 12;
      setThrottle(pin, command, 0);
      sendData();
      break;
    case 12:
      sleepTime = 35;
    case 7 ... 10:
    case 13 ... 21:
    case 32 ... 35:
      setThrottle(pin, command, 0);
      sendData();  // not tested yet
      sendData();
      sendData();
      sendData();
      sendData();
      sendData();
      break;
    default:  // sends also not implemented commands once
      setThrottle(pin, command, 0);
  }
  delay(sleepTime);
  setThrottle(pin, 48, 0);  // send motor speed 0 command
  interrupts();
}
