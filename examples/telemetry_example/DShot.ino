#include "DShot.h"
/*
  THIS IS JUST AN EXAMPLE HOW THE CODE WAS USED

  redefine DSHOT_PORT if you want to change the default PORT

  Defaults
  UNO: PORTD, available pins 0-7 (D0-D7)
  Leonardo: PORTB, available pins 4-7 (D8-D11)
            PORTF, available pins A3-A0 

  e.g.
  #define DSHOT_PORT PORTD
*/

// #define DEBUG 1  // Telemetry
// #define DEBUG2 1 
// #define DEBUG3 1 // Current debug
// #define DEBUG4 1

// Port B
#define M1 8
#define M2 9
#define M3 10
#define M4 11
// Port F
#define M5 18
#define M6 19
#define M7 20
#define M8 21

#define NUM_ESC 8
#define LEN_PAYLOAD (2 + NUM_ESC * 2)
#define TIMEOUT 15000  // ms (default was 100 if connected to program)

// #define ANALOG_CURRENT  // To add the currents in the telemetry message

// Default values for DShot outputs
uint16_t const disarmCommand[] = {48, 48, 48, 48, 48, 48, 48, 48};
uint16_t const ARMCommand[] = {48, 48, 48, 48, 48, 48, 48, 48};
uint16_t const minDShot = 48;  // without commands: 48, with commands: 1 (0 not implemented)
uint16_t const maxDShot = 2047;
uint16_t currThrottle[NUM_ESC];
bool currTelemetry[NUM_ESC];
bool const noTelemetry[] = {0, 0, 0, 0, 0, 0, 0, 0};

#ifdef ANALOG_CURRENT
int analogPin1 = A4;  // current measurements from ESCs
int analogPin2 = A5;
#endif

byte receivedIx = 0;
byte receivedBytes[LEN_PAYLOAD];
bool dataValid = false;

bool armed = false;
bool throttleValid;
unsigned long lastValidMessage = 0;

DShot esc(DShot::Mode::DSHOT600);

void setup() {
  Serial.begin(115200);  // communicating over usb

  Serial1.begin(115200);  // Communication with ESC Telemetry

#ifdef ANALOG_CURRENT
  pinMode(analogPin1, INPUT);
  pinMode(analogPin2, INPUT);
  analogReference(INTERNAL);  // INTERNAL: 2.5V (DEFAULT = 5V)
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // LED On until first serial message received

  while (!Serial.available())
    ;

  // Notice, all pins must be connected to same PORT if possible
  esc.attach(M1);
  esc.setThrottle(M1, disarmCommand[0], 0);
  esc.attach(M2);
  esc.setThrottle(M2, disarmCommand[0], 0);
  esc.attach(M3);
  esc.setThrottle(M3, disarmCommand[0], 0);
  esc.attach(M4);
  esc.setThrottle(M4, disarmCommand[0], 0);
  esc.attach(M5);
  esc.setThrottle(M5, disarmCommand[0], 0);
  esc.attach(M6);
  esc.setThrottle(M6, disarmCommand[0], 0);
  esc.attach(M7);
  esc.setThrottle(M7, disarmCommand[0], 0);
  esc.attach(M8);
  esc.setThrottle(M8, disarmCommand[0], 0);

  // for visual reference
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  /* 
  // to check if the ports are correct
  if(digitalPinToPort(M5) == PORT2){
    digitalWrite(LED_BUILTIN, HIGH);
  }
  */
}

void receiveData() {
  // parse serial code - super simple protocol 0x26 followed by NUM_ESC*2 bytes and checksum (xor of
  // all bytes) Magic pattern NUM_ESC* [0xAA 0xAA] arms the system every value between minDShot and
  // maxDShot (interpreted as unsigned short -> 0x3E8 until 0x7D0) is a setpoint. MSB is telemetry
  // request bit per ESC channel (e.g. 0x80 0x30 is telemetry request and motor off)

  while (Serial.available()) {
    receivedBytes[receivedIx] = Serial.read();

    if (receivedBytes[0] != 0x26) {
      // nonsense received. Do nothing.
      receivedIx = 0x00;
    }
    // last byte
    else if (receivedIx == (LEN_PAYLOAD - 1)) {
      // parse and check XOR of payload
      byte checksum = receivedBytes[1];
      for (int i = 2; i < (LEN_PAYLOAD - 1); i++) {
        checksum ^= receivedBytes[i];
      }
      if (receivedBytes[LEN_PAYLOAD - 1] == checksum) {
        lastValidMessage = millis();
        dataValid = true;
#ifdef DEBUG4
        // for (int i = 0; i < LEN_PAYLOAD - 1; ++i) {  // last byte is just checksum
        //   Serial.print(receivedBytes[receivedIx + 1 - LEN_PAYLOAD + i], HEX);
        //   Serial.print(" ");
        // }
        Serial.print("dataValid");
#endif
      }
      // reset ptr
      receivedIx = 0;
      return;
    } else {
      receivedIx++;
    }
  }
}

const inline bool timeOutReached() { return (millis() - TIMEOUT > lastValidMessage); }

bool dataIsArmingSequence() {
  // check each payload byte
  for (int i = 1; i < LEN_PAYLOAD - 1; i++) {
    // if any byte is not the magic value, we don't
    // have an arming sequence.
    if (receivedBytes[i] != 0xAA) {
      return false;
    }
  }
  return true;
}

bool dataIsDisarmingSequence() {
  // check each payload byte
  for (int i = 1; i < LEN_PAYLOAD - 1; i++) {
    // if any byte is not the magic value, we don't
    // have an arming sequence.
    if (receivedBytes[i] != 0xFF) {
      return false;
    }
  }
  return true;
}

bool updateCurrThrottleFromData() {
  uint16_t currThrottleTemp[NUM_ESC];
  bool telemetryTemp[NUM_ESC];

  for (int i = 0; i < NUM_ESC; i++) {
    int payloadIx = i * 2 + 1;
    currThrottleTemp[i] =
        (((uint16_t)(receivedBytes[payloadIx + 1] & 0x7F)) << 8) | receivedBytes[payloadIx];

    telemetryTemp[i] = receivedBytes[payloadIx + 1] >> 7;

    // check if invalid
    if (currThrottleTemp[i] < minDShot || currThrottleTemp[i] > maxDShot) {
      return false;
    }
  }

  // if we made it to here, all throttles are valid!
  memcpy(currThrottle, currThrottleTemp, sizeof(currThrottle[0]) * NUM_ESC);
  memcpy(currTelemetry, telemetryTemp, sizeof(currTelemetry[0]) * NUM_ESC);
  return true;
}

void finishData() {
  if (dataValid) {
    receivedIx = 0;
    dataValid = false;
  }
}

void arm() {
  armed = true;
  memcpy(currThrottle, ARMCommand, sizeof(currThrottle[0]) * NUM_ESC);
  digitalWrite(LED_BUILTIN, HIGH);  // LED on when armed
}

void disarm() {
  armed = false;
  memcpy(currThrottle, disarmCommand, sizeof(currThrottle[0]) * NUM_ESC);
  digitalWrite(LED_BUILTIN, LOW);  // LED off when disarmed
}

void updateESC() {
#ifndef DEBUG

  // test setup
  esc.setThrottle(M1, (armed ? currThrottle[4] : disarmCommand[0]), currTelemetry[4]);
  esc.setThrottle(M2, (armed ? currThrottle[5] : disarmCommand[0]), currTelemetry[5]);
  esc.setThrottle(M3, (armed ? currThrottle[6] : disarmCommand[0]), currTelemetry[6]);
  esc.setThrottle(M4, (armed ? currThrottle[7] : disarmCommand[0]), currTelemetry[7]);

  esc.setThrottle(M8, (armed ? currThrottle[0] : disarmCommand[0]), currTelemetry[0]);
  esc.setThrottle(M7, (armed ? currThrottle[1] : disarmCommand[0]), currTelemetry[1]);
  esc.setThrottle(M6, (armed ? currThrottle[2] : disarmCommand[0]), currTelemetry[2]);
  esc.setThrottle(M5, (armed ? currThrottle[3] : disarmCommand[0]), currTelemetry[3]);

#else
  if(currTelemetry[0] == 1){ Serial.print("########## NEW TELEMETRY :) ###########"); }
  Serial.print("telemetry requests: ");
  Serial.print("0:");
  Serial.print(esc.setThrottle(M5, (armed ? currThrottle[0] : disarmCommand[0]), currTelemetry[0]), DEC);
  Serial.print(" 1:");
  Serial.print(esc.setThrottle(M3, (armed ? currThrottle[1] : disarmCommand[0]), currTelemetry[1]), DEC);
  Serial.print(" 2:");
  Serial.print(esc.setThrottle(M2, (armed ? currThrottle[2] : disarmCommand[0]), currTelemetry[2]), DEC);
  Serial.print(" 3:");
  Serial.print(esc.setThrottle(M7, (armed ? currThrottle[3] : disarmCommand[0]), currTelemetry[3]), DEC);
  Serial.print(" 4:");
  Serial.print(esc.setThrottle(M6, (armed ? currThrottle[4] : disarmCommand[0]), currTelemetry[4]), DEC);
  Serial.print(" 5:");
  Serial.print(esc.setThrottle(M4, (armed ? currThrottle[5] : disarmCommand[0]), currTelemetry[5]), DEC);
  Serial.print(" 6:");
  Serial.print(esc.setThrottle(M1, (armed ? currThrottle[6] : disarmCommand[0]), currTelemetry[6]), DEC);
  Serial.print(" 7:");
  Serial.println(esc.setThrottle(M8, (armed ? currThrottle[7] : disarmCommand[0]), currTelemetry[7]), DEC);
// for(int i = 0; i < 8; ++i){
//   if(currTelemetry[i] == 1){
//     Serial.print("telemetry request on:");
//     Serial.print(i, DEC);
//     // Serial.print("]: ");
//     // Serial.println(tr, DEC);
//   }
// }
#endif  // DEBUG
}

const inline uint8_t crc8(uint8_t* buffer, uint8_t BufLen) {
  uint8_t remainder = 0;
  const uint8_t polynomial = 0x07;
  uint8_t byte, bit;

  for (byte = 0; byte < BufLen; ++byte) {
    remainder ^= buffer[byte];
    for (bit = 0; bit < 8; ++bit) {
      if (remainder & 0x80) {
        remainder = (remainder << 1) ^ polynomial;
      } else {
        remainder = (remainder << 1);
      }
    }
  }
  return remainder;  // returns zero if crc check is passed
}

void loop() {
  receiveData();

  // new data package and we're armed.
  if (armed && dataValid) {
    if (dataIsDisarmingSequence()) {
      disarm();
    } else {
      // try to update current throttle.
      throttleValid = updateCurrThrottleFromData();
    }
    if (!throttleValid) {
      disarm();

#ifdef DEBUG2  // if something was invalid, we print it out on the serial.
      Serial.print("Invalid data: ");
      Serial.print(receivedBytes[1], HEX);
      for (uint8_t i = 1; i < LEN_PAYLOAD - 1; i += 2) {
        Serial.print(receivedBytes[i], HEX);
        Serial.print(receivedBytes[i + 1], HEX);
        Serial.print(", ");
      }
      Serial.println(receivedBytes[LEN_PAYLOAD - 1], HEX);
#endif
    }
  }
  // new data package while we're not armed
  else if (!armed && dataValid) {
    // check if we should arm
    if (dataIsArmingSequence()) {
      arm();
    }
  } else  // no new data
  {
    // check if we reached timeout
    if (armed && timeOutReached()) {
      disarm();
    }
  }

  // update current state to ESCs
  updateESC();
  memcpy(currTelemetry, noTelemetry, sizeof(noTelemetry[0]) * NUM_ESC);  // reset telemetry request

  // finish data if it was valid
  finishData();


  unsigned char serial_bridge = 0;
// send telemetry back to computer
#ifndef ANALOG_CURRENT  // the telemetry frame can just be forwarded as it is

  for (; Serial1.available() > 0 && serial_bridge < 10; serial_bridge++) {
    Serial.write(Serial1.read());
  }

#else  // add current measurements in telemetry frame (does not work properly yet)

  uint8_t buffer[50];
  uint8_t n = 0;

  if (Serial1.available() >= 10) {
    for (; Serial1.available() > 0 && serial_bridge < 50; serial_bridge++) {
      buffer[serial_bridge] = Serial1.read();
      ++n;

#ifdef DEBUG3
      Serial.print(buffer[serial_bridge], HEX);
      Serial.print(" ");
#endif
    }

    uint8_t startindex = 0;

    while (n > 10 && crc8(buffer[startindex], 10)) {
#ifdef DEBUG3
      Serial.print("CRC=");
      Serial.print(crc8(buffer[startindex], 10), HEX);
#endif
      ++startindex;
      --n;
    }
    if (!crc8(buffer[startindex], 10)) {  // if it is a correct telemetry frame:

      // Total current from both ESCs:
      // Current : 15.2mv/A
      // analogRead: range 0-1023 for 0V to 2.53V (measured)
      // Current in A = analogread * (2530 mV) /1023 / (15.2 mV/A) (= analogread * 0.16270515)
      double current = 0.16270515 * (analogRead(analogPin1) + analogRead(analogPin2));

      // send current in correct frame (our used ESCs send current only analog)
      buffer[3 + startindex] = (uint16_t)(current * 100.) >> 8;
      buffer[4 + startindex] = (uint16_t)(current * 100.) & 0xFF;

      // send changed CRC8
      buffer[9 + startindex] = crc8(buffer[startindex], 9);

#ifdef DEBUG3
      Serial.println("Buffer: ");
      for (int i = 0; i < 10; ++i) {
        Serial.print(buffer[i + startindex], HEX);
        Serial.print(" ");
      }
      Serial.println("end.");
#else

      for (uint8_t i = 0; i < 10; i++) {
        Serial.write(buffer[i + startindex]);
      }
#endif
    }

    // empty buffer for next reading
    while (Serial1.available()) {
      Serial1.read();
    }
  }
#endif  // ANALOG_CURRENT
}

/* Telemetry info from https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
ESC Telemetry
  Byte 0: Temperature
  Byte 1: Voltage high byte
  Byte 2: Voltage low byte
  Byte 3: Current high byte
  Byte 4: Current low byte
  Byte 5: Consumption high byte
  Byte 6: Consumption low byte
  Byte 7: Rpm high byte
  Byte 8: Rpm low byte
  Byte 9: 8-bit CRC

Converting the received values to standard units
  int8_t Temperature = Temperature in 1 degree C
  uint16_t Voltage = Volt *100 so 1000 are 10.00V
  uint16_t Current = Ampere * 100 so 1000 are 10.00A
  uint16_t Consumption = Consumption in 1mAh
  uint16_t ERpm = Electrical Rpm /100 so 100 are 10000 Erpm

  note: to get the real Rpm of the motor you will need to divide the Erpm result
  by the magnetpole count divided by two.

  So with a 14magnetpole motor:
  Rpm = Erpm/7
  rpm = erpm / (motor poles/2)
*/
