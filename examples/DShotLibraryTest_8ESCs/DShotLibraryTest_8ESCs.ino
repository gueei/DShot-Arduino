#include "DShot.h"
/*
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
#define TIMEOUT 15000 // ms (default was 100 if connected to program)
#define MAGNET_POLES 14 // Poles per Motor (needed for telemetry rpm calculations)

// #define ANALOG_CURRENT  // To add the currents in the telemetry message

// Default values for DShot outputs
uint16_t const minDShot = 48; // without commands: 48, with commands: 1 (0 not implemented)
uint16_t const disarmCommand = 48; // without commands: 48, with commands: 1 (0 not implemented)
uint16_t const maxDShot = 2047;

#ifdef ANALOG_CURRENT
int analogPin1 = A4; // current measurements from ESCs
int analogPin2 = A5;
#endif

byte receivedIx = 0;
uint16_t receivedBytes[LEN_PAYLOAD];
bool dataValid = false;

bool armed = false;
bool throttleValid;
unsigned long lastValidMessage = 0;

uint16_t target = 0;
uint16_t throttle = 0;

DShot esc(DShot::Mode::DSHOT600);

void setup() {
  Serial.begin(115200); // communicating over usb

  Serial1.begin(115200); // Communication with ESC Telemetry

#ifdef ANALOG_CURRENT
  pinMode(analogPin1, INPUT);
  pinMode(analogPin2, INPUT);
  analogReference(INTERNAL); // INTERNAL: 2.5V (DEFAULT = 5V)
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED On until first serial message received

  while (!Serial.available())
    ;

  while (!Serial1.available())
    Serial1.read();

  // Notice, all pins must be connected to same PORT if possible
  esc.attach(M1);
  esc.setThrottle(M1, disarmCommand, 0);
  esc.attach(M2);
  esc.setThrottle(M2, disarmCommand, 0);
  esc.attach(M3);
  esc.setThrottle(M3, disarmCommand, 0);
  esc.attach(M4);
  esc.setThrottle(M4, disarmCommand, 0);
  esc.attach(M5);
  esc.setThrottle(M5, disarmCommand, 0);
  esc.attach(M6);
  esc.setThrottle(M6, disarmCommand, 0);
  esc.attach(M7);
  esc.setThrottle(M7, disarmCommand, 0);
  esc.attach(M8);
  esc.setThrottle(M8, disarmCommand, 0);

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

void setAllThrottles(uint16_t throttle, bool telemetry) {
  esc.setThrottle(M1, throttle, telemetry);
  esc.setThrottle(M2, throttle, telemetry);
  esc.setThrottle(M3, throttle, telemetry);
  esc.setThrottle(M4, throttle, telemetry);
  esc.setThrottle(M5, throttle, telemetry);
  esc.setThrottle(M6, throttle, telemetry);
  esc.setThrottle(M7, throttle, telemetry);
  esc.setThrottle(M8, throttle, telemetry);
}

void loop() {
  if (Serial.available() > 0) {
    target = Serial.parseInt();

    if (target < 0) {
      target = 48;
    }

    if (target > 2047) // safety measure, disarm when wrong input
      target = 0;
    Serial.print(target, DEC);
    Serial.print("\t");
    Serial.print(throttle, DEC);
    Serial.print("\n");
  }

  if (throttle < minDShot) { // special commands disabled
    throttle = 48;
  }
  if (target <= 48) {
    setAllThrottles(target, 0);
    if (target == 0)
      throttle = 48;
  } else {
    if (target > throttle) {
      throttle++;
      setAllThrottles(throttle, 0);
    } else if (target < throttle) {
      throttle--;
      setAllThrottles(throttle, 0);
    }
  }

  readtelemetry();

  delay(10);
}

const inline uint8_t crc8(uint8_t *buffer, uint8_t BufLen) {
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
  return remainder; // returns zero if crc check is passed
}

void readtelemetry() {
  unsigned char serial_bridge = 0;
  // send decoded telemetry back to computer (may not work properly!)

  uint8_t buffer[50];
  uint8_t n = 0;

  if (Serial1.available() >= 10) {
    for (; Serial1.available() > 0 && serial_bridge < 50; serial_bridge++) {
      buffer[serial_bridge] = Serial1.read();
      ++n;
    }

    uint8_t startindex = 0;

    while (n > 10 && crc8(buffer[startindex], 10)) {
      ++startindex;
      --n;
    }
    if (!crc8(buffer[startindex], 10)) { // if it is a correct telemetry frame:

#ifndef ANALOG_CURRENT
      Serial.print("Temperature: ");
      Serial.print(buffer[startindex]);
      Serial.println(" °C");
      Serial.print("Voltage: ");
      Serial.print((buffer[startindex + 1] << 8 | buffer[startindex + 2]) / 100);
      Serial.println(" V");
      Serial.print("Current: ");
      Serial.println((buffer[startindex + 3] << 8 | buffer[startindex + 4]) / 100);
      Serial.println(" A");
      Serial.print("Consumption: ");
      Serial.print(buffer[startindex + 5] << 8 | buffer[startindex + 6]);
      Serial.println(" mAh");
      Serial.print("Velocity: ");
      Serial.print((buffer[startindex + 7] << 8 | buffer[startindex + 8]) * 200 / MAGNET_POLES);
      Serial.println(" rpm");
      Serial.println("");

#else
      // Total current from both ESCs:
      // Current : 15.2mv/A (datasheet ESC)
      // analogRead: range 0-1023 for 0V to 2.53V (measured)
      // Current in A = analogread * (2530 mV) /1023 / (15.2 mV/A) (= analogread
      // * 0.16270515)
      double current =
          0.16270515 * (analogRead(analogPin1) + analogRead(analogPin2));

      Serial.print("Temperature: ");
      Serial.print(buffer[startindex]);
      Serial.println(" °C");
      Serial.print("Voltage: ");
      Serial.print((buffer[startindex + 1] << 8 | buffer[startindex + 2]) / 100);
      Serial.println(" V");
      Serial.print("Current: ");
      Serial.print(current);
      Serial.println(" A");
      Serial.print("Consumption: ");
      Serial.print(buffer[startindex + 5] << 8 | buffer[startindex + 6]);
      Serial.println(" mAh");
      Serial.print("Velocity: ");
      Serial.print((buffer[startindex + 7] << 8 | buffer[startindex + 8]) * 200 / MAGNET_POLES);
      Serial.println(" rpm");
      Serial.println("");

#endif // ANALOG_CURRENT
    }

    // empty buffer for next reading
    while (Serial1.available()) {
      Serial1.read();
    }
  }
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
