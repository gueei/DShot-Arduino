#include <DShot.h>

/*

redefine DSHOT_PORT if you want to change the default PORT

Defaults
UNO: PORTD, available pins 0-7 (D0-D7)
Leonardo: PORTB, available pins 4-7 (D8-D11)

e.g.
#define DSHOT_PORT PORTD
*/
DShot esc1, esc2;

void setup() {
  Serial.begin(115200);

  // Notice, all pins must be connected to same PORT
  esc1.attach(7);
  esc2.attach(5);
  esc1.setThrottle(0);
  esc2.setThrottle(0);
}

void loop() {
  if (Serial.available()>0){
    uint16_t throttle = Serial.parseInt();
    uint16_t packet = esc1.setThrottle(throttle);
    Serial.print(throttle, HEX);
    Serial.print("\t");
    Serial.println(packet, HEX);
  }
}
