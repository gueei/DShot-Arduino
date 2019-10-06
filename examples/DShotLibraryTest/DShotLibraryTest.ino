#include <DShot.h>

DShot esc(1);

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial.available()>0){
    uint16_t throttle = Serial.parseInt();
    uint16_t packet = esc.setThrottle(throttle);
    Serial.print(throttle, HEX);
    Serial.print("\t");
    Serial.println(packet, HEX);
  }
}
