#include <DShot.h>

/*

redefine DSHOT_PORT if you want to change the default PORT

Defaults
UNO: PORTD, available pins 0-7 (D0-D7)
Leonardo: PORTB, available pins 4-7 (D8-D11)

e.g.
#define DSHOT_PORT PORTD
*/
DShot esc1(DShot::Mode::DSHOT300);

uint16_t throttle = 0;
uint16_t target = 0;

void setup() {
  Serial.begin(115200);

  // Notice, all pins must be connected to same PORT
  esc1.attach(7);  
  esc1.setThrottle(throttle);
}

void loop() {
  if (Serial.available()>0){
    target = Serial.parseInt();
    if (target>2047)
      target = 2047;
    Serial.print(target, HEX);
    Serial.print("\t");
  }
  if (throttle<48){
    throttle = 48;
  }
  if (target<=48){
    esc1.setThrottle(target);
  }else{
    if (target>throttle){
      throttle ++;
      esc1.setThrottle(throttle);
    }else if (target<throttle){
      throttle --;
      esc1.setThrottle(throttle);
    }
  }
  delay(10);
}
