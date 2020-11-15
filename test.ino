#include "FlexCAN_T4.h"

FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> can;

void setup() {
  // put your setup code here, to run once:
 pinMode(14,OUTPUT);
 can.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(14,LOW);
  delay(50);
  digitalWrite(14,HIGH);
  delay(50);
}
