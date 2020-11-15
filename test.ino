#include "FlexCAN_T4.h"

FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> can;
CAN_message_t buf;

void setup() {
  // put your setup code here, to run once:
 pinMode(13,OUTPUT);
 can.begin();
 can.setBaudRate(1000000);
 buf.id = 0x1b;
 buf.len = 8;
 buf.buf[0] = 0x13;
 buf.buf[1] = 0x13;
 buf.buf[2] = 0x13;
 buf.buf[3] = 0x13;
 buf.buf[4] = 0x13;
 buf.buf[5] = 0x13;
 buf.buf[6] = 0x13;
 buf.buf[7] = 0x13;
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(13,LOW);
  delay(50);
  digitalWrite(13,HIGH);
  delay(50);
  can.write(buf);
}
