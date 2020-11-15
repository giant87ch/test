#include <MsTimer2.h>
#include "ESC_DJI.h"

ESC_DJI motor;

// CAN_message_t buf;
// CAN_message_t msg;

void timerInt(){
  motor.getCanData();
  Serial.println(motor.wEscData[0].angle);
}

void setup() {
  // put your setup code here, to run once:
 pinMode(13,OUTPUT);

 Serial.begin(115200);

 MsTimer2::set(1,timerInt);
 MsTimer2::start();

 motor.init();
 motor.setMotorMaxSpeed(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(13,LOW);
  delay(50);
  digitalWrite(13,HIGH);
  delay(50);
}
