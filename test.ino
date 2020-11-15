#include <MsTimer2.h>
#include "ESC_DJI.h"

ESC_DJI motor;
int motorCurrent[4] = {0};

// CAN_message_t buf;
// CAN_message_t msg;

void setMotorRpm(int id ,int targetRpm){
  int p = 25;//set GAIN
  motorCurrent[id] = p*(targetRpm-motor.wEscData[id].rotation);
  motorCurrent[id] = constrain(motorCurrent[id],-10000,10000);
}

void setMotorPos(int id ,int targetPos){
  int p = 2;//set GAIN
  int rpm = 0;
  rpm = p*(targetPos-motor.exEscData[id].pos);
  setMotorRpm(id,rpm);
}


void timerInt(){
  // motor.getCanData();
  // Serial.println(motor.wEscData[0].angle);
  // motorCurrent[0] = 1000;
  // motor.driveWheel(motorCurrent);

  motor.getCanData();
  // setMotorRpm(0,600);
  setMotorPos(0,20000);
  motor.driveWheel(motorCurrent);
  Serial.println(motor.exEscData[0].pos);
  // Serial.print(", ");
  // Serial.println(motorCurrent[0]);
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
