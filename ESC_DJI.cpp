#include <Arduino.h>
#include "ESC_DJI.h"
#include "FlexCAN_T4.h"

FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> CANbus;

//********************************************************************************
// DJI ESC 制御クラス
//********************************************************************************
ESC_DJI::ESC_DJI(void){

}

//********************************************************************************
// 初期化
//********************************************************************************
void ESC_DJI::init(){
  CANbus.begin();   //CAN通信開始
  CANbus.setBaudRate(1000000);
}

//********************************************************************************
// ESCにモーターの回転速度の指令値を送信(電流値を入力)
//********************************************************************************
void ESC_DJI::driveWheel(int u[4]) {
  msg.len = 8;
  msg.id = 0x200; //走行用モーターの制御用CAN通信
  for ( int idx = 0; idx < msg.len; ++idx ) {
    msg.buf[idx] = 0;
  }

  //緊急停止(脱力) でない場合
  if (flgStop == false && flgNoCanData == false) {
    for (int i = 0; i < 4; i++) {

      u[i] = max(-limitSpeedWheel, min(limitSpeedWheel, u[i]));
      msg.buf[i * 2] = u[i] >> 8;
      msg.buf[i * 2 + 1] = u[i] & 0xFF;
    }
  } else {
    //    printspeed("緊急停止");
  }
  CANbus.write(msg);
}

//********************************************************************************
// ジンバル制御(電流値を入力)
//********************************************************************************
void ESC_DJI::driveGimbal(int pitchAmpere, int yawAmpere, int loaderAmpere, int coverAmpere) {
  int val;

  //limits
  pitchAmpere = max(-limitSpeedGimbal, min(limitSpeedGimbal, pitchAmpere));
  yawAmpere = max(-limitSpeedGimbal, min(limitSpeedGimbal, yawAmpere));
  loaderAmpere = max(-limitSpeedGimbal, min(limitSpeedGimbal, loaderAmpere));
  coverAmpere = max(-limitSpeedGimbal, min(limitSpeedGimbal, coverAmpere));

  msg.len = 8;
  msg.id = 0x1FF; //ヘッド用モーターの制御用CAN通信
  for ( int idx = 0; idx < msg.len; ++idx ) {
    msg.buf[idx] = 0;
  }

  //緊急停止(脱力) でない場合
  if (flgStop == false && flgNoCanData == false) {
    val = yawAmpere;
    msg.buf[0] = val >> 8;  //Yaw
    msg.buf[1] = val & 0x00ff;

    val = pitchAmpere;
    msg.buf[2] = val >> 8; //Pitch
    msg.buf[3] = val & 0x00ff;

    val = loaderAmpere;
    msg.buf[4] = val >> 8; //Loader
    msg.buf[5] = val & 0x00ff;

    val = coverAmpere;
    msg.buf[6] = val >> 8; //Cover
    msg.buf[7] = val & 0x00ff;
  }

  //CAN受信データが正常なときのみジンバルを動かす
  if (flgNoCanData == false) {
    CANbus.write(msg);
  }

}


//********************************************************************************
// CANデータ受信
//********************************************************************************
void ESC_DJI::getCanData() {
  static int canCnt;
  static int lastGimbalYaw = -1;

  while ( CANbus.read(rxmsg) ) {
    // hexDump( sizeof(rxmsg.buf), (uint8_t *)rxmsg.buf );
    // writeSerial(rxmsg);
    if(rxmsg.id == 0x201 || rxmsg.id == 0x202 ||
    rxmsg.id == 0x203 || rxmsg.id == 0x204){
      getESCCanData(rxmsg);//　CANデータ受信(Wheel用ESCデータ)
      canCnt = 0;
    }else if(rxmsg.id == 0x205){
      gimbalYaw = rxmsg.buf[0] * 256 + rxmsg.buf[1];
      if (lastGimbalYaw != -1) {
        int diff;
        diff = gimbalYaw - lastGimbalYaw;
        if (diff < -(encoderOneAround / 2)){
          diff += encoderOneAround;
        }else if ((encoderOneAround / 2) < diff){
          diff -= encoderOneAround;
        }
        generalGimbalYaw += diff;

      } else {      //初回のみ実行
        generalGimbalYaw = gimbalYaw - yawNeutral;
        if (generalGimbalYaw > (encoderOneAround / 2)) {
          generalGimbalYaw -= encoderOneAround;
        }
      }
      lastGimbalYaw = gimbalYaw;
      canCnt = 0;
    }else if(rxmsg.id == 0x206){
      gimbalPitch = rxmsg.buf[0] * 256 + rxmsg.buf[1];
      canCnt = 0;
    }else if(rxmsg.id == 0x207 || rxmsg.id == 0x208){
      getC610Data(rxmsg); //CANデータ受信(Head用ESCデータ)
      canCnt = 0;
    }

  }

  //CAN受信エラーカウンタで分岐
  if (canCnt == 0) {  //正常受信
    flgNoCanData = false;
    canCnt += 1;
  } else if (canCnt < 16) { //エラーだが許容範囲
    canCnt += 1;
  } else {             //CANデータ受信途絶のとき緊急停止。
    flgNoCanData = true;
  }
}

//********************************************************************************
//　CANデータ受信(Wheel用ESCデータ)
//********************************************************************************
void ESC_DJI::getESCCanData(CAN_message_t m) {
  static int id;
  id = m.id;
  if ( id < 0x201 || id > 0x204) { //範囲外
    return;
  }

  id = id - 0x201;  //識別子から添え字に変換
  wEscData[id].angle    = m.buf[0] * 256 + m.buf[1];
  wEscData[id].rotation = m.buf[2] * 256 + m.buf[3];
  wEscData[id].torque   = m.buf[4] * 256 + m.buf[5];
  wEscData[id].temp     = m.buf[6];
  sensRotation[id] = wEscData[id].rotation;

  getExpData(id);
}

//********************************************************************************
//　ESC拡張データ(ESCのID添え字(0～3))
//********************************************************************************
void ESC_DJI::getExpData(int id) {
  static const float LV = -10000;   //LowValue 初期値
  static int deff;
  static float lastangle[4] = {LV, LV, LV, LV};

  if( flgNoCanData == false){
    //CANデータが読めている場合、angle値より frq(回転回数)とpos(累積角)を計算する
    if( lastangle[id] == LV ){  //初回
      exEscData[id].frq = 0;
      exEscData[id].pos = wEscData[id].angle;
    }else{
      deff = wEscData[id].angle - lastangle[id];
      if (deff < -(encoderOneAround / 2)) {
        exEscData[id].frq++;
      }else if ((encoderOneAround / 2) < deff) {
        exEscData[id].frq--;
      }
      exEscData[id].pos = (exEscData[id].frq * encoderOneAround + wEscData[id].angle);
    }
    lastangle[id] = wEscData[id].angle;
  }
}

//********************************************************************************
// wheelESCDataを出力する
//********************************************************************************
void ESC_DJI::printESCData(void){
  int id;
  Serial.print("ESCData  ");
  for (id = 0; id < 4; id++){
    Serial.print("[ ");
    Serial.print(id);
    Serial.print(" ] =");
    Serial.print(wEscData[id].angle);
    Serial.print(", ");
    Serial.print(wEscData[id].rotation);
    Serial.print(", ");
    Serial.print(wEscData[id].torque);
    Serial.print(", ");
    Serial.print(wEscData[id].temp);
    Serial.print(",   ");
    Serial.print("\t");
  }
  Serial.println("");
}

//********************************************************************************
// 16進データを出力する
//********************************************************************************
void ESC_DJI::hexDump(uint8_t dumpLen, uint8_t *bytePtr){
  uint8_t hex[17] = "0123456789abcdef";
  uint8_t working;
  while ( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working >> 4 ] );
    Serial.write( hex[ working & 15 ] );
    Serial.write( ' ' );
  }
  Serial.write('\r');
  Serial.write('\n');
}

//********************************************************************************
// CAN通信内容を出力する
//********************************************************************************
void ESC_DJI::writeSerial(CAN_message_t rxmsg) {
  String wstr = "";
  wstr = String(rxmsg.id);
  writeString(wstr);
  Serial.write(' ');
  hexDump( sizeof(rxmsg.buf), (uint8_t *)rxmsg.buf );
  Serial.write(rxmsg.buf[0]);
  if (rxmsg.id == 0x206 ) {
    Serial.write('\r');
    Serial.write('\n');
  }

}

//********************************************************************************
// 文字列を出力する
//********************************************************************************
void ESC_DJI::writeString(String stringData) { // Used to serially push out a String with Serial.write()
  for (unsigned int i = 0; i < stringData.length(); i++){
    Serial.write(stringData[i]);   // Push each char 1 by 1 on each loop pass
  }
}

//********************************************************************************
// ジンバルYaw軸ニュートラル角度セット(角度)
//********************************************************************************
void ESC_DJI::setYawNeutral(int yawNeutralInput){
  yawNeutral = yawNeutralInput;
}

//********************************************************************************
// モーターの速度制限の設定(0～100(%))
//********************************************************************************
void ESC_DJI::setMotorMaxSpeed(long rate){
  rate = constrain(rate,0,100);
  //走行用モータ制限速度:-16,384～0～16,384(0xC000～0x4000)
  limitSpeedWheel = map(rate,0,100,0,16384);
}

//********************************************************************************
// モーターの制限速度取得
//********************************************************************************
long ESC_DJI::getMotorMaxSpeed(void){
  return limitSpeedWheel;
}

//********************************************************************************
// ジンバルモーターの速度制限の設定(0～100(%))
//********************************************************************************
void ESC_DJI::setGimbalMaxSpeed(long rate){
  rate = constrain(rate,0,100);
  //走行用モータ制限速度:-16,384～0～16,384(0xC000～0x4000)
  limitSpeedGimbal = map(rate,0,100,0,16384);
}

//********************************************************************************
// 拡張データの表示
//********************************************************************************
void ESC_DJI::printExpData(void){
  int id;
  Serial.print("ESCDataEx  ");
  for (id = 0; id < 4; id++){
    Serial.print("[ ");
    Serial.print(id);
    Serial.print(" ] =");
    Serial.print(exEscData[id].frq);
    Serial.print(", ");
    Serial.print(exEscData[id].pos);
    Serial.print("\t");
  }
  Serial.println("");
}


//********************************************************************************
//　CANデータ受信(Head用ESCデータ)
//********************************************************************************
void ESC_DJI::getC610Data(CAN_message_t m) {
  static int id;
  id = m.id;
  if ( id < 0x207 || id > 0x208) { //範囲外
    return;
  }

  id = id - 0x207;  //識別子から添え字に変換
  c610Data[id].angle    = m.buf[0] * 256 + m.buf[1];
  c610Data[id].rotation = m.buf[2] * 256 + m.buf[3];
  c610Data[id].torque   = m.buf[4] * 256 + m.buf[5];
  getExpC610Data(id);
  loaderRotation = c610Data[0].rotation;
  coverAngle = exC610Data[1].pos;
}

//********************************************************************************
//　ESC拡張データ(Head用)(ESCのID添え字(0～1))
//********************************************************************************
void ESC_DJI::getExpC610Data(int id) {
  static const float LV = -10000;   //LowValue 初期値
  static int deff;
  static float lastangle[2] = {LV, LV};
  if( flgNoCanData == false){
    //CANデータが読めている場合、angle値より frq(回転回数)とpos(累積角)を計算する
    if( lastangle[id] == LV ){  //初回
      exC610Data[id].frq = 0;
      exC610Data[id].pos = c610Data[id].angle;
    }else{
      deff = c610Data[id].angle - lastangle[id];
      if (deff < -(encoderOneAround / 2)) {
        exC610Data[id].frq++;
      }else if ((encoderOneAround / 2) < deff) {
        exC610Data[id].frq--;
      }
      exC610Data[id].pos = (long)(exC610Data[id].frq * encoderOneAround) + (long)(c610Data[id].angle);
    }
    lastangle[id] = c610Data[id].angle;
  }
}
