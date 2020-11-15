// #pragma once
#ifndef ESC_DJI_h
#define ESC_DJI_h
#include <Arduino.h>
#include "FlexCAN_T4.h"

//クラスの定義
//クラス名・コンストラクタ名・関数名や使用する変数名を定義します。
class ESC_DJI{
    public:
        ESC_DJI();
        void init();
        void getCanData();
        void driveWheel(int u[4]);
        void driveGimbal(int pitchAmpere, int yawAmpere, int loaderAmpere, int coverAmpere);
        //CAN受信データ途絶時にTrue
        bool flgNoCanData = false;
        //緊急停止時にTrue
        bool flgStop = false;
        typedef struct{
            int16_t angle;
            int16_t rotation;
            int16_t torque;
            short temp;
        } M3508Data;
        //M3508のデータ
        M3508Data wEscData[4];
        void printESCData();
        void setYawNeutral(int yawNeutral);
        void setMotorMaxSpeed(long rate);    //0 ~ 100%
        void setGimbalMaxSpeed(long rate);    //0 ~ 100%
        long getMotorMaxSpeed(void);

        //現在のヨー角度
        int gimbalYaw = -1;
        //現在のヨー角度(累積値)
        int generalGimbalYaw = 0;
        //現在のピッチ角度
        int gimbalPitch = -1;
        //シャーシモータの角度
        int16_t sensRotation[4];
        //ローダーモータの角度
        int16_t loaderRotation = 0;
        //カバーモータの角度
        long coverAngle = 0;

        //CANデータ拡張項目
        typedef struct{
            int16_t frq;
            long pos;
        } exEscDataSt;
        //C620の拡張データ
        exEscDataSt exEscData[4];
        //C610の拡張データ
        exEscDataSt exC610Data[2];
        void printExpData(void);
        //C610のCANデータ格納用構造体
        typedef struct{
            int16_t angle;
            int16_t rotation;
            int16_t torque;
        } C610DataSt;
        //C610のCANデータ
        C610DataSt c610Data[2];

    private:
        CAN_message_t msg;
        CAN_message_t rxmsg;
        // unsigned int txTimer;
        // unsigned int rxTimer;
        void getESCCanData(CAN_message_t m);
        void hexDump(uint8_t dumpLen, uint8_t *bytePtr);
        void writeSerial(CAN_message_t rxmsg);
        void writeString(String stringData);
        void pidRotation(int* error, int* u);
        void getExpData(int id);

        //走行用モータ制限速度:-16,384～0～16,384(0xC000～0x4000)
        long limitSpeedWheel = 1000;
        long limitSpeedGimbal = 1000;

        int timerInterruptInterval = 1; //[ms]
        const int encoderOneAround = 8192;
        int yawNeutral = 0;
        void getC610Data(CAN_message_t m);
        void getExpC610Data(int id);



};
#endif
