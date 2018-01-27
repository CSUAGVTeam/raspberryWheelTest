#ifndef DATASHARE_H
#define DATASHARE_H

#include <QObject>
#include <QString>

class Datashare
{
public:
    int fd;                                 //serial port file identify

    unsigned char gainAccess[12]={0xa5,0x00,0x02,0x07,0x00,0x01,0xda,0xf0,0x0f,0x00,0x01,0x3e};
    unsigned char gainAccess1[12]={0xa5,0x01,0x02,0x07,0x00,0x01,0x70,0xa1,0x0f,0x00,0x01,0x3e};
    unsigned char gainAccess3[12]={0xa5,0x03,0x02,0x07,0x00,0x01,0x34,0x22,0x0f,0x00,0x01,0x3e};
    //unsigned char enableBridge[12]={0xa5,0x00,0x02,0x01,0x00,0x01,0x68,0x50,0x00,0x00,0x00,0x00};
    //unsigned char enableBridge1[12]={0xa5,0x01,0x02,0x01,0x00,0x01,0xc2,0x01,0x00,0x00,0x00,0x00};
    //unsigned char enableBridge3[12]={0xa5,0x03,0x02,0x01,0x00,0x01,0x86,0x82,0x00,0x00,0x00,0x00};
    //unsigned char disableBridge[12]={0xa5,0x00,0x02,0x01,0x00,0x01,0x68,0x50,0x01,0x00,0x33,0x31};
    //QByteArray gainAccessByte = {0xa5,0x01,0x02,0x07,0x00,0x01,0x70,0xa1,0x0f,0x00,0x01,0x3e};
//    char write500RPM[14]={0xa5,0x00,0x02,0x45,0x00,0x02,0x99,0x5e,0x55,0x55,0x03,0x00,0x29,0x13};
//    char write500RPMreverse[14]={0xa5,0x00,0x02,0x45,0x00,0x02,0xf0,0x49,0xab,0xaa,0xfc,0xff,0xc6,0x68};
    char write0RPM[14];
    char commanData[14];
    char commanDataReverse[14];
    float getSpeedString;
    float wheelMoveSpeedSet;
    float wheelAngle;
    unsigned int accum;
    unsigned int Gr1;
    int wheelAddress;
    int delayTimeSet;

     unsigned char accessData[12];
     unsigned char enableData[12];
     unsigned char disableData[12];
     unsigned char writeCurrentData[14];
     unsigned char readCurrentData[8];
     unsigned char writeSpeedData[14];
     unsigned char readSpeedData[8];
     unsigned char writePositionData[14];
     unsigned char readPositionData[8];

public:
    Datashare();
    //~Datashare();
    void writeWheelSpeed(int address, float speedREV,int inputArea, char commandData[14]);

    void writeAccessToDrive(int address);

    void enableBridge(int address);

    void disableBridge(int address);

    void writeWheelCurrent(int address, int inputArea, float currentAMPS);

    void readWheelCurrent(int address);

    void writeWheelSpeed(int address, int inputArea, float speedREV);

    void readWheelSpeed(int address);

    void writeWheelPosition(int address, int inputArea, float positonANGLE);

    void readWheelPositon(int address);

    void crunchCRC(char x);

    void delayTimeMsecs(int msecs);

signals:
    //void startReading();

public slots:
    //void readReady();



};

#endif // DATASHARE_H
