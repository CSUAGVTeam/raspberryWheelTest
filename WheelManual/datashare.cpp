#include "datashare.h"
#include "wiringPi.h"
#include <QMessageBox>
#include "wiringSerial.h"
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <cstdio>
#include <termios.h>
#include <QTime>
#include <QCoreApplication>


Datashare::Datashare()
{
    if(wiringPiSetup()==-1)
        QMessageBox::critical(NULL,"Wrong","Setup WiringPi false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
    fd = serialOpen("/dev/ttyUSB0",115200);
    if(fd<0)
        QMessageBox::critical(NULL,"Wrong","Setup WiringPi serial port false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
    //write(fd,gainAccess,sizeof(gainAccess));fflush(stdout);
    //write(fd,gainAccess1,sizeof(gainAccess1));fflush(stdout);
    //write(fd,gainAccess3,sizeof(gainAccess3));fflush(stdout);
    //write(fd,enableBridge,sizeof(enableBridge));fflush(stdout);
    //write(fd,enableBridge1,sizeof(enableBridge1));fflush(stdout);
    //write(fd,enableBridge3,sizeof(enableBridge3));fflush(stdout);
    Gr1 = 0x0810;
    getSpeedString=0;
    wheelAddress = 0;
    wheelMoveSpeedSet=0;
    wheelAngle = 0;
    delayTimeSet = 5;
    wheelFrontAngleOffset = 0;
    wheelRearAngleOffset = 0;
}

void Datashare::writeWheelSpeed(int address, float speedREV, int inputArea, char commandData[])
{
    int speed;
    speed = speedREV * 4000 / 60 * 131072 / 20000;
    commandData[0] = 0xa5;
    commandData[1] = address & 255;
    commandData[2] = 0x02;
    commandData[3] = 0x45;
    commandData[4] = inputArea & 255;
    commandData[5] = 0x02;
    accum = 0;
    for (int i = 0; i < 6; i++)
    {
        crunchCRC(commandData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);

    commandData[6] = (accum >> 8) & 255;
    commandData[7] = accum & 255;
    commandData[8] = speed & 255;
    commandData[9] = (speed >> 8) & 255;
    commandData[10] = (speed >> 16) & 255;
    commandData[11] = (speed >> 24) & 255;
    accum = 0;
    for (int i = 0; i < 12; i++)
    {
        crunchCRC(commandData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);
    commandData[12] = (accum >> 8) & 255;
    commandData[13] = accum & 255;
}

void Datashare::delayTimeMsecs(int msecs)
{
    QTime _Timer = QTime::currentTime().addMSecs(msecs);
    while(QTime::currentTime()<_Timer)
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
}

void Datashare::crunchCRC(char x)
{
    int i, k;
    for (k = 0; k < 8; k++)
    {
        i = (x >> 7) & 1;
        if (accum & 0x8000)
        {
            accum = ((accum ^ Gr1) << 1) + (i ^ 1);
        }
        else
        {
            accum = (accum << 1) + i;
        }
        accum &= 0x0ffff;
        x <<= 1;
    }
}

void Datashare::writeAccessToDrive(int address)
{
    accessData[0] = 0xa5;
    accessData[1] = address & 255;
    accessData[2] = 0x02;
    accessData[3] = 0x07;
    accessData[4] = 0x00;
    accessData[5] = 0x01;

    accum = 0;
    for (int i = 0; i < 6; i++)
    {
        crunchCRC(accessData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);

    accessData[6] = (accum >> 8) & 255;
    accessData[7] = accum & 255;
    accessData[8] = 0x0f;
    accessData[9] = 0x00;

    accum = 0;
    for (int i = 0; i < 10; i++)
    {
        crunchCRC(accessData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);

    accessData[10] = (accum >> 8) & 255;
    accessData[11] = accum & 255;
}

void Datashare::enableBridge(int address)
{
    enableData[0] = 0xa5;
    enableData[1] = address & 255;
    enableData[2] = 0x02;
    enableData[3] = 0x01;
    enableData[4] = 0x00;
    enableData[5] = 0x01;

    accum = 0;
    for (int i = 0; i < 6; i++)
    {
        crunchCRC(enableData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);

    enableData[6] = (accum >> 8) & 255;
    enableData[7] = accum & 255;
    enableData[8] = 0x00;
    enableData[9] = 0x00;

    accum = 0;
    for (int i = 0; i < 10; i++)
    {
        crunchCRC(enableData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);

    enableData[10] = (accum >> 8) & 255;
    enableData[11] = accum & 255;
}

void Datashare::disableBridge(int address)
{
    disableData[0] = 0xa5;
    disableData[1] = address & 255;
    disableData[2] = 0x02;
    disableData[3] = 0x01;
    disableData[4] = 0x00;
    disableData[5] = 0x01;

    accum = 0;
    for (int i = 0; i < 6; i++)
    {
        crunchCRC(disableData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);

    disableData[6] = (accum >> 8) & 255;
    disableData[7] = accum & 255;
    disableData[8] = 0x01;
    disableData[9] = 0x00;

    accum = 0;
    for (int i = 0; i < 10; i++)
    {
        crunchCRC(disableData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);

    disableData[10] = (accum >> 8) & 255;
    disableData[11] = accum & 255;
}

void Datashare::writeWheelCurrent(int address, int inputArea, float currentAMPS)
{
    int current;
    current = currentAMPS * 32768 / 15 + 0.5;
    writeCurrentData[0] = 0xa5;
    writeCurrentData[1] = address & 255;
    writeCurrentData[2] = 0x02;
    writeCurrentData[3] = 0x45;
    writeCurrentData[4] = inputArea & 255;
    writeCurrentData[5] = 0x02;
    accum = 0;
    for (int i = 0; i < 6; i++)
    {
        crunchCRC(writeCurrentData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);

    writeCurrentData[6] = (accum >> 8) & 255;
    writeCurrentData[7] = accum & 255;
    writeCurrentData[8] = current & 255;
    writeCurrentData[9] = (current >> 8) & 255;
    writeCurrentData[10] = (current >> 16) & 255;
    writeCurrentData[11] = (current >> 24) & 255;
    accum = 0;
    for (int i = 0; i < 12; i++)
    {
        crunchCRC(writeCurrentData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);
    writeCurrentData[12] = (accum >> 8) & 255;
    writeCurrentData[13] = accum & 255;

}

void Datashare::readWheelCurrent(int address)
{
    readCurrentData[0] = 0xa5;
    readCurrentData[1] = address & 255;
    readCurrentData[2] = 0x01;
    readCurrentData[3] = 0x10;
    readCurrentData[4] = 0x03;
    readCurrentData[5] = 0x01;

    accum = 0;
    for (int i = 0; i < 6; i++)
    {
        crunchCRC(readCurrentData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);

    readCurrentData[6] = (accum >> 8) & 255;
    readCurrentData[7] = accum & 255;
}

void Datashare::writeWheelSpeed(int address,int inputArea, float speedREV)
{
    int speed;
    speed = speedREV * 4000 / 60 * 131072 / 20000 + 0.5;
    writeSpeedData[0] = 0xa5;
    writeSpeedData[1] = address & 255;
    writeSpeedData[2] = 0x02;
    writeSpeedData[3] = 0x45;
    writeSpeedData[4] = inputArea & 255;
    writeSpeedData[5] = 0x02;
    accum = 0;
    for (int i = 0; i < 6; i++)
    {
        crunchCRC(writeSpeedData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);

    writeSpeedData[6] = (accum >> 8) & 255;
    writeSpeedData[7] = accum & 255;
    writeSpeedData[8] = speed & 255;
    writeSpeedData[9] = (speed >> 8) & 255;
    writeSpeedData[10] = (speed >> 16) & 255;
    writeSpeedData[11] = (speed >> 24) & 255;
    accum = 0;
    for (int i = 0; i < 12; i++)
    {
        crunchCRC(writeSpeedData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);
    writeSpeedData[12] = (accum >> 8) & 255;
    writeSpeedData[13] = accum & 255;
}

void Datashare::readWheelSpeed(int address)
{
    readSpeedData[0] = 0xa5;
    readSpeedData[1] = address & 255;
    readSpeedData[2] = 0x01;
    readSpeedData[3] = 0x11;
    readSpeedData[4] = 0x02;
    readSpeedData[5] = 0x02;

    accum = 0;
    for (int i = 0; i < 6; i++)
    {
        crunchCRC(readSpeedData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);

    readSpeedData[6] = (accum >> 8) & 255;
    readSpeedData[7] = accum & 255;
}

void Datashare::writeWheelPosition(int address, int inputArea, float positonANGLE)
{
    int position;
    position = positonANGLE * 32512 / 315 + 0.5;
    writePositionData[0] = 0xa5;
    writePositionData[1] = address & 255;
    writePositionData[2] = 0x02;
    writePositionData[3] = 0x45;
    writePositionData[4] = inputArea & 255;
    writePositionData[5] = 0x02;
    accum = 0;
    for (int i = 0; i < 6; i++)
    {
        crunchCRC(writePositionData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);

    writePositionData[6] = (accum >> 8) & 255;
    writePositionData[7] = accum & 255;
    writePositionData[8] = position & 255;
    writePositionData[9] = (position >> 8) & 255;
    writePositionData[10] = (position >> 16) & 255;
    writePositionData[11] = (position >> 24) & 255;
    accum = 0;
    for (int i = 0; i < 12; i++)
    {
        crunchCRC(writePositionData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);
    writePositionData[12] = (accum >> 8) & 255;
    writePositionData[13] = accum & 255;
}

void Datashare::readWheelPositon(int address)
{
    readPositionData[0] = 0xa5;
    readPositionData[1] = address & 255;
    readPositionData[2] = 0x01;
    readPositionData[3] = 0x12;
    readPositionData[4] = 0x00;
    readPositionData[5] = 0x02;

    accum = 0;
    for (int i = 0; i < 6; i++)
    {
        crunchCRC(readPositionData[i]);
    }
    crunchCRC(0);
    crunchCRC(0);

    readPositionData[6] = (accum >> 8) & 255;
    readPositionData[7] = accum & 255;
}

QString Datashare::checkWheelCommunication(void)
{
    unsigned char array[100]={0};
    int numberOFRead;
    QString str;
    numberOFRead = read(fd,array,sizeof(array));
    if(array[0] != 0xa5)
    {
        //add function to make system stop!
        //QMessageBox::critical(NULL,"Communication Wrong!","Wheel Communication has something wrong!");
    }
    if(array[1] != 255)
    {
        //add function to make system stop!
        //QMessageBox::critical(NULL,"Communication Wrong!","Wheel Communication has something wrong!");
    }
    tcflush(fd,TCIOFLUSH);
//    for(int i =0;i<numberOFRead;i++)
//    {
//        if(array[i]<16)
//            testStr += '0' +QString::number(array[i],16).toUpper();
//        else
//            testStr += QString::number(array[i],16).toUpper();
//    }
    for(int i=0;i<numberOFRead;i++)
    {
        if(array[i]<16)
            str += '0' + QString::number(array[i],16).toUpper();
        else
            str += QString::number(array[i],16).toUpper();
    }
    return str;
}

