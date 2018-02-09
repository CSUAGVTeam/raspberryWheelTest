#include "datashare.h"
#include "wiringPi.h"
#include <QMessageBox>
#include "wiringSerial.h"
#include "wiringPiI2C.h"
#include <pcf8574.h>
#include<sys/types.h>  
#include<sys/stat.h> 
#include <sys/ioctl.h> 
#include<fcntl.h>  
#include<termios.h>  
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <cstdio>
#include <termios.h>
#include <string.h>
#include <QTime>
#include <QCoreApplication>
#include <QObject>

Datashare::Datashare(QObject *parent) : QObject(parent)
{
    if(wiringPiSetup()==-1)
            QMessageBox::critical(NULL,"Wrong","Setup WiringPi false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
 
    // fd = serialOpen("/dev/ttyUSB0",115200);
    // if(fd<0)
        // QMessageBox::critical(NULL,"Wrong","Setup WiringPi serial port false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
    openSerial("/dev/ttyUSB0","/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3","/dev/ttyUSB4","/dev/ttyUSB5",115200);

    
	////fd1 = open("/dev/ttyUSB1",O_RDWR|O_NOCTTY| O_NDELAY | O_NONBLOCK); //读写打开 
	// fd1 = serialOpen("/dev/ttyUSB1",115200);
	// if(fd1<0)
        // QMessageBox::critical(NULL,"Wrong","Setup T11 serial port false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
	////fd1 = openSerial("dev/ttyUSB1",115200);

	////set_speed(fd1,115200); //设置波特率 
	////set_Parity(fd1,8,1,'N');

	// fd2 = open("/dev/ttyUSB2",O_RDWR|O_NOCTTY| O_NDELAY | O_NONBLOCK); //读写打开 
	////fd2 = serialOpen("dev/ttyUSB2",115200);
    // if(fd2<0)
        // QMessageBox::critical(NULL,"Wrong","Setup T2 serial port false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
	// set_speed(fd2,115200); //设置波特率 
	// set_Parity(fd2,8,1,'N');
	
    // fd3 = open("/dev/ttyUSB3",O_RDWR|O_NOCTTY| O_NDELAY | O_NONBLOCK); //读写打开 
	////fd3 = serialOpen("dev/ttyUSB3",115200);
	// if(fd3<0)
        // QMessageBox::critical(NULL,"Wrong","Setup T3 serial port false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
	// set_speed(fd3,115200); //设置波特率 
	// set_Parity(fd3,8,1,'N');
	
    // fd4 = open("/dev/ttyUSB4",O_RDWR|O_NOCTTY| O_NDELAY | O_NONBLOCK); //读写打开
	////fd4 = serialOpen("dev/ttyUSB4",115200);
	// if(fd4<0)
        // QMessageBox::critical(NULL,"Wrong","Setup T4 serial port false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
	// set_speed(fd4,115200); //设置波特率 
	// set_Parity(fd4,8,1,'N');
	
    i2c_fd1 = wiringPiI2CSetup(i2c_device1);
    if (i2c_fd1 < 0)
        QMessageBox::critical(NULL,"Wrong","Setup I2C device 1 false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
	
	wiringPiI2CWriteReg8(i2c_fd1,0x00,0x00);

    i2c_fd2 = wiringPiI2CSetup(i2c_device2);
    if (i2c_fd2 < 0)
        QMessageBox::critical(NULL,"Wrong","Setup I2C device 2 false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);

    i2c_fd3 = wiringPiI2CSetup(i2c_device3);
    if (i2c_fd3 < 0)
        QMessageBox::critical(NULL,"Wrong","Setup I2C device 3 false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);


    Gr1 = 0x0810;
    getSpeedString=0;
    wheelAddress = 0;
    wheelMoveSpeedSet=0;
    wheelMoveSpeedSetMax = 2000;
    wheelAngle = 0;
    delayTimeSet = 5;
    wheelFrontAngleOffset = 0;
    wheelRearAngleOffset = 0;
    connect(this,SIGNAL(timingbeginSignal()),this,SLOT(timingNow()));
}

void Datashare::writeWheelSpeed(float speedREV, int inputArea, char commandData[])
{
    int speed;
    speed = speedREV * 437; //4000 / 60 * 131072 / 20000;
    commandData[0] = 0xa5;
    commandData[1] = 0x3f;                  //address & 255;
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

void Datashare::writeAccessToDrive(int address)      //  give up
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

void Datashare::enableBridge(int address)            //  give up
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

void Datashare::disableBridge(int address)           //  give up
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

void Datashare::writeWheelCurrent(int inputArea, float currentAMPS)
{
    int current;
    current = currentAMPS * 32768 / 15 + 0.5;
    writeCurrentData[0] = 0xa5;
    writeCurrentData[1] = 0x3f;                          //address & 255;
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

void Datashare::readWheelCurrent(int address)        // give up
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

void Datashare::writeWheelSpeed(int inputArea, float speedREV)          //give up
{
    int speed;
    speed = speedREV * 4000 / 60 * 131072 / 20000 + 0.5;
    writeSpeedData[0] = 0xa5;
    writeSpeedData[1] = 0x3f;                       //address & 255;
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

void Datashare::readWheelSpeed(int address)          // give up
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

float Datashare::convertTelegramHex2Speed(unsigned char array[])
{
    if(array[0]==0xaf && array[1]==0xff)
    {
        int HH, H, L, LL, Speed, speedREV;
        HH = (int)(array[11]);
        H = (int)(array[10]);
        L = (int)(array[9]);
        LL = (int)(array[8]);
        Speed = LL + L * 256 + H * 65536 + HH * 16777216;
        speedREV = Speed * 20000 / 131072 * 60 / 4000;
        return (float)speedREV;
    }
    else
    {
        wheelCommunicationErrorFlag = false;
        return 0;
    }
}

void Datashare::writeWheelPosition(int inputArea, float positonANGLE)
{
    int position;
    position = positonANGLE * 32512 / 315 + 0.5;
    writePositionData[0] = 0xa5;
    writePositionData[1] = 0x3f;                      //address & 255;
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

void Datashare::readWheelPositon(int address)       //give up
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

float Datashare::convertTelegramHex2Angle(unsigned char array[])
{
    if (array[0]==0xaf && array[1]==0xff)
    {
        int HH, H, L, LL, position, positonANGLE;
        HH = (int)(array[11]);
        H = (int)(array[10]);
        L = (int)(array[9]);
        LL = (int)(array[8]);
        position = LL + L * 256 + H * 65536 + HH * 16777216;
        positonANGLE = position * 315 / 32512;
        return (float)positonANGLE;
    }
    else
    {
        wheelCommunicationErrorFlag = false;
        return 0;
    }
}

QString Datashare::checkWheelCommunication(int filedestiny)//need to fullfill
{
    unsigned char array[20]={0};
    int numberOFRead;
    QString str;
    numberOFRead = read(filedestiny,array,sizeof(array));
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

void Datashare::writeWheelSpeed(float speedREV)
{
    unsigned char array[14];
    int speed;
    speed = speedREV * 4000 / 60 * 131072 / 20000;
    array[0] = 0xa5;
    array[1] = 0x3f;
    array[2] = 0x02;
    array[3] = 0x45;
    array[4] = 0x00;
    array[5] = 0x02;
    accum = 0;
    for (int i = 0; i < 6; i++)
    {
        crunchCRC(array[i]);
    }
    crunchCRC(0);
    crunchCRC(0);

    array[6] = (accum >> 8) & 255;
    array[7] = accum & 255;
    array[8] = speed & 255;
    array[9] = (speed >> 8) & 255;
    array[10] = (speed >> 16) & 255;
    array[11] = (speed >> 24) & 255;
    accum = 0;
    for (int i = 0; i < 12; i++)
    {
        crunchCRC(array[i]);
    }
    crunchCRC(0);
    crunchCRC(0);
    array[12] = (accum >> 8) & 255;
    array[13] = accum & 255;
    write(fd, array, sizeof(array));
}

void Datashare::timingNow()
{
    while(1)
    {
        if(breakFlag == false)
        {
            QTime t1=QTime::currentTime().addSecs(60);
            while(QTime::currentTime()<t1)
            {
                QCoreApplication::processEvents(QEventLoop::AllEvents,65000);
            }
            wheelMoveSpeedSet +=500;
            if (wheelMoveSpeedSet>1600)
                wheelMoveSpeedSet = 500;
            if(wheelMoveSpeedSet>wheelMoveSpeedSetMax)
                wheelMoveSpeedSet = wheelMoveSpeedSetMax;
        }
        else
            break;
        QCoreApplication::processEvents(QEventLoop::AllEvents,65000);
    }
}

void Datashare::sendSignal()
{
    emit timingbeginSignal();
}

int Datashare::i2c_trans_in(int i2c_v,int num)
{
    int temp;
    temp = i2c_v&and_buf[num];
    temp = temp >> num;
    return temp;
}

int Datashare::i2c_trans_out1(int *p)//低位
{
    int temp=0;
    for (int i = 0; i <= 7; i++)
    {
        temp = temp + (p[i] << i);
    }
    return temp;
}

int Datashare::i2c_trans_out2(int *p)//高位
{
    int temp = 0;
    for (int i = 8; i <= 15; i++)
    {
        temp = temp + (p[i] << (i-8));
    }
    return temp;
}

void Datashare::bufTOvariable()                         // waiting to check
{
	systemOnFlag = state_input[0][0];
	systemOnFlag = !systemOnFlag;
	sickFalse = state_input[0][1];
	sickFalse = !sickFalse;
	sickWarningSpaceAlert = state_input[0][2];
	sickWarningSpaceAlert = !sickWarningSpaceAlert;
	steerFrontLimitDetect = state_input[0][3];
	steerFrontLimitDetect = !steerFrontLimitDetect;
    steerFrontLimitDetect2 = state_input[0][4];				//right
	steerFrontLimitDetect2 = !steerFrontLimitDetect2;
    steerBackLimitDetect = state_input[0][5];				//right
	steerBackLimitDetect = !steerBackLimitDetect;
    steerBackLimitDetect2 = state_input[0][6];
	steerBackLimitDetect2 = !steerBackLimitDetect2;
    // chargeContectorConnect = state_input[0][0];
    // reserve1=state_input[0][1];
    // conveyorDriveOKFlag = state_input[0][2];
    // liftSwitchFlag = state_input[0][3];
    // steerFrontLimitDetect = state_input[0][4];
    // steerFrontLimitDetect2 = state_input[0][5];
    // steerBackLimitDetect = state_input[0][6];
    // steerBackLimitDetect2 = state_input[0][7];
    // conveyorBackInterfaceDetect = state_input[0][8];
    // conveyorBackArriveDetect = state_input[0][9];
    // convryorFrontInterfaceDetect = state_input[0][10];
    // conveyorFrontArriveDetect = state_input[0][11];
    // liftBackDownDetect = state_input[0][12];
    // liftBacktUpDetect = state_input[0][13];
    // liftFrontDownDetect=state_input[0][14];
    // liftFrontUpDetect=state_input[0][15];

    // systemOnFlag = state_input[1][0];
    // powerOn = state_input[1][1];
    // chargeConnectReady = state_input[1][2];
    // chargeOn = state_input[1][3];
    // batteryChargeComplete = state_input[1][4];
    // chargerFalse = state_input[1][5];
    // reserve2 = state_input[1][6];
    // reserve3 = state_input[1][7];
    // wheelSteerBackFalseFlag = state_input[1][8];
    // wheelMoveBackFalseFlag = state_input[1][9];
    // wheelSteerFrontFalseFlag = state_input[1][10];
    // wheelMoveFrontFalseFlag = state_input[1][11];
    // sickWarningSpaceAlert = state_input[1][12];
    // sickFalse = state_input[1][13];
    // steerBackZeroPositionDetect = state_input[1][14];
    // steerFrontZeroPositionDetect = state_input[1][15];     //赋值
}

void Datashare::readIO()
{
    unsigned char array[14]= {0};
    unsigned char array3[14]= {0};
    int numberOfRead;
    //read I/O
    int i2c_read[2] = {0};
    i2c_read[0] = wiringPiI2CReadReg8(i2c_fd1,0x00);//一次性读取设备1的8位数据存在buf[0]
    //i2c_read[1] = wiringPiI2CReadReg16(i2c_fd2,0x00);//一次性读取设备2的0x00reg的16位数据存在buf[0]

    for(int i=0;i<8;i++)                               //将输入所得数据进行转换
    {
        state_input[0][i]=i2c_trans_in(i2c_read[0],i);
        //state_input[1][i]=i2c_trans_in(i2c_read[1],i);
    }
    bufTOvariable();

    // //read the speed of wheel
    // write(fd1,readSpeedData,sizeof(readSpeedData));//fflush(stdout);
    // numberOfRead = read(fd1,array,sizeof(array));
    // wheelMoveSpeedReadFront = convertTelegramHex2Speed(array);

    // memset(array,0,14*sizeof(unsigned char));
    // write(fd3,readSpeedData,sizeof(readSpeedData));//fflush(stdout);
    // numberOfRead = read(fd3,array3,sizeof(array3));
    // wheelMoveSpeedReadRear = convertTelegramHex2Speed(array3);

    // //read the angle of wheel
    // memset(array,0,14*sizeof(unsigned char));
    // write(fd2,readPositionData,sizeof(readPositionData));//fflush(stdout);
    // numberOfRead = read(fd2,array,sizeof(array));
    // wheelFrontAngle = convertTelegramHex2Angle(array) - wheelFrontAngleOffset;

    // memset(array,0,14*sizeof(unsigned char));
    // write(fd4,readPositionData,sizeof(readPositionData));//fflush(stdout);
    // numberOfRead = read(fd4,array,sizeof(array));
    // wheelRearAngle = convertTelegramHex2Angle(array) - wheelRearAngleOffset;

    // AGVSpeed = wheelMoveSpeedReadFront * cos(wheelFrontAngle) + wheelMoveSpeedReadRear * cos(wheelRearAngle);
	//
    checkIO();
}

void Datashare::gainAccessAndEnableWheel(void)
{
	unsigned char resetcommand[12] = {0xa5,0x3f,0x02,0x01,0x00,0x01,0x01,0x47,0x00,0x10,0x12,0x31};
    int array[20]= {0};
	// disable the bridge
    write(fd1,disableBridgeCommand,sizeof(disableBridgeCommand));read(fd1,array,20);
    write(fd2,disableBridgeCommand,sizeof(disableBridgeCommand));read(fd2,array,20);
    write(fd3,disableBridgeCommand,sizeof(disableBridgeCommand));read(fd3,array,20);
    write(fd4,disableBridgeCommand,sizeof(disableBridgeCommand));read(fd4,array,20);
	delayTimeMsecs(100);
	// reset 
	write(fd1,resetcommand,sizeof(resetcommand));read(fd1,array,20);
	write(fd2,resetcommand,sizeof(resetcommand));read(fd2,array,20);
	write(fd3,resetcommand,sizeof(resetcommand));read(fd3,array,20);
	write(fd4,resetcommand,sizeof(resetcommand));read(fd4,array,20);
	delayTimeMsecs(100);
	
   //writeAccessToDrive(01);
   //write(fd1,mptr)
   write(fd1,gainAccess,sizeof(gainAccess));
   read(fd1,array,20);
   //delayTimeMsecs(delayTimeSet);
   //writeAccessToDrive(02);
   write(fd2,gainAccess,sizeof(gainAccess));read(fd2,array,20);
//   delayTimeMsecs(delayTimeSet);
//   writeAccessToDrive(03);
   write(fd3,gainAccess,sizeof(gainAccess));read(fd3,array,20);
//   delayTimeMsecs(delayTimeSet);
//   writeAccessToDrive(04);
   write(fd4,gainAccess,sizeof(gainAccess));read(fd4,array,20);
//   delayTimeMsecs(delayTimeSet);
//   enableBridge(01);
   write(fd1,enableBridgeCommand,sizeof(enableBridgeCommand));read(fd1,array,20);
//   delayTimeMsecs(delayTimeSet);
//   enableBridge(02);
   write(fd2,enableBridgeCommand,sizeof(enableBridgeCommand));read(fd2,array,20);
//   delayTimeMsecs(delayTimeSet);
//   enableBridge(03);
   write(fd3,enableBridgeCommand,sizeof(enableBridgeCommand));read(fd3,array,20);
//   delayTimeMsecs(delayTimeSet);
//   enableBridge(04);
   write(fd4,enableBridgeCommand,sizeof(enableBridgeCommand));read(fd4,array,20);
//   delayTimeMsecs(delayTimeSet);
}

void Datashare::checkIO()
{
    if (sickWarningSpaceAlert && (!sickFalse))
    {
        wheelMoveSpeedSetMax -=800; (wheelMoveSpeedSetMax<0) ? wheelMoveSpeedSetMax = 0: 0;
        systemOnLight = 0;
        warmingLight = 1;
        alarmLight = 0;
        write(fd5,seri_send_buzzer3,sizeof(seri_send_buzzer3));
    }      //  unit: r/min ,fix in the future.

    if ((!sickWarningSpaceAlert) && (!sickFalse))
    {
        wheelMoveSpeedSetMax +=800; (wheelMoveSpeedSetMax>2400) ? wheelMoveSpeedSetMax = 2400 : 0;
        systemOnLight = 0;
        warmingLight = 0;
        alarmLight = 0;
        write(fd5,seri_send_buzzer1,sizeof(seri_send_buzzer1));
    }// The space is available, and add the speed upper limit.

    if (sickWarningSpaceAlert & sickFalse)
    {
        wheelMoveSpeedSetMax = 0;    // warning field2 alert, stop AGV
        systemOnLight = 0;
        warmingLight = 0;
        alarmLight = 1;
        write(fd5,seri_send_buzzer2,sizeof(seri_send_buzzer2));
    }

    //if ( sickFalse && (!sickWarningSpaceAlert) )    emergencyFlag = true;                           // check in the future, whether the parameter is useful?

//    if ( AGVSpeed > 2400 )                          {sickA=1; sickB=0; sickC=0;}

//    if ((1600<AGVSpeed) && (AGVSpeed< 2400))        {sickA=1; sickB=0; sickC=1;}

//    if ((800<AGVSpeed) && (AGVSpeed<1600))          {sickA=1; sickB=1; sickC=0;}

//    if (AGVSpeed <800)                             {sickA=1; sickB=1; sickC=1;}
    sickA = 0;
    sickB = 0;
    sickC = 1;
	
	writeIO();


}

void Datashare::writeIO()
{
    int i2c_write[2];
    variableTobuf();
    i2c_write[0]=i2c_trans_out1(state_output);//低位
    i2c_write[1]=i2c_trans_out2(state_output);//高位
    wiringPiI2CWriteReg16(i2c_fd3,i2c_write[0],i2c_write[1]);//向设备3的reg中写入两个字节
}

void Datashare::variableTobuf()
{
    state_output[0]=chargeContactorPickup;
    state_output[1]=wheelMoveFrontBrake;
    state_output[2]=wheelMoveBackBrake;
    state_output[3]=conveyorForward;
    state_output[4]=conveyorBack;
    state_output[5]=conveyorBrake;
    state_output[6]=liftFrontOn;
    state_output[7]=liftBackOn;
	
    // state_output[8]=systemOnLight;
    // state_output[9]=alarmLight;
    // state_output[10]=warmingLight;
    // state_output[11]=sickA;
    
	state_output[8]=sickA;
    state_output[9]=warmingLight;
    state_output[10]=alarmLight;
    state_output[11]=systemOnLight;
	
	state_output[12]=sickB;
    state_output[13]=sickC;
    state_output[14]=batteryChargeCircuitOn;
    state_output[15]=chargeStart;
}

void Datashare::set_speed(int fd, int speed)
{  
  int   i;   
  int   status;   
 struct termios   Opt;//用于存放获得的终端参数信息 
  tcgetattr(fd, &Opt); //获取与终端相关的参数存放在opt中  
  for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) 
  {   
    if  (speed == name_arr[i]) {       
      tcflush(fd, TCIOFLUSH);//清空终端未完成的I/O请求及数据,TCIOFLUSH清除所有正在发生的I/O数据。       
      cfsetispeed(&Opt, speed_arr[i]); //设置输入波特率   
      cfsetospeed(&Opt, speed_arr[i]); //设置输出波特率    
      status = tcsetattr(fd, TCSANOW, &Opt); //设置终端参数   
      if  (status != 0) {          
        perror("tcsetattr fd1");    
        return;       
      }      
      tcflush(fd,TCIOFLUSH);     
    }    
  }  
}  
  
int Datashare::set_Parity(int fd,int databits,int stopbits,int parity)  
{   
    struct termios options;   
    if  ( tcgetattr( fd,&options)  !=  0) 
	{   
        perror("SetupSerial 1");       
        return(FALSE);    
    }  
    options.c_cflag &= ~CSIZE;   
    switch (databits)  //数据位长度 
    {     
    case 7:       
        options.c_cflag |= CS7; //7位数据位  
        break;  
    case 8:       
        options.c_cflag |= CS8;  
        break;     
    default:      
        fprintf(stderr,"Unsupported data size\n"); return (FALSE);    
    }  
    switch (parity)   //奇偶校验位
    {   
        //无校验位	
        case 'n':  
        case 'N':      
            options.c_cflag &= ~PARENB;   /* Clear parity enable */  
            options.c_iflag &= ~INPCK;     /* Enable parity checking */   
            break;    
		///偶校验
        case 'o':     
        case 'O':       
            options.c_cflag |= (PARODD | PARENB);   
            options.c_iflag |= INPCK;             /* Disnable parity checking */   
            break;   
        //奇校验	
        case 'e':    
        case 'E':     
            options.c_cflag |= PARENB;     /* Enable parity */      
            options.c_cflag &= ~PARODD;      
            options.c_iflag |= INPCK;       /* Disnable parity checking */  
            break;  
        case 'S':   
        case 's':  /*as no parity*/     
            options.c_cflag &= ~PARENB;  
            options.c_cflag &= ~CSTOPB;break;    
        default:     
            fprintf(stderr,"Unsupported parity\n");      
            return (FALSE);    
        }    
      
    switch (stopbits)  //停止位
    {     
        case 1:      
            options.c_cflag &= ~CSTOPB;  //一位停止位  
            break;    
        case 2:      
            options.c_cflag |= CSTOPB;    //两位停止位
           break;  
        default:      
             fprintf(stderr,"Unsupported stop bits\n");    
             return (FALSE);   
    }   
    /* Set input parity option */   
    if (parity != 'n')     
        options.c_iflag |= INPCK;   //使能输入奇偶校验
    tcflush(fd,TCIFLUSH);  //清空正在发生的输入数据
    options.c_cc[VTIME] = 5;  // 要等待的时间量，要是在0.1s内没完成数据的读取就强行停止读取
    options.c_cc[VMIN] = 0; //要求等待的最小字节数
    if (tcsetattr(fd,TCSANOW,&options) != 0)    //激活配置使其生效 
    {   
        perror("SetupSerial 3");     
        return (FALSE);    
    }   
    return (TRUE);    
}

int Datashare::openSerial (const char *device, const char *device2, const char *device3, const char *device4, const char *device5, const char *device6, const int baud)
{
  struct termios options ;
  struct termios options1;
  int status1;
  speed_t myBaud ;
  int     status;
  //int fd=0 ;

  switch (baud)
  {
    case      50:	myBaud =      B50 ; break ;
    case      75:	myBaud =      B75 ; break ;
    case     110:	myBaud =     B110 ; break ;
    case     134:	myBaud =     B134 ; break ;
    case     150:	myBaud =     B150 ; break ;
    case     200:	myBaud =     B200 ; break ;
    case     300:	myBaud =     B300 ; break ;
    case     600:	myBaud =     B600 ; break ;
    case    1200:	myBaud =    B1200 ; break ;
    case    1800:	myBaud =    B1800 ; break ;
    case    2400:	myBaud =    B2400 ; break ;
    case    4800:	myBaud =    B4800 ; break ;
    case    9600:	myBaud =    B9600 ; break ;
    case   19200:	myBaud =   B19200 ; break ;
    case   38400:	myBaud =   B38400 ; break ;
    case   57600:	myBaud =   B57600 ; break ;
    case  115200:	myBaud =  B115200 ; break ;
    case  230400:	myBaud =  B230400 ; break ;
    case  460800:	myBaud =  B460800 ; break ;
    case  500000:	myBaud =  B500000 ; break ;
    case  576000:	myBaud =  B576000 ; break ;
    case  921600:	myBaud =  B921600 ; break ;
    case 1000000:	myBaud = B1000000 ; break ;
    case 1152000:	myBaud = B1152000 ; break ;
    case 1500000:	myBaud = B1500000 ; break ;
    case 2000000:	myBaud = B2000000 ; break ;
    case 2500000:	myBaud = B2500000 ; break ;
    case 3000000:	myBaud = B3000000 ; break ;
    case 3500000:	myBaud = B3500000 ; break ;
    case 4000000:	myBaud = B4000000 ; break ;

    default:
      return -2 ;
  }

  if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
  {
	  QMessageBox::critical(NULL,"Wrong","Setup WiringPi serial port false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
	  QMessageBox::critical(NULL,"Wrong",tr("%1").arg(fd),QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
  }
    if ((fd1 = open (device2, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
  {
	  QMessageBox::critical(NULL,"Wrong","Setup WiringPi serial port false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
	  QMessageBox::critical(NULL,"Wrong",tr("%1").arg(fd1),QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
  }
      if ((fd2 = open (device3, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
  {
	  QMessageBox::critical(NULL,"Wrong","Setup T2 port false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
	  QMessageBox::critical(NULL,"Wrong",tr("%1").arg(fd2),QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
  }
      if ((fd3 = open (device4, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
  {
	  QMessageBox::critical(NULL,"Wrong","Setup T3 port false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
	  QMessageBox::critical(NULL,"Wrong",tr("%1").arg(fd3),QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
  }
      if ((fd4 = open (device5, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
  {
	  QMessageBox::critical(NULL,"Wrong","Setup T4 port false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
	  QMessageBox::critical(NULL,"Wrong",tr("%1").arg(fd4),QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
  }
    if ((fd5 = open (device6, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
  {
      QMessageBox::critical(NULL,"Wrong","Setup Speaker port false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
      QMessageBox::critical(NULL,"Wrong",tr("%1").arg(fd5),QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
  }
  
  
    //return -1 ;

  fcntl (fd, F_SETFL, O_RDWR) ;


// Get and modify current options:

  tcgetattr (fd, &options) ;

    cfmakeraw   (&options) ;
    cfsetispeed (&options, myBaud) ;
    cfsetospeed (&options, myBaud) ;

    options.c_cflag |= (CLOCAL | CREAD) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

  tcsetattr (fd, TCSANOW, &options) ;

  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;

  ioctl (fd, TIOCMSET, &status);
  
  //  打开串口1，控制T1
  fcntl (fd1, F_SETFL, O_RDWR) ;
  tcgetattr (fd1, &options) ;
	cfmakeraw   (&options) ;
	cfsetispeed (&options, myBaud) ;
	cfsetospeed (&options, myBaud) ;
	options.c_cflag |= (CLOCAL | CREAD) ;
	options.c_cflag &= ~PARENB ;
	options.c_cflag &= ~CSTOPB ;
	options.c_cflag &= ~CSIZE ;
	options.c_cflag |= CS8 ;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
	options.c_oflag &= ~OPOST ;
	options.c_cc [VMIN]  =   0 ;
	options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)
	tcsetattr (fd1, TCSANOW, &options) ;
	ioctl (fd1, TIOCMGET, &status);
	status |= TIOCM_DTR ;
	status |= TIOCM_RTS ;
	ioctl (fd1, TIOCMSET, &status);
  

  //  打开串口2，控制T2
  fcntl (fd2, F_SETFL, O_RDWR) ;
  tcgetattr (fd2, &options) ;
  	cfmakeraw   (&options) ;
	cfsetispeed (&options, myBaud) ;
	cfsetospeed (&options, myBaud) ;
	options.c_cflag |= (CLOCAL | CREAD) ;
	options.c_cflag &= ~PARENB ;
	options.c_cflag &= ~CSTOPB ;
	options.c_cflag &= ~CSIZE ;
	options.c_cflag |= CS8 ;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
	options.c_oflag &= ~OPOST ;
	options.c_cc [VMIN]  =   0 ;
	options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)
	tcsetattr (fd2, TCSANOW, &options) ;
	ioctl (fd2, TIOCMGET, &status);
	status |= TIOCM_DTR ;
	status |= TIOCM_RTS ;
	ioctl (fd2, TIOCMSET, &status);
  
	// 打开串口3，控制T3
  fcntl (fd3, F_SETFL, O_RDWR) ;
  tcgetattr (fd3, &options) ;
	cfmakeraw   (&options) ;
	cfsetispeed (&options, myBaud) ;
	cfsetospeed (&options, myBaud) ;
	options.c_cflag |= (CLOCAL | CREAD) ;
	options.c_cflag &= ~PARENB ;
	options.c_cflag &= ~CSTOPB ;
	options.c_cflag &= ~CSIZE ;
	options.c_cflag |= CS8 ;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
	options.c_oflag &= ~OPOST ;
	options.c_cc [VMIN]  =   0 ;
	options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)
	tcsetattr (fd3, TCSANOW, &options) ;
	ioctl (fd3, TIOCMGET, &status);
	status |= TIOCM_DTR ;
	status |= TIOCM_RTS ;
	ioctl (fd3, TIOCMSET, &status);
  
  // 打开串口4，控制T4
  fcntl (fd4, F_SETFL, O_RDWR) ;
  tcgetattr (fd4, &options) ;
  	cfmakeraw   (&options) ;
	cfsetispeed (&options, myBaud) ;
	cfsetospeed (&options, myBaud) ;
	options.c_cflag |= (CLOCAL | CREAD) ;
	options.c_cflag &= ~PARENB ;
	options.c_cflag &= ~CSTOPB ;
	options.c_cflag &= ~CSIZE ;
	options.c_cflag |= CS8 ;
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
	options.c_oflag &= ~OPOST ;
	options.c_cc [VMIN]  =   0 ;
	options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)
	tcsetattr (fd4, TCSANOW, &options) ;
	ioctl (fd4, TIOCMGET, &status);
	status |= TIOCM_DTR ;
	status |= TIOCM_RTS ;
	ioctl (fd4, TIOCMSET, &status);

    // open 485 Speaker
    fcntl (fd5, F_SETFL, O_RDWR) ;
    tcgetattr (fd5, &options1) ;
      cfmakeraw   (&options1) ;
      cfsetispeed (&options1, B9600) ;
      cfsetospeed (&options1, B9600) ;
      options1.c_cflag |= (CLOCAL | CREAD) ;
      options1.c_cflag &= ~PARENB ;
      options1.c_cflag &= ~CSTOPB ;
      options1.c_cflag &= ~CSIZE ;
      options1.c_cflag |= CS8 ;
      options1.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
      options1.c_oflag &= ~OPOST ;
      options1.c_cc [VMIN]  =   0 ;
      options1.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)
      tcsetattr (fd5, TCSANOW, &options1) ;
      ioctl (fd5, TIOCMGET, &status1);
      status1 |= TIOCM_DTR ;
      status1 |= TIOCM_RTS ;
      ioctl (fd5, TIOCMSET, &status1);

  usleep (10000) ;	// 10mS

  return fd ;
}
