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
    if(wiringPiSetup()==-1)										//wiringPi启动
            QMessageBox::critical(NULL,"Wrong","Setup WiringPi false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
 
    openSerial("/dev/ttyUSB0","/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3","/dev/ttyUSB4","/dev/ttyUSB5","/dev/ttyUSB6",115200);	//打开串口，全部一起打开。
	
    i2c_fd1 = wiringPiI2CSetup(i2c_device1);					//打开I2C设备（输入点）
    if (i2c_fd1 < 0)
        QMessageBox::critical(NULL,"Wrong","Setup I2C device 1 false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
	
	wiringPiI2CWriteReg8(i2c_fd1,0x00,0x00);

    i2c_fd2 = wiringPiI2CSetup(i2c_device2);					//打开I2C设备（输入点）
    if (i2c_fd2 < 0)
        QMessageBox::critical(NULL,"Wrong","Setup I2C device 2 false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);

    i2c_fd3 = wiringPiI2CSetup(i2c_device3);					//打开I2C设备（输出点）
    if (i2c_fd3 < 0)
        QMessageBox::critical(NULL,"Wrong","Setup I2C device 3 false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);


    Gr1 = 0x0810;									//CRC校验使用参数
    getSpeedString=0;								//放弃不用
    wheelAddress = 0;								//舵机驱动器地址
    wheelMoveSpeedSet=0;							//舵机驱动器速度
    wheelMoveSpeedSetMax = 2000;					//舵机驱动器最大速度
    wheelAngle = 0;									//舵机打角
    delayTimeSet = 5;								//延时参数（改为232后放弃不用了）
    wheelFrontAngleOffset = 0;						//前轮打角偏移量
    wheelRearAngleOffset = 0;						//后轮打角偏移量
    connect(this,SIGNAL(timingbeginSignal()),this,SLOT(timingNow()));		// 定时信号与定时槽连通（放弃不用）
}

/***********************生成速度报文**********************************************************************************************/
void Datashare::writeWheelSpeed(double speedREV, int inputArea, char commandData[])
{
    int speed;
    speed =(int) (speedREV * 17801.1941*60);//437; //4000 / 60 * 131072 / 20000;
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

/***********************延时函数（非阻塞模式）***************************************************************************************/
void Datashare::delayTimeMsecs(int msecs)
{
    QTime _Timer = QTime::currentTime().addMSecs(msecs);
    while(QTime::currentTime()<_Timer)
        QCoreApplication::processEvents(QEventLoop::AllEvents,100);
}

/************************报文校验用函数********************************************************************************************/
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

/************************生成获取舵机权限报文（可以弃用了）********************************************************************************************/
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

/*************************生成桥使能报文（可以弃用了）*******************************************************************************************/
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

/***************************生成桥断连报文（可以弃用了）*****************************************************************************************/
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

/****************************生成电流设置报文****************************************************************************************/
void Datashare::writeWheelCurrent(int inputArea, double currentAMPS)
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

/********************************生成读电流报文************************************************************************************/
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

/*********************************生成写速度报文***********************************************************************************/
void Datashare::writeWheelSpeed(int inputArea, double speedREV)          //give up
{
    int speed;
    speed = (int) (speedREV * 17801.1941*60);//4000 / 60 * 131072 / 20000 + 0.5;
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

/*****************************生成读速度报文***************************************************************************************/
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

/******************************报文转换函数（读取速度）**************************************************************************************/
double Datashare::convertTelegramHex2Speed(unsigned char array[])
{
    if(array[0]==0xa5 && array[1]==0xff)
    {
        int HH, H, L, LL, Speed;
        double speedREV;
        HH = (int)(array[11]);
        H = (int)(array[10]);
        L = (int)(array[9]);
        LL = (int)(array[8]);
        Speed = LL + L * 256 + H * 65536 + HH * 16777216;
        speedREV = Speed / 17801.1941 / 60;//20000 / 131072 * 60 / 4000;
        return speedREV;
    }
    else
    {
        wheelCommunicationErrorFlag = false;
        return 0;
    }
}

/********************************生成位置设置报文（舵机打角）************************************************************************************/
void Datashare::writeWheelPosition(int inputArea, double positonANGLE)
{
    int position;
    position = positonANGLE * 100.794 +0.5;//32512 / 315 * 0.97656 + 0.5;
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

/**********************************生成读取位置报文（可以弃用了）**********************************************************************************/
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

/**********************************报文转换函数(打角）**********************************************************************************/
double Datashare::convertTelegramHex2Angle(unsigned char array[])
{
    if (array[0]==0xa5 && array[1]==0xff)
    {
        int HH, H, L, LL, position, positonANGLE;
        HH = (int)(array[11]);
        H = (int)(array[10]);
        L = (int)(array[9]);
        LL = (int)(array[8]);
        position = LL + L * 256 + H * 65536 + HH * 16777216;
        positonANGLE = position /100.794; //315 / 32512;
        return (double)positonANGLE;
    }
    else
    {
        wheelCommunicationErrorFlag = false;
        return 0;
    }
}

/**********************************报文检验函数（可以弃用了）**********************************************************************************/
QString Datashare::checkWheelCommunication(int filedestiny)//need to fullfill
{
    unsigned char array[50]={0};
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

/************************************生成速度设置函数（重载）********************************************************************************/
void Datashare::writeWheelSpeed(double speedREV)
{
    unsigned char array[14];
    int speed;
    speed = (int) (speedREV * 17801.1941*60);//4000 / 60 * 131072 / 20000;
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

/**************************************计时函数（弃用）******************************************************************************/
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

/***************************************信号发送函数（弃用）*****************************************************************************/
void Datashare::sendSignal()
{
    emit timingbeginSignal();
}

/***************************************i2c读取函数*****************************************************************************/
int Datashare::i2c_trans_in(int i2c_v,int num)
{
    int temp;
    temp = i2c_v&and_buf[num];
    temp = temp >> num;
    return temp;
}

/****************************************i2c输出函数****************************************************************************/
int Datashare::i2c_trans_out1(int *p)//低位
{
    int temp=0;
    for (int i = 0; i <= 7; i++)
    {
        temp = temp + (p[i] << i);
    }
    return temp;
}

/*****************************************i2c输出函数（2）***************************************************************************/
int Datashare::i2c_trans_out2(int *p)//高位
{
    int temp = 0;
    for (int i = 8; i <= 15; i++)
    {
        temp = temp + (p[i] << (i-8));
    }
    return temp;
}

/******************************************IO->变量转换函数**************************************************************************/
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

/***************************************读取IO口函数*****************************************************************************/
void Datashare::readIO()
{
    unsigned char array[20]= {0};
    unsigned char array2[20] = {0};
    unsigned char array3[20]= {0};
    unsigned char array4[20] = {0};
    unsigned char arrayTemp[20] = {0};
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
     write(fd1,readSpeedData,sizeof(readSpeedData));//fflush(stdout);
     numberOfRead = read(fd1,array,sizeof(array));
     wheelMoveSpeedReadFront = convertTelegramHex2Speed(array);

    // memset(array,0,14*sizeof(unsigned char));
     write(fd3,readSpeedData,sizeof(readSpeedData));//fflush(stdout);
     numberOfRead = read(fd3,array3,sizeof(array3));
     wheelMoveSpeedReadRear = convertTelegramHex2Speed(array3);

    // //read the angle of wheel
    // memset(array,0,14*sizeof(unsigned char));
     write(fd2,readPositionData,sizeof(readPositionData));//fflush(stdout);
     numberOfRead = read(fd2,array2,sizeof(array2));
     wheelFrontAngle = convertTelegramHex2Angle(array2) - wheelFrontAngleOffset;

    // memset(array,0,14*sizeof(unsigned char));
     write(fd4,readPositionData,sizeof(readPositionData));//fflush(stdout);
     numberOfRead = read(fd4,array4,sizeof(array4));
     wheelRearAngle = convertTelegramHex2Angle(array4) - wheelRearAngleOffset;

     AGVSpeed=(wheelMoveSpeedReadFront+wheelMoveSpeedReadRear)/2 * cos((wheelFrontAngle+wheelRearAngle)/2*3.14159/180);
     AGVSpeeds.X=AGVSpeed*cos((yaw-yawInt)*3.14159/180);
     AGVSpeeds.Y=AGVSpeed*sin((yaw-yawInt)*3.14159/180);
     // AGVLocationX=AGVLocationX+AGVSpeedX*delayTimeSet/1000;
     //AGVLocationY=AGVLocationY+AGVSpeedY*delayTimeSet/1000;
/**
     AGVSpeedX = wheelMoveSpeedReadFront * cos(wheelFrontAngle*3.14/180) + wheelMoveSpeedReadRear * cos(wheelRearAngle*3.14/180);
     AGVSpeedY = wheelMoveSpeedReadFront * sin(wheelFrontAngle*3.14/180) + wheelMoveSpeedReadRear * sin(wheelRearAngle*3.14/180);
     AGVLocationX = AGVSpeedX * cos(yaw*3.14/180) * deltT - AGVSpeedY * sin(yaw*3.14/180) * deltT;
     AGVLocationY = AGVSpeedX * sin(yaw*3.14/180) * deltT + AGVSpeedY * cos(yaw*3.14/180) * deltT;
**/

	write(fd6,readInertialBuff,sizeof(readInertialBuff));
	delayTimeMsecs(8);
	numberOfRead = read(fd6,arrayTemp,sizeof(arrayTemp));
	//ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd6)).toHex());
    yaw = angle_trans(arrayTemp[4],arrayTemp[3]);
    if(yaw>=0)
        yaw=yaw;
    else
        yaw=yaw+360;
	if (yawFlag == true)
	{
        if ((yaw - yawLast > 20)||(yaw - yawLast) < -20)	yaw = yawLast;	//滤波
	}

	yawLast =	yaw;

    //checkIO();
}

/*****************************************获取舵机权限并使能***************************************************************************/
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

/******************************************检查IO状态并对相应变量做调整**************************************************************************/
void Datashare::checkIO()
{
    int array[20] = {0};
    if (sickWarningSpaceAlert && (!sickFalse))
    {
        wheelMoveSpeedSetMax -=800; (wheelMoveSpeedSetMax<0) ? wheelMoveSpeedSetMax = 0: 0;
        systemOnLight = 0;
        warmingLight = 1;
        alarmLight = 0;
        //write(fd5,seri_send_buzzer3,sizeof(seri_send_buzzer3));
    }      //  unit: r/min ,fix in the future.

    if ((!sickWarningSpaceAlert) && (!sickFalse))
    {
        wheelMoveSpeedSetMax +=800; (wheelMoveSpeedSetMax>2400) ? wheelMoveSpeedSetMax = 2400 : 0;
        systemOnLight = 1;
        warmingLight = 0;
        alarmLight = 0;
        //write(fd5,seri_send_buzzer1,sizeof(seri_send_buzzer1));
    }// The space is available, and add the speed upper limit.

    if (sickWarningSpaceAlert & sickFalse)
    {
        wheelMoveSpeedSetMax = 0;    // warning field2 alert, stop AGV
        systemOnLight = 0;
        warmingLight = 0;
        alarmLight = 1;
        //write(fd5,seri_send_buzzer2,sizeof(seri_send_buzzer2));
    }

    //if ( sickFalse && (!sickWarningSpaceAlert) )    emergencyFlag = true;                           // check in the future, whether the parameter is useful?

//    if ( AGVSpeed > 2400 )                          {sickA=1; sickB=0; sickC=0;}

//    if ((1600<AGVSpeed) && (AGVSpeed< 2400))        {sickA=1; sickB=0; sickC=1;}

//    if ((800<AGVSpeed) && (AGVSpeed<1600))          {sickA=1; sickB=1; sickC=0;}

//    if (AGVSpeed <800)                             {sickA=1; sickB=1; sickC=1;}
    sickA = 0;
    sickB = 0;
    sickC = 1;

    //wheelAngle = Incremental_PI(yaw,yawTarget);                                     //PI control the angle of wheel
    /**
    if(turn_flag==false)
    {
        wheelAngle = Position_PID(yaw, yawTarget);

    }//PID Position control the angle of wheel
    else
    {
        static double Pwm,Integral_bias,Last_Bias;
        Integral_bias+=delta_s;	                                 //Çó³öÆ«²îµÄ»ý·Ö
        if(Integral_bias>300)Integral_bias=300;
        if(Integral_bias<-300)Integral_bias=-300;
        Pwm=KP_turn*delta_s+KI_turn*Integral_bias+KD_turn*(delta_s-Last_Bias);       //Î»ÖÃÊœPID¿ØÖÆÆ÷
        Last_Bias=delta_s;                                       //±£ŽæÉÏÒ»ŽÎÆ«²î
        if (Pwm>45) Pwm = 45;
        if (Pwm<-45) Pwm = -45;
        wheelAngle=-Pwm;
    }
    **/
    wheelAngle = Position_PID2 (delta_s,turn_flag);
    writeWheelPosition(00,-wheelAngle + wheelFrontAngleOffset);
    write(fd2,writePositionData,sizeof(writePositionData));read(fd2,array,sizeof(array));
    writeWheelPosition(00,wheelAngle + wheelRearAngleOffset);
    write(fd4,writePositionData,sizeof(writePositionData));read(fd4,array,sizeof(array));

	writeIO();


}

/***********************************************输出IO口*********************************************************************/
void Datashare::writeIO()
{
    int i2c_write[2];
    variableTobuf();
    i2c_write[0]=i2c_trans_out1(state_output);//低位
    //i2c_write[1]=i2c_trans_out2(state_output);//高位
    wiringPiI2CWrite(i2c_fd3,i2c_write[0]);//向设备3的reg中写入两个字节
	//wiringPiI2CWriteReg16(i2c_fd3,i2c_write[0],i2c_write[1]);//向设备3的reg中写入两个字节
}

/***********************************************变量转换为输出数组*********************************************************************/
void Datashare::variableTobuf()
{
    state_output[0]=systemOnLight;
    state_output[1]=warmingLight;
    state_output[2]=alarmLight;
    state_output[3]=sickA;	
	state_output[4]=sickB;
    state_output[5]=sickC;
    state_output[6]=batteryChargeCircuitOn;
    state_output[7]=chargeStart;
    // state_output[0]=chargeContactorPickup;
    // state_output[1]=wheelMoveFrontBrake;
    // state_output[2]=wheelMoveBackBrake;
    // state_output[3]=conveyorForward;
    // state_output[4]=conveyorBack;
    // state_output[5]=conveyorBrake;
    // state_output[6]=liftFrontOn;
    // state_output[7]=liftBackOn;	
    // state_output[8]=systemOnLight;
    // state_output[9]=alarmLight;
    // state_output[10]=warmingLight;
    // state_output[11]=sickA;	
	// state_output[12]=sickB;
    // state_output[13]=sickC;
    // state_output[14]=batteryChargeCircuitOn;
    // state_output[15]=chargeStart;
}

/****************************************设置串口速度（可以弃置不用）****************************************************************************/
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

/******************************************设置校验位（可以弃置不用）**************************************************************************/  
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

/***********************************打开串口函数 *********************************************************************************/
int Datashare::openSerial (const char *device, const char *device2, const char *device3, const char *device4, const char *device5, const char *device6, const char *device7, const int baud)
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
    if ((fd6 = open (device7, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
  {
      QMessageBox::critical(NULL,"Wrong","Setup Speaker port false",QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
      QMessageBox::critical(NULL,"Wrong",tr("%1").arg(fd6),QMessageBox::Yes|QMessageBox::No,QMessageBox::Yes);
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

      // open navigation part
      fcntl (fd6, F_SETFL, O_RDWR) ;
      tcgetattr (fd6, &options) ;
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
        tcsetattr (fd6, TCSANOW, &options) ;
        ioctl (fd6, TIOCMGET, &status);
        status |= TIOCM_DTR ;
        status |= TIOCM_RTS ;
        ioctl (fd6, TIOCMSET, &status);

  usleep (10000) ;	// 10mS

  return fd ;
}

/***************************************              **************************************************/
int Datashare::Incremental_PI (int Encoder,int Target)
{
     static int Bias,Pwm,Last_bias;
     Bias=Target -  Encoder;                //ŒÆËãÆ«²î
     Pwm+=KP*(Bias-Last_bias)+KI*Bias;   //ÔöÁ¿ÊœPI¿ØÖÆÆ÷
     if(Pwm>45)Pwm=45;
     if(Pwm<-45)Pwm=-45;
     Last_bias=Bias;	                   //±£ŽæÉÏÒ»ŽÎÆ«²î
     return Pwm;                         //ÔöÁ¿Êä³ö
}

/***************************************              **************************************************/
int Datashare::Position_PID (int Encoder,int Target)
{
     static double Bias,Pwm,Integral_bias,Last_Bias;
     //Bias=angle_tran (Target,Encoder);
     Bias=Target - Encoder;                                  //ŒÆËãÆ«²î
     if(Bias>180)
     {
         Bias=Bias-360;
     }else if(Bias>=(-180))
     {
         Bias=Bias;
     }else
     {
         Bias=Bias+360;
     }
     Integral_bias+=Bias;	                                 //Çó³öÆ«²îµÄ»ý·Ö
     if(Integral_bias>120)Integral_bias=120;
     if(Integral_bias<-120)Integral_bias=-120;
     Pwm=KP*Bias+KI*Integral_bias+KD*(Bias-Last_Bias);       //Î»ÖÃÊœPID¿ØÖÆÆ÷
     Last_Bias=Bias;                                       //±£ŽæÉÏÒ»ŽÎÆ«²î
     if (Pwm>45) Pwm = 45;
     if (Pwm<-45) Pwm = -45;
     return Pwm;                                           //ÔöÁ¿Êä³ö
}
/***************************************   新版PID         **************************************************/
double Datashare::Position_PID2 (double delta,bool flag)
{
     double Pwm=0;
     static double Last_delta=0,Integral_delta=0;
     static double Last_delta_turn=0,Integral_delta_turn=0;


     if(flag==false)
     {
         Integral_delta+=delta;	                                 //Çó³öÆ«²îµÄ»ý·Ö
         if(Integral_delta>120)
             Integral_delta=120;
         if(Integral_delta<-120)
             Integral_delta=-120;
         Pwm=KP*delta+KI*Integral_delta+KD*(delta-Last_delta);       //Î»ÖÃÊœPID¿ØÖÆÆ÷
         Last_delta=delta;
     }
     else
     {
         Integral_delta_turn+=delta;	                                 //Çó³öÆ«²îµÄ»ý·Ö
         if(Integral_delta_turn>300)
             Integral_delta_turn=300;
         if(Integral_delta_turn<-300)
             Integral_delta_turn=-300;
         Pwm=KP_turn*delta+KI_turn*Integral_delta_turn+KD_turn*(delta_s-Last_delta_turn);       //Î»ÖÃÊœPID¿ØÖÆÆ÷
         Last_delta_turn=delta;
     }
     if (Pwm>45) Pwm = 45;
     if (Pwm<-45) Pwm = -45;
     return Pwm;                                           //ÔöÁ¿Êä³ö
}
/***************************************   圆周运动的圆心计算函数       **************************************************/
/**
void Datashare::Position_Turn (Position P_Now,Position P_Target,double Yaw_Now,double Yaw_Target)
{
    double K_Now,K_Target;
    double K_Now_vertical,K_Target_vertical;
    //Position P_Centre;

    K_Now=tan(Yaw_Now-yawInt); K_Target=tan(Yaw_Target-yawInt);
    K_Now_vertical=-1/K_Now;   K_Target_vertical=-1/K_Target;
    P_Centre.X=(double)(P_Target.Y-P_Now.Y+K_Now_vertical*P_Now.X-K_Target_vertical*P_Target.X)/(K_Now_vertical-K_Target_vertical);
    P_Centre.Y=K_Target_vertical*(P_Centre.X-P_Target.X)+P_Target.Y;
    //return P_Centre;
}
**/
/***************************************   直线路径规划函数      **************************************************/
double Datashare::Straight_Line (Position P_Now,Position P_Start,Position P_Target)
{
    double K_str=0;
    double delta_x=0;
    double delta_y=0;
    double out=0;

    delta_x=P_Target.X-P_Now.X;
    delta_y=P_Target.Y-P_Now.Y;
    if(abs(P_Target.X-P_Start.X)>=abs(P_Target.Y-P_Start.Y))
    {
        K_str=(double)(P_Target.Y-P_Start.Y)/(P_Target.X-P_Start.X);
        out=P_Target.Y-delta_x*K_str;
        return (out-P_Now.Y);
    }
    else
    {
        K_str=(double)(P_Target.X-P_Start.X)/(P_Target.Y-P_Start.Y);
        out=P_Target.X-delta_y*K_str;
        return (out-P_Now.X);
    }

}
/***************************************   圆周运动的圆心计算函数（优化）      **************************************************/
double Datashare::Position_Turn2 (Position P_Now,Position P_Target,double Yaw_Target)
{
    double K_And,K_Target;
    double K_And_vertical,K_Target_vertical;
    Position P_Middle;
    double Radius_turn_sq;

    if(P_Target.X==P_Now.X)//防止除数为0
    {
            K_And=(double)(P_Target.Y-P_Now.Y)/0.0001;
    }
    else
    {
    K_And=(double)(P_Target.Y-P_Now.Y)/(P_Target.X-P_Now.X);
    }

    if((Yaw_Target-yawInt==90)||(Yaw_Target-yawInt==-90))
    {
       // P_Centre.X=P_Now.X;
       // P_Centre.Y=P_Target.Y;
       // Radius_turn_sq=2;
        K_Target=9999999;
    }
    else
    {
      K_Target=tan(Yaw_Target-yawInt);
    }

      if(K_And==0)
      {
          K_And_vertical=9999999;
      }
      else
      {
      K_And_vertical=(double)-1/K_And;
      }
      if(K_Target==0)
      {
          K_Target_vertical=9999999;
      }
      else
      {
      K_Target_vertical=(double)-1/K_Target;
      }
      P_Middle.X=(double)(P_Target.X+P_Now.X)/2;
      P_Middle.Y=(double)(P_Target.Y+P_Now.Y)/2;

      P_Centre.X=(double)(P_Target.Y-P_Middle.Y+K_And_vertical*P_Middle.X-K_Target_vertical*P_Target.X)/(K_And_vertical-K_Target_vertical);
      if(P_Centre.X==P_Target.X)
          P_Centre.Y=K_And_vertical*(P_Centre.X-P_Middle.X)+P_Middle.Y;
      else
         P_Centre.Y=K_Target_vertical*(P_Centre.X-P_Target.X)+P_Target.Y;
      Radius_turn_sq=(P_Target.X-P_Centre.X)*(P_Target.X-P_Centre.X)+(P_Target.Y-P_Centre.Y)*(P_Target.Y-P_Centre.Y);
    return Radius_turn_sq;
}
/***************************************   转弯方向控制函数       **************************************************/
double Datashare::Position_Turn_crol (Position P_Centre,Position P_Target,Position P_Now,double Radius_turn_sq)
{
    //double Radius_turn_sq;
    //double K_And;                        //弦，斜率
    double Distance_NowToTar_sq;         // 当前点到目标点距离的平方
    double Distance_NowToRad_sq;         // 当前点到圆心的距离的平方
    double Angle ;                       // 返回给PID的参考值
    Position Circle_Point;              //
    double delta_angle=0;//角度的偏差

    //if(P_Target.X!=P_Now.X)
    //K_And=(double)(P_Target.Y-P_Now.Y)/(P_Target.X-P_Now.X);

    //Radius_turn_sq=(P_Target.X-P_Centre.X)*(P_Target.X-P_Centre.X)+(P_Target.Y-P_Centre.Y)*(P_Target.Y-P_Centre.Y);
    Distance_NowToTar_sq=(P_Now.X-P_Target.X)*(P_Now.X-P_Target.X)+(P_Now.Y-P_Target.Y)*(P_Now.Y-P_Target.Y);
    Distance_NowToRad_sq=(P_Now.X-P_Centre.X)*(P_Now.X-P_Centre.X)+(P_Now.Y-P_Centre.Y)*(P_Now.Y-P_Centre.Y);
    //Circle_Point.X= P_Now.X;
    //Circle_Point.Y= sqrt(Radius_turn_sq-(Circle_Point.X-P_Centre.X)*(Circle_Point.X-P_Centre.X))+P_Centre.Y;
    //if(P_Now > P_Centre.Y)
    //Angle=Circle_Point.Y-P_Now.Y;

    //Angle=sqrt(Radius_turn_sq)-sqrt(Distance_NowToRad_sq);
    Angle=sqrt(Radius_turn_sq)-sqrt(Distance_NowToRad_sq);
    if(Distance_NowToTar_sq<=0.025)//到达目标点，精度为30cm
    {
       turn_flag=false;
       num++;
    }
    delta_angle=yawTarget-yaw;
            if(delta_angle>180)
            {
                delta_angle=delta_angle-360;
            }else if(delta_angle>=(-180))
            {
                delta_angle=delta_angle;
            }else
            {
                delta_angle=delta_angle+360;
            }
    if(Angle>0)               //朝外
    {
        if(delta_angle<0)
            return (-Angle);
        else
            return 0;
    }else
    {
        if(delta_angle<0)
             return 0;
        else
            return (-Angle);
    }
    return Angle;


}
/***************************************   角度转换函数      **************************************************/
double Datashare::angle_tran (double Yaw_Target,double Yaw)
{
    double angle_out;
    double delta_angle;

    delta_angle=Yaw_Target-Yaw;
    if(delta_angle>180)
    {
        angle_out=delta_angle-360;
    }else if(delta_angle>=(-180))
    {
        angle_out=angle_out;
    }else
    {
        angle_out=delta_angle+360;
    }
   return angle_out;
}
/***************************************   走函数      **************************************************/
double Datashare::Go (Position P_Target )
{
   double Dis_NowToTar=0;
   //int circle_N=0;
   double R=0;
  Dis_NowToTar=(AGVLocation.X-P_Target.X)*(AGVLocation.X-P_Target.X)+(AGVLocation.Y-P_Target.Y)*(AGVLocation.Y-P_Target.Y);
  if(Dis_NowToTar<0.025)
  {
      if(!turn_flag)
      {
      yawTarget+=90;
      if(yawTarget>360)
          yawTarget-=360;
     // circle_N=yawTarget/360;
     // yawTarget=yawTarget-circle_N*360;
      //R=Position_Turn2 (AGVLocation,P_Target,yawTarget);
      turn_flag=true;
      //num++;
      }
  }
  return Dis_NowToTar;
}
/****************************************              ************************************************/
double Datashare::angle_trans(unsigned char low, unsigned char high)
{
  int temp=high*256+low;
  if((temp<=32767)&(temp>=0))
   {
      temp=temp;
   }
  if((temp>32767)&(temp<=65536))
  {
    temp=temp-65536;
  }
  return temp*0.1;
}

/*****************************************              ************************************************/


