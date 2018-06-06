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
#include <QTextStream>
#include <QFile>



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
    wheelMoveSpeedSetMax = 1.2;					//舵机驱动器最大速度
    wheelAngle = 0;									//舵机打角
    delayTimeSet = 5;								//延时参数（改为232后放弃不用了）
    wheelFrontAngleOffset = 0;						//前轮打角偏移量
    wheelRearAngleOffset = 0;						//后轮打角偏移量
    warningFile.setFileName("/home/pi/lys/errorReport.txt");
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
        int HH, H, L, LL, position;
        double  positonANGLE;
        HH = (int)(array[11]);
        H = (int)(array[10]);
        L = (int)(array[9]);
        LL = (int)(array[8]);
        position = LL + L * 256 + H * 65536 + HH * 16777216;
        positonANGLE = (double)position / 103.2127; //100.794 ???; //315 / 32512;
        return positonANGLE;
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
            /*
            wheelMoveSpeedSet +=500;
            if (wheelMoveSpeedSet>1600)
                wheelMoveSpeedSet = 500;
            if(wheelMoveSpeedSet>wheelMoveSpeedSetMax)
                wheelMoveSpeedSet = wheelMoveSpeedSetMax;
             */
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
     if(numberOfRead <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
     wheelMoveSpeedReadFront = convertTelegramHex2Speed(array);

    // memset(array,0,14*sizeof(unsigned char));
     write(fd3,readSpeedData,sizeof(readSpeedData));//fflush(stdout);
     numberOfRead = read(fd3,array3,sizeof(array3));
     if(numberOfRead <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
     wheelMoveSpeedReadRear = convertTelegramHex2Speed(array3);

    // //read the angle of wheel
    // memset(array,0,14*sizeof(unsigned char));
     write(fd2,readPositionData,sizeof(readPositionData));//fflush(stdout);
     numberOfRead = read(fd2,array2,sizeof(array2));
     if(numberOfRead <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
     frontTelegram.clear();
     for(int i=0;i<numberOfRead;i++)
     {
         if(array2[i]<16)
             frontTelegram += '0' + QString::number(array2[i],16).toUpper();
         else
             frontTelegram += QString::number(array2[i],16).toUpper();
     }
     wheelFrontAngle = convertTelegramHex2Angle(array2) - wheelFrontAngleOffset;

    // memset(array,0,14*sizeof(unsigned char));
     write(fd4,readPositionData,sizeof(readPositionData));//fflush(stdout);
     numberOfRead = read(fd4,array4,sizeof(array4));
     if(numberOfRead <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}

     backTelegram.clear();
     for(int i=0;i<numberOfRead;i++)
     {
         if(array4[i]<16)
             backTelegram += '0' + QString::number(array4[i],16).toUpper();
         else
             backTelegram += QString::number(array4[i],16).toUpper();
     }
     wheelRearAngle = convertTelegramHex2Angle(array4) - wheelRearAngleOffset;

     /**
     AGVSpeed=(wheelMoveSpeedReadFront+wheelMoveSpeedReadRear)/2 * cos((wheelFrontAngle+wheelRearAngle)/2*3.14159/180);
     AGVSpeeds.X=AGVSpeed*cos((yaw-yawInt)*3.14159/180);
     AGVSpeeds.Y=AGVSpeed*sin((yaw-yawInt)*3.14159/180);
     **/
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
    if(numberOfRead <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
	//ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd6)).toHex());
    yaw = angle_trans(arrayTemp[4],arrayTemp[3])-yaw_error;
    while(yaw>=360||yaw<0)
    {
        if(yaw>=360)
            yaw-=360;
        if(yaw<0)
            yaw+=360;
    }
   // if ((yaw - yawLast > 20)||(yaw - yawLast) < -20)	yaw = yawLast;	//滤波

    AGVSpeed=(wheelMoveSpeedReadFront+wheelMoveSpeedReadRear)/2 * cos(wheelRearAngle*3.14159/180);
    AGVSpeeds.X=AGVSpeed*cos((yaw-yawInt)*3.14159/180);
    AGVSpeeds.Y=AGVSpeed*sin((yaw-yawInt)*3.14159/180);



    //checkIO();
}

/*****************************************获取舵机权限并使能***************************************************************************/
void Datashare::gainAccessAndEnableWheel(void)
{
	unsigned char resetcommand[12] = {0xa5,0x3f,0x02,0x01,0x00,0x01,0x01,0x47,0x00,0x10,0x12,0x31};
    int array[20]= {0};
    int number;
	// disable the bridge
    write(fd1,disableBridgeCommand,sizeof(disableBridgeCommand));number=read(fd1,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
    write(fd2,disableBridgeCommand,sizeof(disableBridgeCommand));number=read(fd2,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
    write(fd3,disableBridgeCommand,sizeof(disableBridgeCommand));number=read(fd3,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
    write(fd4,disableBridgeCommand,sizeof(disableBridgeCommand));number=read(fd4,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
	delayTimeMsecs(100);
	// reset 
    write(fd1,resetcommand,sizeof(resetcommand));number=read(fd1,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
    write(fd2,resetcommand,sizeof(resetcommand));number=read(fd2,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
    write(fd3,resetcommand,sizeof(resetcommand));number=read(fd3,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
    write(fd4,resetcommand,sizeof(resetcommand));number=read(fd4,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
	delayTimeMsecs(100);
	
   //writeAccessToDrive(01);
   //write(fd1,mptr)
   write(fd1,gainAccess,sizeof(gainAccess));
   number=read(fd1,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
   //delayTimeMsecs(delayTimeSet);
   //writeAccessToDrive(02);
   write(fd2,gainAccess,sizeof(gainAccess));number=read(fd2,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
//   delayTimeMsecs(delayTimeSet);
//   writeAccessToDrive(03);
   write(fd3,gainAccess,sizeof(gainAccess));number=read(fd3,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
//   delayTimeMsecs(delayTimeSet);
//   writeAccessToDrive(04);
   write(fd4,gainAccess,sizeof(gainAccess));number=read(fd4,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
//   delayTimeMsecs(delayTimeSet);
//   enableBridge(01);
   write(fd1,enableBridgeCommand,sizeof(enableBridgeCommand));number=read(fd1,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
//   delayTimeMsecs(delayTimeSet);
//   enableBridge(02);
   write(fd2,enableBridgeCommand,sizeof(enableBridgeCommand));number=read(fd2,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
//   delayTimeMsecs(delayTimeSet);
//   enableBridge(03);
   write(fd3,enableBridgeCommand,sizeof(enableBridgeCommand));number=read(fd3,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
//   delayTimeMsecs(delayTimeSet);
//   enableBridge(04);
   write(fd4,enableBridgeCommand,sizeof(enableBridgeCommand));number=read(fd4,array,20);if(number <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
//   delayTimeMsecs(delayTimeSet);
}

/******************************************检查IO状态并对相应变量做调整**************************************************************************/
void Datashare::checkIO()
{
    int array[20] = {0};
    double yawTarget_Delta;

    if(direction_flag==true)
    {
        if (sickWarningSpaceAlert && (!sickFalse) )
        {
          //  if(wheelMoveSpeedSet > 0.3)
             //   wheelMoveSpeedSet = 0.3;
            systemOnLight = 0;
            warmingLight = 1;
            alarmLight = 0;
            sickFlag = true;
           // write(fd5,seri_send_buzzer3,sizeof(seri_send_buzzer3));
        }      //  unit: r/min ,fix in the future.

        if ((!sickWarningSpaceAlert) && (!sickFalse) && (sickFlag == true))
        {
           // if(AGVSpeed <= 0.05)
              //  wheelMoveSpeedSet = 0.08;
           // else
            {
               // wheelMoveSpeedSet = wheelSpeedHold;
                sickFlag = false;
            }
            systemOnLight = 1;
            warmingLight = 0;
            alarmLight = 0;
           // write(fd5,seri_send_buzzer1,sizeof(seri_send_buzzer1));
        }// The space is available, and add the speed upper limit.

        if (sickWarningSpaceAlert & sickFalse)
        {
           // wheelMoveSpeedSet = 0;    // warning field2 alert, stop AGV
            sickFlag = true;
            systemOnLight = 0;
            warmingLight = 0;
            alarmLight = 1;
            //write(fd5,seri_send_buzzer2,sizeof(seri_send_buzzer2));
        }
        if ( AGVSpeed >= 0.32 )                          {sickA=0; sickB=0; sickC=1;}
        if ( AGVSpeed < 0.32 )                          {sickA=0; sickB=1; sickC=0;}
    }
    else
    {
        if (sickWarningSpaceAlert && (!sickFalse) )
        {
          //  if(wheelMoveSpeedSet < -0.3)
            //    wheelMoveSpeedSet = -0.3;
            systemOnLight = 0;
            warmingLight = 1;
            alarmLight = 0;
            sickFlag = true;
            //write(fd5,seri_send_buzzer3,sizeof(seri_send_buzzer3));
        }      //  unit: r/min ,fix in the future.

        if ((!sickWarningSpaceAlert) && (!sickFalse) && (sickFlag == true))
        {
           // if(AGVSpeed >= -0.05)
              //  wheelMoveSpeedSet = -0.08;
           // else
            {
              //  wheelMoveSpeedSet = wheelSpeedHold;
                sickFlag = false;
            }
            systemOnLight = 1;
            warmingLight = 0;
            alarmLight = 0;

           // write(fd5,seri_send_buzzer1,sizeof(seri_send_buzzer1));
        }// The space is available, and add the speed upper limit.

        if (sickWarningSpaceAlert & sickFalse)
        {
          //  wheelMoveSpeedSet = 0;    // warning field2 alert, stop AGV
            sickFlag = true;
            systemOnLight = 0;
            warmingLight = 0;
            alarmLight = 1;
           // write(fd5,seri_send_buzzer2,sizeof(seri_send_buzzer2));
        }
        if ( AGVSpeed >= -0.32 )                          {sickA=0; sickB=1; sickC=1;}
        if ( AGVSpeed < -0.32 )                          {sickA=1; sickB=1; sickC=0;}
    }


    if(turn_flag)
    {
        yawTarget_Delta = yawTarget-yawTarget_Last;
        if(yawTarget_Delta>180)
        {
            yawTarget_Delta=yawTarget_Delta-360;
        }else if(yawTarget_Delta>=(-180))
        {
            yawTarget_Delta=yawTarget_Delta;
        }else
        {
            yawTarget_Delta=yawTarget_Delta+360;
        }
        if(yawTarget_Delta>50)   {sickA=1;sickB=0;sickC=0;}
        if(yawTarget_Delta<-50)  {sickA=1;sickB=0;sickC=1;}
        //if ( sickFalse && (!sickWarningSpaceAlert) )    emergencyFlag = true;                           // check in the future, whether the parameter is useful?

    }




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
    //wheelAngle = Position_PID2 (delta_s,turn_flag)+Position_PID(yaw, yawTarget);
    if(turn_flag==false)  //直道
    {
        a = Position_PID2 (delta_s,turn_flag);
        b = Position_PID(yaw, yawTarget);
        if(fabs(AGVSpeed) <= 0.6)
        {
            if(fabs(delta_s)>=0.012)
            {
                //if( (a>0 && b>0) || (a<0 && b<0) )
                //   wheelAngle = 1.7*a + 0.2*b;
                //else
                   wheelAngle = 1.6*a+0.4*b;
            }
            else
            {
                wheelAngle = a+b;
            }
        }
        else
        {
            if(fabs(delta_s)>=0.012)
            {
                //if( (a>0 && b>0) || (a<0 && b<0) )
                //   wheelAngle = 1.7*a + 0.2*b;
                //else
                   wheelAngle = 1.3*a+0.3*b;
            }
            else
            {
                wheelAngle = a+b;
            }
        }

        if(wheelAngle<-12)
            wheelAngle=-12;
        if(wheelAngle>12)
            wheelAngle=12;
    }
    else
    {
        wheelAngle=Position_PID2 (delta_s,turn_flag);
        if(wheelAngle<-45)
            wheelAngle=-45;
        if(wheelAngle>45)
            wheelAngle=45;
    }


    if( ((AGVLocation.X-P_Stop.X)*(AGVLocation.X-P_Stop.X)+(AGVLocation.Y-P_Stop.Y)*(AGVLocation.Y-P_Stop.Y)) <= 0.2) //到起点就不打角
    {
        wheelAngle = 0;
    }


    //wheelAngle = Position_PID(yaw, yawTarget);

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
    options.c_cc [VTIME] = 10 ;	// Ten seconds (100 deciseconds)

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
    options.c_cc [VTIME] = 10 ;	// Ten seconds (100 deciseconds)
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
    options.c_cc [VTIME] = 10 ;	// Ten seconds (100 deciseconds)
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
    options.c_cc [VTIME] = 10 ;	// Ten seconds (100 deciseconds)
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
    options.c_cc [VTIME] = 10 ;	// Ten seconds (100 deciseconds)
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
      options1.c_cc [VTIME] = 10 ;	// Ten seconds (100 deciseconds)
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
        options.c_cc [VTIME] = 10 ;	// Ten seconds (100 deciseconds)
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
double Datashare::Position_PID (double Encoder,double Target)
{
      double Bias=0,Pwm,Integral_bias;
      static double Last_Bias=0;
     //Bias=angle_tran (Target,Encoder);
      if(direction_flag==true)
      {
          Bias=Target - Encoder;                                  //ŒÆËãÆ«²î
      }
      else
      {
          Bias= Encoder - Target;
      }

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
     /*
     Integral_bias+=Bias;	                                 //Çó³öÆ«²îµÄ»ý·Ö
     if(Integral_bias>120)Integral_bias=120;
     if(Integral_bias<-120)Integral_bias=-120;
     */
     if(fabs(Bias)<8)  //20
     {
         if(fabs(KP_Angle*Bias*10)<fabs(KD_Angle*(Bias-Last_Bias)))
         {
             Pwm=KP_Angle*Bias;
         }
         else
         {
             Pwm=KP_Angle*Bias+KD_Angle*(Bias-Last_Bias);
         }
         //Pwm=KP_Angle*Bias+KI_Angle*Integral_bias+KD_Angle*(Bias-Last_Bias);       //Î»ÖÃÊœPID¿ØÖÆÆ÷
         Last_Bias=Bias;                                       //±£ŽæÉÏÒ»ŽÎÆ«²î
     }
     else
     {
         yaw = yawLast;
     }
     yawLast = yaw;

     if (Pwm>45) Pwm = 45;
     if (Pwm<-45) Pwm = -45;
     return Pwm;                                           //ÔöÁ¿Êä³ö
}

/***************************************   QR code information processinng  *********************************/
bool Datashare::Two_bar_codes_Pro(QString information)
{
    int Int_out[20]={0};//输出转化为数字
    int blank[20]={0};//字符串中的空字符所在的位置；
    int i=0,j=0;//只作为循环的变量；--可以随意使用
    int k=0,t=0;//记录数据个数的大小（当前的下标）--不可随意更改
    bool Cut_Chars=false;//空字符的标志
    int m=0,n=0;//当前解读数据的大小
    bool QR_Scan_Flag=false;
    char *Inf;

    //QByteArray ba=information.toLatin1();
    //Inf=ba.data();
    for(i=0;;i++)
    {
        if(Inf[i]==' ')
        {
            blank[k]=i;
            k++;
        }
        if(Inf[i]=='\0')
        {
            break;
        }
    }
    for( i=0;;i++)
    {
        Inf[i]=Inf[i]-'0';
        if(Inf[i]=='\0')
        {
            t=i;
            break;
        }
    }
    for( i=0;i<t;i++)
    {
        n=0;
        Cut_Chars=false;
        char data[10]={0};
        for(j=0;j<=k;j++)
        {
            if(i==blank[j])
            {
                Cut_Chars=true;
                //n=0;
                break;
            }
        }
        if(Cut_Chars==false)
        {
            data[n]=Inf[i];
            n++;
        }
        else
        {
            for(i=0;i<n;i++)
            {
               Int_out[m]+=data[i]*(int)pow(10,(n-i-1));
            }
            m++;
        }
    }
    for(i=0;i<m;i++)
    {
        if(i=0)
        {
            QR_Code_Number=Int_out[i];
        }
        else
        {
            if(m-i>1)
            {
                QR_Point[i/2].X=Int_out[i];
                QR_Point[i/2-1].Y=Int_out[i];
            }
            else
            {
                Angle_QRtoCar=Int_out[i];
            }
        }
    }
    QR_Scan_Flag=true;
    return QR_Scan_Flag;
}
/***************************************  二维码读取信息转换   ***********************************************/
bool Datashare::Two_bar_codes_Pro2(QString information)
{
    bool QR_Scan_Flag=false;

    QStringList Part_Inf=information.split(" ");
    QString a1=Part_Inf[0];       QString a7=Part_Inf[6];
    QString a2=Part_Inf[1];       QString a8=Part_Inf[7];
    QString a3=Part_Inf[2];       QString a9=Part_Inf[8];
    QString a4=Part_Inf[3];       QString a10=Part_Inf[9];
    QString a5=Part_Inf[4];       QString a11=Part_Inf[10];
    QString a6=Part_Inf[5];       QString a12=Part_Inf[11];

    QR_Code_Number = a1.toInt();    QR_Point[2].Y = a7.toInt();
    QR_Point[0].X = a2.toInt();     QR_Point[3].X = a8.toInt();
    QR_Point[0].Y = a3.toInt();     QR_Point[3].Y = a9.toInt();
    QR_Point[1].X = a4.toInt();     QR_Point[4].X = a10.toInt();
    QR_Point[1].Y = a5.toInt();     QR_Point[4].Y = a11.toInt();
    QR_Point[2].X = a6.toInt();     Angle_QRtoCar = a12.toInt();
    QR_Scan_Flag=true;
    return QR_Scan_Flag;
}
/***************************************  二维码信息进行位置，角度矫正处理   ***********************************************/
double Datashare::Information_Corrective()
{
    //static int Target_Location_Last=-1;  //上次的位置标号
    int Target_Location=0;               //位置标号
    int Target_Number=0;                 //序列号
    int Number_Flag;
    double Dis_Out;
    double Dis_In;
    double Side_Twocode=0;               //二维码图像边长
    double Angle_Error=0;                //角度偏差
    double Yaw_Error=0;                  //惯导漂移值

    //double N_YawInt=0;                   //坐标系正方向修正角度
    //double Chord_Length = 0;
    Position delta_TwoCode = {0,0};
    bool Flag_On_Road = false;           //二维码是否在路径上的标志位
    Position Image_Error={0,0};                   //图像上二维码中心到图像中心的像素点偏差
    //int Count_Turn = 0;                  //转弯后朝向记录
    struct Position *Error_Position ;    //偏差数组的指针
    Position Error_Position_Front[4]={{0.0125,-0.0125},{-0.0125,-0.0125},{0.0125,0.0125},{-0.0125,0.0125}};//此处的值是根据所贴二维码大小及之间位置决定
    Position Error_Position_Left[4]={{-0.0125,-0.0125},{-0.0125,0.0125},{0.0125,-0.0125},{0.0125,0.0125}};//此处的值是根据所贴二维码大小及之间位置决定
    Position Error_Position_Back[4]={{-0.0125,0.0125},{0.0125,0.0125},{-0.0125,-0.0125},{0.0125,-0.0125}};//反向跑时二维码的方向
    Position Error_Position_Right[4]={{0.0125,0.0125},{0.0125,-0.0125},{-0.0125,0.0125},{-0.0125,-0.0125}};//此处的值是根据所贴二维码大小及之间位置决定
    Target_Location=(QR_Code_Number-1)/4;
    Target_Number=(QR_Code_Number-1)%4;

    P_protection.X = P_TwoCode[Target_Location].X;
    P_protection.Y = P_TwoCode[Target_Location].Y;

    Image_Error.X=Image_Center.X-QR_Point[4].X;
    Image_Error.Y=Image_Center.Y-QR_Point[4].Y;

    /****此处加像素坐标差转换为实际位置坐标差 *******/


    Side_Twocode=sqrt((QR_Point[0].X-QR_Point[1].X)*(QR_Point[0].X-QR_Point[1].X)+(QR_Point[0].Y-QR_Point[1].Y)*(QR_Point[0].Y-QR_Point[1].Y));
    Image_Error.X=Image_Error.X*(double)(0.021/Side_Twocode);//实际大小与图像倍数关系
    Image_Error.Y=Image_Error.Y*(double)(0.021/Side_Twocode);
    //Chord_Length = 2*Distance_QR_CarCenter*sin((Angle_Error/2+1.8)*3.14159/180);
    //Image_Error.X += Chord_Length;
    //Image_Error.Y -= Chord_Length;
    if(turn_flag == true)
    {
        Dis_In=(AGVLocation.X-P_Target[num-1].X)*(AGVLocation.X-P_Target[num-1].X)+(AGVLocation.Y-P_Target[num-1].Y)*(AGVLocation.Y-P_Target[num-1].Y);
        Dis_Out=(AGVLocation.X-P_Target[num].X)*(AGVLocation.X-P_Target[num].X)+(AGVLocation.Y-P_Target[num].Y)*(AGVLocation.Y-P_Target[num].Y);
        if(Dis_In < Dis_Out)  //如果靠近入弯点就按入弯点方向校正
        {
            Number_Flag = (4+Num_Turn-1)%4;
        }
        else
            Number_Flag = Num_Turn%4;
    }
    else
        Number_Flag = Num_Turn%4;

    switch(Number_Flag)
    {
      case 0:
            {
               Error_Position = Error_Position_Front ; //y轴正
               Error_Position += Target_Number;
               break;
            }
      case 1:
            {
               Error_Position = Error_Position_Left ; //x负
               Error_Position += Target_Number;
               break;
            }
      case 2:
            {
               Error_Position = Error_Position_Back ; //y负
               Error_Position += Target_Number;
               break;
            }
      case 3:
            {
               Error_Position = Error_Position_Right ;
               Error_Position += Target_Number;               
               break;
            }
      default: break;
    }
    Error_Position->X+=Image_Error.X;
    Error_Position->Y-=Image_Error.Y;
    delta_TwoCode.X = Error_Position->X*cos((yawTarget-yawInt-90)*3.14159/180)-Error_Position->Y*sin((yawTarget-yawInt-90)*3.14159/180);
    delta_TwoCode.Y = Error_Position->X*sin((yawTarget-yawInt-90)*3.14159/180)+Error_Position->Y*cos((yawTarget-yawInt-90)*3.14159/180);
/**************
    if(direction_flag==true)
    {
        for(int i=num-1;i<=7;i++)
        {
            if(i<=0)
            {
                i=0;
            }
            if(P_TwoCode[Target_Location].X==P_Target[i].X)
            {
                if(P_TwoCode[Target_Location].Y==P_Target[i].Y)
                {
                    Flag_On_Road = true;
                    break;
                }
            }
        }
    }
    else
    {
        for(int i=num+1;i>=0;i--)
        {
            if(i>=7)
            {
                i=7;
            }
            if(P_TwoCode[Target_Location].X==P_Target[i].X)
            {
                if(P_TwoCode[Target_Location].Y==P_Target[i].Y)
                {
                    Flag_On_Road = true;
                    break;
                }
            }
        }
    }
************/

//    if(Flag_On_Road==true)
    {
        AGVLocation.X = P_TwoCode[Target_Location].X+delta_TwoCode.X;
        AGVLocation.Y = P_TwoCode[Target_Location].Y+delta_TwoCode.Y;
    }


/*角度*/
    Angle_Error = Angle_QRtoCar-180-Num_Turn*90;
//    if(Flag_On_Road==true)   //为什么角度校正要是路径上的点？？？
    {
        if(turn_flag==false)
        {
            if(direction_flag == true)  //前进偏右要加
            {
               yawInt = yaw-90-Angle_Error-ka_for-Num_Turn*90;//坐标系的重建0.2  //0.4
               yawTarget = yaw - Angle_Error-ka_for;//目标方向修正0.2
            }
            else     //后退偏右要减
            {
                yawInt = yaw-90-Angle_Error-ka_ba-Num_Turn*90;//坐标系的重建//0.3
                yawTarget = yaw - Angle_Error-ka_ba;//目标方向修正
            }
            //yawInt = yaw-90-Angle_Error-0.2-Num_Turn*90;//坐标系的重建
            while(yawInt<0||yawInt>=360)
            {
                if(yawInt<0)
                    yawInt+=360;
                if(yawInt>360)
                    yawInt-=360;
            }
            //if(fabs(Angle_Error)<25) //对错角滤波
            //    yawTarget = yaw - Angle_Error-0.2;//目标方向修正
            while(yawTarget<0||yawTarget>=360)
            {
                if(yawTarget<0)
                    yawTarget+=360;
                if(yawTarget>360)
                    yawTarget-=360;
            }
        }
/*********************
        if(Target_Location==0)
        {
            yawInt = yaw-90-Angle_Error-0.2;//坐标系的重建
            while(yawInt<0||yawInt>=360)
            {
                if(yawInt<0)
                    yawInt+=360;
                if(yawInt>360)
                    yawInt-=360;
            }

            yawTarget = yaw - Angle_Error-0.2;//目标方向修正
            while(yawTarget<0||yawTarget>=360)
            {
                if(yawTarget<0)
                    yawTarget+=360;
                if(yawTarget>360)
                    yawTarget-=360;
            }
        }
*********************/
/******************************
        else
        {
            if(turn_flag==false)
            {
                Yaw_Error = yaw-(yawTarget+Angle_Error);
            }
            else
            {
                Yaw_Error = yaw-(yawTarget_Last+Angle_Error);
            }
            while(Yaw_Error<0||Yaw_Error>=360)
            {
                if(Yaw_Error<0)
                    Yaw_Error +=360;
                if(Yaw_Error>360)
                    Yaw_Error -=360;
            }
        }
    *********************************/
    }
    return Yaw_Error;
}

/***************************************   新版PID         **************************************************/
double Datashare::Position_PID2 (double delta,bool flag)
{
     double Pwm=0;
     static double Last_delta=0,Integral_delta=0;
     static double Last_delta_turn=0,Integral_delta_turn=0;
     static double Last_Pwm=0;
     double KP_Line;

     if(flag==false)
     {
         if(fabs(AGVSpeed)<=0.4)
         {
             KP=14;
             KD=1250;
             if(fabs(delta)<0.28)
                KP_Line=(delta*delta)*7000+KP;
             else
                KP_Line=(delta*delta)*8000+KP;
         }
         else if(fabs(AGVSpeed)<=0.55)
         {
             KP=10;
             KD=1200;
             if(fabs(delta)<0.28)
                KP_Line=(delta*delta)*6000+KP;
             else
                KP_Line=(delta*delta)*7000+KP;
         }
         else
         {
             KP=8;
             KD=1000;
             if(fabs(delta)<0.28)
                KP_Line=(delta*delta)*5000+KP;
             else
                KP_Line=(delta*delta)*5000+KP;
         }

         Integral_delta+=delta;	                                 //Çó³öÆ«²îµÄ»ý·Ö
         if(Integral_delta>120)
             Integral_delta=120;
         if(Integral_delta<-120)
             Integral_delta=-120;
         //Pwm=KP_Line*delta+KI*Integral_delta+KD*(delta-Last_delta);       //Î»ÖÃÊœPID¿ØÖÆÆ÷
         if(fabs(KP_Line*delta*10)>fabs(KD*(delta-Last_delta)))
         {
             Pwm=KP_Line*delta+KD*(delta-Last_delta);       //Î»ÖÃÊœPID¿ØÖÆÆ÷
         }
         else
         {
              Pwm=KP_Line*delta;
         }
         //Pwm=KP_Line*delta+KD*(delta-Last_delta);       //Î»ÖÃÊœPID¿ØÖÆÆ÷
         Last_delta=delta;
         Last_delta_turn=0;
     }
     else
     {
         KP_Line=(delta*delta)*KI_turn+KP_turn;
         Integral_delta_turn+=delta;
         if(Integral_delta_turn>300)
             Integral_delta_turn=300;
         if(Integral_delta_turn<-300)
             Integral_delta_turn=-300;

//         if(direction_flag)
//         {
//             KP_turn = 57;
//           //  KI_turn = 30;
//         }
//         else
//         {
//             KP_turn = 58;
//            // KI_turn = 30;
//         }

         //Pwm=KP_turn*delta+KI_turn*Integral_delta_turn+KD_turn*(delta_s-Last_delta_turn);       //Î»ÖÃÊœPID¿ØÖÆÆ÷
         Pwm=KP_turn*delta+KD_turn*(delta_s-Last_delta_turn);       //Î»ÖÃÊœPID¿ØÖÆÆ÷
         Last_delta_turn=delta;
         Last_delta=0;
     }
//     if(fabs(Last_Pwm-Pwm)>3)
//     {
//            Pwm = (Last_Pwm + Pwm)/2;
//     }
     Last_Pwm = Pwm;
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
    if(fabs(P_Target.X-P_Start.X)>=fabs(P_Target.Y-P_Start.Y))
    {
        if(fabs(P_Target.Y-P_Start.Y)<=0.00001)
        {
           if(P_Target.X-P_Start.X>0)
           {
               out=P_Target.Y;
               if(direction_flag==true)
               {
                   return (out-P_Now.Y);
               }
               else
               {
                   return (P_Now.Y-out);
               }

           }
           else
           {
               out=P_Target.Y;
               if(direction_flag==true)
               {
                   return (P_Now.Y-out);
               }
               else
               {
                   return (out-P_Now.Y);
               }

           }
        }
        else
        {
            K_str=(double)(P_Target.Y-P_Start.Y)/(P_Target.X-P_Start.X);
            out=P_Target.Y-delta_x*K_str;
            return (out-P_Now.Y);
        }
    }
    else
    {
        if(fabs(P_Target.X-P_Start.X)<=0.00001)
        {
            if(P_Target.Y-P_Start.Y>0)
            {
                out=P_Target.X;
                if(direction_flag==true)
                {
                    return (P_Now.X-out);
                }
                else
                {
                    return (out-P_Now.X);
                }

            }
            else
            {
                out=P_Target.X;
                if(direction_flag==true)
                {
                    return (out-P_Now.X);
                }
                else
                {
                    return (P_Now.X-out);
                }

            }
        }
        else
        {
        K_str=(double)(P_Target.X-P_Start.X)/(P_Target.Y-P_Start.Y);
        out=P_Target.X-delta_y*K_str;
        return (out-P_Now.X);
        }
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
    double yawTarget_Delta=0;
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
    if(Distance_NowToTar_sq<=0.1)//到达目标点，精度为30cm(出弯点)
    {
       if(targetNumber-num-1 == 1)
       {
           if(direction_flag)
           {
               wheelMoveSpeedSet = 0.1;
               wheelSpeedHold = wheelMoveSpeedSet;
           }
           else
           {
               wheelMoveSpeedSet = -0.1;
               wheelSpeedHold = wheelMoveSpeedSet;
           }
       }
       num++;
       turn_flag=false;
       num_AddSpeed = num;
       //Centre_num++; //圆心加1

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

            yawTarget_Delta = yawTarget-yawTarget_Last;
            if(yawTarget_Delta>180)
            {
                yawTarget_Delta=yawTarget_Delta-360;
            }else if(yawTarget_Delta>=(-180))
            {
                yawTarget_Delta=yawTarget_Delta;
            }else
            {
                yawTarget_Delta=yawTarget_Delta+360;
            }

            if(direction_flag==true)
            {
                if(yawTarget_Delta>0)
                {
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
                }
                else
                {
                    if(Angle>0)               //朝外
                    {
                        if(delta_angle>0)
                            return (Angle);
                        else
                            return 0;
                    }else
                    {
                        if(delta_angle>0)
                             return 0;
                        else
                            return (Angle);
                    }
                }

            }
            else
            {
                if(yawTarget_Delta<0)
                {
                    if(Angle>0)               //朝外
                    {
                        if(delta_angle<0)
                            return 0;
                        else
                            return (-Angle);
                    }else
                    {
                        if(delta_angle<0)
                             return (-Angle);
                        else
                            return 0;
                    }
                }
                else
                {
                    if(Angle>0)               //朝外
                    {
                        if(delta_angle<0)
                            return (Angle);
                        else
                            return 0;
                    }else
                    {
                        if(delta_angle<0)
                             return 0;
                        else
                            return (Angle);
                    }
                }

            }

    return 0;


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
  if(Dis_NowToTar<0.0025)
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
/***************************************   走函数(加直线)      **************************************************/
double Datashare::Go2 (Position P_Targets )
{
   double Dis_NowToTar=0;
   double Dis_SpeedAdd = 0;
   double Dis_Stop = 0;  //到停车点停车
   double Dis_protec = 0;  //检测不到二维码停车
   double Distance_stop=0;
   //int circle_N=0;
   double R=0;
   bool Straight_Bend = false;
   double a=0.0025;
   Vector Vector_Now={0,0,0};
   Vector Vector_NowTar ={0,0,0};
   Vector Vector_Cross_Product = {0,0,0};          //左右转弯判断
   //double Angle_NowTar=0;
   //static int Num_Bend = 0;

  Dis_NowToTar=(AGVLocation.X-P_Targets.X)*(AGVLocation.X-P_Targets.X)+(AGVLocation.Y-P_Targets.Y)*(AGVLocation.Y-P_Targets.Y);

  Dis_Stop=(AGVLocation.X-P_Target[targetNumber-1].X)*(AGVLocation.X-P_Target[targetNumber-1].X)+(AGVLocation.Y-P_Target[targetNumber-1].Y)*(AGVLocation.Y-P_Target[targetNumber-1].Y);

  Dis_protec = (AGVLocation.X-P_protection.X)*(AGVLocation.X-P_protection.X)+(AGVLocation.Y-P_protection.Y)*(AGVLocation.Y-P_protection.Y);

  Dis_SpeedAdd = (AGVLocation.X-P_Target[num_AddSpeed].X)*(AGVLocation.X-P_Target[num_AddSpeed].X)+(AGVLocation.Y-P_Target[num_AddSpeed].Y)*(AGVLocation.Y-P_Target[num_AddSpeed].Y);

  if(targetNumber-num-1 >= 3)
  {
      if((Dis_SpeedAdd <= 0.01) && (turn_flag == false))  //出弯一个点后加速
      {
           Flag_SpeedAdd = true;
           Flag_SpeedDe = false;
           //num_AddSpeed = 120;
      }
  }
  else
  {
      if(direction_flag == true)
      {
          switch((Num_Turn+4)%4) //确定停车距离
          {
              case 0:
              {
                  if(AGVLocation.Y >= P_Target[targetNumber-1].Y)
                  {
                      breakFlag = true;
                      wheelMoveSpeedSet = 0;
                  }
                  break;
              }
              case 1:
              {
                  if(AGVLocation.X <= P_Target[targetNumber-1].X)
                  {
                      breakFlag = true;
                      wheelMoveSpeedSet = 0;
                  }
                  break;
              }
              case 2:
              {
                  if(AGVLocation.Y <= P_Target[targetNumber-1].Y)
                  {
                      breakFlag = true;
                      wheelMoveSpeedSet = 0;
                  }
                  break;
              }
              case 3:
              {
                  if(AGVLocation.X >= P_Target[targetNumber-1].X)
                  {
                      breakFlag = true;
                      wheelMoveSpeedSet = 0;
                  }
                  break;
              }
            default: break;
          }
      }
      else
      {
          switch((Num_Turn+4)%4) //确定停车距离
          {
              case 0:
              {
                  if(AGVLocation.Y <= P_Target[targetNumber-1].Y)
                  {
                      breakFlag = true;
                      wheelMoveSpeedSet = 0;
                  }
                  break;
              }
              case 1:
              {
                  if(AGVLocation.X >= P_Target[targetNumber-1].X)
                  {
                      breakFlag = true;
                      wheelMoveSpeedSet = 0;
                  }
                  break;
              }
              case 2:
              {
                  if(AGVLocation.Y >= P_Target[targetNumber-1].Y)
                  {
                      breakFlag = true;
                      wheelMoveSpeedSet = 0;
                  }
                  break;
              }
              case 3:
              {
                  if(AGVLocation.X <= P_Target[targetNumber-1].X)
                  {
                      breakFlag = true;
                      wheelMoveSpeedSet = 0;
                  }
                  break;
              }
            default: break;
         }
     }
      if(Dis_Stop<=0.0016 )
      {

          switch((Num_Turn+4)%2) //确定停车距离
          {
              case 0:
              {
                  Distance_stop = (AGVLocation.X-P_Target[targetNumber-1].X)*(AGVLocation.X-P_Target[targetNumber-1].X)+0.00003;
                  break;
              }
              case 1:
              {
                  Distance_stop = (AGVLocation.Y-P_Target[targetNumber-1].Y)*(AGVLocation.Y-P_Target[targetNumber-1].Y)+0.00003;
                  break;
              }
            default: break;
          }
/***
          if(direction_flag == true)
          {
              switch((Num_Turn+4)%4) //确定停车距离
              {
                  case 0:
                  {
                      if(AGVLocation.Y < P_Target[targetNumber-1].Y)
                         Distance_stop = (AGVLocation.X-P_Target[targetNumber-1].X)*(AGVLocation.X-P_Target[targetNumber-1].X)+0.00003;  //0.00005
                      else
                      {
                          breakFlag = true;
                          wheelMoveSpeedSet = 0;
                      }
                      break;
                  }
                  case 1:
                  {
                      if(AGVLocation.X > P_Target[targetNumber-1].X)
                         Distance_stop = (AGVLocation.Y-P_Target[targetNumber-1].Y)*(AGVLocation.Y-P_Target[targetNumber-1].Y)+0.00003;
                      else
                      {
                          breakFlag = true;
                          wheelMoveSpeedSet = 0;
                      }
                      break;
                  }
                  case 2:
                  {
                      if(AGVLocation.Y > P_Target[targetNumber-1].Y)
                          Distance_stop = (AGVLocation.X-P_Target[targetNumber-1].X)*(AGVLocation.X-P_Target[targetNumber-1].X)+0.00003;
                      else
                      {
                          breakFlag = true;
                          wheelMoveSpeedSet = 0;
                      }
                      break;
                  }
                  case 3:
                  {
                      if(AGVLocation.X < P_Target[targetNumber-1].X)
                          Distance_stop = (AGVLocation.Y-P_Target[targetNumber-1].Y)*(AGVLocation.Y-P_Target[targetNumber-1].Y)+0.00003;
                      else
                      {
                          breakFlag = true;
                          wheelMoveSpeedSet = 0;
                      }
                      break;
                  }
                default: break;
              }
          }
          else
          {
              switch((Num_Turn+4)%4) //确定停车距离
              {
                  case 0:
                  {
                      if(AGVLocation.Y > P_Target[targetNumber-1].Y)
                         Distance_stop = (AGVLocation.X-P_Target[targetNumber-1].X)*(AGVLocation.X-P_Target[targetNumber-1].X)+0.00003;
                      else
                      {
                          breakFlag = true;
                          wheelMoveSpeedSet = 0;
                      }
                      break;
                  }
                  case 1:
                  {
                      if(AGVLocation.X < P_Target[targetNumber-1].X)
                         Distance_stop = (AGVLocation.Y-P_Target[targetNumber-1].Y)*(AGVLocation.Y-P_Target[targetNumber-1].Y)+0.00003;
                      else
                      {
                          breakFlag = true;
                          wheelMoveSpeedSet = 0;
                      }
                      break;
                  }
                  case 2:
                  {
                      if(AGVLocation.Y < P_Target[targetNumber-1].Y)
                          Distance_stop = (AGVLocation.X-P_Target[targetNumber-1].X)*(AGVLocation.X-P_Target[targetNumber-1].X)+0.00003;
                      else
                      {
                          breakFlag = true;
                          wheelMoveSpeedSet = 0;
                      }
                      break;
                  }
                  case 3:
                  {
                      if(AGVLocation.X > P_Target[targetNumber-1].X)
                          Distance_stop = (AGVLocation.Y-P_Target[targetNumber-1].Y)*(AGVLocation.Y-P_Target[targetNumber-1].Y)+0.00003;
                      else
                      {
                          breakFlag = true;
                          wheelMoveSpeedSet = 0;
                      }
                      break;
                  }
                default: break;
             }
          }
         ****/
      }
      if(Dis_Stop <= Distance_stop) //停车
      {
          breakFlag = true;
          wheelMoveSpeedSet = 0;
          Flag_SpeedAdd = false;
          Flag_SpeedDe = false;
          Flag_Stop = false;
      }
  }


  if(turn_flag == false)
  {
      if(Dis_protec >= 7) //检测不到二维码停车
      {
          wheelMoveSpeedSet = 0;
          lostQRCodeFlag = true;
//          breakFlag = true;
//          fileClearFlag =false;
      }
  }

  if(turn_flag==true)
      a=0.04;
  else
      a=0.04;

  if(breakFlag == false)  //停车标志
  {
      if(Dis_NowToTar<a)//直线点以及入弯点精度
      {
              //num++;
               if(targetNumber-num-1 <= 4)  //是否快到终点
               {
                  if(P_Target[num+1].X==P_Target[targetNumber-1].X)
                  {
                      if(P_Target[num+1].Y==P_Target[targetNumber-1].Y)
                      {
                          Flag_SpeedAdd = false;
                          Flag_SpeedDe = false;
                          Flag_Stop = true;  //停车前减速标志位
                      }
                  }

                  if(fabs(AGVSpeed) > 0.6 )
                  {
                      if(P_Target[num+4].X==P_Target[targetNumber-1].X)
                      {
                          if(P_Target[num+4].Y==P_Target[targetNumber-1].Y)
                          {
                              if(direction_flag == true)
                                  wheelMoveSpeedSet = 0.6;  //停车前四个二维码减速
                              else
                                  wheelMoveSpeedSet = -0.6;
                          }
                      }
                  }
                  if(fabs(AGVSpeed) > 0.3 )
                  {
                      if(P_Target[num+3].X==P_Target[targetNumber-1].X)
                      {
                          if(P_Target[num+3].Y==P_Target[targetNumber-1].Y)
                          {
                              if(direction_flag == true)
                                  wheelMoveSpeedSet = 0.3;  //停车前三个二维码减速
                              else
                                  wheelMoveSpeedSet = -0.3;
                          }
                      }
                  }

                  if(fabs(wheelMoveSpeedSet) > 0.3 )
                  {
                      if(P_Target[num+2].X==P_Target[targetNumber-1].X)
                      {
                          if(P_Target[num+2].Y==P_Target[targetNumber-1].Y)
                          {
                              if(direction_flag == true)
                                  wheelMoveSpeedSet = 0.3;  //停车前三个二维码减速
                              else
                                  wheelMoveSpeedSet = -0.3;
                          }
                      }
                  }

                }
                for(int i=0;i<numberOfTurnCentre;i++)
                {
                      if(P_Target[num+1].X==P_Target3[i].X)
                      {
                          if(P_Target[num+1].Y==P_Target3[i].Y)
                          {
                              Flag_SpeedAdd = false; //入弯提前一个点减速
                              Flag_SpeedDe = true;
                              break;
                          }
                      }

                      if(fabs(AGVSpeed) > 0.5)  //高速入弯提前两个点减速
                      {
                          if(P_Target[num+2].X==P_Target3[i].X)
                          {
                              if(P_Target[num+2].Y==P_Target3[i].Y)
                              {
                                  if(direction_flag == true)
                                      wheelMoveSpeedSet = 0.5;
                                  else
                                      wheelMoveSpeedSet = -0.5;
                                  break;
                              }
                          }
                      }

                      if(P_Target[num].X==P_Target3[i].X)
                      {
                          if(P_Target[num].Y==P_Target3[i].Y)
                          {
                              P_protection.X = P_Target[num+1].X;
                              P_protection.Y = P_Target[num+1].Y;
                              P_Centre = P_Target4[i];
                              Straight_Bend =true;
                              break;
                          }
                      }
                }

                  if(Straight_Bend==true)
                  {
                        Vector_Now.X = P_Target[num].X- P_Target[num-1].X;
                        Vector_Now.Y = P_Target[num].Y- P_Target[num-1].Y;
                        Vector_Now.Z = 0;
                        Vector_NowTar.X = P_Target[num+1].X- P_Target[num].X;
                        Vector_NowTar.Y = P_Target[num+1].Y- P_Target[num].Y;
                        Vector_NowTar.Z = 0;
                        Vector_Cross_Product.Z = Vector_Now.X*Vector_NowTar.Y-Vector_Now.Y*Vector_NowTar.X;
                        if(Vector_Cross_Product.Z>0)
                        {
                            yawTarget_Last = yawTarget;
                            Num_Turn+=1;
                           yawTarget+=90;
                        }
                        if(Vector_Cross_Product.Z<0)
                        {
                            yawTarget_Last = yawTarget;
                            Num_Turn-=1;
                            yawTarget-=90;
                        }
                      //yawTarget+=(90);
                      //Angle_Bend+=90;
                      if(yawTarget>360)
                         yawTarget-=360;
                      if(yawTarget<0)
                          yawTarget+=360;
                      turn_flag=true;
                  }
                  else
                  {
                      //num++;
                      //turn_flag=false;
                  }
                  if(turn_flag!=true)//位置不能变
                  {
                     num++;
                  }
                  if(turn_flag==true&&Straight_Bend==true)//弯道入口处
                  {
                     num++;

                  }

              while(Num_Turn<0||Num_Turn>=4)
              {
                  if(Num_Turn<0)
                  {
                      Num_Turn+=4;
                  }
                  if(Num_Turn>=4)
                  {
                      Num_Turn-=4;
                  }

              }
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

/***************************************   位置ESO   **************************************************/
void Datashare::ESO(float u,float bk,float output,float h)
{
    double e;
    int sign;
    double u0,u1;

    //z1_Position ... 初值如何设置？？？
    e = z1_Position - output;

    //sign函数
    if(e>0)
        sign = 1;
    if(e<0)
        sign = -1;

    //fal函数
    if(fabs(e)>0.01)
    {
        u0 = sign*(pow(fabs(e),0.5));
        u1 = sign*(pow(fabs(e),0.25));
    }
    else
    {
        u0 = e/(pow(0.01,0.5));
        u1 = e/(pow(0.01,0.25));
    }

    z1_Position += h*(z2_Position-beta1*e);
    z2_Position += h*(z3_Position-beta2*(u0) + bk*u );
    z3_Position += h*(-beta3*u1);

}

/*************************************fhan        *************************************************************************************/
float Datashare::fhan(float x1,float x2, float r, float h)
{
    double td_y=0;
    double a0=0;
    double a=0;
    double u=0;
    double d=0;
    double d0=0;
    int sign =0;
    //float flag_y=0;
    //float flag_a=0;

    d=r*h;
    d0=h*d;
    td_y = x1+h*x2;
    a0 = sqrt(d*d+8*r*fabs(td_y));
    //if(td_y>0) flag_y=1;           //sign(td_y);
    //else 	   flag_y=-1;
    if(td_y>0)
        sign = 1;
    if(td_y<0)
        sign = -1;

    if(fabs(td_y)>d0)
        a=x2+0.5*(a0-d)*sign;
    else
        a=x2+td_y/h;

    //if(a>0) flag_a=1;
    //else 	flag_a=-1;
    if(a>0)
        sign = 1;
    if(a<0)
        sign = -1;

    if (fabs(a)>d)
        u=-r*sign;
    else
        u=-r*a/d;
    return(u);
}

//速度调节
void Datashare::Speed_Adj(void)
{
    if(fabs(Speed_Td_x1-wheelMoveSpeedSet) < 0.01)  //最后给定速度值固定不变
    {

        Speed_Td_x1 = wheelMoveSpeedSet;
        Speed_Td_x2 = 0;
    }
    else
    {
        Speed_Td_x1 += Speed_h*Speed_Td_x2; //速度TD
        Speed_Td_x2 += Speed_h* fhan(Speed_Td_x1-wheelMoveSpeedSet,Speed_Td_x2,Speed_r,Speed_h);
    }


    Td_SpeedSet= Speed_Td_x1;

    if(direction_flag == true)
    {
        if(Flag_Stop == true)
        {
            if(((AGVLocation.X-P_Target[targetNumber-1].X)*(AGVLocation.X-P_Target[targetNumber-1].X)+(AGVLocation.Y-P_Target[targetNumber-1].Y)*(AGVLocation.Y-P_Target[targetNumber-1].Y))>0.04)
            {
                wheelMoveSpeedSet = 0.1;
                wheelSpeedHold = wheelMoveSpeedSet;
            }
            else
            {
                wheelMoveSpeedSet = 0.02;
                wheelSpeedHold = wheelMoveSpeedSet;
            }
            //Flag_Stop = false;  //这会不会有问题
        }
        else
        {
            if(Flag_SpeedDe == true) //入弯减速
            {
                wheelMoveSpeedSet = wheelTurnSpeed;
                wheelSpeedHold = wheelMoveSpeedSet;
                Flag_SpeedDe = false;
            }
    
            if(Flag_SpeedAdd == true) //出弯加速
            {
                wheelMoveSpeedSet = wheelSpeedTarget;
                wheelSpeedHold = wheelMoveSpeedSet;
                Flag_SpeedAdd = false;
            }
        }
    
    }
    else
    {
        if(Flag_Stop == true)
        {
            if(((AGVLocation.X-P_Target[targetNumber-1].X)*(AGVLocation.X-P_Target[targetNumber-1].X)+(AGVLocation.Y-P_Target[targetNumber-1].Y)*(AGVLocation.Y-P_Target[targetNumber-1].Y))>0.04)
            {
                wheelMoveSpeedSet = -0.1;
                wheelSpeedHold = wheelMoveSpeedSet;
            }
            else
            {
                wheelMoveSpeedSet = -0.02;
                wheelSpeedHold = wheelMoveSpeedSet;
            }
        }
        else
        {
            if(Flag_SpeedDe == true) //入弯减速
            {
                wheelMoveSpeedSet = -wheelTurnSpeed;
                wheelSpeedHold = wheelMoveSpeedSet;
                Flag_SpeedDe = false;
            }

            if(Flag_SpeedAdd == true) //出弯加速
            {
                wheelMoveSpeedSet = wheelSpeedTarget;
                wheelSpeedHold = wheelMoveSpeedSet;
                Flag_SpeedAdd = false;
            }
        }

    }

}

void Datashare::Code_Init()
{
    int Target_Location;
    int i=0;
    unsigned char arrayTemp[50];

    Speed_h=0.03;
    yaw_error = 0;

//    if(buf!=buf_last)//此处加扫描到二维码的判断信息
    {
        if(buf!=NULL)
        {
            QR_Flag=Two_bar_codes_Pro2(buf);
        }
    }

   if(QR_Flag==true)
   {
       //加上位置，角度处理函数
       Target_Location=(QR_Code_Number-1)/4;      
       for(i=0;i<targetNumber;i++)
       {
            if( ((P_Target[i].X-P_TwoCode[Target_Location].X)*(P_Target[i].X-P_TwoCode[Target_Location].X)+(P_Target[i].Y-P_TwoCode[Target_Location].Y)*(P_Target[i].Y-P_TwoCode[Target_Location].Y)) < 0.1)
            {
                AGVLocation.X = P_Target[i].X;
                AGVLocation.Y = P_Target[i].Y;                
                num = i;
                break;
            }

       }

       P_Stop.X = P_Target[targetNumber-1].X;
       P_Stop.Y = P_Target[targetNumber-1].Y;
       turn_flag = false;
       num_AddSpeed =targetNumber-1; //出弯加速点出始化

       if(fabs(Angle_QRtoCar-180)<20)//确定车身方向
           Num_Turn = 0; //y正
       else if(fabs(Angle_QRtoCar-90)<20)
           Num_Turn = 3; //x正
       else if(fabs(Angle_QRtoCar-270)<20)
           Num_Turn = 1; //x负
       else
           Num_Turn = 2; //y负

       switch((Num_Turn+4)%4)
       {
           case 0:
           {
              if(P_Target[num+1].Y-P_Target[num].Y>0)
              {
                  direction_flag = true;
                  break;
              }
              if(P_Target[num+1].Y-P_Target[num].Y<0)
              {
                  direction_flag = false;
                  break;
              }
           }
           case 1:
           {
               if(P_Target[num+1].X-P_Target[num].X>0)
               {
                   direction_flag = false;
                   break;
               }
               if(P_Target[num+1].X-P_Target[num].X<0)
               {
                   direction_flag = true;
                   break;
               }
           }
           case 2:
           {
              if(P_Target[num+1].Y-P_Target[num].Y>0)
              {
                  direction_flag = false;
                  break;
              }
              if(P_Target[num+1].Y-P_Target[num].Y<0)
              {
                  direction_flag = true;
                  break;
              }
           }
           case 3:
           {
               if(P_Target[num+1].X-P_Target[num].X>0)
               {
                   direction_flag = true;
                   break;
               }
               if(P_Target[num+1].X-P_Target[num].X<0)
               {
                   direction_flag = false;
                   break;
               }
           }
           default: break;
       }

       P_Target[targetNumber].X=P_Target[targetNumber-1].X;
       P_Target[targetNumber].Y=P_Target[targetNumber-1].Y;
       P_Target[targetNumber+1].X=P_Target[targetNumber-1].X;
       P_Target[targetNumber+1].Y=P_Target[targetNumber-1].Y;

       //读惯导角度
       write(fd6,readInertialBuff,sizeof(readInertialBuff));
       delayTimeMsecs(8);
       int numberOfRead = read(fd6,arrayTemp,sizeof(arrayTemp));if(numberOfRead <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
       //ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd6)).toHex());
       yaw = angle_trans(arrayTemp[4],arrayTemp[3])-yaw_error;
       while(yaw>=360||yaw<0)
       {
           if(yaw>=360)
               yaw-=360;
           if(yaw<0)
               yaw+=360;
       }

       yaw_error = Information_Corrective();
       QR_Flag=false;
       buf_last=buf;
       //mptr.buf.clear();
   }

}


/***********************        自动生成路径坐标函数        *****************************/
int Datashare::Trace(int x1,int y1,int x2,int y2, int init )
{
    int num_cor;
    int plus_dec;
    if (x1 == x2)
    {
        num_cor = abs(y1 -y2);
        P_Target[init-1].X = x1;
        P_Target[init-1].Y = y1;
        P_Target[init-1+num_cor].X = x2;
        P_Target[init-1+num_cor].Y = y2;
        if (y2 > y1)
            plus_dec = 1;
        else plus_dec = -1;
        for (int i = init; i <init-1+ num_cor; i++)
        {
            P_Target[i].X = x1;
            P_Target[i].Y = y1 + plus_dec*(i-init+1);
        }
    }
    if (y1 == y2)
    {
        num_cor = abs(x1 - x2);
        P_Target[init-1].X = x1;
        P_Target[init-1].Y = y1;
        P_Target[init-1+num_cor].X = x2;
        P_Target[init-1+num_cor].Y = y2;
        if (x2 > x1)
            plus_dec = 1;
        else plus_dec = -1;
        for (int i = init; i < init-1+num_cor; i++)
        {
            P_Target[i].X = x1 + plus_dec*(i-init+1);
            P_Target[i].Y = y1 ;
        }
    }
    init = init + num_cor+1;
    return init;
}

/*************    读电池参数    ******************************/
void Datashare::readBattery(void)
{
    unsigned char array[50]={0};
    unsigned char array2[50] ={0};
    unsigned char array3[50] = {0};
    unsigned char array4[50] = {0};
    int number = 0;
    QString str;
    int batteryCollumTemp=0;
    int batteryCurrentTemp=0;
    int batteryVoltageTemp=0;
    int batteryErrorTemp = 0;

    tcflush(fd5,TCIOFLUSH);

    write(fd5,readBatteryVoltage,sizeof(readBatteryVoltage));
    delayTimeMsecs(25);
    number = read(fd5,array3,sizeof(array3));
    batteryVoltage = array3[3] * 255 + array3[4];
    batteryVoltage = batteryVoltage * 0.001;


    while((batteryVoltageTemp != batteryVoltage)||(batteryCurrentTemp != batteryCurrent)
          ||(batteryCollumTemp != batteryCollum))
    {
        batteryVoltageTemp = batteryVoltage;
        write(fd5,readBatteryVoltage,sizeof(readBatteryVoltage));
        delayTimeMsecs(25);
        number = read(fd5,array3,sizeof(array3));
        batteryVoltage = array3[3] * 256 + array3[4];
        batteryVoltage = batteryVoltage * 0.001;
        for(int i=0;i<number;i++)
        {
            if(array3[i]<16)
                str += '0' + QString::number(array3[i],16).toUpper();
            else
                str += QString::number(array3[i],16).toUpper();
        }
        batteryArray3 = str;
        str.clear();


        write(fd5,readBatteryError,sizeof(readBatteryError));
        delayTimeMsecs(25);
        number = read(fd5,array4,sizeof(array4));
        for(int i=0;i<number;i++)
        {
            if(array4[i]<16)
                str += '0' + QString::number(array4[i],16).toUpper();
            else
                str += QString::number(array4[i],16).toUpper();
        }
        batteryArray4 = str;
        str.clear();

        batteryCollumTemp = batteryCollum;
        write(fd5,readBatteryCollum,sizeof(readBatteryCollum));
        delayTimeMsecs(25);
        number = read(fd5,array,sizeof(array));
        batteryCollum = array[3] * 256 + array[4];
        batteryCollum = batteryCollum * 0.4;
        for(int i=0;i<number;i++)
        {
            if(array[i]<16)
                str += '0' + QString::number(array[i],16).toUpper();
            else
                str += QString::number(array[i],16).toUpper();
        }
        batteryArray = str;
        str.clear();

        batteryCurrentTemp = batteryCurrent;
        write(fd5,readBatteryCurrent,sizeof(readBatteryCurrent));
        delayTimeMsecs(25);
        number = read(fd5,array2,sizeof(array2));
        batteryCurrent = array2[3] * 16 + array2[4];
        batteryCurrent = batteryCurrent * 0.01;
        for(int i=0;i<number;i++)
        {
            if(array[i]<16)
                str += '0' + QString::number(array2[i],16).toUpper();
            else
                str += QString::number(array2[i],16).toUpper();
        }
        batteryArray2 = str;
        str.clear();
    }
}

void Datashare::warningRecord(void)
{
    if(!errorReportFlag)
    {
        int number = 0;
        int number1 = 0;
        unsigned char array[50] = {0};
        unsigned char array1[50] = {0};
        bool communicationErrorFlag = false;
        int fdfile = 0;
        warningFile.open(QIODevice::WriteOnly|QIODevice::Append);
        QTextStream out(&warningFile);
        QString a,b;

        for(int i =0;i<6;i++)
        {
           switch (i) {
           case 0:
               fdfile = fd1;break;
           case 1:
               fdfile = fd2;break;
           case 2:
               fdfile = fd3;break;
           case 3:
               fdfile = fd4;break;
           case 4:
               fdfile = fd5;break;
           case 5:
               fdfile = fd6;break;
           default:
               break;
           }
           if(0==i | 1==i | 2==i | 3==i)
           {
               write(fdfile,readDriveBridgeStatus,sizeof(readDriveBridgeStatus));
               delayTimeMsecs(100);
               number = read(fdfile,array,sizeof(array));
           }
           if(4 == i)
           {
               write(fdfile,seri_send_buzzer1,sizeof(seri_send_buzzer1));
               delayTimeMsecs(100);
               number = read(fdfile,array,sizeof(array));
           }
           if(5 == i)
           {
               write(fdfile,readInertialBuff,sizeof(readInertialBuff));
               delayTimeMsecs(100);
               number = read(fdfile,array,sizeof(array));
           }
           if(number <= 0)
           {
               //a += QTime::currentTime().toString();
               //a += "  ";
               a += b.setNum(i+1);
               a += "号串口通信故障";
               errorInformation += a + "  ";
               a.prepend("  ");
               a.prepend(QTime::currentTime().toString());
               out<<a<<endl;
               a.clear();b.clear();
               communicationErrorFlag = true;
           }
           //memset(array,0,sizeof(unsigned char)*20);                   //清空array数组
           for(int i=0;i<50;i++)
           {
               array[i] = 0;
           }
        }
        if(!communicationErrorFlag)
        {
    //       out<<QTime::currentTime().toString()<<'\t';
           //a += QTime::currentTime().toString() + "  ";
           for(int i=0;i<4;i++)
           {
               switch (i) {
               case 0:
                   fdfile = fd1;
                   break;
               case 1:
                   fdfile = fd2;break;
               case 2:
                   fdfile = fd3;break;
               case 3:
                   fdfile = fd4;break;
               default:
                   break;
               }
               //memset(array,0,sizeof(unsigned char)*20);               //清空array数组
               for(int i=0;i<50;i++)
               {
                   array[i] = 0;
               }
               write(fdfile,readDriveProtectionStatus,sizeof(readDriveProtectionStatus));
               number = read(fdfile,array,sizeof(array));
               if(array[8]&0x01)  {driveReset[i]=1;         a += "驱动器"+ b.setNum(i+1) + "drive reset" + "  ";}
               if(array[8]&0x02)  {driveInternalEror[i]=1;  a += "驱动器"+ b.setNum(i+1) + "internal error"+ "  ";}
               if(array[8]&0x04)  {shortCircuit[i]=1;       a += "驱动器"+ b.setNum(i+1) + "short circuit" + "  ";}
               if(array[8]&0x08)  {currentOverShoot[i]=1;   a += "驱动器"+ b.setNum(i+1) + "current over shoot" + "  ";}
               if(array[8]&0x10)  {underVoltage[i]=1;       a += "驱动器"+ b.setNum(i+1) + "under voltage" + "  ";}
               if(array[8]&0x20)  {overVoltage[i]=1;        a += "驱动器"+ b.setNum(i+1) + "over voltage" + "  ";}
               if(array[8]&0x40)  {driveOverTemprature[i]=1;a += "驱动器"+ b.setNum(i+1) + "over temprature" + "  ";}

    //           memset(array,0,sizeof(unsigned char)*20);
               for(int i=0;i<50;i++)
               {
                   array[i] = 0;
               }
               write(fdfile,readSystemProtectionStatus,sizeof(readSystemProtectionStatus));
               number=read(fdfile,array,sizeof(array));
               if(array[8]&0x01)  {parameterRestoreError[i]=1;      a += "驱动器"+ b.setNum(i+1) + "parameter Restore Error" + "  ";}
               if(array[8]&0x02)  {parameterStoreError[i] = 1;      a += "驱动器"+ b.setNum(i+1) + "parameter Store Error" + "  " ;}
               if(array[8]&0x04)  {invalidHallState[i] = 1;         a += "驱动器"+ b.setNum(i+1) + "invalid Hall State" + "  ";}
               if(array[8]&0x08)   {phaseSyncError[i] = 1;          a += "驱动器"+ b.setNum(i+1) + "phase Sync Error" + "  ";}
               if(array[8]&0x10)   {motorOverTemprature[i]=1;       a += "电机"+ b.setNum(i+1) + "过温度" + "  ";}
               if(array[8]&0x20)   {phaseDetectionFault[i]=1;       a += "驱动器"+ b.setNum(i+1) + "phase Detection Fault" + "  ";}
               if(array[8]&0x40)   {feedBackSensorError[i]=1;       a += "驱动器"+ b.setNum(i+1) + "传感器反馈错误" + "  ";}
               if(array[8]&0x80)   {motorOverSpeed[i]=1;            a += "驱动器"+ b.setNum(i+1) + "电机失速" + "  ";}
               if(array[9]&0x01)   {}
               if(array[9]&0x02)   {}
               if(array[9]&0x04)   {comError[i]=1;                  a += "驱动器"+ b.setNum(i+1) + "驱动器与舵轮通信错误" + "  ";}
               if(array[9]&0x08)   {PWMandDirBrokenWire[i];         a += "驱动器"+ b.setNum(i+1) + "PWM和方向断线"+ "  ";}
               if(array[9]&0x10)   {motionEngineError[i];           a += "驱动器"+ b.setNum(i+1) + "电机引擎错误" + "  ";}
               if(array[9]&0x20)   {motionEngineAbort[i];           a += "驱动器"+ b.setNum(i+1) + "电机引擎 abort" + "  ";}

    //           memset(array,0,sizeof(unsigned char)*20);
               for(int i=0;i<50;i++)
               {
                   array[i] = 0;
               }
               write(fdfile,readDriveSystemStatus2,sizeof(readDriveSystemStatus2));
               number1=read(fdfile,array1,sizeof(array1));
               if(array1[8]&0x04)    {velocityFollowingError[i]=1;   a+= "驱动器"+ b.setNum(i+1) + "速度跟随错误" + "  "; }
               if(array1[8]&0x80)    {positionFollowingError[i]=1;   a+= "驱动器"+ b.setNum(i+1) + "位置跟随错误" + "  ";}
            //    if(array1[9]&0x04)    {velocityFollowingError[i]=1;   a+= "驱动器"+ b.setNum(i+1) + "速度跟随错误" + "  "; }
            //    if(array1[9]&0x80)    {positionFollowingError[i]=1;   a+= "驱动器"+ b.setNum(i+1) + "位置跟随错误" + "  ";}
               if(array1[8]&0x01)    {zeroVelocity[i]=1;             a+= "舵轮"+ b.setNum(i+1) + "零速度" + "  ";}
    //           for(int i=0;i<number1;i++)
    //           {
    //               if(array1[i]<16)
    //                   a += '0' + QString::number(array1[i],16).toUpper();
    //               else
    //                   a += QString::number(array1[i],16).toUpper();
    //           }

               //错误代码转变为变量
           }
    //       errorInformation.clear();
           errorInformation += a;
           a.prepend("  ");
           a.prepend(QTime::currentTime().toString());

            out<<a<<endl;
            a.clear();b.clear();
        }

        out.flush();

        warningFile.close();
        breakFlag = true;
        fileClearFlag = false;
        errorReportFlag = true;
    }

}

/*********************************差速PID  *****************************/
double Datashare::Position_PID3 (double delta)
{
     double Pwm=0;
     static double Last_delta=0;
     static double Last_delta_turn=0;
     static double Last_Pwm=0;

     if(delta > 180)
         delta = delta - 360;
     if(delta < -180)
         delta = delta + 360;
     Pwm=0.01*delta+0.005*(delta-Last_delta_turn);       //Î»ÖÃÊœPID¿ØÖÆÆ÷
     Last_delta_turn=delta;
     Last_delta=0;

     Last_Pwm = Pwm;
     if (Pwm>0.04) Pwm = 0.04;
     if (Pwm<-0.04) Pwm = -0.04;
     return Pwm;                                           //ÔöÁ¿Êä³ö
}

void Datashare::readIO1()
{
    int numberOfRead = 0;
    unsigned char array[20]={0};
    unsigned char array3[20]={0};
    unsigned char arrayTemp[20]={0};
    // //read the speed of wheel
     write(fd1,readSpeedData,sizeof(readSpeedData));//fflush(stdout);
     numberOfRead = read(fd1,array,sizeof(array));
     if(numberOfRead <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
     wheelMoveSpeedReadFront = convertTelegramHex2Speed(array);

    // memset(array,0,14*sizeof(unsigned char));
     write(fd3,readSpeedData,sizeof(readSpeedData));//fflush(stdout);
     numberOfRead = read(fd3,array3,sizeof(array3));
     if(numberOfRead <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
     wheelMoveSpeedReadRear = convertTelegramHex2Speed(array3);

    write(fd6,readInertialBuff,sizeof(readInertialBuff));
    delayTimeMsecs(8);
    numberOfRead = read(fd6,arrayTemp,sizeof(arrayTemp));
    if(numberOfRead <= 0)  {warningRecord();breakFlag=true;fileClearFlag=false;}
    //ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd6)).toHex());
    yaw = angle_trans(arrayTemp[4],arrayTemp[3])-yaw_error;
    while(yaw>=360||yaw<0)
    {
        if(yaw>=360)
            yaw-=360;
        if(yaw<0)
            yaw+=360;
    }
   // if ((yaw - yawLast > 20)||(yaw - yawLast) < -20)	yaw = yawLast;	//滤波

    AGVSpeed=(wheelMoveSpeedReadFront+wheelMoveSpeedReadRear)/2;
}

/*****************************************              ************************************************/


