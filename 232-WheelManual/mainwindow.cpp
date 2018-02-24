#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <wiringPi.h>
#include "wiringSerial.h"
#include "wiringPiI2C.h"
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include "datashare.h"
#include <QTime>
#include <QString>
#include <QByteArray>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QString c1;
    ui->speedEdit->setText(c1.setNum(mptr.wheelMoveSpeedSet));
    ui->addressEdit->setText(c1.setNum(mptr.wheelAddress));
    ui->angleEdit->setText(c1.setNum(mptr.wheelAngle));
    ui->timeEdit->setText(c1.setNum(mptr.delayTimeSet));
    //ui->rotateButton->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

/****************系统初始化（包括舵轮权限获取、使能、舵轮校零）并启动系统开始工作**************************************************************************/
void MainWindow::initialTheSystem(void)
{
	mptr.gainAccessAndEnableWheel();
	mptr.readIO();			
	ui->CommunicationEdit->append(tr("%1 %2 %3").arg(mptr.systemOnFlag).arg(mptr.sickFalse).arg(mptr.sickWarningSpaceAlert));
                                                                            //read I/O and store the data
    if (mptr.calibrationFlag == false)
        wheelZeroCalibration();                                             //find the zero point of the steering wheel

                                                    //Initial alignment of inertial navigation
                                                    //judge the status whether the system can operate
    systemOn();
}

/*****************系统运行函数****************************************************************************************************************************/
void MainWindow::systemOn(void)
{
//    float position = 0;
//    float destination = 600;
    int countLoop = 0;

    float speedBefore=1;
    bool loopFlag=true;
	unsigned char array[20]= {0};
    while(1)
    {
        countLoop += 1;
        if(countLoop > 6) break;
		QTime t1;
        t1 = QTime::currentTime().addSecs(10);							//延时1min
		while (QTime::currentTime()<t1)
        {
//			if (loopFlag == true)
//			{
//				mptr.wheelMoveSpeedSet +=500;
//				if (mptr.wheelMoveSpeedSet>1600)
//					mptr.wheelMoveSpeedSet = 500;
//                if(mptr.wheelMoveSpeedSet > mptr.wheelMoveSpeedSetMax)
//					mptr.wheelMoveSpeedSet = mptr.wheelMoveSpeedSetMax;
//			}
//			loopFlag =false;
            mptr.wheelMoveSpeedSet = 0;									//给舵轮速度为零（之后会由控制函数计算出来）
            if(mptr.wheelMoveSpeedSet > mptr.wheelMoveSpeedSetMax)
                mptr.wheelMoveSpeedSet = mptr.wheelMoveSpeedSetMax;

			if(mptr.breakFlag == false)
			{
                mptr.readIO();                          //读取数据，IO、舵机等,检查IO数据，输出对应IO数据
				ui->CommunicationEdit->append(tr("%1 %2 %3").arg(mptr.systemOnFlag).arg(mptr.sickFalse).arg(mptr.sickWarningSpaceAlert));
                showIOResult();                         // 界面显示读数结果，如果急停进入等待复位状态。

				if(mptr.emergencyFlag == false)			//emergencyFlag目前不使用，可忽略
				{
					if(speedBefore != mptr.wheelMoveSpeedSet)	//如果速度没变化不在对舵轮写速度。  
					{
						speedBefore = mptr.wheelMoveSpeedSet;
                        if (mptr.wheelMoveSpeedSet>mptr.wheelMoveSpeedSetMax)
                            mptr.wheelMoveSpeedSet = mptr.wheelMoveSpeedSetMax;
                        mptr.writeWheelSpeed(mptr.wheelMoveSpeedSet,00,mptr.commanData);	//生成速度设置报文
                        write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
                        read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
                        mptr.writeWheelSpeed(mptr.wheelMoveSpeedSet,00,mptr.commanData);
                        write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
                        read(mptr.fd3,array,sizeof(array));
					}
				//此处以后添加为WiFi串口读取函数，检查上位机命令
				}
				else
				{
					//报警并显示状态到触摸屏（函数未添加）
                    mptr.writeWheelSpeed(00,00,mptr.commanData);			//将速度置为零
                    write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));
                    read(mptr.fd1,array,sizeof(array));
					mptr.delayTimeMsecs(mptr.delayTimeSet);
                    mptr.writeWheelSpeed(00,00,mptr.commanData);
                    write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
                    read(mptr.fd3,array,sizeof(array));
				}
			}
			else
				break;
            //mptr.writeIO();                             // 数据输出（目前将此函数至于readIO末尾此处可以去掉）
			QApplication::processEvents(QEventLoop::AllEvents,1000);			//Qt任务队列等待，释放系统资源
		}
		loopFlag = true;
        if (mptr.breakFlag == true)												//手自变换时候，AGV停车
        {
            mptr.writeWheelSpeed(00,00,mptr.commanData);
            write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));
            read(mptr.fd1,array,sizeof(array));
            mptr.delayTimeMsecs(mptr.delayTimeSet);
            mptr.writeWheelSpeed(00,00,mptr.commanData);
            write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
            read(mptr.fd3,array,sizeof(array));
            break;
        }
    }
	mptr.writeWheelSpeed(00,00,mptr.commanData);								//任务队列完成后，停车
	write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));
	read(mptr.fd1,array,sizeof(array));
	mptr.delayTimeMsecs(mptr.delayTimeSet);
	mptr.writeWheelSpeed(00,00,mptr.commanData);
	write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
	read(mptr.fd3,array,sizeof(array));
	ui->CommunicationEdit->append("Halt Success!");
	mptr.sickA = 0;
    mptr.sickB = 0;
    mptr.sickC = 0;
	mptr.writeIO();
}

/***********************前进按钮，按压前进，速度为设定值***************************************************************************************/
void MainWindow::on_forwardButton_pressed()
{
    switch(mptr.wheelAddress)
    {
        case 1:mptr.writeWheelSpeed(mptr.wheelMoveSpeedSet,0,mptr.commanData);
                write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));//fflush(stdout);
                ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd1)).toHex());
                mptr.writeWheelSpeed(mptr.wheelMoveSpeedSet,0,mptr.commanData);
                write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));//fflush(stdout);
                ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
                break;
        case 3:mptr.writeWheelSpeed(mptr.wheelMoveSpeedSet,0,mptr.commanData);
                write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));//fflush(stdout);
                ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
                break;
        default: break;
    }
}

/************************前进按钮，弹起停车**************************************************************************************************/
void MainWindow::on_forwardButton_released()
{
    switch (mptr.wheelAddress) {
    case 1:
        mptr.writeWheelSpeed(00,00,mptr.write0RPM);
        write(mptr.fd1,mptr.write0RPM,sizeof(mptr.write0RPM));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd1)).toHex());
        mptr.writeWheelSpeed(00,00,mptr.write0RPM);
        write(mptr.fd3,mptr.write0RPM,sizeof(mptr.write0RPM));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
        break;
    case 3:
        mptr.writeWheelSpeed(00,00,mptr.write0RPM);
        write(mptr.fd3,mptr.write0RPM,sizeof(mptr.write0RPM));//fflush(stdout);
        //mptr.delayTimeMsecs(mptr.delayTimeSet);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
        break;
    default:
        break;
    }
}

/****************************后退按钮，按压后退，速度为设定值**********************************************************************************/
void MainWindow::on_backwardButton_pressed()
{
    switch (mptr.wheelAddress) {
    case 1:
        mptr.writeWheelSpeed(-mptr.wheelMoveSpeedSet,0,mptr.commanDataReverse);
        write(mptr.fd1,mptr.commanDataReverse,sizeof(mptr.commanDataReverse));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd1)).toHex());
        mptr.writeWheelSpeed(-mptr.wheelMoveSpeedSet,0,mptr.commanDataReverse);
        write(mptr.fd3,mptr.commanDataReverse,sizeof(mptr.commanDataReverse));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
        break;
    case 3:
        mptr.writeWheelSpeed(-mptr.wheelMoveSpeedSet,0,mptr.commanDataReverse);
        write(mptr.fd3,mptr.commanDataReverse,sizeof(mptr.commanDataReverse));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
        break;
    default:
        break;
    }
}

/*****************************后退按钮，弹起停车***********************************************************************************/
void MainWindow::on_backwardButton_released()
{
    switch (mptr.wheelAddress) {
    case 1:
        mptr.writeWheelSpeed(0, 0, mptr.write0RPM);
        write(mptr.fd1,mptr.write0RPM,sizeof(mptr.write0RPM));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd1)).toHex());
        mptr.writeWheelSpeed(0,0,mptr.write0RPM);
        write(mptr.fd3,mptr.write0RPM,sizeof(mptr.write0RPM));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
        break;
    case 3:
        mptr.writeWheelSpeed(0,0,mptr.write0RPM);
        write(mptr.fd3,mptr.write0RPM,sizeof(mptr.write0RPM));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
        break;
    default:
        break;
    }
}

/******************************速度设定按钮，增速，步长100rpm，最大值2400********************************************************************************/
void MainWindow::on_setSpeed_clicked()
{
    QString s;
    if (mptr.wheelMoveSpeedSet>=2400)
        mptr.wheelMoveSpeedSet = 2400;
    else
        mptr.wheelMoveSpeedSet += 100;
    ui->speedEdit->setText(s.setNum(mptr.wheelMoveSpeedSet));
}

/*******************************速度设定按钮，减速，步长100rpm，最小值0*******************************************************************************/
void MainWindow::on_setButton2_clicked()//speed decrease
{
    QString s;
    std::string str;
    if (mptr.wheelMoveSpeedSet<=0)
        mptr.wheelMoveSpeedSet = 0;
    else
        mptr.wheelMoveSpeedSet -= 100;
    ui->speedEdit->setText(s.setNum(mptr.wheelMoveSpeedSet));
}

/********************************舵机权限获取按钮：1，获取T1、T3权限，2：获取T2、T4权限，3：获取T3权限，4：获取T4权限************************************/
void MainWindow::on_pushButton_clicked()
{
    switch (mptr.wheelAddress) {
    case 1:
        write(mptr.fd1,mptr.gainAccess,sizeof(mptr.gainAccess));//fflush(stdout);
		mptr.delayTimeMsecs(mptr.delayTimeSet);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd1)).toHex());
        write(mptr.fd3,mptr.gainAccess,sizeof(mptr.gainAccess));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
        break;
    case 2:
        write(mptr.fd2,mptr.gainAccess,sizeof(mptr.gainAccess));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd2)).toHex());
        write(mptr.fd4,mptr.gainAccess,sizeof(mptr.gainAccess));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd4)).toHex());
        break;
    case 3:
        write(mptr.fd3,mptr.gainAccess,sizeof(mptr.gainAccess));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
        break;
    case 4:
        write(mptr.fd4,mptr.gainAccess,sizeof(mptr.gainAccess));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd4)).toHex());
        break;
    default:
        break;
    }
}

/*********************************舵机使能按钮，效果同上*****************************************************************************/
void MainWindow::on_pushButton_2_clicked()
{
    switch (mptr.wheelAddress) {
    case 1:
        write(mptr.fd1,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd1)).toHex());
        write(mptr.fd3,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
        break;
    case 2:
        write(mptr.fd2,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd2)).toHex());
        write(mptr.fd4,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd4)).toHex());
        break;
    case 3:
        write(mptr.fd3,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
        break;
    case 4:
        write(mptr.fd4,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd4)).toHex());
        break;
    default:
        break;
    }
}

/************************************角度增加按钮，用于手动控制舵机打角**************************************************************************/
void MainWindow::on_addAngleButton_clicked()
{
    QString s;
    if(mptr.wheelAngle>=90)
        mptr.wheelAngle = 90;
    else
        mptr.wheelAngle += 5;
    ui->angleEdit->setText(s.setNum(mptr.wheelAngle));
}

/*************************************角度减少按钮，用于手动控制舵机打角*************************************************************************/
void MainWindow::on_decAngleButton_clicked()
{
    QString s;
    if(mptr.wheelAngle<=-90)
        mptr.wheelAngle = -90;
    else
        mptr.wheelAngle -=5;
    ui->angleEdit->setText(s.setNum(mptr.wheelAngle));
}

/*************************************单个舵机旋转按钮，根据地址选择****************************************************************************/
void MainWindow::on_rotateButton_clicked()
{
    int array[20] = {0};
    switch (mptr.wheelAddress) {
    case 2:
        mptr.writeWheelPosition(00,mptr.wheelAngle+mptr.wheelFrontAngleOffset);
        write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));//fflush(stdout);
        read(mptr.fd2,array,sizeof(array));
        break;
    case 4:
        mptr.writeWheelPosition(00,mptr.wheelAngle+mptr.wheelRearAngleOffset);
        write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));//fflush(stdout);
        read(mptr.fd4,array,sizeof(array));
    default:
        break;
    }
}

/***************************************驱动器地址改变按钮，1-4循环***********************************************************************/
void MainWindow::on_changeAddButton_clicked()
{
    QString s;
    mptr.wheelAddress ++;
    if(mptr.wheelAddress>4)
        mptr.wheelAddress = 1;
    ui->addressEdit->setText(s.setNum(mptr.wheelAddress));
    if(mptr.wheelAddress==1||mptr.wheelAddress==3)
        mptr.writeWheelSpeed(00,00,mptr.write0RPM);
}

/****************************************断连按钮，效果同获取权限按钮**********************************************************************/
void MainWindow::on_disableBridge_clicked()
{
    switch (mptr.wheelAddress) {
    case 1:
        write(mptr.fd1,mptr.disableBridgeCommand,sizeof(mptr.disableBridgeCommand));
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd1)).toHex());
        write(mptr.fd3, mptr.disableBridgeCommand, sizeof(mptr.disableBridgeCommand));
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
        break;
    case 2:
        write(mptr.fd2,mptr.disableBridgeCommand,sizeof(mptr.disableBridgeCommand));
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd2)).toHex());
        write(mptr.fd4, mptr.disableBridgeCommand, sizeof(mptr.disableBridgeCommand));
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd4)).toHex());
        break;
    case 3:
        write(mptr.fd3, mptr.disableBridgeCommand, sizeof(mptr.disableBridgeCommand));
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
        break;
    case 4:
        write(mptr.fd4, mptr.disableBridgeCommand, sizeof(mptr.disableBridgeCommand));
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd4)).toHex());
        break;
    default:
        break;
    }
}

/*******************************************延时控制按钮（加），通讯方式改变后放弃不用***********************************************************/
void MainWindow::on_addTimeButton_clicked()
{
    QString s;
    mptr.delayTimeSet++;
    ui->timeEdit->setText(s.setNum(mptr.delayTimeSet));
}

/*******************************************延时控制按钮（减），通讯方式改变后放弃不用***********************************************************/
void MainWindow::on_decTimeButton_clicked()
{
    QString s;
    mptr.delayTimeSet--;
    if(mptr.delayTimeSet<0)
        mptr.delayTimeSet = 0;
    ui->timeEdit->setText(s.setNum(mptr.delayTimeSet));
}

/*********************************************前后轮联动旋转按钮，同时从零位转动相同角度***********************************************************/
void MainWindow::on_rotateTogetherButton_clicked()
{
    if(mptr.wheelAddress ==2 )
    {
        mptr.writeWheelPosition(00, mptr.wheelAngle + mptr.wheelFrontAngleOffset);
        write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd2)).toHex());
        mptr.writeWheelPosition(00, mptr.wheelAngle + mptr.wheelRearAngleOffset);
        write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));//fflush(stdout);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd4)).toHex());
    }
}

/***********************************************reset命令发送按钮，用于舵机驱动器重置***************************************************************/
void MainWindow::on_resetButton_clicked()
{
    int array[20] = {0};
    unsigned char resetcommand[12] = {0xa5,0x3f,0x02,0x01,0x00,0x01,0x01,0x47,0x00,0x10,0x12,0x31};
    write(mptr.fd1,resetcommand,sizeof(resetcommand));//fflush(stdout);
    read(mptr.fd1,array,sizeof(array));
    write(mptr.fd2,resetcommand,sizeof(resetcommand));//fflush(stdout);
    read(mptr.fd2,array,sizeof(array));
    write(mptr.fd3,resetcommand,sizeof(resetcommand));//fflush(stdout);
    read(mptr.fd3,array,sizeof(array));
    write(mptr.fd4,resetcommand,sizeof(resetcommand));//fflush(stdout);
    read(mptr.fd4,array,sizeof(array));
}

/**********************************************报文显示函数****************************************************************/
QByteArray MainWindow::QString2Hex(QString str)
{
    QByteArray senddata;
            int hexdata,lowhexdata;
            int hexdatalen = 0;
            int len = str.length();
            senddata.resize(len/2);
            char lstr,hstr;
            for(int i=0; i<len; )
            {
                hstr=str[i].toLatin1();   //字符型
                if(hstr == ' ')
                {
                    i++;
                    continue;
                }
                i++;
                if(i >= len)
                    break;
                lstr = str[i].toLatin1();
                hexdata = ConvertHexChar(hstr);
                lowhexdata = ConvertHexChar(lstr);
                if((hexdata == 16) || (lowhexdata == 16))
                    break;
                else
                    hexdata = hexdata*16+lowhexdata;
                i++;
                senddata[hexdatalen] = (char)hexdata;
                hexdatalen++;
            }
            senddata.resize(hexdatalen);
            return senddata;
}

/*********************************************16进制与字符穿互换函数*****************************************************************/
int MainWindow::ConvertHexChar(char ch)
{
    if((ch >= '0') && (ch <= '9'))
        return ch-'0';
    else if((ch >= 'A') && (ch <= 'F'))
        return ch-'A'+10;
    else if((ch >= 'a') && (ch <= 'f'))
        return ch-'a'+10;
    else return (-1);
}

/*********************************************清楚显示区*****************************************************************/
void MainWindow::on_cleanCommunicationButton_clicked()
{
    ui->CommunicationEdit->clear();
}

/**********************************************设置前轮偏移量函数****************************************************************/
void MainWindow::on_setFrontOffsetButton_clicked()
{
    QString s;
    if(mptr.wheelAddress ==2)
    {
        mptr.wheelFrontAngleOffset = mptr.wheelAngle;
        mptr.wheelAngle = 0;
        ui->angleEdit->setText(s.setNum(mptr.wheelAngle));
    }
}

/**********************************************设置后轮偏移量函数***************************************************************/
void MainWindow::on_setRearOffsetButton_clicked()
{
    QString s;
    if(mptr.wheelAddress == 4)
    {
        mptr.wheelRearAngleOffset = mptr.wheelAngle;
        mptr.wheelAngle = 0;
        ui->angleEdit->setText(s.setNum(mptr.wheelAngle));
    }
}

/***********************************************232测试按钮函数，目前放弃不用***************************************************************/
void MainWindow::on_testButton_clicked()
{
    unsigned char testarray[12] = {0xA5, 0x3F, 0x02, 0x07, 0x00, 0x01, 0xB3, 0xE7, 0x0F, 0x00, 0x10, 0x3E};
    unsigned char testEnable[12] ={0xA5, 0x3F, 0x02, 0x01, 0x00, 0x01, 0x01, 0x47, 0x00, 0x00, 0x00, 0x00};
    unsigned char testDisable[12]={0xA5, 0x3F, 0x02, 0x01, 0x00, 0x01, 0x01, 0x47, 0x01, 0x00, 0x33, 0x31};
    unsigned char test0RPM[14] = {0xA5, 0x3F, 0x02, 0x45, 0x00, 0x02, 0xF0, 0x49, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    unsigned char test500RPM[14] = {0xA5, 0x3F, 0x02, 0x45, 0x00, 0x02, 0xF0, 0x49, 0x55, 0x55, 0x03, 0x00, 0x29, 0x13};
    unsigned char testRead[8] = {0xA5, 0x3F, 0x01, 0x11, 0x02, 0x02, 0x8F, 0xF9};
    write(mptr.fd,testarray,sizeof(testarray));
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd)).toHex());

    write(mptr.fd,testEnable,sizeof(testEnable));
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd)).toHex());

    write(mptr.fd,test500RPM,sizeof(test500RPM));
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd)).toHex());

}

/************************************************自动运行按钮**************************************************************/
void MainWindow::on_autoRunButton_clicked()
{
    ui->addTimeButton->setEnabled(false);                               //we can change a window next time to avoid write so much code to hide the widgets
    ui->addAngleButton->setEnabled(false);								//我们后期可以改变成为两个串口，便无需设定无效这么多控件
    ui->backwardButton->setEnabled(false);
    ui->changeAddButton->setEnabled(false);
    ui->pushButton->setEnabled(false);
    ui->pushButton_2->setEnabled(false);
    ui->disableBridge->setEnabled(false);
    ui->resetButton->setEnabled(false);
    ui->setButton2->setEnabled(false);
    ui->setSpeed->setEnabled(false);
    ui->forwardButton->setEnabled(false);
    ui->backwardButton->setEnabled(false);
    ui->addAngleButton->setEnabled(false);
    ui->decAngleButton->setEnabled(false);
    ui->rotateButton->setEnabled(false);
    ui->rotateTogetherButton->setEnabled(false);
    ui->setFrontOffsetButton->setEnabled(false);
    ui->setRearOffsetButton->setEnabled(false);
    ui->testButton->setEnabled(false);
    ui->testButton2->setEnabled(false);

    mptr.breakFlag = false;
    if (mptr.wheelMoveSpeedSet==500||mptr.wheelMoveSpeedSet==1000||mptr.wheelMoveSpeedSet==1500)
        mptr.wheelMoveSpeedSet=500;

    initialTheSystem();

}

/**************************************************停止自动运行按钮，自动运行转为手动模式，可视为急停****************************************************/
void MainWindow::on_stopAutorunButton_clicked()
{
    QString s;
    ui->addTimeButton->setEnabled(true);									//手动按钮重新使能
    ui->addAngleButton->setEnabled(true);
    ui->backwardButton->setEnabled(true);
    ui->changeAddButton->setEnabled(true);
    ui->pushButton->setEnabled(true);
    ui->pushButton_2->setEnabled(true);
    ui->disableBridge->setEnabled(true);
    ui->resetButton->setEnabled(true);
    ui->setButton2->setEnabled(true);
    ui->setSpeed->setEnabled(true);
    ui->forwardButton->setEnabled(true);
    ui->backwardButton->setEnabled(true);
    ui->addAngleButton->setEnabled(true);
    ui->decAngleButton->setEnabled(true);
    ui->rotateButton->setEnabled(true);
    ui->rotateTogetherButton->setEnabled(true);
    ui->setFrontOffsetButton->setEnabled(true);
    ui->setRearOffsetButton->setEnabled(true);
    ui->testButton->setEnabled(true);
    ui->testButton2->setEnabled(true);
    mptr.wheelAddress = 0;																//部分手动参数归零，归位
    mptr.wheelMoveSpeedSet=0;
    mptr.wheelAngle = 0;
    mptr.delayTimeSet = 5;
//    mptr.wheelFrontAngleOffset = 0;
//    mptr.wheelRearAngleOffset = 0;
    mptr.wheelMoveSpeedSetMax = 2400;
    ui->addressEdit->setText(s.setNum(mptr.wheelAddress));
    ui->speedEdit->setText(s.setNum(mptr.wheelMoveSpeedSet));
    ui->angleEdit->setText(s.setNum(mptr.wheelAngle));
    ui->timeEdit->setText(s.setNum(mptr.delayTimeSet));
    mptr.breakFlag = true;
}

/*******************************舵机转向轮校零函数*******************************************************************************/
void MainWindow::wheelZeroCalibration()
{
    int array[20] = {0};
    QString s;
    bool steerFrontLimitTemp=false;
    bool steerBackLimitTemp = false;
    bool limitFlag2=false;
    bool limitFlag4=false;
    write(mptr.fd1,mptr.gainAccess,sizeof(mptr.gainAccess));read(mptr.fd1,array,sizeof(array));//fflush(stdout);                     //gain access
    write(mptr.fd3,mptr.gainAccess,sizeof(mptr.gainAccess));read(mptr.fd2,array,sizeof(array));//fflush(stdout);
    write(mptr.fd2,mptr.gainAccess,sizeof(mptr.gainAccess));read(mptr.fd3,array,sizeof(array));//fflush(stdout);
    write(mptr.fd4,mptr.gainAccess,sizeof(mptr.gainAccess));read(mptr.fd4,array,sizeof(array));//fflush(stdout);

    write(mptr.fd1,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));read(mptr.fd1,array,sizeof(array));//fflush(stdout);   //enable the bridge
    write(mptr.fd2,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));read(mptr.fd2,array,sizeof(array));//fflush(stdout);
    write(mptr.fd3,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));read(mptr.fd3,array,sizeof(array));//fflush(stdout);
    write(mptr.fd4,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));read(mptr.fd4,array,sizeof(array));//fflush(stdout);

    while((limitFlag2==false) || (limitFlag4==false))
    {
        if(mptr.breakFlag==false)
        {
            mptr.readIO();                                   //read I/O		读取IO，
				ui->CommunicationEdit->append(tr("%1 %2 %3").arg(mptr.systemOnFlag).arg(mptr.sickFalse).arg(mptr.sickWarningSpaceAlert));

            do{                                              //filter		IO口滤波
                steerFrontLimitTemp = mptr.steerFrontLimitDetect2;
                steerBackLimitTemp = mptr.steerFrontLimitDetect;
                mptr.readIO();
				ui->CommunicationEdit->append(tr("%1 %2 %3").arg(mptr.systemOnFlag).arg(mptr.sickFalse).arg(mptr.sickWarningSpaceAlert));
            }while((steerFrontLimitTemp != mptr.steerFrontLimitDetect2) && (steerBackLimitTemp != mptr.steerBackLimitDetect));

            ui->CommunicationEdit->append(s.setNum((int)mptr.wheelAngle2));
            ui->CommunicationEdit->append(tr("Front: %1, Back: %2").arg(mptr.steerFrontLimitDetect2).arg(mptr.steerBackLimitDetect));
            //if(mptr.steerFrontLimitDetect == false && mptr.steerFrontLimitDetect2 ==false)
            if(mptr.steerFrontLimitDetect2 == false)
            {
                mptr.wheelAngle2 += 1;
                mptr.writeWheelPosition(00,mptr.wheelAngle2);
                write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,sizeof(array));//fflush(stdout);
            }
            else
                limitFlag2 = true;
            //if(mptr.steerBackLimitDetect == false && mptr.steerBackLimitDetect2 == false)
            if(mptr.steerBackLimitDetect == false)
            {
                mptr.wheelAngle4 += 1;
                mptr.writeWheelPosition(00,mptr.wheelAngle4);
                write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,sizeof(array));
            }
            else
                limitFlag4 = true;
            mptr.delayTimeMsecs(500);
        }
        else
            break;
    }
    if(mptr.breakFlag == false)														
    {
        mptr.wheelFrontAngleOffset = mptr.wheelAngle2 - 102;                  
        mptr.wheelRearAngleOffset = mptr.wheelAngle4 - 102;

        mptr.writeWheelPosition(00,mptr.wheelFrontAngleOffset);
        write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,sizeof(array));
        ui->CommunicationEdit->append(tr("The wheelFrontAngleOffset is: %1").arg(mptr.wheelFrontAngleOffset));

        mptr.writeWheelPosition(00,mptr.wheelRearAngleOffset);
        write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,sizeof(array));
        ui->CommunicationEdit->append(tr("The wheelRearAngleOffset is: %1").arg(mptr.wheelRearAngleOffset));
        mptr.delayTimeMsecs(6000);
		mptr.calibrationFlag = true;		
    }
	

}

/*********************************读取到的IO口状态显示到界面上*****************************************************************************/
void MainWindow::showIOResult()
{
    QString s;
    ui->speedLabel->setText(s.setNum(mptr.AGVSpeed));
    if(mptr.systemOnFlag)                                                 // waiting to check the status of normal//////////////////////////////////////
    {
        while(mptr.breakFlag == false)
        {
            ui->CommunicationEdit->append("Need Manual Reset");            // show on pannel: need manual reset
            ui->CommunicationEdit->append(tr("systemOnFlag is :%1").arg(mptr.systemOnFlag));
            mptr.readIO();
				ui->CommunicationEdit->append(tr("%1 %2 %3").arg(mptr.systemOnFlag).arg(mptr.sickFalse).arg(mptr.sickWarningSpaceAlert));
			
            if(!mptr.systemOnFlag)
            {
				mptr.delayTimeMsecs(1000);
                mptr.gainAccessAndEnableWheel();
                break;
            }
            QCoreApplication::processEvents(QEventLoop::AllEvents,100);
        }
    }
    if (mptr.emergencyFlag)
    {
        ui->CommunicationEdit->append("Please check the system and restart the robot");
    }
}

/***********************************刷IO口Q点，全部置为零***************************************************************************/
void MainWindow::on_refreshIOButton_clicked()
{
    int i2c_write[2];
    i2c_write[0]=0x00;//低位
    i2c_write[1]=0x00;//高位
    wiringPiI2CWriteReg16(mptr.i2c_fd3,i2c_write[0],i2c_write[1]);//向设备3的reg中写入两个字节
}

/************************************右平移按钮，按压执行**************************************************************************/
void MainWindow::on_parallelRightButton_pressed()
{
    int array[20]= {0};
    mptr.writeWheelPosition(00,90+mptr.wheelFrontAngleOffset);
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,90+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

    mptr.delayTimeMsecs(6000);						//舵轮从开始打角到平移开始，延时

    mptr.writeWheelSpeed(00,100);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,100);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);
}

/*************************************右面平移按钮，弹起归位*************************************************************************/
void MainWindow::on_parallelRightButton_released()
{
    int array[20]={0};
    mptr.writeWheelSpeed(00,00);                        //stop AGV
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,00);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

    mptr.delayTimeMsecs(6000);							//延时

    mptr.writeWheelPosition(00,0+mptr.wheelFrontAngleOffset);           // angle to zero
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,0+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

}

/*************************************左平移按钮，按压执行*************************************************************************/
void MainWindow::on_parallelLeftButton_pressed()
{
    int array[20]= {0};
    mptr.writeWheelPosition(00,-90+mptr.wheelFrontAngleOffset);
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,-90+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

    mptr.delayTimeMsecs(6000);							//延时

    mptr.writeWheelSpeed(00,100);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,100);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

}

/***************************************左平移按钮，弹起归位***********************************************************************/
void MainWindow::on_parallelLeftButton_released()
{
    int array[20]={0};
    mptr.writeWheelSpeed(00,00);                        //stop AGV
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,00);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

    mptr.delayTimeMsecs(6000);							//延时

    mptr.writeWheelPosition(00,0+mptr.wheelFrontAngleOffset);           // angle to zero
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,0+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

}

/***************************************顺时针旋转按钮，按压执行***********************************************************************/
void MainWindow::on_rotateLeftButton_pressed()
{
    int array[20]= {0};
    mptr.writeWheelPosition(00,90+mptr.wheelFrontAngleOffset);
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,90+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

    mptr.delayTimeMsecs(6000);									//延时

    mptr.writeWheelSpeed(00,100);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,-100);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

}

/****************************************顺时针旋转按钮，弹起归位**********************************************************************/
void MainWindow::on_rotateLeftButton_released()
{
    int array[20]={0};
    mptr.writeWheelSpeed(00,00);                        										//stop AGV
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,00);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

    mptr.delayTimeMsecs(6000);																	//延时

    mptr.writeWheelPosition(00,0+mptr.wheelFrontAngleOffset);          	 						// angle to zero
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,0+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

}

/****************************************逆时针旋转按钮，按压执行**********************************************************************/
void MainWindow::on_rotateRightButton_pressed()
{
    int array[20]= {0};
    mptr.writeWheelPosition(00,90+mptr.wheelFrontAngleOffset);
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,90+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

    mptr.delayTimeMsecs(6000);																//延时

    mptr.writeWheelSpeed(00,-100);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,100);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

}

/***************************************逆时针旋转按钮，弹起归位***********************************************************************/
void MainWindow::on_rotateRightButton_released()
{
    int array[20]={0};
    mptr.writeWheelSpeed(00,00);                        									//stop AGV
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,00);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

    mptr.delayTimeMsecs(6000);																//延时

    mptr.writeWheelPosition(00,0+mptr.wheelFrontAngleOffset);           					// angle to zero
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,0+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

}
