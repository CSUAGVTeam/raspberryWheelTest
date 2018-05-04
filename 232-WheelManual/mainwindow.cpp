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
#include <QSpinBox>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QString c1;
/*****************  二维码TCPIP协议的连接  ****************/
    tcpSocket = NULL;
    tcpSocket = new QTcpSocket(this);
    QString ip="192.168.20.61";
    //QString ip="192.168.10.11";
    //QString ip="10.0.0.3";
    qint16 port=23;
    tcpSocket->connectToHost(ip,port);
    connect(tcpSocket,SIGNAL(readyRead()),this,SLOT(ReadData()));
/*****************   UI界面的初始设计  ******************/
    ui->speedEdit->setText(c1.setNum(mptr.wheelMoveSpeedSet));
    ui->addressEdit->setText(c1.setNum(mptr.wheelAddress));
    ui->angleEdit->setText(c1.setNum(mptr.wheelAngle));
    ui->timeEdit->setText(c1.setNum(mptr.delayTimeSet));
    ui->kpSpinBox->setValue(mptr.KP);
    ui->kiSpinBox->setValue(mptr.KI);
    ui->kdSpinBox->setValue(mptr.KD);
    //ui->rotateButton->setEnabled(false);
    ui->kpSpinBox->setSingleStep(1.0);
    ui->kiSpinBox->setSingleStep(100);
    ui->kdSpinBox->setSingleStep(10);
    ui->kpSpinBox->setValue(60);
    ui->kiSpinBox->setValue(300);
    ui->kdSpinBox->setValue(300);
}

MainWindow::~MainWindow()
{
    delete ui;
}

/****************系统初始化（包括舵轮权限获取、使能、舵轮校零）并启动系统开始工作**************************************************************************/
void MainWindow::initialTheSystem(void)
{
    mptr.gainAccessAndEnableWheel();
    mptr.readIO();                               //read I/O and store the data
    ui->CommunicationEdit->append(tr("%1 %2 %3").arg(mptr.systemOnFlag).arg(mptr.sickFalse).arg(mptr.sickWarningSpaceAlert));

    if (mptr.calibrationFlag == false)
        {
            if(ui->calibrationCheck->isChecked())
            {
                wheelZeroCalibration();                  //find the zero point of the steering wheel
                mptr.yawTarget = mptr.yaw;
                mptr.yawInt=mptr.yaw-90;
                if(mptr.yawInt<0)
                    mptr.yawInt+=360;
                if(mptr.yawInt>=360)
                    mptr.yawInt-=360;
            }
        }
    systemOn();
}

/*****************系统运行函数****************************************************************************************************************************/
void MainWindow::systemOn(void)
{
    double speedBefore=1;
	unsigned char array[20]= {0};
    double speedTemp =0;
    double R=0;
    //mptr.AGVLocation={0,0};

    mptr.readIO();
    if(mptr.wheelMoveSpeedSet>=0)
    {
        mptr.direction_flag=true;
    }
    else
    {
        mptr.direction_flag=false;
    }
    if(mptr.direction_flag==true)
    {
        if(mptr.direction_flag_last==false)
        {
            mptr.num+=1;
        }

        mptr.direction_flag_last=true;
    }
    else
    {
        if(mptr.direction_flag_last=true)
        {
            mptr.num-=1;
        }
        mptr.direction_flag_last=false;
    }
    //mptr.yawLast = mptr.yaw;
	mptr.yawFlag == true;
	speedTemp = mptr.wheelMoveSpeedSet;
    //QTime t1;
    //t1 = QTime::currentTime().addSecs(60);
    while(1)
    {
        int t_1,t_2,delta_t=0;
        mptr.wheelAddress=2;
        t_1=QTime::currentTime().msec();
        /**
        if(QTime::currentTime()>t1)
        {
            break;
        }
        **/
        if(mptr.breakFlag == false)
        {
                mptr.readIO();                          //读取数据，IO、舵机等,检查IO数据，输出对应IO数据
                mptr.wheelMoveSpeedSet = speedTemp;//重新设定速度

                if(mptr.wheelMoveSpeedSet > mptr.wheelMoveSpeedSetMax)
                    mptr.wheelMoveSpeedSet = mptr.wheelMoveSpeedSetMax;
                mptr.checkIO();
                ui->CommunicationEdit->append(tr("%1\t").arg(mptr.buf));
                ui->CommunicationEdit->append(tr("%1\t").arg(mptr.buf_last));
                ui->CommunicationEdit->append(tr("%1\t%2\t%3").arg(mptr.yawTarget).arg(mptr.yaw).arg(mptr.yawInt));
                ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.wheelFrontAngle).arg(mptr.wheelRearAngle));
                //ui->CommunicationEdit->append(tr("%1\t").arg(mptr.wheelAngle));
                ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.delta_s).arg(mptr.yaw_error));

//                write(mptr.fd2,mptr.readPositionData,sizeof(mptr.readPositionData));
//                ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd2)).toHex());
//                write(mptr.fd4,mptr.readPositionData,sizeof(mptr.readPositionData));
//                ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd4)).toHex());
                ui->CommunicationEdit->append(mptr.frontTelegram);
                ui->CommunicationEdit->append(mptr.backTelegram);

                //ui->CommunicationEdit->append(tr("FrontAngleOffset: %1").arg(mptr.wheelFrontAngleOffset));
                //ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.P_Centre.X).arg(mptr.P_Centre.Y));

                ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.wheelMoveSpeedReadFront).arg(mptr.wheelMoveSpeedReadRear));
                ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.AGVLocation.X).arg(mptr.AGVLocation.Y));
                ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.a).arg(mptr.b));
                ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.num).arg(mptr.wheelAngle));
                //ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.QR_Code_Number).arg(mptr.Angle_QRtoCar));
                ui->read->append(tr("%1\t%2").arg(mptr.turn_flag).arg(mptr.direction_flag));

                //ui->read->append(tr("%1\t").arg(mptr.turn_flag));
                //ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.QR_Point[0].X).arg(mptr.QR_Point[0].Y));
                //ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.QR_Point[1].X).arg(mptr.QR_Point[1].Y));
                //ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.QR_Point[2].X).arg(mptr.QR_Point[2].Y));
                //ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.QR_Point[3].X).arg(mptr.QR_Point[3].Y));
                //ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.QR_Point[4].X).arg(mptr.QR_Point[4].Y));

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
					mptr.writeWheelPosition(00,mptr.wheelFrontAngleOffset);
					write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,sizeof(array));
					mptr.writeWheelPosition(00, mptr.wheelRearAngleOffset);
					write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,sizeof(array));
					
                }

        }
			else
                break;
        QApplication::processEvents(QEventLoop::AllEvents,1000);			//Qt任务队列等待，释放系统资源
        if (mptr.breakFlag == true)												//手自变换时候，AGV停车
        {   
            mptr.writeWheelSpeed(00,00,mptr.commanData);
            write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));
            read(mptr.fd1,array,sizeof(array));

            mptr.writeWheelSpeed(00,00,mptr.commanData);
            write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
            read(mptr.fd3,array,sizeof(array));

            mptr.writeWheelPosition(00,mptr.wheelFrontAngleOffset);
            write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,sizeof(array));

            mptr.writeWheelPosition(00, mptr.wheelRearAngleOffset);
			write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,sizeof(array));
            break;
        }
        t_2=QTime::currentTime().msec();
        if(t_2>t_1)
            delta_t=t_2-t_1;
        else
            delta_t=1000+t_2-t_1;

        mptr.AGVLocation.X=mptr.AGVLocation.X+mptr.AGVSpeeds.X*delta_t/1000;        //  航位推算
        mptr.AGVLocation.Y=mptr.AGVLocation.Y+mptr.AGVSpeeds.Y*delta_t/1000;

         if(mptr.buf!=mptr.buf_last)//此处加扫描到二维码的判断信息
        {
             if(mptr.buf!=NULL)
             {
                 mptr.QR_Flag=mptr.Two_bar_codes_Pro2(mptr.buf);
             }
        }
        if(mptr.QR_Flag==true)
        {

            //加上位置，角度处理函数
            mptr.yaw_error = mptr.Information_Corrective();
            mptr.QR_Flag=false;
            mptr.buf_last=mptr.buf;
            //mptr.buf.clear();
        }
        /**         路径规划        **/
        /*****
        if(mptr.direction_flag==true)
        {
            R=mptr.Go2 (mptr.P_Target[mptr.num]);
        }
        else
        {

            R=mptr.Go2 (mptr.P_Target[mptr.num]);
        }
        ******/

         R=mptr.Go2 (mptr.P_Target[mptr.num]);

        if(mptr.direction_flag==true)
        {
            if(mptr.num==25)
                if(mptr.turn_flag==false)
                {
                    //mptr.breakFlag = true;
                    mptr.num = 2;
                }
        }
        else
        {
            if(mptr.num==1)
                if(mptr.turn_flag==false)
                {
                    //mptr.breakFlag = true;
                    mptr.num = 23;
                }
        }

        if(mptr.turn_flag==true)
        {
            /**
            if(mptr.direction_flag==true)
            {
                if(mptr.num==8)
                mptr.P_Centre={-2,7};
                if(mptr.num==13)
                mptr.P_Centre={-6,7};
                if(mptr.num==19)
                mptr.P_Centre={-6,2};
                if(mptr.num==24)
                mptr.P_Centre={-2,2};
            }
            else
            {
                if(mptr.num==7)
                mptr.P_Centre={-2,7};
                if(mptr.num==12)
                mptr.P_Centre={-6,7};
                if(mptr.num==18)
                mptr.P_Centre={-6,2};
                if(mptr.num==23)
                mptr.P_Centre={-2,2};

                //if(mptr.num==23)
                //mptr.P_Centre={-2,2};
            }
            **/
            if(mptr.direction_flag==true)
            {
                if(mptr.num==3)
                mptr.P_Centre={2,2};
            }
            else
            {
                if(mptr.num==2)
                mptr.P_Centre={2,2};
            }


            mptr.delta_s=mptr.Position_Turn_crol (mptr.P_Centre,mptr.P_Target[mptr.num],mptr.AGVLocation,4);
        }
        else
        {
            //mptr.delta_s=mptr.Straight_Line (mptr.AGVLocation,mptr.P_Target[mptr.num][0],mptr.P_Target[mptr.num][1]);
            if(mptr.direction_flag==true)
            {
                mptr.delta_s=mptr.Straight_Line (mptr.AGVLocation,mptr.P_Target[mptr.num-1],mptr.P_Target[mptr.num]);
            }
            else
            {
                mptr.delta_s=mptr.Straight_Line (mptr.AGVLocation,mptr.P_Target[mptr.num+1],mptr.P_Target[mptr.num]);
            }

        }
    }
	mptr.writeWheelSpeed(00,00,mptr.commanData);								//任务队列完成后，停车
	write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));
	read(mptr.fd1,array,sizeof(array));
	mptr.delayTimeMsecs(mptr.delayTimeSet);
	mptr.writeWheelSpeed(00,00,mptr.commanData);
	write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
	read(mptr.fd3,array,sizeof(array));
	mptr.writeWheelPosition(00,mptr.wheelFrontAngleOffset);
	write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,sizeof(array));
	mptr.writeWheelPosition(00, mptr.wheelRearAngleOffset);
	write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,sizeof(array));
	ui->CommunicationEdit->append("Halt Success!");
	mptr.sickA = 0;
    mptr.sickB = 0;
    mptr.sickC = 0;
	mptr.writeIO();
}
/***********************    二维码信息读取   **************************/

void MainWindow::ReadData ()
{
     //if(mptr.buf == NULL)
     {
         QByteArray buffer=tcpSocket->readAll();    //
         mptr.buf = buffer;
         //buffer.clear();
     }

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
        mptr.wheelMoveSpeedSet += 0.05;
    ui->speedEdit->setText(s.setNum(mptr.wheelMoveSpeedSet));
}

/*******************************速度设定按钮，减速，步长100rpm，最小值0*******************************************************************************/
void MainWindow::on_setButton2_clicked()//speed decrease
{
    QString s;
    std::string str;
    //if (mptr.wheelMoveSpeedSet<=0)
      //  mptr.wheelMoveSpeedSet = 0;
    //else
        mptr.wheelMoveSpeedSet -= 0.05;
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
        mptr.wheelAngle += 0.1;
    ui->angleEdit->setText(s.setNum(mptr.wheelAngle));
}

/*************************************角度减少按钮，用于手动控制舵机打角*************************************************************************/
void MainWindow::on_decAngleButton_clicked()
{
    QString s;
    if(mptr.wheelAngle<=-90)
        mptr.wheelAngle = -90;
    else
        mptr.wheelAngle -=0.1;
    ui->angleEdit->setText(s.setNum(mptr.wheelAngle));
}

/*************************************单个舵机旋转按钮，根据地址选择****************************************************************************/
void MainWindow::on_rotateButton_clicked()
{
    int array[20] = {0};
    QString str;
    switch (mptr.wheelAddress) {
    case 2:
        mptr.writeWheelPosition(00,mptr.wheelAngle + mptr.wheelFrontAngleOffset);
        write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));//fflush(stdout);
        read(mptr.fd2,array,sizeof(array));
//        for(int i=0; i<sizeof(mptr.writePositionData); i++)
//        {
//            if(mptr.writePositionData[i]<16)
//                str += '0' + QString::number(mptr.writePositionData[i],16).toUpper();
//            else
//                str += QString::number(mptr.writePositionData[i],16).toUpper();
//        }
//        ui->CommunicationEdit->append(str);
        write(mptr.fd2,mptr.readPositionData,sizeof(mptr.readPositionData));
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd2)).toHex());
        break;
    case 4:
        mptr.writeWheelPosition(00,mptr.wheelAngle + mptr.wheelRearAngleOffset);
        write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));//fflush(stdout);
//        for(int i=0; i<sizeof(mptr.writePositionData); i++)
//        {
//            if(mptr.writePositionData[i]<16)
//                str += '0' + QString::number(mptr.writePositionData[i],16).toUpper();
//            else
//                str += QString::number(mptr.writePositionData[i],16).toUpper();
//        }
//        ui->CommunicationEdit->append(str);
        read(mptr.fd4,array,sizeof(array));
        write(mptr.fd4,mptr.readPositionData,sizeof(mptr.readPositionData));
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd4)).toHex());
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
        mptr.wheelFrontAngleOffset = mptr.wheelAngle + mptr.wheelFrontAngleOffset;
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
        mptr.wheelRearAngleOffset = mptr.wheelAngle + mptr.wheelRearAngleOffset;
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
//    if (mptr.wheelMoveSpeedSet==500||mptr.wheelMoveSpeedSet==1000||mptr.wheelMoveSpeedSet==1500)
//        mptr.wheelMoveSpeedSet=500;

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
    bool limitFlag2left = false;
    bool limitFlag4left = false;
    double frontLeftTemp = 0.0;
    double backLeftTemp = 0.0;
//    double frontRightTemp = 0.0;
//    double backRightTemp =0.0;
    write(mptr.fd1,mptr.gainAccess,sizeof(mptr.gainAccess));read(mptr.fd1,array,sizeof(array));//fflush(stdout);                     //gain access
    write(mptr.fd3,mptr.gainAccess,sizeof(mptr.gainAccess));read(mptr.fd2,array,sizeof(array));//fflush(stdout);
    write(mptr.fd2,mptr.gainAccess,sizeof(mptr.gainAccess));read(mptr.fd3,array,sizeof(array));//fflush(stdout);
    write(mptr.fd4,mptr.gainAccess,sizeof(mptr.gainAccess));read(mptr.fd4,array,sizeof(array));//fflush(stdout);

    write(mptr.fd1,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));read(mptr.fd1,array,sizeof(array));//fflush(stdout);   //enable the bridge
    write(mptr.fd2,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));read(mptr.fd2,array,sizeof(array));//fflush(stdout);
    write(mptr.fd3,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));read(mptr.fd3,array,sizeof(array));//fflush(stdout);
    write(mptr.fd4,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));read(mptr.fd4,array,sizeof(array));//fflush(stdout);

//    ui->CommunicationEdit->append("left detect!");
//    while((limitFlag2left==false)||(limitFlag4left==false))
//    {
//        if(mptr.breakFlag == false)
//        {
//            mptr.readIO();
//            do{
//                steerFrontLimitTemp = mptr.steerFrontLimitDetect;
//                steerBackLimitTemp = mptr.steerBackLimitDetect2;
//                mptr.readIO();
//            }while((steerFrontLimitTemp != mptr.steerFrontLimitDetect) && (steerBackLimitTemp != mptr.steerBackLimitDetect2));
//            if(mptr.steerFrontLimitDetect == false)
//            {
//                mptr.wheelAngle2 -= 1;
//                mptr.writeWheelPosition(00,mptr.wheelAngle2);
//                write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,sizeof(array));//fflush(stdout);
//                limitFlag2left =false;
//            }
//            else
//            {
//                frontLeftTemp = mptr.wheelAngle2 ;
//                limitFlag2left = true;
//            }
//            if(mptr.steerBackLimitDetect2 == false)
//            {
//                mptr.wheelAngle4 -= 1;
//                mptr.writeWheelPosition(00,mptr.wheelAngle4);
//                write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,sizeof(array));
//                limitFlag4left = false;
//            }
//            else
//            {
//                backLeftTemp = mptr.wheelAngle4;
//                limitFlag4left = true;
//            }
//            mptr.delayTimeMsecs(100);
//        }
//        else
//            break;

//    }

    ui->CommunicationEdit->append("right detect!");
    while((limitFlag2==false) || (limitFlag4==false))
    {
        if(mptr.breakFlag==false)
        {
            mptr.readIO();                                   //read I/O		读取IO，
            //mptr.checkIO();
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
                limitFlag2 =false;
            }
            else
                limitFlag2 = true;
            //if(mptr.steerBackLimitDetect == false && mptr.steerBackLimitDetect2 == false)
            if(mptr.steerBackLimitDetect == false)
            {
                mptr.wheelAngle4 += 1;
                mptr.writeWheelPosition(00,mptr.wheelAngle4);
                write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,sizeof(array));
                limitFlag4 = false;
            }
            else
                limitFlag4 = true;
            mptr.delayTimeMsecs(10);
        }
        else
            break;
    }


    ui->CommunicationEdit->append("Go back!");
    while((limitFlag2==true) || (limitFlag4==true))
    {
        if(mptr.breakFlag==false)
        {
            mptr.readIO();                                   //read I/O		读取IO，
            //mptr.checkIO();
//				ui->CommunicationEdit->append(tr("%1 %2 %3").arg(mptr.systemOnFlag).arg(mptr.sickFalse).arg(mptr.sickWarningSpaceAlert));

            do{                                              //filter		IO口滤波
                steerFrontLimitTemp = mptr.steerFrontLimitDetect2;
                steerBackLimitTemp = mptr.steerFrontLimitDetect;
                mptr.readIO();
                ui->CommunicationEdit->append(tr("%1 %2 %3").arg(mptr.systemOnFlag).arg(mptr.sickFalse).arg(mptr.sickWarningSpaceAlert));
            }while((steerFrontLimitTemp != mptr.steerFrontLimitDetect2) && (steerBackLimitTemp != mptr.steerBackLimitDetect));

            if(mptr.steerFrontLimitDetect2 == true)
            {
                mptr.wheelAngle2 -= 0.1;
                mptr.writeWheelPosition(00,mptr.wheelAngle2);
                write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,sizeof(array));//fflush(stdout);
                limitFlag2 =true;
            }
            else
                limitFlag2 = false;
            if(mptr.steerBackLimitDetect == true)
            {
                mptr.wheelAngle4 -= 0.1;
                mptr.writeWheelPosition(00,mptr.wheelAngle4);
                write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,sizeof(array));
                limitFlag4 = true;
            }
            else
                limitFlag4 = false;

            ui->CommunicationEdit->append(tr("Front: %1, Back: %2").arg(mptr.wheelAngle2).arg(mptr.wheelAngle4));
        }
        else
            break;
        mptr.delayTimeMsecs(100);
    }

    if(mptr.breakFlag == false)
    {
        mptr.wheelFrontAngleOffset = mptr.wheelAngle2 - 101.4;
        mptr.wheelRearAngleOffset = mptr.wheelAngle4 - 99.8;

        mptr.writeWheelPosition(00,mptr.wheelFrontAngleOffset);
        write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,sizeof(array));
        ui->CommunicationEdit->append(tr("The wheelFrontAngleOffset is: %1").arg(mptr.wheelFrontAngleOffset));

        mptr.writeWheelPosition(00,mptr.wheelRearAngleOffset);
        write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,sizeof(array));
        ui->CommunicationEdit->append(tr("The wheelRearAngleOffset is: %1").arg(mptr.wheelRearAngleOffset));
        mptr.delayTimeMsecs(6000);
        mptr.calibrationFlag = true;
//        write(mptr.fd2,mptr.resetcommand,sizeof(mptr.resetcommand));read(mptr.fd2,array,sizeof(array));
        write(mptr.fd2,mptr.gainAccess,sizeof(mptr.gainAccess));read(mptr.fd3,array,sizeof(array));//fflush(stdout);
        write(mptr.fd2,mptr.enableBridgeCommand,sizeof(mptr.enableBridgeCommand));read(mptr.fd2,array,sizeof(array));//fflush(stdout);
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
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));

    mptr.writeWheelPosition(00,90+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));
    read(mptr.fd4,array,20);read(mptr.fd2,array,20);

    mptr.delayTimeMsecs(6000);						//舵轮从开始打角到平移开始，延时

    mptr.writeWheelSpeed(00,mptr.wheelMoveSpeedSet);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    //mptr.writeWheelSpeed(00,mptr.wheelMoveSpeedSet);
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

    //mptr.delayTimeMsecs(6000);							//延时

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
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));

    mptr.writeWheelPosition(00,-90+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));

    read(mptr.fd4,array,20);read(mptr.fd2,array,20);

    mptr.delayTimeMsecs(6000);							//延时

    mptr.writeWheelSpeed(00,mptr.wheelMoveSpeedSet);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    //mptr.writeWheelSpeed(00,mptr.wheelMoveSpeedSet);
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

    //mptr.delayTimeMsecs(6000);							//延时

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
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));

    mptr.writeWheelPosition(00,90+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));
    read(mptr.fd4,array,20);read(mptr.fd2,array,20);

    mptr.delayTimeMsecs(6000);									//延时

    mptr.writeWheelSpeed(00,mptr.wheelMoveSpeedSet);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,-mptr.wheelMoveSpeedSet);
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

    //mptr.delayTimeMsecs(6000);																	//延时

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
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));

    mptr.writeWheelPosition(00,90+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));
    read(mptr.fd4,array,20);read(mptr.fd2,array,20);

    mptr.delayTimeMsecs(6000);																//延时

    mptr.writeWheelSpeed(00,-mptr.wheelMoveSpeedSet);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,mptr.wheelMoveSpeedSet);
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

    //mptr.delayTimeMsecs(6000);																//延时

    mptr.writeWheelPosition(00,0+mptr.wheelFrontAngleOffset);           					// angle to zero
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,0+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

}

/*************************************        **************************************************************************************/
int MainWindow::Position_PID (int Encoder,int Target)
{
     static double Bias,Pwm,Integral_bias,Last_Bias;
     Bias=Target-Encoder;                                  //ŒÆËãÆ«²î
     Integral_bias+=Bias;	                                 //Çó³öÆ«²îµÄ»ý·Ö
     if(Integral_bias>120)Integral_bias=120;
     if(Integral_bias<-120)Integral_bias=-120;
     Pwm=mptr.KP*Bias+mptr.KI*Integral_bias+mptr.KD*(Bias-Last_Bias);       //Î»ÖÃÊœPID¿ØÖÆÆ÷
     Last_Bias=Bias;                                       //±£ŽæÉÏÒ»ŽÎÆ«²î
     if (Pwm>45) Pwm = 45;
     if (Pwm<-45) Pwm = -45;
     return Pwm;                                           //ÔöÁ¿Êä³ö
}

/*************************************         *************************************************************************************/
int MainWindow::Incremental_PI (int Encoder,int Target)
{
     static int Bias,Pwm,Last_bias;
     Bias=Encoder-Target;                //ŒÆËãÆ«²î
     Pwm+=mptr.KP*(Bias-Last_bias)+mptr.KI*Bias;   //ÔöÁ¿ÊœPI¿ØÖÆÆ÷
     if(Pwm>45)Pwm=45;
     if(Pwm<-45)Pwm=-45;
     Last_bias=Bias;	                   //±£ŽæÉÏÒ»ŽÎÆ«²î
     return Pwm;                         //ÔöÁ¿Êä³ö
}

/*************************************         *************************************************************************************/
void MainWindow::on_setPIDButton_clicked()
{
    mptr.KP_turn = ui->kpSpinBox->value();
    mptr.KI_turn = ui->kiSpinBox->value();
    mptr.KD_turn = ui->kdSpinBox->value();
}

void MainWindow::on_label_3_linkActivated(const QString &link)
{

}

void MainWindow::on_kdSpinBox_editingFinished()
{

}
