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
#include <math.h>
#include <QIODevice>
#include <QFile>
#include <QTextStream>
#include <QTextCodec>


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
    mptr.delayTimeMsecs(1000);
    tcpSocket->write(mptr.startReadGun);

/*****************  上位机TCPIP协议的连接  ****************/
   tcpSocket1 = NULL;
   tcpSocket1 = new QTcpSocket(this);
   QString ip1= "192.168.10.12"; //"10.0.0.2";
   //QString ip="192.168.10.11";
   //QString ip="10.0.0.3";
   qint16 port1=6666;
   tcpSocket1->connectToHost(ip1,port1);
   connect(tcpSocket1,SIGNAL(readyRead()),this,SLOT(readWifi()));
   connect(tcpSocket1,SIGNAL(disconnected()),this,SLOT(reconnect()));
   connect(tcpSocket1,SIGNAL(connected()),this,SLOT(TCPconnected()));
/****************    文件设置   ***************************/
   routeFile.setFileName("/home/pi/lys/oldRoute.txt");          //路径需要修改
   yawFile.setFileName("/home/pi/lys/yawOfInertial.txt");       //惯导记录文件

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
    ui->kdSpinBox->setSingleStep(2);
    ui->kpSpinBox->setValue(60);
    ui->kiSpinBox->setValue(300);
    ui->kdSpinBox->setValue(30);
    ui->turnSpeedSetEdit->setText(c1.setNum(mptr.wheelTurnSpeed));
    errorReport = new errorReportWindow(this);

//    write(mptr.fd5,mptr.seri_send_buzzer4,sizeof(mptr.seri_send_buzzer4));
    mptr.systemOnLight = 0;
    mptr.alarmLight = 0;
    mptr.warmingLight = 0;
    mptr.sickA = 0;
    mptr.sickB = 0;
    mptr.sickC = 0;
    mptr.writeIO();
}

MainWindow::~MainWindow()
{
    routeFile.close();
    dontShutDownFlag = false;
    delete ui;
}

/****************系统初始化（包括舵轮权限获取、使能、舵轮校零）并启动系统开始工作**************************************************************************/
void MainWindow::initialTheSystem(void)
{
    QString temp;
    QStringList oldRouteList;
    QString strTemp;
    mptr.gainAccessAndEnableWheel();
    mptr.readIO();                               //read I/O and store the data
    ui->CommunicationEdit->append(tr("%1 %2 %3").arg(mptr.systemOnFlag).arg(mptr.sickFalse).arg(mptr.sickWarningSpaceAlert));

//    write(mptr.fd5,mptr.seri_send_buzzer1,sizeof(mptr.seri_send_buzzer1));
    mptr.systemOnLight = 1;
    mptr.alarmLight = 0;
    mptr.warmingLight = 0;
    mptr.sickA = 0;
    mptr.sickB = 0;
    mptr.sickC = 0;
    mptr.writeIO();

    if (mptr.calibrationFlag == false)
        {
            if(ui->calibrationCheck->isChecked())
            {
                wheelZeroCalibration();                  //find the zero point of the steering wheel
            }
        }

    mptr.readBattery();
    ui->CommunicationEdit->append("readBattery");
    //mptr.autoRuningFlag = true;
    mptr.readBattery();
    ui->batteryCollumLabel->setNum(mptr.batteryCollum);
    ui->batteryCurrentLble->setNum(mptr.batteryCurrent);
    ui->batteryVoltageLable->setNum(mptr.batteryVoltage);

    mptr.Code_Init();  //给上位机发小车坐标
    ui->CommunicationEdit->append("code_init ready!");
    while(dontShutDownFlag)
    {
        routeFile.open(QIODevice::ReadWrite);
          QString string1;
          QTextStream in(&routeFile);
          ui->CommunicationEdit->append("read Data！");
          string1 = in.readAll();
          if(string1 != "")
          {
              mptr.oldRouteExistFlag = true;
              ui->labelOfMissionStatus->setText("任务进行中");
              //读取文件内容，并且将路径放置于相应的数组中
              oldRouteList = string1.split(":");                  //读取路径
              for(int i=0;i<(oldRouteList.count())/2;i++)
              {
                  temp = string1.split(":")[2*i];
                  mptr.P_Target[i].X=temp.toDouble();
                  temp = string1.split(":")[2*i+1];
                  mptr.P_Target[i].Y=temp.toDouble();
              }
              mptr.targetNumber = oldRouteList.count()/2;         //路径点个数
              ui->CommunicationEdit->append("读取路径");

              oldRouteList = string1.split(";");                  //读取圆心
              mptr.numberOfTurnCentre = oldRouteList.count()/2 - 1;
              for(int m =0;m<mptr.numberOfTurnCentre;m++)
              {
                  temp = string1.split(";")[2*m+1];
                  mptr.P_Target4[m].X = temp.toDouble();
                  temp = string1.split(";")[2*m+2];
                  mptr.P_Target4[m].Y = temp.toDouble();
              }
              ui->CommunicationEdit->append("读取yuanxin");

              oldRouteList = string1.split("/");                  //读取入弯点
              mptr.numberOfTurnIn = oldRouteList.count()/2-1;
              for(int i=0;i<mptr.numberOfTurnIn;i++)
              {
                  temp = string1.split("/")[2*i+1];
                  mptr.P_Target3[i].X = temp.toDouble();
                  temp = string1.split("/")[2*i+2];
                  mptr.P_Target3[i].Y = temp.toDouble();
              }
              ui->CommunicationEdit->append("读取ruwandian");

              routeFile.close();
              mptr.getRouteFlag = true;
          }
          else
          {
              mptr.oldRouteExistFlag = false;

          }
          routeFile.close();




          if(mptr.getRouteFlag)                                               //如果获取到新路径
          {
              mptr.oldRouteExistFlag = true;
              routeFile.open(QIODevice::WriteOnly);                               //清空旧文件
              routeFile.close();

              routeFile.open(QIODevice::WriteOnly|QIODevice::Text);               //写文件，存储路径
              QTextStream out(&routeFile);
              string1.clear();

              for(int i=0;i<mptr.targetNumber;i++)          //存路径
              {
                  string1 += strTemp.setNum(mptr.P_Target[i].X) + ":";
                  string1 += strTemp.setNum(mptr.P_Target[i].Y) + ":";
              }

              string1 += ";";                             //存圆心
              for(int i=0;i<mptr.numberOfTurnCentre;i++)
              {
                  string1 += strTemp.setNum(mptr.P_Target4[i].X) + ";";
                  string1 += strTemp.setNum(mptr.P_Target4[i].Y) + ";";
              }

              string1 += "/";
              for(int i=0;i<mptr.numberOfTurnIn;i++ )     //存入弯点
              {
                  string1 += strTemp.setNum(mptr.P_Target3[i].X) + "/";
                  string1 += strTemp.setNum(mptr.P_Target3[i].Y) + "/";
              }
              ui->CommunicationEdit->append("new file:");
//                ui->CommunicationEdit->append(string1);
              out<<string1<<endl;
              out.flush();
              routeFile.close();
              ui->labelOfMissionStatus->setText("任务进行中");
              ui->CommunicationEdit->append("initial");
              mptr.Code_Init();

              if(mptr.initialReady == true)
              {
                  //ui->CommunicationEdit->append("systemON");
                  systemOn();
                  if(mptr.fileClearFlag)
                  {
                      routeFile.open(QIODevice::WriteOnly);               //执行完成清空路径存储文件
                      routeFile.close();
                  }
                  for(int i=0;i<100;i++)
                  {
                      mptr.P_Target4[i].X=0;mptr.P_Target4[i].Y=0;//清空圆心存放结构体数组
                      mptr.P_Target[i].X=0;mptr.P_Target[i].Y=0;//清空路径存放结构体数组
                      mptr.P_Target3[i].X=0;mptr.P_Target3[i].Y=0;//清空入弯点存放结构体数组
                      mptr.start_end[i].X=0;mptr.start_end[i].Y=0;//清空接收数组
                  }
                  mptr.getRouteFlag = false;                          //执行完成，清空标志位
                  mptr.numberOfTurnCentre = 0;                        //圆心数量
                  mptr.targetNumber = 0;                              //目标点数量
                  mptr.numberOfTurnIn = 0;                  
                  mptr.wheelMoveSpeedSet = mptr.wheelSpeedTarget;
                  mptr.breakFlag = false;

                  ui->labelOfMissionStatus->setText("任务已完成");
              }
          }
          else            //运行结束后，getRouteFlag置为false，此时，需要将文件清空
          {
              routeFile.open(QIODevice::WriteOnly);       //清空文件
              routeFile.close();
          }
          writeWifi();                                //与上位机进行交互

        //ui->CommunicationEdit->append(tr("init "));
        mptr.readBattery();                                   //读电池数据
        ui->batteryCollumLabel->setNum(mptr.batteryCollum);
        ui->batteryCurrentLble->setNum(mptr.batteryCurrent);
        ui->batteryVoltageLable->setNum(mptr.batteryVoltage);
        QApplication::processEvents(QEventLoop::AllEvents,1000);
//        write(mptr.fd5,mptr.seri_send_buzzer4,sizeof(mptr.seri_send_buzzer4));

        mptr.delayTimeMsecs(2000);
    }

}

/*****************系统运行函数****************************************************************************************************************************/
void MainWindow::systemOn(void)
{
    QString strYaw;
    QString a;
    double speedBefore=1;
    int j,i=0;
    double data_yaw[10];
	unsigned char array[20]= {0};
   // double speedTemp =0;
    double R=0;
    //mptr.AGVLocation={0,0};
//    mptr.breakFlag = false;
    mptr.wheelSpeedHold = mptr.wheelSpeedTarget;
    mptr.Speed_Td_x1 = 0;
    mptr.Flag_Stop = false;
    mptr.Flag_SpeedDe = false;
    mptr.Flag_SpeedAdd = false;
    mptr.Flag_char = false;
    mptr.Flag_charOver = false;

    for(i=0;i<10;i++)  //采10次角度
    {
        mptr.readIO();
        data_yaw[i] = mptr.yaw;
    }

    for(i=0;i<10;i++) //排序
    {
        for(j=i;j<10;j++)
        {
            if(data_yaw[i]<data_yaw[j])
            {
                double temp;
                temp = data_yaw[i];
                data_yaw[i] = data_yaw[j];
                data_yaw[j] = temp;
            }
        }
    }

    for(i=2;i<8;i++) //去掉最大和最小
    {
        double sun_yaw=0;
        sun_yaw += data_yaw[i];
        if(i==7)
           mptr.yaw = sun_yaw/6;
    }
    mptr.yawLast = mptr.yaw;

//    write(mptr.fd5,mptr.seri_send_buzzer1,sizeof(mptr.seri_send_buzzer1));
    mptr.alarmLight = 0;
    mptr.systemOnLight = 1;
    mptr.warmingLight = 0;
    mptr.writeIO();

    if(mptr.direction_flag == false)
    {
        if(mptr.wheelMoveSpeedSet > 0)
        {
            mptr.wheelMoveSpeedSet = -mptr.wheelMoveSpeedSet;
            mptr.wheelSpeedTarget = mptr.wheelMoveSpeedSet;
            mptr.wheelSpeedHold =  mptr.wheelMoveSpeedSet;
        }
        else
        {
            mptr.wheelMoveSpeedSet = mptr.wheelMoveSpeedSet;
            mptr.wheelSpeedTarget =  mptr.wheelMoveSpeedSet;
            mptr.wheelSpeedHold =  mptr.wheelMoveSpeedSet;
        }

    }
    else
    {
        if(mptr.wheelMoveSpeedSet < 0)
        {
            mptr.wheelMoveSpeedSet = -mptr.wheelMoveSpeedSet;
            mptr.wheelSpeedTarget = mptr.wheelMoveSpeedSet;
            mptr.wheelSpeedHold =  mptr.wheelMoveSpeedSet;
        }
        else
        {
            mptr.wheelMoveSpeedSet = mptr.wheelMoveSpeedSet;
            mptr.wheelSpeedTarget =  mptr.wheelMoveSpeedSet;
            mptr.wheelSpeedHold =  mptr.wheelMoveSpeedSet;
        }
    }

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
                writeWifi();
                mptr.checkIO();
                ui->CommunicationEdit->append(tr("%1\t").arg(mptr.buf));
                ui->CommunicationEdit->append(tr("%1\t").arg(mptr.buf_last));
                ui->CommunicationEdit->append(tr("%1\t%2\t%3").arg(mptr.yawTarget).arg(mptr.yaw).arg(mptr.yawInt));
                ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.Td_SpeedSet).arg(mptr.wheelRearAngle));
                ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.turn_flag).arg(mptr.targetNumber));
                ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.delta_s).arg(mptr.yaw_error));
                ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.a).arg(mptr.b));
                ui->CommunicationEdit->append(tr("%1\t%2\t%3").arg(mptr.Flag_Stop).arg(mptr.wheelAngle).arg(mptr.num));
//                write(mptr.fd2,mptr.readPositionData,sizeof(mptr.readPositionData));
//                ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd2)).toHex());
//                write(mptr.fd4,mptr.readPositionData,sizeof(mptr.readPositionData));
//                ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd4)).toHex());
                //ui->CommunicationEdit->append(mptr.Flag_SpeedAdd);
                //ui->CommunicationEdit->append(mptr.Flag_SpeedDe);

                //ui->CommunicationEdit->append(tr("FrontAngleOffset: %1").arg(mptr.wheelFrontAngleOffset));
                //ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.P_protection.X).arg(mptr.P_protection.Y));

                ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.P_Target[mptr.num].X).arg(mptr.P_Target[mptr.num].Y));
                ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.AGVLocation.X).arg(mptr.AGVLocation.Y));                

                //ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.QR_Code_Number).arg(mptr.Angle_QRtoCar));
                ui->read->append(tr("%1\t%2").arg(mptr.turn_flag).arg(mptr.direction_flag));

                //ui->read->append(tr("%1\t").arg(mptr.turn_flag));
                //ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.QR_Point[0].X).arg(mptr.QR_Point[0].Y));
                //ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.QR_Point[1].X).arg(mptr.QR_Point[1].Y));
                //ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.QR_Point[2].X).arg(mptr.QR_Point[2].Y));
                //ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.QR_Point[3].X).arg(mptr.QR_Point[3].Y));
                //ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.QR_Point[4].X).arg(mptr.QR_Point[4].Y));

                strYaw += a.setNum(mptr.yaw) + '\n';
                showIOResult();                         // 界面显示读数结果，如果急停进入等待复位状态。

				if(mptr.emergencyFlag == false)			//emergencyFlag目前不使用，可忽略
                {
                    mptr.Speed_Adj();
                    if (fabs(mptr.Td_SpeedSet)>fabs(mptr.wheelMoveSpeedSetMax))
                        mptr.Td_SpeedSet = mptr.wheelMoveSpeedSetMax;
                    if(speedBefore != mptr.Td_SpeedSet)	//如果速度没变化不在对舵轮写速度。
					{
                        speedBefore = mptr.Td_SpeedSet;

                        mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);	//生成速度设置报文
                        write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
                        read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
                        mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);
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
        {
            break;
        }
        QApplication::processEvents(QEventLoop::AllEvents,1000);			//Qt任务队列等待，释放系统资源

        t_2=QTime::currentTime().msec();
        if(t_2>t_1)
            delta_t=t_2-t_1+0.2;
        else
            delta_t=1000+t_2-t_1+0.2;

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
        R=mptr.Go2(mptr.P_Target[mptr.num]);

        if(mptr.turn_flag==true)
        {
            //mptr.P_Centre = mptr.P_Target4[mptr.Centre_num];

            mptr.delta_s=mptr.Position_Turn_crol (mptr.P_Centre,mptr.P_Target[mptr.num],mptr.AGVLocation,4);
        }
        else
        {
            mptr.delta_s=mptr.Straight_Line (mptr.AGVLocation,mptr.P_Target[mptr.num-1],mptr.P_Target[mptr.num]);
        }
    }
    while(1)
    {
        mptr.wheelSpeedHold = 0;
        mptr.wheelMoveSpeedSet = 0;
        if(fabs(mptr.Speed_Td_x1)>0.02)
        {
            mptr.Speed_h=0.01;
            mptr.Speed_Adj();
            if (mptr.Td_SpeedSet>mptr.wheelMoveSpeedSetMax)
                mptr.Td_SpeedSet = mptr.wheelMoveSpeedSetMax;
            if(mptr.Speed_Td_x1 <=0 || mptr.Speed_Td_x1 > 2)
                mptr.Speed_Td_x1 = 0;
             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);	//生成速度设置报文
             write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
             read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);
             write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
             read(mptr.fd3,array,sizeof(array));

        }
        else
        {
            mptr.writeWheelSpeed(00,00,mptr.commanData);	//生成速度设置报文
            write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
            read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
            mptr.writeWheelSpeed(00,00,mptr.commanData);
            write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
            read(mptr.fd3,array,sizeof(array));
            break;
        }
        //QApplication::processEvents(QEventLoop::AllEvents,1000);
    }

    if(mptr.P_Target[mptr.targetNumber-1].X == mptr.P_charge.X)
    {
        if(mptr.P_Target[mptr.targetNumber-1].Y == mptr.P_charge.Y)
        {
            mptr.Flag_char = true;
            mptr.sickA = 0;
            mptr.sickB = 0;
            mptr.sickC = 0;
            mptr.writeIO();
            if(mptr.num == mptr.targetNumber)
               mptr.readyToChargeFlag = true;
        }
    }
    if(mptr.Flag_char && (mptr.readyToChargeFlag))
    {
        mptr.writeWheelPosition(00,mptr.wheelFrontAngleOffset+90);
        write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,sizeof(array));
        mptr.writeWheelPosition(00, mptr.wheelRearAngleOffset+90);
        write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,sizeof(array));
        mptr.delayTimeMsecs(8000);
        mptr.wheelMoveSpeedSet = 0.05;
    }

    while(mptr.Flag_char && (mptr.readyToChargeFlag))
    {
        int t_3,t_4;
        int delta_t;
        t_3 = QTime::currentTime().msec();
        mptr.readIO1();
        mptr.Speed_charge = mptr.Position_PID3(mptr.yawTarget - mptr.yaw);
        ui->CommunicationEdit->append(tr("%1\t%2\t%3").arg(mptr.yawTarget).arg(mptr.yaw).arg(mptr.yawInt));
        ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.AGVLocation.X).arg(mptr.AGVLocation.Y));
        ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.AGVSpeed).arg(mptr.AGVLocation.Y));

        mptr.writeWheelSpeed(mptr.wheelMoveSpeedSet - mptr.Speed_charge,00,mptr.commanData);								//任务队列完成后，停车
        write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));
        read(mptr.fd1,array,sizeof(array));
        mptr.delayTimeMsecs(mptr.delayTimeSet);
        mptr.writeWheelSpeed(mptr.wheelMoveSpeedSet + mptr.Speed_charge,00,mptr.commanData);
        write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
        read(mptr.fd3,array,sizeof(array));

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
           ui->CommunicationEdit->append(tr("%1\t").arg(mptr.buf));
           ui->CommunicationEdit->append(tr("%1\t").arg(mptr.buf_last));
       }

        t_4 = QTime::currentTime().msec();
        if(t_4>t_3)
            delta_t=t_4-t_3;
        else
            delta_t=1000+t_4-t_3;
        //ui->CommunicationEdit->append(tr("%1").arg(delta_t));
        mptr.AGVLocation.Y = mptr.AGVLocation.Y-fabs(mptr.AGVSpeed) * delta_t/1000;
        if(mptr.AGVLocation.Y <= -1.5)
        {
            mptr.Flag_char = false;
            mptr.Td_SpeedSet = 0;
            mptr.Speed_Td_x1 = 0;
            mptr.waitingForCharging = true;
        }
        QApplication::processEvents(QEventLoop::AllEvents,100);
    }

    while(1)
    {
        ui->CommunicationEdit->append("De Success!");
        mptr.wheelSpeedHold = 0;
        mptr.wheelMoveSpeedSet = 0;
        if(fabs(mptr.Speed_Td_x1)>0.02)
        {
            mptr.Speed_h=0.01;
            mptr.Speed_Adj();
            if (mptr.Td_SpeedSet>mptr.wheelMoveSpeedSetMax)
                mptr.Td_SpeedSet = mptr.wheelMoveSpeedSetMax;
            if(mptr.Speed_Td_x1 <=0 || mptr.Speed_Td_x1 > 2)
                mptr.Speed_Td_x1 = 0;
             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);	//生成速度设置报文
             write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
             read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);
             write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
             read(mptr.fd3,array,sizeof(array));

        }
        else
        {

            break;
        }
        //QApplication::processEvents(QEventLoop::AllEvents,1000);
    }

    mptr.writeWheelSpeed(00,00,mptr.commanData);								//任务队列完成后，停车
    write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));
    read(mptr.fd1,array,sizeof(array));
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    mptr.writeWheelSpeed(00,00,mptr.commanData);
    write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
    read(mptr.fd3,array,sizeof(array));

    while(mptr.waitingForCharging)              //等待充电
    {
        ui->CommunicationEdit->append("wait Success!");
        for(int i = 0; i<10;i++)
            mptr.delayTimeMsecs(1000);
        mptr.readBattery();
        ui->batteryCollumLabel->setNum(mptr.batteryCollum);
        ui->batteryCurrentLble->setNum(mptr.batteryCurrent);
        ui->batteryVoltageLable->setNum(mptr.batteryVoltage);
        if(mptr.batteryCollum>=95)
        {
            mptr.waitingForCharging = false;
            mptr.Flag_charOver = true;
            mptr.wheelMoveSpeedSet = -0.05;
        }
    }

    while(mptr.Flag_charOver && (mptr.readyToChargeFlag))
    {
        int t_3,t_4;
        int delta_t;
        t_3 = QTime::currentTime().msec();
        mptr.readIO1();
        mptr.Speed_charge = mptr.Position_PID3(mptr.yawTarget - mptr.yaw);
        ui->CommunicationEdit->append(tr("%1\t%2\t%3").arg(mptr.yawTarget).arg(mptr.yaw).arg(mptr.yawInt));
        ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.AGVLocation.X).arg(mptr.AGVLocation.Y));
        ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.AGVSpeed).arg(mptr.AGVLocation.Y));

        mptr.writeWheelSpeed(mptr.wheelMoveSpeedSet - mptr.Speed_charge,00,mptr.commanData);								//任务队列完成后，停车
        write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));
        read(mptr.fd1,array,sizeof(array));
        mptr.delayTimeMsecs(mptr.delayTimeSet);
        mptr.writeWheelSpeed(mptr.wheelMoveSpeedSet + mptr.Speed_charge,00,mptr.commanData);
        write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
        read(mptr.fd3,array,sizeof(array));

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
           ui->CommunicationEdit->append(tr("%1\t").arg(mptr.buf));
           ui->CommunicationEdit->append(tr("%1\t").arg(mptr.buf_last));
       }

        t_4 = QTime::currentTime().msec();
        if(t_4>t_3)
            delta_t=t_4-t_3;
        else
            delta_t=1000+t_4-t_3;
        //ui->CommunicationEdit->append(tr("%1").arg(delta_t));
        mptr.AGVLocation.Y = mptr.AGVLocation.Y+fabs(mptr.AGVSpeed) * delta_t/1000;
        if(mptr.AGVLocation.Y >= 0)
        {
            mptr.Flag_char = false;
            mptr.Flag_charOver = false;
            mptr.Td_SpeedSet = 0;
            mptr.Speed_Td_x1 = 0;
        }
        QApplication::processEvents(QEventLoop::AllEvents,100);
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

    mptr.delayTimeMsecs(30);
//    write(mptr.fd5,mptr.seri_send_buzzer4,sizeof(mptr.seri_send_buzzer4));
    mptr.systemOnLight = 0;
    mptr.alarmLight = 0;
    mptr.warmingLight = 0;
	mptr.sickA = 0;
    mptr.sickB = 0;
    mptr.sickC = 0;
	mptr.writeIO();

    QString strError;
    QTextStream out(&mptr.warningFile);
    if(mptr.errorReportFlag)
    {
        if(mptr.lostQRCodeFlag)
        {
            mptr.errorInformation += QTime::currentTime().toString() + "  ";
            mptr.errorInformation += "丢码停车";
            mptr.warningFile.open(QIODevice::WriteOnly|QIODevice::Append);
            strError.clear();
            strError += QTime::currentTime().toString() + "  " + "丢码停车";
            out<<strError<<endl; out.flush();strError.clear();
            mptr.warningFile.close();
        }
        ui->CommunicationEdit->append("错误！请从文件中提取信息");
        ui->CommunicationEdit->append(mptr.errorInformation);
        writeErrorInformation();
        mptr.errorReportFlag = false;

    }
    yawFile.open(QIODevice::WriteOnly|QIODevice::Append);
    QTextStream out1(&yawFile);
    out1<<strYaw<<endl;
    out1.flush();
    yawFile.close();
    strYaw.clear();
}
/***********************    二维码信息读取   **************************/
void MainWindow::ReadData ()
{
     //if(mptr.buf == NULL)

         QString buffer=tcpSocket->readAll();    //
         mptr.buf = buffer;
         //buffer.clear();

     //数据处理
    QStringList Part_Inf= buffer.split(" ");
    QString a1=Part_Inf[0];       QString a7=Part_Inf[6];
    ui->labelOfQRCode->setText(a1);

     tcpSocket->write(mptr.startReadGun);

}

/**********************    断开重新连接上位机   ***********************/
void MainWindow::reconnect()
{
    QString a;
    tcpSocket1 = new QTcpSocket(this);
    QString ip1= "192.168.10.12"; //"10.0.0.2";
    //QString ip="192.168.10.11";
    //QString ip="10.0.0.3";
    qint16 port1=6666;
    tcpSocket1->connectToHost(ip1,port1);
    connect(tcpSocket1,SIGNAL(readyRead()),this,SLOT(readWifi()));
    connect(tcpSocket1,SIGNAL(connected()),this,SLOT(TCPconnected()));
    connect(tcpSocket1,SIGNAL(disconnected()),this,SLOT(reconnect()));

    ui->TCPStatusLabel->setText("上位机连接失败，手动重连");
    if(mptr.TCPconnectFlag == true)
    {
        mptr.warningFile.open(QIODevice::WriteOnly|QIODevice::Append);
        QTextStream out(&mptr.warningFile);
        a +=QTime::currentTime().toString() + "与上位机无线信号断开" + "  ";
        out<<a<<endl;
        out.flush();
        mptr.warningFile.close();
        mptr.TCPconnectFlag = false;
    }

}

void MainWindow::TCPconnected()
{
    mptr.TCPconnectFlag = true;
    ui->TCPStatusLabel->setText("上位机连接成功");
}

/**********************     读取wifi   *********************************/
void MainWindow::readWifi()
{
    //加条件，运行时候不接受新信息
    if(mptr.oldRouteExistFlag)
    {
//        tcpSocket1->write("invacant!");

    }
    else
    {
        int init=1;         //初值
        int target_angel=0;
        int u,v;
        static QString bufTemp;
        int lengthall =0;
        QString temp;
        QString buffer = tcpSocket1->readAll();
    //    std::string bufferStdString = (buffer.toStdString());
    //    QString bufferString;
    //    bufferString.fromStdString(bufferStdString);
        ui->CommunicationEdit->append("收到的字符串：");
        ui->CommunicationEdit->append(buffer);      //显示接收到的字符串

        //QByteArray splitCoeffcient= ":";
        //tcpSocket 清除缓冲区
//        if((buffer != bufTemp))
        {
            bufTemp = buffer;
            mptr.bufWifi = buffer;
            lengthall=buffer.length();
            //信息校验有问题，需要请求重发
            if(buffer=="")
            {
               // QMessageBox::information(NULL,QString("提示"),QString("无历史路径，请重新设置"));
            }
            else
            {
                mptr.getRouteFlag = true;
                mptr.oldRouteExistFlag = true;
                for(int i=0;i<lengthall;i++)
                {
                    if(buffer.mid(i,1)==":")
                        mptr.numberOfStaEnd++;
                }

                for(int i=0;i<(mptr.numberOfStaEnd-2)/4;i++)
                {
                    temp=buffer.split(":")[2+4*i];
                    mptr.start_end[2*i].X=temp.toInt();
                    temp=buffer.split(":")[3+4*i];
                    mptr.start_end[2*i].Y=temp.toInt();
                    temp=buffer.split(":")[4+4*i];
                    mptr.start_end[2*i+1].X=temp.toInt();
                    temp=buffer.split(":")[5+4*i];
                    mptr.start_end[2*i+1].Y=temp.toInt();
                }
                mptr.numberOfStaEnd = (mptr.numberOfStaEnd-2)/2;//起点终点的数量
                for(int i=0; i<mptr.numberOfStaEnd/2; i++)
                {
                    init = mptr.Trace(mptr.start_end[2*i].X, mptr.start_end[2*i].Y, mptr.start_end[2*i+1].X, mptr.start_end[2*i+1].Y,init);
                    mptr.targetNumber = init-1;
                }

                for(int i =0; i<init-1; i++)              //显示
                {
                    ui->CommunicationEdit->append(tr("%1\t%2").arg(mptr.P_Target[i].X).arg(mptr.P_Target[i].Y));
                }

                for(int i=1; i<mptr.numberOfStaEnd-2; i+=2)                     //自动生成圆心
                {
                    if(mptr.start_end[i+1].X== mptr.start_end[i+2].X)                   //计算圆心角
                    {
                        if(mptr.start_end[i+2].Y> mptr.start_end[i+1].Y)
                            target_angel=90;
                        else target_angel=270;
                    }
                    if(mptr.start_end[i+1].Y==mptr.start_end[i+2].Y)
                    {
                        if(mptr.start_end[i+2].X>mptr.start_end[i+1].X)
                            target_angel=0;
                        else target_angel=180;
                    }
                    u=0, v=0;//定义转弯向量（0,2）经旋转的向量
                    if(cos(target_angel*pi/180)<1.1&&cos(target_angel*pi/180)>0.9)
                        u = 2*1;
                    if(cos(target_angel*pi/180)<0.1&&cos(target_angel*pi/180)>-0.1)
                        u = 2*0;
                    if(cos(target_angel*pi/180)<-0.9&&cos(target_angel*pi/180)>-1.1)
                        u = 2*(-1);
                    if(sin(target_angel*pi/180)<1.1&&sin(target_angel*pi/180)>0.9)
                        v = 2*1;
                    if(sin(target_angel*pi/180)<0.1&&sin(target_angel*pi/180)>-0.1)
                        v = 2*0;
                    if(sin(target_angel*pi/180)<-0.9&&sin(target_angel*pi/180)>-1.1)
                        v = 2*(-1);
                   // u = 2*cos(target_angel*pi/180);
                    //v = 2*sin(target_angel*pi/180);
                    mptr.P_Target4[(i-1)/2].X = mptr.start_end[i].X + u;
                    mptr.P_Target4[(i-1)/2].Y = mptr.start_end[i].Y + v;

                    mptr.numberOfTurnCentre++;

    //                center_x？？？？？？？？ = start_end[i-1].X + u;//计算转弯圆心
    //                center_y ？？？？？？？？？？？？？？？？= start_end[i-1].Y + v;

                }
                mptr.numberOfTurnIn = mptr.numberOfTurnCentre;

                for(int i=1;i<mptr.numberOfStaEnd-2;i +=2)            //存放入弯点
                {
                    mptr.P_Target3[(i-1)/2].X = mptr.start_end[i].X;
                    mptr.P_Target3[(i-1)/2].Y = mptr.start_end[i].Y;
                }
                mptr.numberOfStaEnd = 0;
            }
        }
//        routeFile.close();
    }
        //tcpSocket 清除缓冲区
}

/**************************  写WiFi （将AGV的信息拼接字符串之后送到上位机）  ***********************/
void MainWindow::writeWifi(void)
{
    //L:sta:x:y:theta:speed:baterry:voltage:current:end

    QString intergrate="sta:";
    QString addTemp;
    //addTemp.setNum((int)mptr.AGVSpeed);
    intergrate += addTemp.setNum(mptr.AGVLocation.X) + ":" +addTemp.setNum(mptr.AGVLocation.Y) + ":";
    intergrate += addTemp.setNum(mptr.yaw) + ":";
    intergrate += addTemp.setNum(mptr.AGVSpeed) + ":";
    intergrate += addTemp.setNum(mptr.batteryCollum) + ":" + addTemp.setNum(mptr.batteryVoltage) + ":"
            + addTemp.setNum(mptr.batteryCurrent) + ":";
   // intergrate += "00000000:";
    intergrate += addTemp.setNum(!mptr.oldRouteExistFlag) + ":";
    intergrate += "end";

//    int length = intergrate.length();
//    intergrate.prepend(":");
//    intergrate.prepend(addTemp.setNum(length));

//    ui->CommunicationEdit->append(intergrate);

    QByteArray message;
    message = intergrate.toLocal8Bit();

//    QDataStream out(&message,QIODevice::ReadWrite);
//    out.setVersion(QDataStream::Qt_5_0);
//    out<<intergrate;
    tcpSocket1->write(message);

}

void MainWindow::writeErrorInformation()
{
    QString a;
//    QTextCodec *code = QTextCodec::codecForName("GBK");
//    QTextCodec::setCodecForLocale(code);


    a += mptr.errorInformation;
    a.prepend("err:");
    a += ";end";
    QByteArray message;
    message = a.toUtf8();

//    message = a.tou;
//    a = code->toUnicode(a.toLocal8Bit());

    tcpSocket1->write(message);

    mptr.errorInformation.clear();
//    QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
}

/***********************前进按钮，按压前进，速度为设定值***************************************************************************************/
void MainWindow::on_forwardButton_pressed()
{
    unsigned char array[20];
//    switch(mptr.wheelAddress)
//    {
//        case 1:mptr.writeWheelSpeed(mptr.wheelMoveSpeedSet,0,mptr.commanData);
//                write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));//fflush(stdout);
//                ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd1)).toHex());
//                mptr.writeWheelSpeed(mptr.wheelMoveSpeedSet,0,mptr.commanData);
//                write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));//fflush(stdout);
//                ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
//                break;
//        case 3:mptr.writeWheelSpeed(mptr.wheelMoveSpeedSet,0,mptr.commanData);
//                write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));//fflush(stdout);
//                ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication(mptr.fd3)).toHex());
//                break;
//        default: break;
//    }
    mptr.Flag_Forward_push = true;
    while(mptr.Flag_Forward_push)
    {

        if(fabs(mptr.Speed_Td_x1 - mptr.wheelMoveSpeedSet)>0.01)
        {
            mptr.Speed_h=0.01;
            mptr.Speed_Adj();
            if (mptr.Td_SpeedSet>mptr.wheelMoveSpeedSetMax)
                mptr.Td_SpeedSet = mptr.wheelMoveSpeedSetMax;

             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);	//生成速度设置报文
             write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
             read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);
             write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
             read(mptr.fd3,array,sizeof(array));

        }
        else
        {
            break;
        }
        QApplication::processEvents(QEventLoop::AllEvents,1000);
    }
}

/************************前进按钮，弹起停车**************************************************************************************************/
void MainWindow::on_forwardButton_released()
{
    unsigned char array[20];
    double Speed_De;
    /*****
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
    *********/
    Speed_De = mptr.wheelMoveSpeedSet;
    mptr.Flag_Forward_push = false;
    while(1)
    {
        mptr.wheelMoveSpeedSet = 0;
        if(fabs(mptr.Speed_Td_x1 - mptr.wheelMoveSpeedSet)>0.01)
        {
            mptr.Speed_h=0.01;
            mptr.Speed_Adj();
            if (mptr.Td_SpeedSet>mptr.wheelMoveSpeedSetMax)
                mptr.Td_SpeedSet = mptr.wheelMoveSpeedSetMax;

             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);	//生成速度设置报文
             write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
             read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);
             write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
             read(mptr.fd3,array,sizeof(array));

        }
        else
        {
            mptr.writeWheelSpeed(00,00,mptr.commanData);	//生成速度设置报文
            write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
            read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
            mptr.writeWheelSpeed(00,00,mptr.commanData);
            write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
            read(mptr.fd3,array,sizeof(array));

            break;
        }

    }
    mptr.wheelMoveSpeedSet = Speed_De;

}

/****************************后退按钮，按压后退，速度为设定值**********************************************************************************/
void MainWindow::on_backwardButton_pressed()
{
    unsigned char array[20];
    /***
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
    *****/
    mptr.Flag_backward_push = true;
    if(mptr.wheelMoveSpeedSet > 0)
        mptr.wheelMoveSpeedSet = -mptr.wheelMoveSpeedSet;
    while(mptr.Flag_backward_push)
    {

        if(fabs(mptr.Speed_Td_x1 - mptr.wheelMoveSpeedSet)>0.01)
        {
            mptr.Speed_h=0.01;
            mptr.Speed_Adj();
            if (mptr.Td_SpeedSet>mptr.wheelMoveSpeedSetMax)
                mptr.Td_SpeedSet = mptr.wheelMoveSpeedSetMax;

             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);	//生成速度设置报文
             write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
             read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);
             write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
             read(mptr.fd3,array,sizeof(array));

        }
        else
        {
            break;
        }
        QApplication::processEvents(QEventLoop::AllEvents,1000);
    }
}

/*****************************后退按钮，弹起停车***********************************************************************************/
void MainWindow::on_backwardButton_released()
{
    unsigned char array[20];
    double Speed_De;
    /**********
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
    } **********/
    Speed_De = mptr.wheelMoveSpeedSet;
    mptr.Flag_backward_push = false;
    while(1)
    {
        mptr.wheelMoveSpeedSet = 0;
        if(fabs(mptr.Speed_Td_x1 - mptr.wheelMoveSpeedSet)>0.01)
        {
            mptr.Speed_h=0.01;
            mptr.Speed_Adj();
            if (mptr.Td_SpeedSet>mptr.wheelMoveSpeedSetMax)
                mptr.Td_SpeedSet = mptr.wheelMoveSpeedSetMax;

             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);	//生成速度设置报文
             write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
             read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);
             write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
             read(mptr.fd3,array,sizeof(array));

        }
        else
        {
            mptr.writeWheelSpeed(00,00,mptr.commanData);	//生成速度设置报文
            write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
            read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
            mptr.writeWheelSpeed(00,00,mptr.commanData);
            write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
            read(mptr.fd3,array,sizeof(array));

            break;
        }
    }
    mptr.wheelMoveSpeedSet = -Speed_De;
}

/******************************速度设定按钮，增速，步长100rpm，最大值2400********************************************************************************/
void MainWindow::on_setSpeed_clicked()
{
    QString s;
    if (mptr.wheelMoveSpeedSet>=1)
        mptr.wheelMoveSpeedSet = 1;
    else
        mptr.wheelMoveSpeedSet += 0.05;
    mptr.wheelSpeedTarget = mptr.wheelMoveSpeedSet;
    mptr.wheelSpeedHold = mptr.wheelMoveSpeedSet;
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
    mptr.wheelSpeedTarget = mptr.wheelMoveSpeedSet;
    mptr.wheelSpeedHold = mptr.wheelMoveSpeedSet;
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
    ui->showErrorLogButton->setEnabled(false);
    ui->errorReportButton->setEnabled(false);
    ui->clearFileButton->setEnabled(false);
    ui->manulCalibrationButton->setEnabled(false);
    ui->turnSpeedSetButton->setEnabled(false);

    mptr.breakFlag = false;
    mptr.fileClearFlag = true;
    mptr.initialReady = true;
//    mptr.readyToChargeFlag = true;
//    mptr.lostQRCodeFlag = false;
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
    ui->showErrorLogButton->setEnabled(true);
    ui->errorReportButton->setEnabled(true);
    ui->clearFileButton->setEnabled(true);
    ui->manulCalibrationButton->setEnabled(true);
    ui->turnSpeedSetButton->setEnabled(true);
    mptr.wheelAddress = 0;																//部分手动参数归零，归位
    mptr.wheelMoveSpeedSet=0;
    mptr.wheelAngle = 0;
    mptr.delayTimeSet = 5;
//    mptr.wheelFrontAngleOffset = 0;
//    mptr.wheelRearAngleOffset = 0;
    mptr.wheelMoveSpeedSetMax = 1.2;
    ui->addressEdit->setText(s.setNum(mptr.wheelAddress));
    ui->speedEdit->setText(s.setNum(mptr.wheelMoveSpeedSet));
    ui->angleEdit->setText(s.setNum(mptr.wheelAngle));
    ui->timeEdit->setText(s.setNum(mptr.delayTimeSet));
    mptr.breakFlag = true;
    mptr.fileClearFlag = false;
    mptr.initialReady = false;
    mptr.lostQRCodeFlag = false;
    mptr.readyToChargeFlag = false;
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
//        if(mptr.breakFlag==false)
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
//        else
//            break;
    }


    ui->CommunicationEdit->append("Go back!");
    while((limitFlag2==true) || (limitFlag4==true))
    {
//        if(mptr.breakFlag==false)
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
//        else
//            break;
        mptr.delayTimeMsecs(100);
    }

//    if(mptr.breakFlag == false)
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
    }
    mptr.wheelAngle2 = 0;
    mptr.wheelAngle4 = 0;


}
/*********************************读取到的IO口状态显示到界面上*****************************************************************************/
void MainWindow::showIOResult()
{
    QString s;
    int count = 0;
    QTextStream out(&mptr.warningFile);
    ui->speedLabel->setText(s.setNum(mptr.AGVSpeed));
    ui->locationXlabel->setText(s.setNum(mptr.AGVLocation.X));
    ui->locationYlabel->setText(s.setNum(mptr.AGVLocation.Y));
    ui->labelOfWheelAngle->setText(s.setNum(mptr.wheelAngle));
    ui->labelOfWheelAngle4->setText(s.setNum(mptr.wheelRearAngle-mptr.wheelRearAngleOffset));
    ui->sickABox->setChecked(mptr.sickA);
    ui->sickBBox->setChecked(mptr.sickB);
    ui->sickCBox->setChecked(mptr.sickC);
    if(mptr.systemOnFlag)                                                 // waiting to check the status of normal//////////////////////////////////////
    {
        while(mptr.breakFlag == false)
        {
            if(count == 0)      //只进一次
            {
                mptr.warningFile.open(QIODevice::WriteOnly|QIODevice::Append);
                s.clear();
                s += QTime::currentTime().toString() + "触边、急停或sick保护区出发" +"  ";
                out<<s<<endl; out.flush();s.clear();
                ui->CommunicationEdit->append("触边、急停或sick保护区出发");
                count++;
                mptr.warningFile.close();
            }
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
/*******
    mptr.writeWheelSpeed(00,mptr.wheelMoveSpeedSet);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    //mptr.writeWheelSpeed(00,mptr.wheelMoveSpeedSet);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);
********/
    mptr.Flag_Right_push = true;
    while(mptr.Flag_Right_push)
    {

        if(fabs(mptr.Speed_Td_x1 - mptr.wheelMoveSpeedSet)>0.01)
        {
            mptr.Speed_h=0.01;
            mptr.Speed_Adj();
            if (mptr.Td_SpeedSet>mptr.wheelMoveSpeedSetMax)
                mptr.Td_SpeedSet = mptr.wheelMoveSpeedSetMax;

             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);	//生成速度设置报文
             write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
             read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);
             write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
             read(mptr.fd3,array,sizeof(array));

        }
        else
        {
            break;
        }
        QApplication::processEvents(QEventLoop::AllEvents,1000);
    }
}

/*************************************右面平移按钮，弹起归位*************************************************************************/
void MainWindow::on_parallelRightButton_released()
{
    int array[20]={0};
    double Speed_De;

/********
    mptr.writeWheelSpeed(00,00);                        //stop AGV
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,00);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

    //mptr.delayTimeMsecs(6000);							//延时

    mptr.writeWheelPosition(00,0+mptr.wheelFrontAngleOffset);           // angle to zero
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,0+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);
***********/

    Speed_De = mptr.wheelMoveSpeedSet;
    mptr.Flag_Right_push = false;
    while(1)
    {
        mptr.wheelMoveSpeedSet = 0;
        if(fabs(mptr.Speed_Td_x1 - mptr.wheelMoveSpeedSet)>0.01)
        {
            mptr.Speed_h=0.01;
            mptr.Speed_Adj();
            if (mptr.Td_SpeedSet>mptr.wheelMoveSpeedSetMax)
                mptr.Td_SpeedSet = mptr.wheelMoveSpeedSetMax;

             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);	//生成速度设置报文
             write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
             read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);
             write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
             read(mptr.fd3,array,sizeof(array));

        }
        else
        {
            mptr.writeWheelSpeed(00,00,mptr.commanData);	//生成速度设置报文
            write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
            read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
            mptr.writeWheelSpeed(00,00,mptr.commanData);
            write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
            read(mptr.fd3,array,sizeof(array));

            mptr.writeWheelPosition(00,0+mptr.wheelFrontAngleOffset);           // angle to zero
            write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

            mptr.writeWheelPosition(00,0+mptr.wheelRearAngleOffset);
            write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);
            break;
        }

    }
    mptr.wheelMoveSpeedSet = Speed_De;
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
/*******
    mptr.writeWheelSpeed(00,mptr.wheelMoveSpeedSet);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    //mptr.writeWheelSpeed(00,mptr.wheelMoveSpeedSet);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);
***********/
    mptr.Flag_Left_push = true;
    while(mptr.Flag_Left_push)
    {

        if(fabs(mptr.Speed_Td_x1 - mptr.wheelMoveSpeedSet)>0.01)
        {
            mptr.Speed_h=0.01;
            mptr.Speed_Adj();
            if (mptr.Td_SpeedSet>mptr.wheelMoveSpeedSetMax)
                mptr.Td_SpeedSet = mptr.wheelMoveSpeedSetMax;

             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);	//生成速度设置报文
             write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
             read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);
             write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
             read(mptr.fd3,array,sizeof(array));

        }
        else
        {
            break;
        }
        QApplication::processEvents(QEventLoop::AllEvents,1000);
    }
}

/***************************************左平移按钮，弹起归位***********************************************************************/
void MainWindow::on_parallelLeftButton_released()
{
    int array[20]={0};
    double Speed_De;
/**********
    mptr.writeWheelSpeed(00,00);                        //stop AGV
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,00);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

    //mptr.delayTimeMsecs(6000);							//延时

    mptr.writeWheelPosition(00,0+mptr.wheelFrontAngleOffset);           // angle to zero
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,0+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);
***********/

    Speed_De = mptr.wheelMoveSpeedSet;
    mptr.Flag_Left_push = false;
    while(1)
    {
        mptr.wheelMoveSpeedSet = 0;
        if(fabs(mptr.Speed_Td_x1 - mptr.wheelMoveSpeedSet)>0.01)
        {
            mptr.Speed_h=0.01;
            mptr.Speed_Adj();
            if (mptr.Td_SpeedSet>mptr.wheelMoveSpeedSetMax)
                mptr.Td_SpeedSet = mptr.wheelMoveSpeedSetMax;

             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);	//生成速度设置报文
             write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
             read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
             mptr.writeWheelSpeed(mptr.Td_SpeedSet,00,mptr.commanData);
             write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
             read(mptr.fd3,array,sizeof(array));

        }
        else
        {
            mptr.writeWheelSpeed(00,00,mptr.commanData);	//生成速度设置报文
            write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));			//发送速度设置报文到驱动器
            read(mptr.fd1,array,sizeof(array));									//读取串口缓冲区，主要目的是为了清除缓冲区为之后读取速度留下空间。
            mptr.writeWheelSpeed(00,00,mptr.commanData);
            write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));
            read(mptr.fd3,array,sizeof(array));

            mptr.writeWheelPosition(00,0+mptr.wheelFrontAngleOffset);           // angle to zero
            write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

            mptr.writeWheelPosition(00,0+mptr.wheelRearAngleOffset);
            write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);
            break;
        }

    }
    mptr.wheelMoveSpeedSet = Speed_De;
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

    mptr.writeWheelSpeed(00,0.05);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,-0.05);
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

    mptr.writeWheelSpeed(00,-0.05);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,0.05);
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


void MainWindow::on_reconnectButton_clicked()
{
    tcpSocket1 = NULL;
    tcpSocket1 = new QTcpSocket(this);
    QString ip1= "192.168.10.12"; //"10.0.0.2";
    //QString ip="192.168.10.11";
    //QString ip="10.0.0.3";
    qint16 port1=6666;
    tcpSocket1->connectToHost(ip1,port1);
    connect(tcpSocket1,SIGNAL(readyRead()),this,SLOT(readWifi()));
    connect(tcpSocket1,SIGNAL(disconnected()),this,SLOT(reconnect()));
    connect(tcpSocket1,SIGNAL(connected()),this,SLOT(TCPconnected()));
}

void MainWindow::on_manulCalibrationButton_clicked()
{
    mptr.gainAccessAndEnableWheel();
    mptr.readIO();
    wheelZeroCalibration();
}

void MainWindow::on_clearFileButton_clicked()
{
    routeFile.open(QIODevice::WriteOnly);                               //清空旧文件
    routeFile.close();
    for(int i=0;i<100;i++)
    {
        mptr.P_Target4[i].X=0;mptr.P_Target4[i].Y=0;//清空圆心存放结构体数组
        mptr.P_Target[i].X=0;mptr.P_Target[i].Y=0;//清空路径存放结构体数组
        mptr.P_Target3[i].X=0;mptr.P_Target3[i].Y=0;//清空入弯点存放结构体数组
        mptr.start_end[i].X=0;mptr.start_end[i].Y=0;//清空接收数组
    }
    mptr.getRouteFlag = false;                          //执行完成，清空标志位
    mptr.numberOfTurnCentre = 0;                        //圆心数量
    mptr.targetNumber = 0;                              //目标点数量
    mptr.numberOfTurnIn = 0;
}

void MainWindow::on_turnSpeedSetButton_clicked()
{
    QString a;
    mptr.wheelTurnSpeed += 0.1;
    if(mptr.wheelTurnSpeed > 0.4)
        mptr.wheelTurnSpeed = 0.1;
    ui->turnSpeedSetEdit->setText(a.setNum(mptr.wheelTurnSpeed));
}

void MainWindow::on_errorReportButton_clicked()
{
    mptr.warningRecord();
    ui->CommunicationEdit->append("错误读取完成");
    errorReport->show();
}

void MainWindow::on_showErrorLogButton_clicked()
{
    errorReport->show();
}



void MainWindow::on_setCameraAngleButton_clicked()
{
    mptr.ka_for = ui->ka_forDoubleSpinBox->value();
    mptr.ka_ba = ui->ka_baDoubleSpinBox->value();
}
