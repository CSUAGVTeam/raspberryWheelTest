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
        t1 = QTime::currentTime().addSecs(10);
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
            mptr.wheelMoveSpeedSet = 0;
            if(mptr.wheelMoveSpeedSet > mptr.wheelMoveSpeedSetMax)
                mptr.wheelMoveSpeedSet = mptr.wheelMoveSpeedSetMax;

			if(mptr.breakFlag == false)
			{
                mptr.readIO();                          //read I/O
				ui->CommunicationEdit->append(tr("%1 %2 %3").arg(mptr.systemOnFlag).arg(mptr.sickFalse).arg(mptr.sickWarningSpaceAlert));
                //mptr.checkIO();                         //check whether there are some alarm signals, and act correspondingly. And pull up enmergencyFlag to 1.
                showIOResult();                         // show the result and jump into a loop waitting for action.

				if(mptr.emergencyFlag == false)
				{
					if(speedBefore != mptr.wheelMoveSpeedSet)
					{
						speedBefore = mptr.wheelMoveSpeedSet;
                        if (mptr.wheelMoveSpeedSet>mptr.wheelMoveSpeedSetMax)
                            mptr.wheelMoveSpeedSet = mptr.wheelMoveSpeedSetMax;
                        mptr.writeWheelSpeed(mptr.wheelMoveSpeedSet,00,mptr.commanData);
                        write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));//fflush(stdout);
                        read(mptr.fd1,array,sizeof(array));
						

                        mptr.writeWheelSpeed(mptr.wheelMoveSpeedSet,00,mptr.commanData);
                        write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));//fflush(stdout);
                        read(mptr.fd3,array,sizeof(array));
					}
                //wifi remote connection, check the command from upper machine.
				}
				else
				{
					//Alarm and show the mistake
                    mptr.writeWheelSpeed(00,00,mptr.commanData);
                    write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));//fflush(stdout);
                    read(mptr.fd1,array,sizeof(array));
					mptr.delayTimeMsecs(mptr.delayTimeSet);
                    mptr.writeWheelSpeed(00,00,mptr.commanData);
                    write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));//fflush(stdout);
                    read(mptr.fd3,array,sizeof(array));
				}
			}
			else
				break;
            mptr.writeIO();                             // write I/O
			QApplication::processEvents(QEventLoop::AllEvents,1000);
		}
		loopFlag = true;
        if (mptr.breakFlag == true)
        {
            mptr.writeWheelSpeed(00,00,mptr.commanData);
            write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));//fflush(stdout);
            read(mptr.fd1,array,sizeof(array));
            mptr.delayTimeMsecs(mptr.delayTimeSet);
            mptr.writeWheelSpeed(00,00,mptr.commanData);
            write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));//fflush(stdout);
            read(mptr.fd3,array,sizeof(array));
            break;
        }
    }
	mptr.writeWheelSpeed(00,00,mptr.commanData);
	write(mptr.fd1,mptr.commanData,sizeof(mptr.commanData));//fflush(stdout);
	read(mptr.fd1,array,sizeof(array));
	mptr.delayTimeMsecs(mptr.delayTimeSet);
	mptr.writeWheelSpeed(00,00,mptr.commanData);
	write(mptr.fd3,mptr.commanData,sizeof(mptr.commanData));//fflush(stdout);
	read(mptr.fd3,array,sizeof(array));
	ui->CommunicationEdit->append("Halt Success!");
	mptr.sickA = 0;
    mptr.sickB = 0;
    mptr.sickC = 0;
	mptr.writeIO();
}

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

void MainWindow::on_setSpeed_clicked()
{
    QString s;
    if (mptr.wheelMoveSpeedSet>=2400)
        mptr.wheelMoveSpeedSet = 2400;
    else
        mptr.wheelMoveSpeedSet += 100;
    ui->speedEdit->setText(s.setNum(mptr.wheelMoveSpeedSet));
}

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

void MainWindow::on_addAngleButton_clicked()
{
    QString s;
    if(mptr.wheelAngle>=90)
        mptr.wheelAngle = 90;
    else
        mptr.wheelAngle += 5;
    ui->angleEdit->setText(s.setNum(mptr.wheelAngle));
}

void MainWindow::on_decAngleButton_clicked()
{
    QString s;
    if(mptr.wheelAngle<=-90)
        mptr.wheelAngle = -90;
    else
        mptr.wheelAngle -=5;
    ui->angleEdit->setText(s.setNum(mptr.wheelAngle));
}

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

void MainWindow::on_changeAddButton_clicked()
{
    QString s;
    mptr.wheelAddress ++;
    if(mptr.wheelAddress>4)
    //if(mptr.wheelAddress>2)
        mptr.wheelAddress = 1;
    ui->addressEdit->setText(s.setNum(mptr.wheelAddress));
    if(mptr.wheelAddress==1||mptr.wheelAddress==3)
        mptr.writeWheelSpeed(00,00,mptr.write0RPM);
}

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

void MainWindow::on_addTimeButton_clicked()
{
    QString s;
    mptr.delayTimeSet++;
    ui->timeEdit->setText(s.setNum(mptr.delayTimeSet));
}

void MainWindow::on_decTimeButton_clicked()
{
    QString s;
    mptr.delayTimeSet--;
    if(mptr.delayTimeSet<0)
        mptr.delayTimeSet = 0;
    ui->timeEdit->setText(s.setNum(mptr.delayTimeSet));
}

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

void MainWindow::on_cleanCommunicationButton_clicked()
{
    ui->CommunicationEdit->clear();
}

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

void MainWindow::on_autoRunButton_clicked()
{
    ui->addTimeButton->setEnabled(false);                               //we can change a window next time to avoid write so much code to hide the widgets
    ui->addAngleButton->setEnabled(false);
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

void MainWindow::on_stopAutorunButton_clicked()
{
    QString s;
    ui->addTimeButton->setEnabled(true);
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
    mptr.wheelAddress = 0;
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
            mptr.readIO();                                   //read I/O
				ui->CommunicationEdit->append(tr("%1 %2 %3").arg(mptr.systemOnFlag).arg(mptr.sickFalse).arg(mptr.sickWarningSpaceAlert));

            do{                                              //filter
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
                write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,sizeof(array));//fflush(stdout);
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
        mptr.wheelFrontAngleOffset = mptr.wheelAngle2 - 102;                        //   right(clockwise):107, (anti-clockwise)left:97
        mptr.wheelRearAngleOffset = mptr.wheelAngle4 - 102;

        mptr.writeWheelPosition(00,mptr.wheelFrontAngleOffset);
        write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,sizeof(array));//fflush(stdout);
        ui->CommunicationEdit->append(tr("The wheelFrontAngleOffset is: %1").arg(mptr.wheelFrontAngleOffset));

        mptr.writeWheelPosition(00,mptr.wheelRearAngleOffset);
        write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,sizeof(array));//fflush(stdout);
        ui->CommunicationEdit->append(tr("The wheelRearAngleOffset is: %1").arg(mptr.wheelRearAngleOffset));
        mptr.delayTimeMsecs(6000);
		mptr.calibrationFlag = true;		
    }
	

}

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

void MainWindow::on_refreshIOButton_clicked()
{
    int i2c_write[2];
    i2c_write[0]=0x00;//低位
    i2c_write[1]=0x00;//高位
    wiringPiI2CWriteReg16(mptr.i2c_fd3,i2c_write[0],i2c_write[1]);//向设备3的reg中写入两个字节
}

void MainWindow::on_parallelRightButton_pressed()
{
    int array[20]= {0};
    mptr.writeWheelPosition(00,90+mptr.wheelFrontAngleOffset);
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,90+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

    mptr.delayTimeMsecs(6000);

    mptr.writeWheelSpeed(00,100);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,100);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);
}

void MainWindow::on_parallelRightButton_released()
{
    int array[20]={0};
    mptr.writeWheelSpeed(00,00);                        //stop AGV
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,00);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

    mptr.delayTimeMsecs(6000);

    mptr.writeWheelPosition(00,0+mptr.wheelFrontAngleOffset);           // angle to zero
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,0+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

}

void MainWindow::on_parallelLeftButton_pressed()
{
    int array[20]= {0};
    mptr.writeWheelPosition(00,-90+mptr.wheelFrontAngleOffset);
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,-90+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

    mptr.delayTimeMsecs(6000);

    mptr.writeWheelSpeed(00,100);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,100);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

}

void MainWindow::on_parallelLeftButton_released()
{
    int array[20]={0};
    mptr.writeWheelSpeed(00,00);                        //stop AGV
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,00);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

    mptr.delayTimeMsecs(6000);

    mptr.writeWheelPosition(00,0+mptr.wheelFrontAngleOffset);           // angle to zero
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,0+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

}

void MainWindow::on_rotateLeftButton_pressed()
{
    int array[20]= {0};
    mptr.writeWheelPosition(00,90+mptr.wheelFrontAngleOffset);
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,90+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

    mptr.delayTimeMsecs(6000);

    mptr.writeWheelSpeed(00,100);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,-100);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

}


void MainWindow::on_rotateLeftButton_released()
{
    int array[20]={0};
    mptr.writeWheelSpeed(00,00);                        //stop AGV
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,00);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

    mptr.delayTimeMsecs(6000);

    mptr.writeWheelPosition(00,0+mptr.wheelFrontAngleOffset);           // angle to zero
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,0+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

}

void MainWindow::on_rotateRightButton_pressed()
{
    int array[20]= {0};
    mptr.writeWheelPosition(00,90+mptr.wheelFrontAngleOffset);
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,90+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);

    mptr.delayTimeMsecs(6000);

    mptr.writeWheelSpeed(00,-100);
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,100);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

}


void MainWindow::on_rotateRightButton_released()
{
    int array[20]={0};
    mptr.writeWheelSpeed(00,00);                        //stop AGV
    write(mptr.fd1,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd1,array,20);

    mptr.writeWheelSpeed(00,00);
    write(mptr.fd3,mptr.writeSpeedData,sizeof(mptr.writeSpeedData));read(mptr.fd3,array,20);

    mptr.delayTimeMsecs(6000);

    mptr.writeWheelPosition(00,0+mptr.wheelFrontAngleOffset);           // angle to zero
    write(mptr.fd2,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd2,array,20);

    mptr.writeWheelPosition(00,0+mptr.wheelRearAngleOffset);
    write(mptr.fd4,mptr.writePositionData,sizeof(mptr.writePositionData));read(mptr.fd4,array,20);


}
