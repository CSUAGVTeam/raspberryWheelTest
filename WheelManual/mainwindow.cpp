#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <wiringPi.h>
#include "wiringSerial.h"
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

void MainWindow::on_forwardButton_pressed()
{
    mptr.writeWheelSpeed(mptr.wheelAddress,mptr.wheelMoveSpeedSet,0,mptr.commanData);
    write(mptr.fd,mptr.commanData,sizeof(mptr.commanData));fflush(stdout);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());
	
    mptr.writeWheelSpeed(mptr.wheelAddress+2,mptr.wheelMoveSpeedSet,0,mptr.commanData);
	write(mptr.fd,mptr.commanData,sizeof(mptr.commanData));fflush(stdout);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());
}

void MainWindow::on_forwardButton_released()
{
    mptr.writeWheelSpeed(mptr.wheelAddress,00,00,mptr.write0RPM);
    write(mptr.fd,mptr.write0RPM,sizeof(mptr.write0RPM));fflush(stdout);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());

    mptr.writeWheelSpeed(mptr.wheelAddress+2,00,00,mptr.write0RPM);
    write(mptr.fd,mptr.write0RPM,sizeof(mptr.write0RPM));fflush(stdout);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());
}

void MainWindow::on_backwardButton_pressed()
{
    //serialPuts(mptr.fd,mptr.write500RPMreverse);
    mptr.writeWheelSpeed(mptr.wheelAddress,-mptr.wheelMoveSpeedSet,0,mptr.commanDataReverse);
    write(mptr.fd,mptr.commanDataReverse,sizeof(mptr.commanDataReverse));fflush(stdout);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());

    mptr.writeWheelSpeed(mptr.wheelAddress+2,-mptr.wheelMoveSpeedSet,0,mptr.commanDataReverse);
    write(mptr.fd,mptr.commanDataReverse,sizeof(mptr.commanDataReverse));fflush(stdout);
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());
    mptr.delayTimeMsecs(mptr.delayTimeSet);
}

void MainWindow::on_backwardButton_released()
{
    mptr.writeWheelSpeed(mptr.wheelAddress, 0, 0, mptr.write0RPM);
    write(mptr.fd,mptr.write0RPM,sizeof(mptr.write0RPM));fflush(stdout);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());

    mptr.writeWheelSpeed(mptr.wheelAddress+2,0,0,mptr.write0RPM);
    write(mptr.fd,mptr.write0RPM,sizeof(mptr.write0RPM));fflush(stdout);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());
}

void MainWindow::on_setSpeed_clicked()
{
    QString s;
    if (mptr.wheelMoveSpeedSet>=1900)
        mptr.wheelMoveSpeedSet = 1900;
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
    mptr.writeAccessToDrive(mptr.wheelAddress);
    write(mptr.fd,mptr.accessData,sizeof(mptr.accessData));fflush(stdout);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());

	mptr.writeAccessToDrive(mptr.wheelAddress+2);
    write(mptr.fd,mptr.accessData,sizeof(mptr.accessData));fflush(stdout);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());

}

void MainWindow::on_pushButton_2_clicked()
{
    mptr.enableBridge(mptr.wheelAddress);
    write(mptr.fd,mptr.enableData,sizeof(mptr.enableData));fflush(stdout);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());

    mptr.enableBridge(mptr.wheelAddress+2);
    write(mptr.fd,mptr.enableData,sizeof(mptr.enableData));fflush(stdout);
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());
    mptr.delayTimeMsecs(mptr.delayTimeSet);
}

void MainWindow::on_addAngleButton_clicked()
{
    QString s;
    if(mptr.wheelAngle>=90)
        mptr.wheelAngle = 90;
    else
        mptr.wheelAngle += 5;
    mptr.writeWheelPosition(mptr.wheelAddress,00,mptr.wheelAngle);
    ui->angleEdit->setText(s.setNum(mptr.wheelAngle));
}

void MainWindow::on_decAngleButton_clicked()
{
    QString s;
    if(mptr.wheelAngle<=-90)
        mptr.wheelAngle = -90;
    else
        mptr.wheelAngle -=5;
    mptr.writeWheelPosition(mptr.wheelAddress,00,mptr.wheelAngle);
    ui->angleEdit->setText(s.setNum(mptr.wheelAngle));
}

void MainWindow::on_rotateButton_clicked()
{
    write(mptr.fd,mptr.writePositionData,sizeof(mptr.writePositionData));fflush(stdout);
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
        mptr.writeWheelSpeed(mptr.wheelAddress,00,00,mptr.write0RPM);
}

void MainWindow::on_disableBridge_clicked()
{
    mptr.disableBridge(mptr.wheelAddress);
    write(mptr.fd,mptr.disableData,sizeof(mptr.disableData));
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());

    mptr.disableBridge(mptr.wheelAddress+2);
    write(mptr.fd, mptr.disableData, sizeof(mptr.disableData));
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());
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
        mptr.writeWheelPosition(mptr.wheelAddress, 00, mptr.wheelAngle + mptr.wheelFrontAngleOffset);
        write(mptr.fd,mptr.writePositionData,sizeof(mptr.writePositionData));fflush(stdout);
        mptr.delayTimeMsecs(mptr.delayTimeSet);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());

        mptr.writeWheelPosition(mptr.wheelAddress+2, 00, mptr.wheelAngle + mptr.wheelRearAngleOffset);
        write(mptr.fd,mptr.writePositionData,sizeof(mptr.writePositionData));fflush(stdout);
        mptr.delayTimeMsecs(mptr.delayTimeSet);
        ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());
    }
}

void MainWindow::on_resetButton_clicked()
{
    unsigned char add1[12] = {0xa5,0x01,0x02,0x01,0x00,0x01,0xc2,0x01,0x00,0x10,0x12,0x31};
    unsigned char add2[12] = {0xa5,0x02,0x02,0x01,0x00,0x01,0x2c,0xd3,0x00,0x10,0x12,0x31};
    unsigned char add3[12] = {0xa5,0x03,0x02,0x01,0x00,0x01,0x86,0x82,0x00,0x10,0x12,0x31};
    unsigned char add4[12] = {0xa5,0x04,0x02,0x01,0x00,0x01,0xe1,0x56,0x00,0x10,0x12,0x31};

    write(mptr.fd,add1,sizeof(add1));fflush(stdout);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    write(mptr.fd,add2,sizeof(add2));fflush(stdout);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    write(mptr.fd,add3,sizeof(add3));fflush(stdout);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
    write(mptr.fd,add4,sizeof(add4));fflush(stdout);

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
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());

    write(mptr.fd,testEnable,sizeof(testEnable));
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());

    write(mptr.fd,test500RPM,sizeof(test500RPM));
    ui->CommunicationEdit->append(QString2Hex(mptr.checkWheelCommunication()).toHex());

}
