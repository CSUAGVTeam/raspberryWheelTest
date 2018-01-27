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

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QString c1;
    ui->speedEdit->setText(c1.setNum(mptr.wheelMoveSpeedSet));
    ui->addressEdit->setText(c1.setNum(mptr.wheelAddress));
    ui->angleEdit->setText(c1.setNum(mptr.wheelAngle));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_forwardButton_pressed()
{
    //serialPuts(mptr.fd,mptr.commanData);
    write(mptr.fd,mptr.commanData,sizeof(mptr.commanData));
    fflush(stdout);
}

void MainWindow::on_forwardButton_released()
{
    //serialPuts(mptr.fd,mptr.commanData);
    write(mptr.fd,mptr.write0RPM,sizeof(mptr.write0RPM));
    fflush(stdout);
}

void MainWindow::on_backwardButton_pressed()
{
    //serialPuts(mptr.fd,mptr.write500RPMreverse);
    write(mptr.fd,mptr.commanDataReverse,sizeof(mptr.commanDataReverse));
    fflush(stdout);
}


void MainWindow::on_backwardButton_released()
{
    //serialPuts(mptr.fd,mptr.write0RPM);
    write(mptr.fd,mptr.write0RPM,sizeof(mptr.write0RPM));
    fflush(stdout);
}

void MainWindow::on_setSpeed_clicked()
{
    QString s;
//    std::string str;
//    QByteArray qbytearray;
    if (mptr.wheelMoveSpeedSet>=1900)
        mptr.wheelMoveSpeedSet = 1900;
    else
        mptr.wheelMoveSpeedSet += 100;
    //mptr.writeWheelSpeed(mptr.wheelAddress,00,mptr.wheelMoveSpeedSet);
    mptr.writeWheelSpeed(mptr.wheelAddress,mptr.wheelMoveSpeedSet,0,mptr.commanData);
    mptr.writeWheelSpeed(mptr.wheelAddress,-(mptr.wheelMoveSpeedSet),0,mptr.commanDataReverse);
    ui->speedEdit->setText(s.setNum(mptr.wheelMoveSpeedSet));
    //ui->speedEdit->setText(s);
//    str = (char *)(mptr.commanData);
//    s = QString::fromRawData();
//    qbytearray = QByteArray((char *)mptr.commanData,14);
    //s = s.append(mptr.commanData);
    //qbytearray.
//    s="";
//    ui->label->setText(s.fromLocal8Bit(reinterpret_cast<char*>(mptr.commanData)));
}

void MainWindow::on_setButton2_clicked()
{
    QString s;
    std::string str;
    if (mptr.wheelMoveSpeedSet<=0)
        mptr.wheelMoveSpeedSet = 0;
    else
        mptr.wheelMoveSpeedSet -= 100;
    mptr.writeWheelSpeed(mptr.wheelAddress,mptr.wheelMoveSpeedSet,0,mptr.commanData);
    mptr.writeWheelSpeed(mptr.wheelAddress,-(mptr.wheelMoveSpeedSet),0,mptr.commanDataReverse);
    ui->label->setText(s.setNum(mptr.wheelMoveSpeedSet));
    ui->speedEdit->setText(s);
//    str = (char *)mptr.commanData;
//    s = QString::fromStdString(str);
    s="";
    ui->label->setText(s.fromLatin1(static_cast<char*>(mptr.commanData)));
    ui->label->setText(s);
}

void MainWindow::on_pushButton_clicked()
{
    mptr.writeAccessToDrive(mptr.wheelAddress);
    write(mptr.fd,mptr.accessData,sizeof(mptr.accessData));fflush(stdout);
    //write(mptr.fd,mptr.gainAccess,sizeof(mptr.gainAccess));
    //fflush(stdout);
    //write(mptr.fd,mptr.gainAccess1,sizeof(mptr.gainAccess1));fflush(stdout);
    //write(mptr.fd,mptr.gainAccess3,sizeof(mptr.gainAccess3));fflush(stdout);
//    read(mptr.fd,a1,8);
//    QString string(a1);
//    ui->lineEdit->setText(string);
}


void MainWindow::on_pushButton_2_clicked()
{
    mptr.enableBridge(mptr.wheelAddress);
    write(mptr.fd,mptr.enableData,sizeof(mptr.enableData));fflush(stdout);
    //write(mptr.fd,mptr.enableBridge,sizeof(mptr.enableBridge));
    //fflush(stdout);
    //write(mptr.fd,mptr.enableBridge1,sizeof(mptr.enableBridge1));fflush(stdout);
   // write(mptr.fd,mptr.enableBridge3,sizeof(mptr.enableBridge3));fflush(stdout);
    //read(mptr.fd,a1,8);
//    QString string(a1);
//    ui->lineEdit->setText(string);

}

void MainWindow::on_addAngleButton_clicked()
{
    QString s;
    if(mptr.wheelAngle>=60)
        mptr.wheelAngle = 60;
    else
        mptr.wheelAngle += 10;
    mptr.writeWheelPosition(mptr.wheelAddress,00,mptr.wheelAngle);
    ui->angleEdit->setText(s.setNum(mptr.wheelAngle));

}

void MainWindow::on_decAngleButton_clicked()
{
    QString s;
    if(mptr.wheelAngle<=-60)
        mptr.wheelAngle = -60;
    else
        mptr.wheelAngle -=10;
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
        mptr.wheelAddress = 1;
    ui->addressEdit->setText(s.setNum(mptr.wheelAddress));
}
