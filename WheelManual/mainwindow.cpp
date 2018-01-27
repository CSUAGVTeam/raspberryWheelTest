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
    ui->rotateButton->setEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_forwardButton_pressed()
{
    mptr.writeWheelSpeed(mptr.wheelAddress,mptr.wheelMoveSpeedSet,0,mptr.commanData);
    write(mptr.fd,mptr.commanData,sizeof(mptr.commanData));
    fflush(stdout);
	
//	QTime n=QTime::currentTime();
//    QTime now;
//    do{
//        // qDebug()<<"jjmm";
//        now=QTime::currentTime();
//    }   while(n.msecsTo(now)<=mptr.delayTimeSet);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
	
    mptr.writeWheelSpeed(mptr.wheelAddress+2,mptr.wheelMoveSpeedSet,0,mptr.commanData);
	write(mptr.fd,mptr.commanData,sizeof(mptr.commanData));fflush(stdout);
}

void MainWindow::on_forwardButton_released()
{
    mptr.writeWheelSpeed(mptr.wheelAddress,00,00,mptr.write0RPM);
    write(mptr.fd,mptr.write0RPM,sizeof(mptr.write0RPM));fflush(stdout);

//	QTime n=QTime::currentTime();
//    QTime now;
//    do{
//        // qDebug()<<"jjmm";
//        now=QTime::currentTime();
//    }   while(n.msecsTo(now)<=mptr.delayTimeSet);
    mptr.delayTimeMsecs(mptr.delayTimeSet);

    mptr.writeWheelSpeed(mptr.wheelAddress+2,00,00,mptr.write0RPM);
    write(mptr.fd,mptr.write0RPM,sizeof(mptr.write0RPM));fflush(stdout);
}

void MainWindow::on_backwardButton_pressed()
{
    //serialPuts(mptr.fd,mptr.write500RPMreverse);
    mptr.writeWheelSpeed(mptr.wheelAddress,-mptr.wheelMoveSpeedSet,0,mptr.commanDataReverse);
    write(mptr.fd,mptr.commanDataReverse,sizeof(mptr.commanDataReverse));fflush(stdout);

//	QTime n=QTime::currentTime();
//    QTime now;
//    do{
//        // qDebug()<<"jjmm";
//        now=QTime::currentTime();
//    }   while(n.msecsTo(now)<=mptr.delayTimeSet);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
	
    mptr.writeWheelSpeed(mptr.wheelAddress+2,-mptr.wheelMoveSpeedSet,0,mptr.commanDataReverse);
    write(mptr.fd,mptr.commanDataReverse,sizeof(mptr.commanDataReverse));fflush(stdout);
}


void MainWindow::on_backwardButton_released()
{
    mptr.writeWheelSpeed(mptr.wheelAddress, 0, 0, mptr.write0RPM);
    write(mptr.fd,mptr.write0RPM,sizeof(mptr.write0RPM));fflush(stdout);

//	QTime n=QTime::currentTime();
//    QTime now;
//    do{
//        // qDebug()<<"jjmm";
//        now=QTime::currentTime();
//    }   while(n.msecsTo(now)<=mptr.delayTimeSet);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
	
    mptr.writeWheelSpeed(mptr.wheelAddress+2,0,0,mptr.write0RPM);
    write(mptr.fd,mptr.write0RPM,sizeof(mptr.write0RPM));fflush(stdout);
}

void MainWindow::on_setSpeed_clicked()
{
    QString s;
    if (mptr.wheelMoveSpeedSet>=1900)
        mptr.wheelMoveSpeedSet = 1900;
    else
        mptr.wheelMoveSpeedSet += 100;
//    mptr.writeWheelSpeed(mptr.wheelAddress,mptr.wheelMoveSpeedSet,0,mptr.commanData);
//    mptr.writeWheelSpeed(mptr.wheelAddress,-(mptr.wheelMoveSpeedSet),0,mptr.commanDataReverse);
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
//    mptr.writeWheelSpeed(mptr.wheelAddress,mptr.wheelMoveSpeedSet,0,mptr.commanData);
//    mptr.writeWheelSpeed(mptr.wheelAddress,-(mptr.wheelMoveSpeedSet),0,mptr.commanDataReverse);
    ui->speedEdit->setText(s.setNum(mptr.wheelMoveSpeedSet));
}

void MainWindow::on_pushButton_clicked()
{
    mptr.writeAccessToDrive(mptr.wheelAddress);
    write(mptr.fd,mptr.accessData,sizeof(mptr.accessData));fflush(stdout);

//    QTime n=QTime::currentTime();
//    QTime now;
//    do{
//        // qDebug()<<"jjmm";
//        now=QTime::currentTime();
//    }   while(n.msecsTo(now)<=mptr.delayTimeSet);
    mptr.delayTimeMsecs(mptr.delayTimeSet);

	mptr.writeAccessToDrive(mptr.wheelAddress+2);
    write(mptr.fd,mptr.accessData,sizeof(mptr.accessData));fflush(stdout);
}


void MainWindow::on_pushButton_2_clicked()
{
    mptr.enableBridge(mptr.wheelAddress);
    write(mptr.fd,mptr.enableData,sizeof(mptr.enableData));fflush(stdout);

//    QTime n=QTime::currentTime();
//    QTime now;
//    do{
//        // qDebug()<<"jjmm";
//        now=QTime::currentTime();
//    }   while(n.msecsTo(now)<=mptr.delayTimeSet);
    mptr.delayTimeMsecs(mptr.delayTimeSet);

	mptr.enableBridge(mptr.wheelAddress+2);
    write(mptr.fd,mptr.enableData,sizeof(mptr.enableData));fflush(stdout);
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
    //if(mptr.wheelAddress>4)
    if(mptr.wheelAddress>2)
        mptr.wheelAddress = 1;
    ui->addressEdit->setText(s.setNum(mptr.wheelAddress));
    if(mptr.wheelAddress==1||mptr.wheelAddress==3)
        mptr.writeWheelSpeed(mptr.wheelAddress,00,00,mptr.write0RPM);
}

void MainWindow::on_disableBridge_clicked()
{
    mptr.disableBridge(mptr.wheelAddress);
    write(mptr.fd,mptr.disableData,sizeof(mptr.disableData));

//	QTime n=QTime::currentTime();
//    QTime now;
//    do{
//        // qDebug()<<"jjmm";
//        now=QTime::currentTime();
//    }   while(n.msecsTo(now)<=mptr.delayTimeSet);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
	
    mptr.disableBridge(mptr.wheelAddress+2);
    write(mptr.fd, mptr.disableData, sizeof(mptr.disableData));
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
    mptr.writeWheelPosition(mptr.wheelAddress,00,mptr.wheelAngle);
    write(mptr.fd,mptr.writePositionData,sizeof(mptr.writePositionData));fflush(stdout);

//	QTime n=QTime::currentTime();
//    QTime now;
//    do{
//        // qDebug()<<"jjmm";
//        now=QTime::currentTime();
//    }   while(n.msecsTo(now)<=mptr.delayTimeSet);
    mptr.delayTimeMsecs(mptr.delayTimeSet);
	
    mptr.writeWheelPosition(mptr.wheelAddress+2,00,mptr.wheelAngle);
    write(mptr.fd,mptr.writePositionData,sizeof(mptr.writePositionData));fflush(stdout);
}
