#include "errorreportwindow.h"
#include "ui_errorreportwindow.h"
#include <QIODevice>
#include <QFile>
#include <QString>
#include <QTextStream>

errorReportWindow::errorReportWindow(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::errorReportWindow)
{
    ui->setupUi(this);
    errorFile.setFileName("/home/pi/lys/errorReport.txt");

}

errorReportWindow::~errorReportWindow()
{
    ui->errorReportEdit->clear();
    delete ui;
}

void errorReportWindow::on_readErrorButton_clicked()
{
    errorFile.open(QIODevice::ReadOnly);
    ui->errorReportEdit->clear();
    QString str;
    QTextStream in(&errorFile);
    str = in.readAll();
    ui->errorReportEdit->append(str);

    errorFile.close();
}

void errorReportWindow::on_clearFileButton_clicked()
{
    errorFile.open(QIODevice::WriteOnly);
    errorFile.close();
    ui->errorReportEdit->append("清除完成");
}
