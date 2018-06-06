#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "datashare.h"
#include "errorreportwindow.h"
#include <QFile>
#define pi 3.14159

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    Datashare mptr;
    QFile routeFile;
    QFile yawFile;
    bool dontShutDownFlag = true;
    errorReportWindow *errorReport;

public:
                                                                                        //read I/O and save the data

    void initialTheSystem(void);                                                        //Initial the system

    void systemOn(void);                                                                //The whole loop of the software

    void wheelZeroCalibration(void);                                                    //calibrate the steering wheel to zero

private slots:

    void ReadData ();

    void reconnect();

    void TCPconnected();

    void readWifi();

    void writeWifi();

    void writeErrorInformation();

    void on_forwardButton_pressed();

    void on_forwardButton_released();

    void on_backwardButton_pressed();

    void on_backwardButton_released();

    void on_setSpeed_clicked();

    void on_setButton2_clicked();

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_addAngleButton_clicked();

    void on_decAngleButton_clicked();

    void on_rotateButton_clicked();

    void on_changeAddButton_clicked();

    void on_disableBridge_clicked();

    void on_addTimeButton_clicked();

    void on_decTimeButton_clicked();

    void on_rotateTogetherButton_clicked();

    void on_resetButton_clicked();

    QByteArray QString2Hex(QString str);

    int ConvertHexChar(char ch);

    void on_cleanCommunicationButton_clicked();

    void on_setFrontOffsetButton_clicked();

    void on_setRearOffsetButton_clicked();

    void on_testButton_clicked();

    void on_autoRunButton_clicked();

    void on_stopAutorunButton_clicked();

    void showIOResult(void);

    void on_refreshIOButton_clicked();

    void on_parallelRightButton_pressed();

    void on_parallelRightButton_released();

    void on_parallelLeftButton_pressed();

    void on_parallelLeftButton_released();

    void on_rotateLeftButton_pressed();

    void on_rotateLeftButton_released();

    void on_rotateRightButton_pressed();

    void on_rotateRightButton_released();

    int Position_PID (int Encoder,int Target);

    int Incremental_PI (int Encoder,int Target);

    void on_setPIDButton_clicked();


    void on_reconnectButton_clicked();

    void on_manulCalibrationButton_clicked();

    void on_clearFileButton_clicked();

    void on_turnSpeedSetButton_clicked();

    void on_errorReportButton_clicked();

    void on_showErrorLogButton_clicked();


    void on_setCameraAngleButton_clicked();


private:
    Ui::MainWindow *ui;

    QTcpSocket *tcpSocket;
    QTcpSocket *tcpSocket1;
};

#endif // MAINWINDOW_H
