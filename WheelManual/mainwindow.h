#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "datashare.h"

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

private slots:
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

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
