#ifndef ERRORREPORTWINDOW_H
#define ERRORREPORTWINDOW_H

#include <QDialog>
#include <QFile>

namespace Ui {
class errorReportWindow;
}

class errorReportWindow : public QDialog
{
    Q_OBJECT

public:
    explicit errorReportWindow(QWidget *parent = 0);
    ~errorReportWindow();
    QFile errorFile;

private slots:
    void on_readErrorButton_clicked();

    void on_clearFileButton_clicked();

private:
    Ui::errorReportWindow *ui;
};

#endif // ERRORREPORTWINDOW_H
