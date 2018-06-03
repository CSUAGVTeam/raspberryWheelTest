#include "mainwindow.h"
//#include "datashare.h"
#include <QApplication>
#include <QTextCodec>

int main(int argc, char *argv[])
{

    QApplication a(argc, argv);
    MainWindow w;
    //Datashare datashare;
//    Datashare datashare;
    w.show();

    return a.exec();
}
