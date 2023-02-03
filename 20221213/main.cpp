#include "widget.h"
#include <QApplication>
#include "mainwindow.h"
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
//    Widget w;
//    w.show();
    MainWindow s;
    s.show();
    return a.exec();
}
