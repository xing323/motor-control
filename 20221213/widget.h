#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTime>
#include "qcustomplot.h"
#include <QtWidgets/QApplication>
#include <QDate>
#include <QTextStream>
#include <QFile>
#include <QString>
#include <QtCharts>

using namespace QtCharts;

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE


class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();
    char receive_Data[1024];             //接收数据
    float data_Save[255];                //储存接收到的数据
    float angle1[6];                     //角度数据

signals:
    void data_Transmision(float data);//把数据同步到mainUI

private slots:
    void on_Btn_Open_Serial_clicked();             //开启串口
    void on_Btn_Close_Serial_clicked();            //关闭串口
    void on_Btn_End_clicked();                     //点击结束按钮
    void on_Btn_LM1_clicked();                     //电机1
    void on_Btn_LM2_clicked();                     //电机2
    void on_Btn_LM3_clicked();                     //电机3
    void on_Btn_RM1_clicked();                     //电机4
    void on_Btn_RM2_clicked();                     //电机5
    void on_Btn_RM3_clicked();                     //电机6
    void on_Btn_Angle_Clear_clicked();             //初始化，角度清零
    void on_Btn_Check_Serial_clicked();            //检测串口
    int read_Data(char *buffer);                   //读取串口数据
    void on_Btn_Switch_UI_clicked();
    void data_PreProsess(uint8_t *receive_Buff, uint8_t length);

    void custom_plot_Init();//绘图初始化

    void custom_Real_Plot(void);//绘图函数


    void on_Btn_BackZero_clicked();

    void on_Btn_to_Curve_Setting_clicked();

    //写文件
    void SaveDataToCSVFile(float* saved_Data);

private:
    Ui::Widget *ui;
    QSerialPort *serialport; //串口对象
    QCustomPlot *customPlot; //绘图

};
#endif // WIDGET_H
