#ifndef EXO_MAIN_H
#define EXO_MAIN_H

#include <QWidget>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QWidget>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTime>
#include <QtWidgets/QApplication>
#include <QDate>
#include <QTextStream>
#include <QFile>
#include <QString>

QT_BEGIN_NAMESPACE
namespace Ui { class Exo_Main; }
QT_END_NAMESPACE

class Exo_Main : public QWidget
{
    Q_OBJECT

public:
    Exo_Main(QWidget *parent = nullptr);
    ~Exo_Main();


    QByteArray serialData_buff;
    char receive_Data[1024];             //接收数据
    float data_Save[255];                //储存接收到的数据
    float angle1[6];                     //角度数据
    float curent[6];                     //电流数据
    uint8_t LeftLeg_Flag;
    uint8_t RightLeg_Flag;
    uint8_t QuShen_Flag;
    uint8_t ZhanShou_Flag;
    uint8_t NeiWaiXuan_Flag;

    uint8_t Received_Position_zeroed_Flag;
    uint8_t Received_angle_Clear_Flag;
    uint8_t Received_Init_WangCheng_Flag;
    uint8_t Received_ShouDongKangFu_mode;
    uint8_t Received_AutoKangFu_mode;
    uint8_t Received_Para_Send_Flag;
    uint8_t Received_ShouDongKangFu_SpeedSet_Flag;
    uint8_t Received_Curve_Type;
    uint8_t Received_AutoKangFu_CiShu;

    uint8_t Received_Motor1_Enabled;
    uint8_t Received_Motor2_Enabled;
    uint8_t Received_Motor3_Enabled;
    uint8_t Received_Motor4_Enabled;
    uint8_t Received_Motor5_Enabled;
    uint8_t Received_Motor6_Enabled;

private slots:
    void on_Btn_Init_clicked();

    void on_Btn_SerialScan_clicked();

    void on_Btn_Open_Serial_clicked();

    void on_Btn_Close_Serial_clicked();

    void on_Btn_BackZero_clicked();

    void on_Btn_LM1_clicked();

    void on_Btn_LM2_clicked();

    void on_Btn_LM3_clicked();

    void on_Btn_RM1_clicked();

    void on_Btn_RM2_clicked();

    void on_Btn_RM3_clicked();

    void on_groupBox_ShouDongKangFu_clicked();

    void on_Btn_ShouDong_Start_clicked();

    void on_Btn_Auto_Start_clicked();

    void on_radioBtn_LeftLeg_clicked();

    void on_radioBtn_RightLeg_clicked();

    void on_radioBtn_BothLeg_clicked();

    void on_radioBtn_QuShen_clicked();

    void on_radioBtn_ZhanShou_clicked();

    void on_radioBtn_NeiWaiXuan_clicked();

    void on_radioBtn_LeftLeg_2_clicked();

    void on_radioBtn_RightLeg_2_clicked();

    void on_radioBtn_BothLeg_2_clicked();

    void on_radioBtn_QuShen_2_clicked();

    void on_radioBtn_ZhanShou_2_clicked();

    void on_radioBtn_NeiWaiXuan_2_clicked();

    void on_Btn_ShouDong_ParaSend_clicked();

    void on_combBox_SpeedChose_activated(int index);

    void data_PreProsess(uint8_t *receive_Buff, uint8_t length);

    int read_Data(char *buffer);

    void Menu_UI_Init(void);

    void on_Btn_ShouDongEnd_clicked();

    void on_AutoMode_End_clicked();

    void on_Btn_Auto_ParaSend_clicked();

    void on_spinBox_AutoKangFuCiShu_editingFinished();

    void received_state_updata();

    void on_comboBox_KangFuCurveChose_activated(int index);

    void on_radioBtn_ShouDong_clicked(bool checked);

    void on_radioBtn_ShouDong_2_clicked(bool checked);

private:
    Ui::Exo_Main *ui;
    QSerialPort *serialport; //串口对象
};
#endif // EXO_MAIN_H
