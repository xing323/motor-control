#include "exo_main.h"
#include "ui_exo_main.h"
#include <QMessageBox>
#include <QtEndian>//大小端转换

Exo_Main::Exo_Main(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Exo_Main)
{
    ui->setupUi(this);
    //创建串口对象
    serialport = new QSerialPort;
    Menu_UI_Init();
    //ui->Title_all->setStyleSheet("color: blue; background-color: yellow");
}

Exo_Main::~Exo_Main()
{
    delete ui;
}

//画面初始化
void Exo_Main::Menu_UI_Init(void)
{   //部件初始化使能
    ui->Btn_Close_Serial ->setEnabled(false);
    ui->groupBox_MotorState->setEnabled(0);
    ui->groupBox_ShouDongKangFu->setEnabled(0);
    ui->groupBox_AutoKangFu->setEnabled(0);
    ui->Btn_SerialScan->setEnabled(1);
    ui->Btn_Open_Serial->setEnabled(1);

    ui->lcdNum_M1->display(0);
    ui->lcdNum_M2->display(0);
    ui->lcdNum_M3->display(0);
    ui->lcdNum_M4->display(0);
    ui->lcdNum_M5->display(0);
    ui->lcdNum_M6->display(0);

    ui->Label_ShouDongKangFu_Speed->hide();
    ui->Label_KangFuYunDong       ->hide();
    ui->Label_KangFuCiShu         ->hide();
    ui->Label_CurveType           ->hide();
}

void Exo_Main::on_Btn_SerialScan_clicked()
{
    //清空当前列表
    ui->comBox_Serial_num->clear();
    //查找可用的串口
    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts())
    {
        QSerialPort serial;
        serial.setPort(info);   //设置串口
        if(serial.open(QIODevice::ReadWrite))
        {
            //显示串口名字
            ui->comBox_Serial_num->addItem(serial.portName());
            serial.close();
        }
        //查找可用的串口
    }
}

void Exo_Main::on_Btn_Open_Serial_clicked()
{
    serialport->setPortName(ui->comBox_Serial_num->currentText());
    if(serialport->open(QIODevice::ReadWrite))              //打开串口成功
    {
        serialport->setBaudRate(256000);       //设置波特率
        serialport->setDataBits(QSerialPort::Data8);
        serialport->setParity(QSerialPort::NoParity);//设置奇偶校验
        serialport->setStopBits(QSerialPort::OneStop);
        serialport->setFlowControl(QSerialPort::NoFlowControl);     //设置流控制

        ui->groupBox_MotorState->setEnabled(1);
//        ui->groupBox_ShouDongKangFu->setEnabled(1);
//        ui->groupBox_AutoKangFu->setEnabled(1);
        //设置按钮
        ui->Btn_Open_Serial ->setEnabled(false);
        ui->Btn_Close_Serial ->setEnabled(true);
        //设置下拉菜单不可用
        ui->comBox_Serial_num->setEnabled(false);
        ui->Btn_SerialScan ->setEnabled(false);

        //串口数据接收
        connect(serialport,&QSerialPort::readyRead,[=](){
                //    qDebug()<<"readyReadOK";
            qDebug()<<read_Data(receive_Data);
            //数据协议处理
            data_PreProsess((uint8_t* )receive_Data, 29);
            //SaveDataToCSVFile(angle1);
            //custom_Real_Plot();
        });
    }
    else    //打开失败提示
    {
        QMessageBox::information(this,tr("Error"),tr("Open the failure"),QMessageBox::Ok);
    }
}



void Exo_Main::on_Btn_Close_Serial_clicked()
{
    serialport->clear();        //清空缓存区
    serialport->close();        //关闭串口

    ui->Btn_Open_Serial ->setEnabled(true);
    ui->Btn_Close_Serial ->setEnabled(false);
    //设置下拉菜单可用
    ui->comBox_Serial_num->setEnabled(true);
    ui->Btn_SerialScan->setEnabled(true);
}

//接收固定长度数据
int Exo_Main::read_Data(char *buffer)
{
    QByteArray buff;
    if(serialport->isReadable())
    {
        int n           = serialport->bytesAvailable();
        serialData_buff.append(serialport->readAll());
        if(serialData_buff.size() >= 29)
        {
            qDebug()<<"拷贝地址为："<<memcpy(buffer, serialData_buff, 29);
            serialData_buff.remove(0,29);
            qDebug()<<"数组长度为："<<serialData_buff.size();
        }

        //serialData_buff = serialData_buff.trimmed(); //返回一个字节数组，该数组从开始和结束处移除空白。
//        qDebug()<<"lengh_array:"<<lengh_array<<"lengh_buffer:"<<lengh_buffer;
//        qDebug()<<buffer[0]<<buffer[1]<<len<<"bytesAvailable"<<n;
        return n;
    }else
    {
    return -1;
    }

//    QByteArray receive_Buff,buff;
//    if(serialport->isReadable())
//    {
//        int n = serialport->bytesAvailable();
//        receive_Buff = serialport->readAll();
//        buff = receive_Buff.mid(0,29);
//        memcpy(buffer, receive_Buff, 29);
////        qDebug()<<"lengh_array:"<<lengh_array<<"lengh_buffer:"<<lengh_buffer;
////        qDebug()<<buffer[0]<<buffer[1]<<len<<"bytesAvailable"<<n;
//        return n;
//    }else
//    {
//    return -1;
//    }
}

//接收报文的预处理
void Exo_Main::data_PreProsess(uint8_t *receive_Buff, uint8_t length)
{
    uint8_t sum = 0;
    //校验帧头0xAA 0xAA
    if((receive_Buff[0] != 0xAA)||(receive_Buff[1] != 0xAA))
    {
        qDebug()<<"校验帧头0xAA 0xAA"<<":error!";
        return;
    }
    //校验和
    for(int i=0;i<length-1;i++)
    {
        sum += receive_Buff[i];
    }
    if(sum != receive_Buff[length-1])
    {
        qDebug()<<"校验和"<<":error!";
        return;
    }
    uint8_t data_trans[4];// = {0x6C,0xA0,0x91,0x42};
    uint8_t cnt;
    switch(receive_Buff[2])
    {
    case 0xF1://角度数据
        for(int j=0;j<6;j++) //存储角度数据并转换为float
        {
            for(int i=0;i<4;i++)
            {
                data_trans[4-i] = receive_Buff[i+3+4*j];
            }
            angle1[j] = *((float *)data_trans);
            //qDebug()<<"angle"<<j<<":"<<angle1[j];
        }
        ui->lcdNum_M1->display((int16_t)angle1[0]/20  );
        ui->lcdNum_M2->display((int16_t)angle1[1]/20  );
        ui->lcdNum_M3->display((int16_t)angle1[2]/20  );
        ui->lcdNum_M4->display((int16_t)angle1[3]/20  );
        ui->lcdNum_M5->display((int16_t)angle1[4]/20  );
        ui->lcdNum_M6->display((int16_t)angle1[5]/20  );
        break;

    case 0xF2:
        //存储角度数据并转换为float
        for(int j=0;j<6;j++)
        {
            for(int i=0;i<4;i++)
            {
                data_trans[4-i] = receive_Buff[i+3+4*j];
            }
            curent[j] = *((float *)data_trans);
            qDebug()<<"curent"<<j<<":"<<curent[j];
        }
        break;

    case 0xF3:
        cnt = 4;
        Received_Position_zeroed_Flag          = receive_Buff[cnt++];
        Received_angle_Clear_Flag              = receive_Buff[cnt++];
        Received_ShouDongKangFu_mode           = receive_Buff[cnt++];
        Received_AutoKangFu_mode               = receive_Buff[cnt++];
        Received_ShouDongKangFu_SpeedSet_Flag  = receive_Buff[cnt++];
        Received_AutoKangFu_CiShu              = receive_Buff[cnt++];
        Received_Curve_Type                    = receive_Buff[cnt++];
        Received_Para_Send_Flag                = receive_Buff[cnt++];
        cnt = 20;
        Received_Motor1_Enabled                = receive_Buff[cnt++];
        Received_Motor2_Enabled                = receive_Buff[cnt++];
        Received_Motor3_Enabled                = receive_Buff[cnt++];
        Received_Motor4_Enabled                = receive_Buff[cnt++];
        Received_Motor5_Enabled                = receive_Buff[cnt++];
        Received_Motor6_Enabled                = receive_Buff[cnt++];
        //接收状态更新
        received_state_updata();
        //qDebug()<<Received_AutoKangFu_CiShu;
        break;
        qDebug()<<"功能位"<<":error!";

    }
}
//接收数据之后的状态更新
void Exo_Main::received_state_updata()
{
    //康复模式标志更新UI
    if(Received_ShouDongKangFu_mode)
    {
        ui->groupBox_ShouDongKangFu->setTitle("手动康复（运行）");
    }
    else
    {
        ui->groupBox_ShouDongKangFu->setTitle("手动康复");
    }

    if(Received_AutoKangFu_mode)
    {
        ui->groupBox_AutoKangFu->setTitle("自动康复（运行）");
    }
    else
    {
        ui->groupBox_AutoKangFu->setTitle("自动康复");
    }
    //设置康复曲线选择失能使能
    ui->comboBox_KangFuCurveChose->setEnabled(!Received_AutoKangFu_mode);
    //设置康复次数选择失能使能
    ui->spinBox_AutoKangFuCiShu  ->setEnabled(!Received_AutoKangFu_mode);
    //设置康复参数传递失能使能
    ui->Btn_Auto_ParaSend        ->setEnabled(!Received_AutoKangFu_mode);

    ui->Label_KangFuCiShu->setText("剩余康复次数："+QString::number(Received_AutoKangFu_CiShu)+"次");

    //康复速度标志更新UI
    switch(Received_ShouDongKangFu_SpeedSet_Flag)
    {
    case 0:
        ui->Label_ShouDongKangFu_Speed->setText("康复速度：低速");
        break;
    case 1:
        ui->Label_ShouDongKangFu_Speed->setText("康复速度：中速");
        break;
    case 2:
        ui->Label_ShouDongKangFu_Speed->setText("康复速度：高速");
        break;
    case 3:
        ui->Label_ShouDongKangFu_Speed->setText("康复速度：限幅");
        break;
    }
    //曲线UI
    switch(Received_Curve_Type)
    {
    case 0:
        ui->Label_CurveType->setText("sin曲线1");
        break;
    case 1:
        ui->Label_CurveType->setText("sin曲线2");
        break;
    case 2:
        ui->Label_CurveType->setText("sin曲线3");
        break;
    case 3:
        ui->Label_CurveType->setText("康复曲线");
        break;
    }

    //康复腿以及康复运动UI
    switch(Received_Para_Send_Flag)
    {
    case 1:
        ui->Label_KangFuYunDong->setText("左腿，屈曲伸展运动");
        break;
    case 2:
        ui->Label_KangFuYunDong->setText("左腿，外展内收运动");
        break;
    case 3:
        ui->Label_KangFuYunDong->setText("左腿，内旋外旋运动");
        break;
    case 4:
        ui->Label_KangFuYunDong->setText("右腿，屈曲伸展运动");
        break;
    case 5:
        ui->Label_KangFuYunDong->setText("右腿，外展内收运动");
        break;
    case 6:
        ui->Label_KangFuYunDong->setText("右腿，内旋外旋运动");
        break;
    case 7:
        ui->Label_KangFuYunDong->setText("双腿，屈曲伸展运动");
        break;
    case 8:
        ui->Label_KangFuYunDong->setText("双腿，外展内收运动");
        break;
    case 9:
        ui->Label_KangFuYunDong->setText("双腿，内旋外旋运动");
        break;
    }
    ui->Title_all->setStyleSheet("color: blue; background-color: yellow");
}


void Exo_Main::on_Btn_LM1_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA1;//调整模式
    send_data[5]=0x01;//数据0001
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    qDebug("电机1");
}

void Exo_Main::on_Btn_LM2_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA1;//调整模式
    send_data[5]=0x02;//数据0010
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    qDebug("电机2");
}

void Exo_Main::on_Btn_LM3_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA1;//调整模式
    send_data[5]=0x04;//数据0100
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    qDebug("电机3");
}

void Exo_Main::on_Btn_RM1_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA1;//调整模式
    send_data[5]=0x08;//数据1000
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    qDebug("电机4");
}

void Exo_Main::on_Btn_RM2_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA1;//调整模式
    send_data[5]=0x10;//数据00010000
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    qDebug("电机5");
}

void Exo_Main::on_Btn_RM3_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA1;//调整模式
    send_data[5]=0x20;//数据00 100  000
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    qDebug("电机6");
}

void Exo_Main::on_Btn_BackZero_clicked()
{
    uint8_t sum = 0;
    uint8_t send_data[6];
    uint8_t i=5;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x01;//数据长度
    send_data[4]=0xA2;//数据
    for(i=0;i<5;i++){sum+= send_data[i];}
    send_data[5]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,6);
    serialport->write((char*)send_data,6);
    serialport->write((char*)send_data,6);
}

void Exo_Main::on_groupBox_ShouDongKangFu_clicked()
{
     qDebug("测试点击效果");
}
//手动康复模式开始（自动康复结束）
void Exo_Main::on_Btn_ShouDong_Start_clicked()
{
    uint8_t sum = 0;
    uint8_t send_data[6];
    uint8_t i=5;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x01;//数据长度
    send_data[4]=0xA4;//初始化完成
    for(i=0;i<5;i++){sum+= send_data[i];}
    send_data[5]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,6);
    serialport->write((char*)send_data,6);
    serialport->write((char*)send_data,6);
}


//自动康复模式开始
void Exo_Main::on_Btn_Auto_Start_clicked()
{
    uint8_t sum = 0;
    uint8_t send_data[6];
    uint8_t i=5;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x01;//数据长度
    send_data[4]=0xA5;//初始化完成
    for(i=0;i<5;i++){sum+= send_data[i];}
    send_data[5]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,6);
    serialport->write((char*)send_data,6);
    serialport->write((char*)send_data,6);

}

void Exo_Main::on_radioBtn_LeftLeg_clicked()
{
    LeftLeg_Flag  = 1;
    RightLeg_Flag = 0;
}


void Exo_Main::on_radioBtn_RightLeg_clicked()
{
    LeftLeg_Flag  = 0;
    RightLeg_Flag = 1;
}

void Exo_Main::on_radioBtn_BothLeg_clicked()
{
    LeftLeg_Flag  = 1;
    RightLeg_Flag = 1;
}

void Exo_Main::on_radioBtn_QuShen_clicked()
{
    QuShen_Flag     = 1;
    ZhanShou_Flag   = 0;
    NeiWaiXuan_Flag = 0;
}

void Exo_Main::on_radioBtn_ZhanShou_clicked()
{
    QuShen_Flag     = 0;
    ZhanShou_Flag   = 1;
    NeiWaiXuan_Flag = 0;
}

void Exo_Main::on_radioBtn_NeiWaiXuan_clicked()
{
    QuShen_Flag     = 0;
    ZhanShou_Flag   = 0;
    NeiWaiXuan_Flag = 1;
}

void Exo_Main::on_radioBtn_LeftLeg_2_clicked()
{
    LeftLeg_Flag  = 1;
    RightLeg_Flag = 0;
}

void Exo_Main::on_radioBtn_RightLeg_2_clicked()
{
    LeftLeg_Flag  = 0;
    RightLeg_Flag = 1;
}

void Exo_Main::on_radioBtn_BothLeg_2_clicked()
{
    LeftLeg_Flag  = 1;
    RightLeg_Flag = 1;
}

void Exo_Main::on_radioBtn_QuShen_2_clicked()
{
    QuShen_Flag     = 1;
    ZhanShou_Flag   = 0;
    NeiWaiXuan_Flag = 0;
}

void Exo_Main::on_radioBtn_ZhanShou_2_clicked()
{
    QuShen_Flag     = 0;
    ZhanShou_Flag   = 1;
    NeiWaiXuan_Flag = 0;
}

void Exo_Main::on_radioBtn_NeiWaiXuan_2_clicked()
{
    QuShen_Flag     = 0;
    ZhanShou_Flag   = 0;
    NeiWaiXuan_Flag = 1;
}

void Exo_Main::on_Btn_ShouDong_ParaSend_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;

    if((!RightLeg_Flag) && LeftLeg_Flag && QuShen_Flag)
        send_data[5]=0x01;

    if((!RightLeg_Flag) && LeftLeg_Flag && ZhanShou_Flag)
        send_data[5]=0x02;

    if((!RightLeg_Flag) && LeftLeg_Flag && NeiWaiXuan_Flag)
        send_data[5]=0x03;

    if((!LeftLeg_Flag) && RightLeg_Flag && QuShen_Flag)
        send_data[5]=0x04;

    if((!LeftLeg_Flag) && RightLeg_Flag && ZhanShou_Flag)
        send_data[5]=0x05;

    if((!LeftLeg_Flag) && RightLeg_Flag && NeiWaiXuan_Flag)
        send_data[5]=0x06;

    if(LeftLeg_Flag && RightLeg_Flag && QuShen_Flag)
        send_data[5]=0x07;

    if(LeftLeg_Flag && RightLeg_Flag && ZhanShou_Flag)
        send_data[5]=0x08;

    if(LeftLeg_Flag && RightLeg_Flag && NeiWaiXuan_Flag)
        send_data[5]=0x09;

    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA6;//调整模式

    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    serialport->write((char*)send_data,7);
    serialport->write((char*)send_data,7);
    qDebug()<<("手动参数传递")<<send_data[5];
}

void Exo_Main::on_combBox_SpeedChose_activated(int index)
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA7;//调整模式
    send_data[5]=0x00;//初始数据为0x00
    //速度选择：0x01低速  0x02中速 0x03 高速
    switch(index)
    {
    case 0:
        send_data[5] = 0x01;
        break;
    case 1:
        send_data[5] = 0x02;
        break;
    case 2:
        send_data[5] = 0x03;
        break;
    case 3:
        send_data[5] = 0x04;
        break;
    }
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    serialport->write((char*)send_data,7);
    serialport->write((char*)send_data,7);
    qDebug("康复速度为");
}

void Exo_Main::on_Btn_Init_clicked()
{
    uint8_t sum = 0;
    uint8_t send_data[6];
    uint8_t i=5;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x01;//数据长度
    send_data[4]=0xA3;//初始化完成
    for(i=0;i<5;i++){sum+= send_data[i];}
    send_data[5]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,6);
    serialport->write((char*)send_data,6);
    serialport->write((char*)send_data,6);
}

//手动训练结束
void Exo_Main::on_Btn_ShouDongEnd_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[6];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x01;//数据长度
    send_data[4]=0xA8;//数据
    for(i=0;i<5;i++){sum+= send_data[i];}
    send_data[5]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,6);
    //手动康复失能
//    ui->groupBox_ShouDongKangFu->setEnabled(0);
}

void Exo_Main::on_AutoMode_End_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[6];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x01;//数据长度
    send_data[4]=0xA9;//数据
    for(i=0;i<5;i++){sum+= send_data[i];}
    send_data[5]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,6);
    //自动康复失能
    //ui->groupBox_AutoKangFu ->setEnabled(0);
}

void Exo_Main::on_Btn_Auto_ParaSend_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;

    if((!RightLeg_Flag) && LeftLeg_Flag && QuShen_Flag)
        send_data[5]=0x01;

    if((!RightLeg_Flag) && LeftLeg_Flag && ZhanShou_Flag)
        send_data[5]=0x02;

    if((!RightLeg_Flag) && LeftLeg_Flag && NeiWaiXuan_Flag)
        send_data[5]=0x03;

    if((!LeftLeg_Flag) && RightLeg_Flag && QuShen_Flag)
        send_data[5]=0x04;

    if((!LeftLeg_Flag) && RightLeg_Flag && ZhanShou_Flag)
        send_data[5]=0x05;

    if((!LeftLeg_Flag) && RightLeg_Flag && NeiWaiXuan_Flag)
        send_data[5]=0x06;

    if(LeftLeg_Flag && RightLeg_Flag && QuShen_Flag)
        send_data[5]=0x07;

    if(LeftLeg_Flag && RightLeg_Flag && ZhanShou_Flag)
        send_data[5]=0x08;

    if(LeftLeg_Flag && RightLeg_Flag && NeiWaiXuan_Flag)
        send_data[5]=0x09;

    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA6;//调整模式

    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    serialport->write((char*)send_data,7);
    serialport->write((char*)send_data,7);
    qDebug()<<("自动训练参数传递")<<send_data[5];
}

void Exo_Main::on_spinBox_AutoKangFuCiShu_editingFinished()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xAA;//调整模式
    send_data[5]=(char)(ui->spinBox_AutoKangFuCiShu->value());//初始数据为0x00
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    qDebug()<<"康复次数为"<<send_data[5];
}


//曲线选择
void Exo_Main::on_comboBox_KangFuCurveChose_activated(int index)
{
    if(index==0){ui->radioBtn_QuShen_2    ->setChecked(1);}
    if(index==1){ui->radioBtn_ZhanShou_2  ->setChecked(1);}
    if(index==2){ui->radioBtn_NeiWaiXuan_2->setChecked(1);}
    if(index==3){ui->radioBtn_QuShen_2    ->setChecked(1);}
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x01;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xAB;//调整模式
    send_data[5]=index;//初始数据
    //曲线选择：0x00sin  0x01步态
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    qDebug()<<"选择曲线为"<<send_data[5];
}


//界面手动使能
void Exo_Main::on_radioBtn_ShouDong_clicked(bool checked)
{
    //手动康复使能
    ui->groupBox_ShouDongKangFu->setEnabled(checked);
    //自动康复使能
    ui->groupBox_AutoKangFu    ->setEnabled(!checked);
    ui->Label_ShouDongKangFu_Speed->show();
    ui->Label_KangFuCiShu         ->hide();
    ui->Label_KangFuYunDong       ->show();
    ui->Label_CurveType           ->hide();
}

void Exo_Main::on_radioBtn_ShouDong_2_clicked(bool checked)
{
    //自动康复使能
    ui->groupBox_AutoKangFu ->setEnabled(checked);
    //手动康复使能
    ui->groupBox_ShouDongKangFu->setEnabled(!checked);
    ui->Label_KangFuCiShu         ->show();
    ui->Label_ShouDongKangFu_Speed->hide();
    ui->Label_KangFuYunDong       ->show();
    ui->Label_CurveType           ->show();
}
