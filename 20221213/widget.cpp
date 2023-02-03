#include "widget.h"
#include "ui_widget.h"
#include "mainwindow.h"
#include <QMessageBox>
#include <QtEndian>//大小端转换

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    setWindowTitle("髋关节康复系统");
    //创建串口对象
    serialport = new QSerialPort;
    //查找可用串口
    //    ui->Btn_Start ->setEnabled(false);
    //    ui->Btn_End ->setEnabled(false);
    ui->Btn_Close_Serial ->setEnabled(false);
    customPlot = new QCustomPlot;
    customPlot = ui->customPlot;
    custom_plot_Init();
    //定时器测试
//    QTimer *angle_Plot_Timer = new QTimer(this);
//    static int test;
//    connect(angle_Plot_Timer,&QTimer::timeout,[=](){
//        ui->label->setText("测试时间："+(QString::number(test++)));
//                //        custom_Real_Plot();
//        if(test>100) angle_Plot_Timer->stop();
//    });
    //    angle_Plot_Timer->start(500);
    //数据写入测试
    angle1[0] = 0.22;
    angle1[1] = 0.233;
    angle1[2] = 0.2456;
    angle1[3] = -0.25;
    QTimer *angle_Plot_Timer = new QTimer(this);
    static int test;
    connect(angle_Plot_Timer,&QTimer::timeout,[=](){
        ui->label->setText("测试时间："+(QString::number(test++)));
        //        custom_Real_Plot();
        SaveDataToCSVFile(angle1);
        angle1[0] = angle1[0] +1;
        angle1[1] = angle1[1] +1;
        angle1[2] = angle1[2] +1;
        angle1[3] = angle1[3] -1;
        if(test>10000) angle_Plot_Timer->stop();
    });
    angle_Plot_Timer->start(1);


}


Widget::~Widget()
{
    delete ui;
}

//查找串口
void Widget::on_Btn_Check_Serial_clicked()
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

//打开串口
void Widget::on_Btn_Open_Serial_clicked()
{
//    find_port();     //重新查找com
    //初始化串口//设置串口名
    serialport->setPortName(ui->comBox_Serial_num->currentText());
    if(serialport->open(QIODevice::ReadWrite))              //打开串口成功
    {
        serialport->setBaudRate(256000);       //设置波特率
        serialport->setDataBits(QSerialPort::Data8);
        serialport->setParity(QSerialPort::NoParity);//设置奇偶校验
        serialport->setStopBits(QSerialPort::OneStop);
        serialport->setFlowControl(QSerialPort::NoFlowControl);     //设置流控制
        //设置按钮

        ui->Btn_Open_Serial ->setEnabled(false);
        ui->Btn_Close_Serial ->setEnabled(true);
        //设置下拉菜单不可用
        ui->comBox_Serial_num->setEnabled(false);
        ui->Btn_Check_Serial->setEnabled(false);

        //串口数据接收
        connect(serialport,&QSerialPort::readyRead,[=](){
                    qDebug()<<"readyReadOK";
            read_Data(receive_Data);
            //数据协议处理
            data_PreProsess((uint8_t* )receive_Data, 29);
            custom_Real_Plot();
        });
    }
    else    //打开失败提示
    {
        QMessageBox::information(this,tr("Erro"),tr("Open the failure"),QMessageBox::Ok);
    }
}
//关闭串口
void Widget::on_Btn_Close_Serial_clicked()
{
    serialport->clear();        //清空缓存区
    serialport->close();        //关闭串口


    ui->Btn_Open_Serial ->setEnabled(true);
    ui->Btn_Close_Serial ->setEnabled(false);
    //设置下拉菜单可用
    ui->comBox_Serial_num->setEnabled(true);
    ui->Btn_Check_Serial->setEnabled(true);
}

//点击结束按钮
void Widget::on_Btn_End_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[6];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x02;//功能码
    send_data[3]=0x01;//数据长度
    send_data[4]=0xA3;//数据
    for(i=0;i<5;i++){sum+= send_data[i];}
    send_data[5]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,6);
}



//调整模式
//16进制数    高（2）  高（1）    低（4） 低（3）  低（2）  低（1）
//电机       // 电机6 // 电机5 //电机4 // 电机3 // 电机2 //电机1 //
void Widget::on_Btn_LM1_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x02;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA4;//调整模式
    send_data[5]=0x01;//数据0001
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    qDebug("电机1");
}

void Widget::on_Btn_LM2_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x02;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA4;//调整模式
    send_data[5]=0x02;//数据0010
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    qDebug("电机2");

}

void Widget::on_Btn_LM3_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x02;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA4;//调整模式
    send_data[5]=0x04;//数据0100
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    qDebug("电机3");
}

void Widget::on_Btn_RM1_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x02;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA4;//调整模式
    send_data[5]=0x08;//数据1000
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
qDebug("电机4");
}

void Widget::on_Btn_RM2_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x02;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA4;//调整模式
    send_data[5]=0x10;//数据00010000
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
qDebug("电机5");
}

void Widget::on_Btn_RM3_clicked()
{
    //信号内容
    uint8_t sum = 0;
    uint8_t send_data[7];
    uint8_t i=0;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x02;//功能码
    send_data[3]=0x02;//数据长度
    send_data[4]=0xA4;//调整模式
    send_data[5]=0x20;//数据00 100  000
    for(i=0;i<6;i++){sum+= send_data[i];}
    send_data[6]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,7);
    qDebug("电机6");
}

//初始化完成后设置
void Widget::on_Btn_Angle_Clear_clicked()
{
    uint8_t sum = 0;
    uint8_t send_data[6];
    uint8_t i=5;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x02;//功能码
    send_data[3]=0x01;//数据长度
    send_data[4]=0xA5;//初始化完成
    for(i=0;i<5;i++){sum+= send_data[i];}
    send_data[5]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,6);
}

//接收数据,会卡住，换用另一种写法
int Widget::read_Data(char *buffer)
{
    QByteArray receive_Buff,buff;
    if(serialport->isReadable())
    {
        int n = serialport->bytesAvailable();
        receive_Buff = serialport->readAll();
        buff = receive_Buff.mid(0,29);
        memcpy(buffer, receive_Buff, 29);
//        qDebug()<<"lengh_array:"<<lengh_array<<"lengh_buffer:"<<lengh_buffer;
//        qDebug()<<buffer[0]<<buffer[1]<<len<<"bytesAvailable"<<n;
        return n;
    }else
    {
    return -1;
    }
}



//切换UI
void Widget::on_Btn_Switch_UI_clicked()
{
//    MainWindow *Main_Menu = new MainWindow();//初始化
//    Main_Menu->show();
//    //UI之间的信号传输
//    connect(this,&Widget::data_Transmision,Main_Menu,&MainWindow::data_Received);
//    //this->close();
//    //串口数据接收
//    connect(serialport,&QSerialPort::readyRead,[=](){
//                qDebug()<<"readyReadOK";
//        read_Data(receive_Data);
//        //数据协议处理
//        data_PreProsess((uint8_t* )receive_Data, 29);
//    });
}

//接收报文的预处理
void Widget::data_PreProsess(uint8_t *receive_Buff, uint8_t length)
{
    uint8_t sum = 0;
    //校验帧头0xAA
    if((receive_Buff[0] != 0xAA)||(receive_Buff[1] != 0xAA))
    {
        return;
    }
    //校验和
    for(int i=0;i<length-1;i++)
    {
        sum += receive_Buff[i];
    }
    if(sum != receive_Buff[length-1])
    {
        return;
    }
    switch(receive_Buff[2])
    {
    //角度数据
    case 0xF1:
        uint8_t data_trans[4];// = {0x6C,0xA0,0x91,0x42};
         qDebug()<<"data_PreProsess_OK";
        //计算数据长度，默认都是浮点数：
        uint8_t data_Len = receive_Buff[3]/4 -1;
        //存储角度数据
        for(int j=0;j<data_Len;j++)
        {
            for(int i=0;i<4;i++)
            {
                data_trans[4-i] = receive_Buff[i+4+4*j];
            }
            data_Save[j] = *((float *)data_trans);//将接收到的数据储存起来
//            emit data_Transmision(angle1[j]);
            qDebug()<<"angle"<<j<<":"<<angle1[j];
        }
        break;
    }
}

//绘图初始化
void Widget::custom_plot_Init()
{
    /*字体设置*/
    QFont font;
    font.setStyleStrategy(QFont::NoAntialias);//设置字体样式
    customPlot->xAxis->setTickLabelFont(font);//x轴字体设置
    customPlot->yAxis->setTickLabelFont(font);//y轴字体设置
    customPlot->legend->setFont(font);//图例字体设置
    /*曲线1*/
    customPlot->addGraph(); //新建图层
    customPlot->graph(0)->setPen(QPen(Qt::blue));//颜色设置：蓝色
    customPlot->graph(0)->setName("曲线1");//图例名称
    /*曲线2*/
    customPlot->addGraph();
    customPlot->graph(1)->setPen(QPen(Qt::red));//颜色设置：红色
    customPlot->graph(1)->setName("曲线2");
    /*曲线3*/
    customPlot->addGraph(); //新建图层
    customPlot->graph(2)->setPen(QPen(Qt::cyan));//颜色设置：色
    customPlot->graph(2)->setName("曲线3");//图例名称
    /*曲线4*/
    customPlot->addGraph();
    customPlot->graph(3)->setPen(QPen(Qt::green));//颜色设置：绿色
    customPlot->graph(3)->setName("曲线4");
    /*曲线5*/
    customPlot->addGraph(); //新建图层
    customPlot->graph(4)->setPen(QPen(Qt::darkGreen ));//颜色设置：墨绿
    customPlot->graph(4)->setName("曲线5");//图例名称
    /*曲线6*/
    customPlot->addGraph();
    customPlot->graph(5)->setPen(QPen(Qt::darkBlue ));//颜色设置：=深蓝色
    customPlot->graph(5)->setName("曲线6");
    //坐标轴样式设置
    customPlot->xAxis->setLabel("时间/（s）");
    customPlot->yAxis->setLabel("角度/（°）");
    customPlot->xAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);  // 结束时加个箭头
    customPlot->yAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);  // 结束时加个箭头


}
//绘图函数
void Widget::custom_Real_Plot(void)
{
    static uint64_t cnt = 0;//记录点个数
    qreal serial_Time = 0.01*cnt;
    cnt++;
//    static qreal min_xRange,max_xRange,min_yRange = -10,max_yRange= 20;//横轴纵轴绘图范围

    double angle_m1 = angle1[0];//电机1角度
    double angle_m2 = angle1[1];//电机2角度
    double angle_m3 = angle1[2];//电机3角度
    double angle_m4 = angle1[3];//电机4角度
    double angle_m5 = angle1[4];//电机5角度
    double angle_m6 = angle1[5];//电机6角度

    customPlot->graph(0)->addData(serial_Time, angle_m1);//添加数据1到曲线1
    customPlot->graph(1)->addData(serial_Time, angle_m2);//添加数据2到曲线2
    customPlot->graph(2)->addData(serial_Time, angle_m3);//添加数据3到曲线3
    customPlot->graph(3)->addData(serial_Time, angle_m4);//添加数据4到曲线4
    customPlot->graph(4)->addData(serial_Time, angle_m5);//添加数据5到曲线5
    customPlot->graph(5)->addData(serial_Time, angle_m6);//添加数据6到曲线6

    //x轴范围变化时启动槽函数setRange，以实现x轴左移
    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
    //y轴的范围自动调整，避免数据超出y轴
    customPlot->graph(0)->rescaleValueAxis(true);
    customPlot->graph(1)->rescaleValueAxis(true);
    customPlot->graph(2)->rescaleValueAxis(true);
    customPlot->graph(3)->rescaleValueAxis(true);
    customPlot->graph(4)->rescaleValueAxis(true);
    customPlot->graph(5)->rescaleValueAxis(true);
    // axis configurations:坐标轴配置
    customPlot->xAxis->setRange(serial_Time, 8, Qt::AlignRight);//设定x轴的范围为key到key+8，方式为右对齐
    customPlot->replot();//重绘
}


//归零
void Widget::on_Btn_BackZero_clicked()
{
    uint8_t sum = 0;
    uint8_t send_data[6];
    uint8_t i=5;
    send_data[0]=0xAA;
    send_data[1]=0xAF;//帧头
    send_data[2]=0x02;//功能码
    send_data[3]=0x01;//数据长度
    send_data[4]=0xA2;//数据
    for(i=0;i<5;i++){sum+= send_data[i];}
    send_data[5]=sum;//校验和
    //发送数据
    serialport->write((char*)send_data,6);
}

void Widget::on_Btn_to_Curve_Setting_clicked()
{
    MainWindow *curve_Setting = new MainWindow();//初始化
    curve_Setting->show();

}


//写文件测试
void Widget::SaveDataToCSVFile(float* saved_Data)
{
    //保存数据的文件路径
    QString  csvFileName = QApplication::applicationDirPath() + "/TestData/" + \
            QDateTime::currentDateTime().toString("yyyyMMdd") + ".csv";
    QFile file(csvFileName);
    if (!file.exists())		//文件不存在的时候便新建，以当天日期为命名
    {
        file.open(QIODevice::WriteOnly);
        QTextStream txtOutPut(&file);
        txtOutPut << "Unit(mm)\n";
        txtOutPut << "Time,Data1,Data2,Data3,Data4\n";	//注意，每行数据结束后要加换行符
        file.close();
    }
    //写入数据
    file.open(QIODevice::WriteOnly|QIODevice::Append);
    QTextStream txtOutPut(&file);

    QString msg = QDateTime::currentDateTime().toString("hh:mm:ss") + "," \
            + QString::number(saved_Data[0],'f',3) + ","\
            + QString::number(saved_Data[1],'f',3) + ","\
            + QString::number(saved_Data[2],'f',3) + ","\
            + QString::number(saved_Data[3],'f',3) + "\n";	//这里只是模拟数据，具体数据做相对改动即可

    txtOutPut << msg;
    file.flush();
    file.close();

}
