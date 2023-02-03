#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "widget.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),m_chart(new QChart),
    m_series(new QLineSeries)
{
    ui->setupUi(this);
    main_window_Init();
    creat_Qchart();//创建图


}

MainWindow::~MainWindow()
{
    delete ui;
}

//创建图表并初始化
void MainWindow::creat_Qchart()
{
//    //创建图标
    QChart *chart = new QChart(m_chart);
    chart->setTitle("电机角度曲线");
    ui->chartView->setChart(chart);
    ui->chartView->setRenderHint(QPainter::Antialiasing);//抗锯齿


    //创建曲线序列
    QSplineSeries *line1 = new QSplineSeries();
    QSplineSeries *line2 = new QSplineSeries();
    QSplineSeries *line3 = new QSplineSeries();
    QSplineSeries *line4 = new QSplineSeries();
    QSplineSeries *line5 = new QSplineSeries();
    QSplineSeries *line6 = new QSplineSeries();

//    //曲线名字
//    line1->setName("LAlpha");
//    line2->setName("LBeta");
//    line3->setName("LGamma");
//    line4->setName("RAlpha");
//    line5->setName("RBeta");
//    line6->setName("RGamma");

    //定义曲线线型
    QPen pen1;
    pen1.setWidth(2);
    pen1.setStyle(Qt::DotLine);
    pen1.setColor(Qt::red);
    line1->setPen(pen1);
    pen1.setStyle(Qt::SolidLine);
    pen1.setColor(Qt::blue);
    line2->setPen(pen1);

    chart->addSeries(line1);
    chart->addSeries(line2);
    chart->addSeries(line3);
    chart->addSeries(line4);
    chart->addSeries(line5);
    chart->addSeries(line6);

    QValueAxis *axisX = new QValueAxis;
    axisX->setRange(0,1);  //坐标轴范围
    axisX->setLabelFormat("%.1f");
    axisX->setTickCount(5);
    axisX->setMinorTickCount(4);//不知道什么意思
//    axisX->setTitleText("时间/（s）");

    QValueAxis *axis_Angle = new QValueAxis;
    axis_Angle->setRange(-100,100);  //坐标轴范围
    axis_Angle->setLabelFormat("%.1f");
    axis_Angle->setTickCount(9);
    axis_Angle->setMinorTickCount(3);//不知道什么意思
//    axis_Angle->setTitleText("角度/（°）");
    //给曲线添加轴
    chart->addAxis ( axisX, Qt::AlignBottom );
    line1->attachAxis(axisX);
    line2->attachAxis(axisX);
    line3->attachAxis(axisX);
    line4->attachAxis(axisX);
    line5->attachAxis(axisX);
    line6->attachAxis(axisX);

    chart->addAxis ( axis_Angle, Qt::AlignLeft );
    line1->attachAxis(axis_Angle);
    line2->attachAxis(axis_Angle);
    line3->attachAxis(axis_Angle);
    line4->attachAxis(axis_Angle);
    line5->attachAxis(axis_Angle);
    line6->attachAxis(axis_Angle);

    //修改说明样式
    chart->legend()->setVisible(false);
    chart->legend()->setAlignment(Qt::AlignRight);//底部对齐
    chart->legend()->setBackgroundVisible(true);//设置背景是否可视
    chart->legend()->setAutoFillBackground(true);//设置背景自动填充
    chart->legend()->setColor(QColor(222,233,251));//设置颜色
    chart->legend()->setLabelColor(QColor(0,100,255));//设置标签颜色
    chart->legend()->setMaximumHeight(500);

}

//数据准备
void MainWindow::prepare_Data()
{
    //    QLineSeries *line1 = (QLineSeries *)ui->chartView->chart()->series().at(0);
    //    QLineSeries *line2 = (QLineSeries *)ui->chartView->chart()->series().at(1);
    //    QLineSeries *line3 = (QLineSeries *)ui->chartView->chart()->series().at(2);
    //    QLineSeries *line4 = (QLineSeries *)ui->chartView->chart()->series().at(3);
    //    QLineSeries *line5 = (QLineSeries *)ui->chartView->chart()->series().at(4);
    //    QLineSeries *line6 = (QLineSeries *)ui->chartView->chart()->series().at(5);
    //    line1->clear();
    //    line2->clear();
    //    line3->clear();
    //    line4->clear();
    //    line5->clear();
    //    line6->clear();

}
//更新图
void MainWindow::updata_Chart()
{
//    static uint64_t cnt = 0;//记录点个数
//    qreal serial_Time = 0.01*cnt;
//    static qreal min_xRange,max_xRange,min_yRange = -10,max_yRange= 20;//横轴纵轴绘图范围
//    //线条
//    QSplineSeries *line1 = (QSplineSeries *)ui->chartView->chart()->series().at(0);
//    QSplineSeries *line2 = (QSplineSeries *)ui->chartView->chart()->series().at(1);
//    QSplineSeries *line3 = (QSplineSeries *)ui->chartView->chart()->series().at(2);
//    QSplineSeries *line4 = (QSplineSeries *)ui->chartView->chart()->series().at(3);
//    QSplineSeries *line5 = (QSplineSeries *)ui->chartView->chart()->series().at(4);
//    QSplineSeries *line6 = (QSplineSeries *)ui->chartView->chart()->series().at(5);

//    QValueAxis *axisX = (QValueAxis *)ui->chartView->chart()->axes().at(0);
//    QValueAxis *axisY = (QValueAxis *)ui->chartView->chart()->axes().at(1);
//    //x轴范围动态变化
//    if(serial_Time<10)
//    {
//        min_xRange = 0;max_xRange = 10;
//    }
//    else
//    {
//        min_xRange = serial_Time-10;max_xRange = serial_Time;
//    }
//    //y轴范围动态变化
//    for(int i=0;i<6;i++)
//    {
//        if(min_yRange>(angle[i]-10))
//            min_yRange = angle[i]-10;
//        if(max_yRange<(angle[i]+10))
//            max_yRange = angle[i]+10;
//    }
//    axisX->setRange(min_xRange,max_xRange);
//    axisY->setRange(min_yRange,max_yRange);
//    //将接收到的数据点添加到曲线
//    line1->append(serial_Time,angle[0]);
//    line2->append(serial_Time,angle[1]);
//    line3->append(serial_Time,angle[2]);
//    line4->append(serial_Time,angle[3]);
//    line5->append(serial_Time,angle[4]);
//    line6->append(serial_Time,angle[5]);
//    qDebug()<<cnt++;
}
//曲线清除
void MainWindow::on_Btn_Clear_clicked()
{
//    QLineSeries *line1 = (QLineSeries *)ui->chartView->chart()->series().at(0);
//    QLineSeries *line2 = (QLineSeries *)ui->chartView->chart()->series().at(1);
//    QLineSeries *line3 = (QLineSeries *)ui->chartView->chart()->series().at(2);
//    QLineSeries *line4 = (QLineSeries *)ui->chartView->chart()->series().at(3);
//    QLineSeries *line5 = (QLineSeries *)ui->chartView->chart()->series().at(4);
//    QLineSeries *line6 = (QLineSeries *)ui->chartView->chart()->series().at(5);
//    line1->clear();
//    line2->clear();
//    line3->clear();
//    line4->clear();
//    line5->clear();
//    line6->clear();

}

//void MainWindow::on_Btn_ConnectSet_clicked()
//{
//    Widget *UI1 = new Widget();
//    UI1->show();
//    this->hide();
//}
//
void MainWindow::data_Received(float data)
{
//    static uint8_t cnt = 0;
//    angle[cnt % 6] = data;
//    cnt++;
//   qDebug()<<"data_Received_OK";
//    if(cnt % 6  == 0)
//    {
//        updata_Chart();
//        cnt = 0;
//    }

}
//界面初始化设置
void MainWindow::main_window_Init()
{
    ui->curvePreview->resize(550,450);
    ui->chartView->resize(450,350);

    uint row_Num = 6;                             //行数
    uint column_Num = 11;                         //列数
    for(uint i=0;i<row_Num;i++)
    {
        for(uint j=0;j<column_Num;j++)
        {
            curve_para[i][j] = 0;
//            qDebug()<<i<<j<<curve_para[i][j];
        }
    }


}
//曲线预览
void MainWindow::on_Btn_linepreview_clicked()
{
    if(ui->checkBox_Alpha->checkState() == Qt::Checked)
    {
        //曲线显示
        ui->chartView->chart()->series().at(0)->show();
        ui->checkBox_Alpha->clicked();
    }
    else
    {
        ui->chartView->chart()->series().at(0)->hide();
    }
    if(ui->checkBox_Beta->checkState() == Qt::Checked)
    {
        //曲线显示
        ui->chartView->chart()->series().at(1)->show();
        ui->checkBox_Beta->clicked();
    }
    else
    {
        ui->chartView->chart()->series().at(1)->hide();
    }
    if(ui->checkBox_Gamma->checkState() == Qt::Checked)
    {
        //曲线显示
        ui->chartView->chart()->series().at(2)->show();
        ui->checkBox_Gamma->clicked();
    }
    else
    {
        ui->chartView->chart()->series().at(2)->hide();
    }
}

//fourier级数计算
qreal MainWindow:: fourier_curve(float a1,float b1,float a2,float b2,float a3,float b3,float a4,float b4,float a5,float b5,float phy,float time)
{
    qreal res;
    res = a1*cos(6.283f*time+phy)+b1*sin(6.283f*time+phy)+
            a2*cos(6.283f*2*time+phy)+b2*sin(6.283f*2*time+phy)+
            a3*cos(6.283f*3*time+phy)+b3*sin(6.283f*3*time+phy)+
            a4*cos(6.283f*4*time+phy)+b4*sin(6.283f*4*time+phy)+
            a5*cos(6.283f*5*time+phy)+b5*sin(6.283f*5*time+phy);
    return res;
}

//切换栏时更新赋值
void MainWindow::on_tabWidget_tabBarClicked(int index)
{
    qDebug()<<"选中了"<<index;
    switch(index)
    {
    case 0:
        //参数赋值
        ui->lineEdit_a1->setText(QString::number(curve_para[0][0] ,'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_b1->setText(QString::number(curve_para[0][1],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_a2->setText(QString::number(curve_para[0][2],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_b2->setText(QString::number(curve_para[0][3],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_a3->setText(QString::number(curve_para[0][4],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_b3->setText(QString::number(curve_para[0][5],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_a4->setText(QString::number(curve_para[0][6],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_b4->setText(QString::number(curve_para[0][7],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_phy->setText(QString::number(curve_para[0][8],'f', 2 ));//保留两位小数,形式为1.1
        break;
    case 1:
        //参数赋值
        ui->lineEdit_a1_2->setText(QString::number(curve_para[1][0] ,'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_b1_2->setText(QString::number(curve_para[1][1],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_a2_2->setText(QString::number(curve_para[1][2],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_b2_2->setText(QString::number(curve_para[1][3],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_a3_2->setText(QString::number(curve_para[1][4],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_b3_2->setText(QString::number(curve_para[1][5],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_a4_2->setText(QString::number(curve_para[1][6],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_b4_2->setText(QString::number(curve_para[1][7],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_phy_2->setText(QString::number(curve_para[1][8],'f', 2 ));//保留两位小数,形式为1.1
        break;
    case 2:
        //参数赋值
        ui->lineEdit_a1_3->setText(QString::number(curve_para[2][0] ,'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_b1_3->setText(QString::number(curve_para[2][1],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_a2_3->setText(QString::number(curve_para[2][2],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_b2_3->setText(QString::number(curve_para[2][3],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_a3_3->setText(QString::number(curve_para[2][4],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_b3_3->setText(QString::number(curve_para[2][5],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_a4_3->setText(QString::number(curve_para[2][6],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_b4_3->setText(QString::number(curve_para[2][7],'f', 2 ));//保留两位小数,形式为1.1
        ui->lineEdit_phy_3->setText(QString::number(curve_para[2][8],'f', 2 ));//保留两位小数,形式为1.1
        break;

    }

}
//参数编辑栏
void MainWindow::on_lineEdit_a1_editingFinished()
{
    curve_para[0][0] = ui->lineEdit_a1->text().toFloat();
    ui->lineEdit_a1->setText(QString::number(curve_para[0][0] ,'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_b1_editingFinished()
{
    curve_para[0][1] = ui->lineEdit_b1->text().toFloat();
    ui->lineEdit_b1->setText(QString::number(curve_para[0][1],'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_a2_editingFinished()
{
    curve_para[0][2] = ui->lineEdit_a2->text().toFloat();
    ui->lineEdit_a2->setText(QString::number(curve_para[0][2] ,'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_b2_editingFinished()
{
    curve_para[0][3] = ui->lineEdit_b2->text().toFloat();
    ui->lineEdit_b2->setText(QString::number(curve_para[0][3] ,'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_a3_editingFinished()
{
    curve_para[0][4] = ui->lineEdit_a3->text().toFloat();
    ui->lineEdit_a3->setText(QString::number(curve_para[0][4] ,'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_b3_editingFinished()
{
    curve_para[0][5] = ui->lineEdit_b3->text().toFloat();
    ui->lineEdit_b3->setText(QString::number(curve_para[0][5],'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_a4_editingFinished()
{
    curve_para[0][6] = ui->lineEdit_a4->text().toFloat();
    ui->lineEdit_a4->setText(QString::number(curve_para[0][6] ,'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_b4_editingFinished()
{
    curve_para[0][7] = ui->lineEdit_b4->text().toFloat();
    ui->lineEdit_b4->setText(QString::number(curve_para[0][7],'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_phy_editingFinished()
{
    curve_para[0][8] = ui->lineEdit_phy->text().toFloat();
    ui->lineEdit_phy->setText(QString::number(curve_para[0][10],'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_a1_2_editingFinished()
{
    curve_para[1][0] = ui->lineEdit_a1_2->text().toFloat();
    ui->lineEdit_a1_2->setText(QString::number(curve_para[1][0] ,'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_b1_2_editingFinished()
{
    curve_para[1][1] = ui->lineEdit_b1_2->text().toFloat();
    ui->lineEdit_b1_2->setText(QString::number(curve_para[1][1],'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_a2_2_editingFinished()
{
    curve_para[1][2] = ui->lineEdit_a2_2->text().toFloat();
    ui->lineEdit_a2_2->setText(QString::number(curve_para[1][2] ,'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_b2_2_editingFinished()
{
    curve_para[1][3] = ui->lineEdit_b2_2->text().toFloat();
    ui->lineEdit_b2_2->setText(QString::number(curve_para[1][3],'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_a3_2_editingFinished()
{
    curve_para[1][4] = ui->lineEdit_a3_2->text().toFloat();
    ui->lineEdit_a3_2->setText(QString::number(curve_para[1][4] ,'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_b3_2_editingFinished()
{
    curve_para[1][5] = ui->lineEdit_b3_2->text().toFloat();
    ui->lineEdit_b3_2->setText(QString::number(curve_para[1][5],'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_a4_2_editingFinished()
{
    curve_para[1][6] = ui->lineEdit_a4_2->text().toFloat();
    ui->lineEdit_a4_2->setText(QString::number(curve_para[1][6] ,'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_b4_2_editingFinished()
{
    curve_para[1][7] = ui->lineEdit_b4_2->text().toFloat();
    ui->lineEdit_b4_2->setText(QString::number(curve_para[1][7],'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_phy_2_editingFinished()
{
    curve_para[1][8] = ui->lineEdit_phy_2->text().toFloat();
    ui->lineEdit_phy_2->setText(QString::number(curve_para[1][10],'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_a1_3_editingFinished()
{
    curve_para[2][0] = ui->lineEdit_a1_3->text().toFloat();
    ui->lineEdit_a1_3->setText(QString::number(curve_para[2][0] ,'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_b1_3_editingFinished()
{
    curve_para[2][1] = ui->lineEdit_b1_3->text().toFloat();
    ui->lineEdit_b1_3->setText(QString::number(curve_para[2][1],'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_a2_3_editingFinished()
{
    curve_para[2][2] = ui->lineEdit_a2_3->text().toFloat();
    ui->lineEdit_a2_3->setText(QString::number(curve_para[2][2] ,'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_b2_3_editingFinished()
{
    curve_para[2][3] = ui->lineEdit_b2_3->text().toFloat();
    ui->lineEdit_b2_3->setText(QString::number(curve_para[2][3],'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_a3_3_editingFinished()
{
    curve_para[2][4] = ui->lineEdit_a3_3->text().toFloat();
    ui->lineEdit_a3_3->setText(QString::number(curve_para[2][4] ,'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_b3_3_editingFinished()
{
    curve_para[2][5] = ui->lineEdit_b3_3->text().toFloat();
    ui->lineEdit_b3_3->setText(QString::number(curve_para[2][5],'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_a4_3_editingFinished()
{
    curve_para[2][6] = ui->lineEdit_a4_3->text().toFloat();
    ui->lineEdit_a4_3->setText(QString::number(curve_para[2][6] ,'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_b4_3_editingFinished()
{
    curve_para[2][7] = ui->lineEdit_b4_3->text().toFloat();
    ui->lineEdit_b4_3->setText(QString::number(curve_para[2][7],'f', 2 ));//保留两位小数,形式为1.1
}

void MainWindow::on_lineEdit_phy_3_editingFinished()
{
    curve_para[2][8] = ui->lineEdit_phy_3->text().toFloat();
    ui->lineEdit_phy_3->setText(QString::number(curve_para[2][10] ,'f', 2 ));//保留两位小数,形式为1.1
}

//曲线显示与否
void MainWindow::on_checkBox_Alpha_clicked()
{
    //绘图点数
    int point_Num = 1000;
    //创建曲线序列
    QSplineSeries *line1 = (QSplineSeries *)ui->chartView->chart()->series().at(0);
    //清除之前的曲线
    line1->clear();
    for(int i=0;i<point_Num;i++)
    {
        qreal t = 0.001* i;
        qreal y = fourier_curve(curve_para[0][0],curve_para[0][1],
                curve_para[0][2],curve_para[0][3],
                curve_para[0][4],curve_para[0][5],
                curve_para[0][6],curve_para[0][7],
                curve_para[0][8],curve_para[0][9],
                curve_para[0][10],t);
        //添加序列点到曲线
        line1->append(t,y);
    }
}
void MainWindow::on_checkBox_Beta_clicked()
{
    //绘图点数
    int point_Num = 1000;
    //创建曲线序列
    QSplineSeries *line2 = (QSplineSeries *)ui->chartView->chart()->series().at(1);
    //清除之前的曲线
    line2->clear();
    for(int i=0;i<point_Num;i++)
    {
        qreal t = 0.001* i;
        qreal y = fourier_curve(curve_para[1][0],curve_para[1][1],
                curve_para[1][2],curve_para[1][3],
                curve_para[1][4],curve_para[1][5],
                curve_para[1][6],curve_para[1][7],
                curve_para[1][8],curve_para[1][9],
                curve_para[1][10],t);
        //添加序列点到曲线
        line2->append(t,y);
    }
}

void MainWindow::on_checkBox_Gamma_clicked()
{
    //绘图点数
    int point_Num = 1000;
    //创建曲线序列
    QSplineSeries *line3 = (QSplineSeries *)ui->chartView->chart()->series().at(2);
    //清除之前的曲线
    line3->clear();
    for(int i=0;i<point_Num;i++)
    {
        qreal t = 0.001* i;
        qreal y = fourier_curve(curve_para[2][0],curve_para[2][1],
                curve_para[2][2],curve_para[2][3],
                curve_para[2][4],curve_para[2][5],
                curve_para[2][6],curve_para[2][7],
                curve_para[2][8],curve_para[2][9],
                curve_para[2][10],t);
        //添加序列点到曲线
        line3->append(t,y);
    }
}

void MainWindow::on_Btn_control_Window_clicked()
{

    Widget *control_Window = new Widget();//初始化
    control_Window->setAttribute(Qt::WA_DeleteOnClose,true);//关闭窗口释放对象
    control_Window->show();
//    control_Window->close();
}
