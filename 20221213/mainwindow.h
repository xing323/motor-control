#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCharts>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>

using namespace QtCharts;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void data_Received(float data);//接收从UI的数据
    qreal fourier_curve(float a1,float b1,float a2,float b2,float a3,float b3,float a4,float b4,float a5,float b5, float phy,float time);
    qreal curve_para[6][11];//储存参数值
public slots:
    void creat_Qchart();//创建图
    void prepare_Data();//数据准备
    void updata_Chart();//更新图
    void on_Btn_Clear_clicked();//清除图像

    void main_window_Init();   //界面


private slots:
//    void on_Btn_ConnectSet_clicked();

    void on_Btn_linepreview_clicked();

    void on_tabWidget_tabBarClicked(int index);

    void on_lineEdit_a1_editingFinished();

    void on_lineEdit_b1_editingFinished();

    void on_lineEdit_a2_editingFinished();

    void on_lineEdit_phy_editingFinished();

    void on_lineEdit_b2_editingFinished();

    void on_lineEdit_a3_editingFinished();

    void on_lineEdit_b3_editingFinished();

    void on_lineEdit_a4_editingFinished();

    void on_lineEdit_b4_editingFinished();

    void on_lineEdit_a1_2_editingFinished();

    void on_lineEdit_b1_2_editingFinished();

    void on_lineEdit_a2_2_editingFinished();

    void on_lineEdit_b2_2_editingFinished();

    void on_lineEdit_a3_2_editingFinished();

    void on_lineEdit_b3_2_editingFinished();

    void on_lineEdit_a4_2_editingFinished();

    void on_lineEdit_b4_2_editingFinished();

    void on_lineEdit_phy_2_editingFinished();

    void on_lineEdit_a1_3_editingFinished();

    void on_lineEdit_b1_3_editingFinished();

    void on_lineEdit_a2_3_editingFinished();

    void on_lineEdit_b2_3_editingFinished();

    void on_lineEdit_a3_3_editingFinished();

    void on_lineEdit_b3_3_editingFinished();

    void on_lineEdit_a4_3_editingFinished();

    void on_lineEdit_b4_3_editingFinished();

    void on_lineEdit_phy_3_editingFinished();

    void on_checkBox_Beta_clicked();

    void on_checkBox_Alpha_clicked();

    void on_checkBox_Gamma_clicked();

    void on_Btn_control_Window_clicked();

private:
    Ui::MainWindow *ui;
    QChart *m_chart;
    QLineSeries *m_series ;

};

#endif // MAINWINDOW_H
