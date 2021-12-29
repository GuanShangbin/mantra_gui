/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <QDateTime>
#include "../include/mantra_gui/main_window.hpp"
#include<string>

#include "../include/mantra_gui/script_controll.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ebox_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ScriptController::startScripts();
    Sleep(5000);        // 初始等待roscore三秒

    //连接ROS Master节点，并初始化Rviz组件
    int failtime = 0;   // 启动qnode失败的次数，超过5次即报错
    //
    while(!qnode.init("http://gg-HP-ENVY-Notebook:11311/", "192.168.123.34")){
        failtime++;
        if(failtime == 5)
        {
            showNoMasterMessage();
        }
        Sleep(1000);    // 再多等1秒再启动
    }

    ui.setupUi(this);
    setWindowIcon(QIcon(":/images/icon.png"));

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QTimer *timer = new QTimer(this);
    // ui.lv_laser_item->setModel(qnode.loggingModel_sub());  //add
    // QObject::connect(&qnode, SIGNAL(loggingUpdated_sub()), this, SLOT(updateLoggingView_sub()));  //add

    // // ui.lv_camera_item->setModel(qnode.loggingModel_sub2());  //add
    // QObject::connect(&qnode, SIGNAL(loggingUpdated_sub2()), this, SLOT(updateLoggingView_sub2()));  //add
    
    QObject::connect(&qnode,SIGNAL(Show_camera_image(int,QImage)),this,SLOT(slot_show_image(int,QImage)));//add 1105

    QObject::connect(ui.robotConnect, SIGNAL(clicked()), this, SLOT(robotConnect_clicked()));
    QObject::connect(ui.robotDisconnect, SIGNAL(clicked()), this, SLOT(robotDisconnect_clicked()));
    QObject::connect(ui.lidar_show_btn, SIGNAL(clicked()), this, SLOT(lidar_show_clicked()));
    QObject::connect(ui.caliBtn, SIGNAL(clicked()), this, SLOT(caliBtn_clicked()));
    QObject::connect(ui.graspBtn, SIGNAL(clicked()), this, SLOT(graspBtn_clicked()));
    QObject::connect(ui.stopGraspBtn, SIGNAL(clicked()), this, SLOT(stopGraspBtn_clicked()));
    QObject::connect(ui.handResetBtn, SIGNAL(clicked()), this, SLOT(handResetBtn_clicked()));
    QObject::connect(ui.camera_show_btn, SIGNAL(clicked()), this, SLOT(camera_show_clicked()));
    QObject::connect(ui.clearInfo, SIGNAL(clicked()), this, SLOT(clearInfo_clicked()));
    QObject::connect(timer,SIGNAL(timeout()),this,SLOT(timerUpdate()));
    timer->start(1000);
    show_lidar_ = false;
    show_camera_ = false;
    ui.lidar_show_widget->hide();
    ui.camera_show_widget->hide();

    rviz_plugin_= new RvizPlugin(this->ui.lo_laser_data);
    rviz_plugin_->initLidarDisplay();
    //new RvizPlugin(this->ui.lo_camera_data, RvizPlugin::TYPE_CAMERA);     //这么用会直接显示弹窗

    QObject::connect(&qnode, SIGNAL(Show_state(rs_perception::state)), this, SLOT(updateState(rs_perception::state)));

    //ui.camera_image->setBackgroundRole(NoBrush);

    ROS_INFO_STREAM("MainWindow Prepared");
    //ROS_INFO_STREAM(QCoreApplication::applicationDirPath().toStdString());
}


MainWindow::~MainWindow() {
    ScriptController::endScripts();
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::updateLoggingView_sub() {
    // ui.lv_laser_item->scrollToBottom();
}

void MainWindow::updateLoggingView_sub2() {
    // ui.lv_camera_item->scrollToBottom();
}

void MainWindow::timerUpdate()
{
    QDateTime time = QDateTime::currentDateTime();
    QString str = time.toString("yyyy-MM-dd hh:mm:ss dddd");
    ui.timeInfo->setText(str);
    // Sleep(1000);
}
void MainWindow::slot_show_image(int frame_id, QImage image)
{
    if(frame_id==0)
        ui.camera_image->setPixmap(QPixmap::fromImage(image).scaled(ui.camera_image->width(),ui.camera_image->height()));
    // else if(frame_id==1)
    //     ui.depth_image->setPixmap(QPixmap::fromImage(image).scaled(ui.camera_image->width(),ui.camera_image->height()));
}

void MainWindow::lidar_show_clicked(){
    if(show_lidar_){
        // 已经显示雷达信息，此时转换为不显示
        ui.lidar_show_widget->hide();
        ui.lidar_show_btn->setText("显示图像");
    }
    else{
        ui.lidar_show_widget->show();
        ui.lidar_show_btn->setText("隐藏图像");
    }
    show_lidar_ = !show_lidar_;
}

void MainWindow::camera_show_clicked(){
    if(show_camera_){
        ui.camera_show_widget->hide();
        ui.camera_image->clear();
        ui.camera_show_btn->setText("显示图像");
    }
    else{
        ui.camera_show_widget->show();
        ui.camera_show_btn->setText("隐藏图像");
    }
    show_camera_ = !show_camera_;
}

void MainWindow::robotConnect_clicked()
{
    QString robotIP=ui.robotIP->text();
    QString robotPort=ui.robotPort->text();
    ScriptController::connectScripts(robotIP,robotPort);
    ROS_INFO_STREAM("Robot connecting");
    std::string temp="连接到机器人"+robotIP.toStdString()+":"+robotPort.toStdString();
    ui.infoPrint->append(QString::fromStdString(temp));
    ui.robot_state->setText("已连接");
    
    //TODO
    ui.robotDisconnect->setEnabled(true);
    ui.robotConnect->setEnabled(false);
}

void MainWindow::robotDisconnect_clicked()
{
    ScriptController::disconnectScripts();
    ROS_INFO_STREAM("Robot disconnecting");
    ui.infoPrint->append("断开机器人的连接...");
    ui.robot_state->setText("未连接");
    //TODO
    ui.robotDisconnect->setEnabled(false);
    ui.robotConnect->setEnabled(true);
}

void MainWindow::clearInfo_clicked()
{
    ROS_INFO_STREAM("Info clearing");
    ui.infoPrint->clear();
}


void MainWindow::caliBtn_clicked()
{
    ROS_INFO_STREAM("Calibration started...");
    ui.infoPrint->append("开始标定...");
    //TODO
    ui.stopGraspBtn->setEnabled(true);
    ui.graspBtn->setEnabled(false); //按钮无法再控制
    ui.caliBtn->setEnabled(false);
    ui.handResetBtn->setEnabled(false);
}

void MainWindow::graspBtn_clicked()
{
    ScriptController::graspScripts();
    ROS_INFO_STREAM("Grasping started...");
    ui.infoPrint->append("开始抓取...");
    //TODO
    ui.stopGraspBtn->setEnabled(true);
    ui.graspBtn->setEnabled(false); //按钮无法再控制
    ui.caliBtn->setEnabled(false);
    ui.handResetBtn->setEnabled(false);
}

void MainWindow::stopGraspBtn_clicked()
{
    ScriptController::stopScripts();
    ROS_INFO_STREAM("Grasping stopped...");
    ui.infoPrint->append("运行终止...");
    //TODO
    ui.caliBtn->setEnabled(true);
    ui.graspBtn->setEnabled(true);
    ui.handResetBtn->setEnabled(true);
    ui.stopGraspBtn->setEnabled(false); //按钮无法再控制
}

void MainWindow::handResetBtn_clicked()
{
    ROS_INFO_STREAM("Restting robot hand...");
    ui.infoPrint->append("机械臂复位中...");
    //TODO
    ui.stopGraspBtn->setEnabled(true);
    ui.graspBtn->setEnabled(false); //按钮无法再控制
    ui.caliBtn->setEnabled(false);
    ui.handResetBtn->setEnabled(false);
}

void MainWindow::updateState(const rs_perception::state msg){

    QDateTime time=QDateTime::currentDateTime();
    QString str = time.toString("yyyy-MM-dd hh:mm:ss"); //2021/12/06  -  10:10:10
    ui.le_system_time->setText(str);
}

void MainWindow::Sleep(int msec)
{
    QTime dieTime = QTime::currentTime().addMSecs(msec);
    while( QTime::currentTime() < dieTime )
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

//在其上编写程序
}


