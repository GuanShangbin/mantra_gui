/**
 * @file /include/mantra_gui/main_window.hpp
 *
 * @brief Qt based gui for mantra_gui.
 *
 * @date November 2010
 **/
#ifndef mantra_gui_MAIN_WINDOW_H
#define mantra_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QWidget>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "rviz.hpp"
#include <rs_perception/state.h>
#include <string>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ebox_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    void showNoMasterMessage();
    void Sleep(int msec);

public Q_SLOTS:
    /******************************************
    ** Auto-connections (connectSlotsByName())
    *******************************************/
    void timerUpdate();
    void updateLoggingView_sub(); //add
    void updateLoggingView_sub2(); //add
    void slot_show_image(int frame_id, QImage image);//add 1105
    void updateState(const rs_perception::state);
    void lidar_show_clicked();
    void camera_show_clicked();
    void caliBtn_clicked();
    void graspBtn_clicked();
    void stopGraspBtn_clicked();
    void handResetBtn_clicked();
    void clearInfo_clicked();
    void robotConnect_clicked();
    void robotDisconnect_clicked();

private:
    Ui::MainWindowDesign ui;
    RvizPlugin* rviz_plugin_;
    QNode qnode;
    bool show_lidar_;
    bool show_camera_;
};

}  // namespace ebox_gui

#endif // ebox_gui_MAIN_WINDOW_H
