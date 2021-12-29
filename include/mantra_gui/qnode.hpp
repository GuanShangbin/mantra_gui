/**
 * @file /include/mantra_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef mantra_gui_QNODE_HPP_
#define mantra_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QImage>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <rs_perception/obstacle_ros_msg.h>
#include <rs_perception/obstacle_set_ros_msg.h>
#include <rs_perception/freespace_ros_msg.h>
#include <rs_perception/freespace_set_ros_msg.h>
#include <rs_perception/odometry_ros_msg.h>
#include <rs_perception/state.h>
//darknet yolo
#include <darknet_ros_msgs/ObjectCount.h>
#include <visualization_msgs/MarkerArray.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

//#include <python2.7/Python.h>
//#include <opencv2/opencv.hpp>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ebox_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();

    /*********************
    ** Logging
    **********************/
    enum LogLevel {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    };

    QStringListModel* loggingModel_sub() { return &logging_model_sub; } //add
    QStringListModel* loggingModel_sub2() { return &logging_model_sub2; } //add
    void lidarCallback(const rs_perception::obstacle_set_ros_msg& msg);
    void cameraCallback(const darknet_ros_msgs::ObjectCount& msg);
    void obstacleCallback(visualization_msgs::MarkerArray marker_array);
    void stateCallback(const rs_perception::state& msg);

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);      //测试用，不要管它
    // void depthCallback(const sensor_msgs::ImageConstPtr& msg);

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();
    void loggingUpdated_sub();  //add
    void loggingUpdated_sub2();  //add
    void Show_camera_image(int,QImage);//add1105
    void Show_state(const rs_perception::state);

private:
    int init_argc;
    char** init_argv;

    QStringListModel logging_model;

    ros::Subscriber laser_subscriber;
    ros::Subscriber camera_subscriber;
    ros::Subscriber obstacle_subscriber;
    ros::Subscriber state_subscriber;
    ros::Subscriber detection_image_subscriber;
    // ros::Subscriber depth_image_subscriber;
    ros::Publisher filtedObs_publisher;
    QStringListModel logging_model_sub;
    QStringListModel logging_model_sub2;
    QImage Mat2QImage(cv::Mat const& src);

};

}  // namespace ebox_gui

#endif /* ebox_gui_QNODE_HPP_ */
