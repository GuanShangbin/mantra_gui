/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/mantra_gui/qnode.hpp"

#include <rs_perception/obstacle_ros_msg.h>
#include <rs_perception/obstacle_set_ros_msg.h>
#include <rs_perception/freespace_ros_msg.h>
#include <rs_perception/freespace_set_ros_msg.h>
#include <rs_perception/odometry_ros_msg.h>
//darknet yolo
#include <darknet_ros_msgs/ObjectCount.h>

#include <visualization_msgs/MarkerArray.h>
#include "../include/mantra_gui/script_controll.h"
/*****************************************************************************
** Namespaces
*****************************************************************************/

typedef struct{
    int sensor_num = 0;
    int count_lidar = 0;//默认只取最近的5个障碍物
    int type_lidar[5] = {0};
    float distance_lidar[5]= {0};
    float yaw_lidar[5] = {0};

    int count_camera = 0;//相机检测到的障碍物
    int type_camera[5] = {0};//障碍物类别
    float distance_camera[5] ={0};//障碍物距离
    float yaw_camera[5] ={0};//障碍物距离
}obstacle_distance;

static obstacle_distance msg_obstacle_distance;//接收障碍物检测结果

std::vector<int> sort_arry(std::vector<float> array, int& count){
    std::map<float, int> mp;
    for(int i = 0; i < count; ++i){
        mp[array[i]] = i;
    }
    count = mp.size();
    std::map<float, int>::iterator iter = mp.begin();
    // for(int i = 0; i< count;++i){
    //   array[i] = iter->first;
    //   iter++;
    // }
    std::vector<int> index;
    while(iter != mp.end()){
        index.push_back(iter->second);
        iter++;
    }
    //std::cout<< "index size " <<index.size()<<std::endl;
    return index;
}

namespace ebox_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"ebox_gui");
    ROS_INFO_STREAM("ros node init");

    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.

    laser_subscriber = n.subscribe("rs_obstacle", 1, &QNode::lidarCallback, this); //add
    camera_subscriber = n.subscribe("/darknet_ros/found_object", 1, &QNode::cameraCallback, this); //add
    start();
    return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"ebox_gui");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    // Add your ros communications here.

    obstacle_subscriber = n.subscribe("/rs_obstacle_rviz", 10, &QNode::obstacleCallback, this);
    filtedObs_publisher = n.advertise<visualization_msgs::MarkerArray&>("obstacle_cubes", 1);
    laser_subscriber = n.subscribe("rs_obstacle", 1, &QNode::lidarCallback, this); //add
    camera_subscriber = n.subscribe("/darknet_ros/found_object", 1, &QNode::cameraCallback, this); //add
    state_subscriber = n.subscribe("/state_msg", 10, &QNode::stateCallback, this);

    image_transport::ImageTransport it_(n);     //使用image_transport获取ROS中的image类型数据
    //    it_.subscribe("/darknet_ros/detection_image",1, &QNode::imageCallback, this);
    //    n.subscribe("/darknet_ros/detection_image",100, &QNode::imageCallback, this);       //以上3句测试用，不需要管
    detection_image_subscriber =n.subscribe("/camera/color/image_raw",1, &QNode::imageCallback, this); 
    // depth_image_subscriber =n.subscribe("/camera/aligned_depth_to_color/image_raw",2, &QNode::depthCallback, this);       //以上3句测试用，不需要管
    start();
    return true;
}

//void QNode::imageCallback(const sensor_msgs::ImageConstPtr &msg){       //测试用，不要管
//    ROS_INFO("接收到图片了！");
//}

void QNode::run() {
    ros::Rate loop_rate(1);
    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    ScriptController::endScripts();
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::obstacleCallback(visualization_msgs::MarkerArray marker_array){
    visualization_msgs::MarkerArray *newMarkerArray = new visualization_msgs::MarkerArray();
    for(int i=0; i<marker_array.markers.size(); i++){
        //printf("%d\n",marker_array.markers[i].type);
        //只需要中心距离原点小于30m的marker
        // ns类型：cube/box外接盒，box_info外接盒信息，polygon外接多边形，
        //        velocity_dir速度箭头，acc_dir加速度箭头，track_info跟踪信息，label_info标签信息
        if(marker_array.markers[i].ns.compare("cube")==0){      //外接盒
            float x = marker_array.markers[i].pose.position.x;
            float y = marker_array.markers[i].pose.position.y;
            if((x*x+y*y)<=900)
                newMarkerArray->markers.push_back(marker_array.markers[i]);
        }
        else if(marker_array.markers[i].ns.compare("label_info")==0){   //标签信息
            float x = marker_array.markers[i].pose.position.x;
            float y = marker_array.markers[i].pose.position.y;
            if((x*x+y*y)<=900){
                //根据概率判断加不加问号
                float prob = 0;
                std::stringstream ss;
                int pos;
                if((pos = marker_array.markers[i].text.find(">>")) > -1){   //字符串流式转换为数字
                    pos = pos + 2;
                    ss << marker_array.markers[i].text.substr(pos);
                    ss >> prob;
                }

                if(marker_array.markers[i].text.find("ped") != std::string::npos){
                    marker_array.markers[i].text = "Person";
                }
                else if(marker_array.markers[i].text.find("bike") != std::string::npos){
                    marker_array.markers[i].text = "Bike";
                }
                else if(marker_array.markers[i].text.find("car") != std::string::npos){
                    marker_array.markers[i].text = "Car";
                    marker_array.markers[i].color.r = 1.0;
                    marker_array.markers[i].color.g = 0.3;
                    marker_array.markers[i].color.b = 0.3;
                }
                else if(marker_array.markers[i].text.find("truck") != std::string::npos){
                    marker_array.markers[i].text = "Truck";
                    marker_array.markers[i].color.r = 0.3;
                    marker_array.markers[i].color.g = 0.3;
                    marker_array.markers[i].color.b = 1.0;
                }
                else{
                    marker_array.markers[i].text = "Unknow";
                }

                if(prob<0.6)    //根据概率判断加不加问号
                    marker_array.markers[i].text = marker_array.markers[i].text + "?";

                marker_array.markers[i].scale.z *= 2;   //字体放大
                newMarkerArray->markers.push_back(marker_array.markers[i]);
            }
        }
    }
    filtedObs_publisher.publish(*newMarkerArray);
}


void QNode::lidarCallback(const rs_perception::obstacle_set_ros_msg& msg)
{
    //ROS_INFO("lidar+++++++++++++++++++++++==");

    logging_model_sub.removeRows(0, logging_model_sub.rowCount());

    if(msg.obstcles.size() == 0){
        ROS_INFO("no obstacle detected!!!\n");
        return;
    }
    //printf("msg.obstcles.size() =%d\n", msg.obstcles.size());

    //printf("show pointcloud2");
    // Q_EMIT lidarPanelUpdated();

    // 0 means unknown, 1 means pedestrain, 2 means bicycle, 3 means car, 4 means truck/bus, 5 means very huge long cargo truck
    //设置最多只显示处理最近的五个障碍物
    // int count = 0;
    int msgsize = msg.obstcles.size();

    std::vector<int> type_tmp;
    std::vector<float> distance_tmp;
    std::vector<float> yaw_tmp;

    msg_obstacle_distance.count_lidar = 0;
    for (size_t i = 0, j = 0; j < msgsize; i++, j++) //忽略0 mmeans unknown
    {
        while (msg.obstcles[j].type == 0)
        {
            ++j;
        }
        type_tmp.push_back( msg.obstcles[j].type );
        distance_tmp.push_back( msg.obstcles[j].distance );
        yaw_tmp.push_back( msg.obstcles[j].yaw );
        //printf("recive data :  type:[%d] distance:[%.2f] yaw:[%.2f]\n",msg.obstcles[j].type, msg.obstcles[j].distance, msg.obstcles[j].yaw);
        ++msg_obstacle_distance.count_lidar;
    }

    std::vector<int> index;
    index = sort_arry(distance_tmp, msg_obstacle_distance.count_lidar);
    msg_obstacle_distance.count_lidar = index.size() > 5 ? 5: index.size();


    std::stringstream logging_model_msg;
    QString angle_sign=u8"°";

    std::map<int,QString> map_type = {
        {1,"person"},
        {2,"bicycle"},
        {3,"car"},
        {4,"truck/bus"},
        {5,"huge truck"}
    };

    QStringList strList;


    for (size_t i = 0; i < 5 && i < index.size(); i++)
    {
        msg_obstacle_distance.type_lidar[i] = type_tmp[index[i]];
        msg_obstacle_distance.distance_lidar[i] = distance_tmp[index[i]];
        msg_obstacle_distance.yaw_lidar[i] = yaw_tmp[index[i]];
        //printf("zuijin data :num:[%d]  type:[%d] distance:[%.2f] yaw:[%.2f]\n", msg_obstacle_distance.count_lidar, msg_obstacle_distance.type_lidar[i], msg_obstacle_distance.distance_lidar[i], msg_obstacle_distance.yaw_lidar[i]);
    }

    //      激光雷达障碍物检测列表（数目、类型、距离、角度）
    strList.append("latest detect:" + QString::number(msg_obstacle_distance.count_lidar));
    for(int i = 0 ; i < msg_obstacle_distance.count_lidar; ++i){
        strList.append(map_type[msg_obstacle_distance.type_lidar[i]] + ":" + \
                QString::number(msg_obstacle_distance.distance_lidar[i],'f', 6) + "m, " + \
                QString::number(57.3*msg_obstacle_distance.yaw_lidar[i], 'f', 6) + "°");//float,小数1位
    }

    int nCount = strList.size();
    for(int i = 0; i < nCount; i++)
    {
        logging_model_sub.insertRows(logging_model_sub.rowCount(),1);
        QString string = static_cast<QString>(strList.at(i));
        QVariant new_row(string);
        logging_model_sub.setData(logging_model_sub.index(logging_model_sub.rowCount()-1),new_row);
        Q_EMIT loggingUpdated_sub(); // used to readjust the scrollbar
    }

}

void QNode::cameraCallback(const  darknet_ros_msgs::ObjectCount& msg)
{
    ROS_INFO("camera+++++++++++++++++++++++==");

    logging_model_sub2.removeRows(0, logging_model_sub2.rowCount());

    if(msg.count <= 0){
        printf("camera no detect object!!!");
        return;
    }

    printf("camera msg.obstcles.size() =%d\n", msg.count);
    msg_obstacle_distance.count_camera = msg.count;
    for(size_t i = 0; i< msg_obstacle_distance.count_camera; ++i){
        msg_obstacle_distance.type_camera[i] = msg.type[i];
        msg_obstacle_distance.distance_camera[i] = msg.distance[i];
        msg_obstacle_distance.yaw_camera[i] = msg.yaw[i];
        printf("camera :  type:[%d] distance:[%.2f] yaw:[%.2f]\n",msg_obstacle_distance.type_camera[i], msg_obstacle_distance.distance_camera[i], msg_obstacle_distance.yaw_camera[i]);
    }

    std::stringstream logging_model_msg;
    QString angle_sign=u8"°";

    std::map<int,QString> map_type = {
        {1,"person"},
        {2,"bicycle"},
        {3,"car"},
        {4,"motorcycle"},
        {6,"bus"},
        {8,"truck"},
        {14,"bench"},
        {16,"cat"},
        {17,"dog"},
        {57,"chair"}
    };

    QStringList strList;

    //    相机障碍物检测列表（数目、类型、距离、角度）
    strList.append("detect num:" + QString::number(msg_obstacle_distance.count_camera));

    for(int i = 0 ; i < msg_obstacle_distance.count_camera; ++i){
        strList.append(map_type[msg_obstacle_distance.type_camera[i]] + ":" + \
                QString::number(msg_obstacle_distance.distance_camera[i],'f', 6) + "m, " + \
                QString::number(msg_obstacle_distance.yaw_camera[i], 'f', 5) + "°");
    }

    int nCount = strList.size();
    for(int i = 0; i < nCount; i++)
    {
        logging_model_sub2.insertRows(logging_model_sub2.rowCount(),1);
        QString string = static_cast<QString>(strList.at(i));
        QVariant new_row(string);
        logging_model_sub2.setData(logging_model_sub2.index(logging_model_sub2.rowCount()-1),new_row);
        Q_EMIT loggingUpdated_sub2(); // used to readjust the scrollbar
    }
}

void QNode::stateCallback(const rs_perception::state& msg)
{
    ROS_INFO("state update+++++++++++++++++++++++==");
    Q_EMIT Show_state(msg);
}

void QNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // ROS_INFO("image update+++++++++++++++++++++++==");
    cv_bridge::CvImagePtr cv_ptr;

     try
       {
         //深拷贝转换为opencv类型
         cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
         QImage im=Mat2QImage(cv_ptr->image);
         emit Show_camera_image(0,im);
       }
       catch (cv_bridge::Exception& e)
       {
        printf("error img transport\n");
         return;
       }

}
// void QNode::depthCallback(const sensor_msgs::ImageConstPtr& msg)
// {
//     // ROS_INFO("image update+++++++++++++++++++++++==");
//     cv_bridge::CvImagePtr cv_ptr;

//      try
//        {
//          //深拷贝转换为opencv类型
//          cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
//          QImage im=Mat2QImage(cv_ptr->image);
//          emit Show_camera_image(1,im);
//        }
//        catch (cv_bridge::Exception& e)
//        {
//         printf("error img transport\n");
//          return;
//        }

// }

 QImage QNode::Mat2QImage(cv::Mat const& src)
 {
   QImage dest(src.cols, src.rows, QImage::Format_ARGB32);

   const float scale = 255.0;

   if (src.depth() == CV_8U) {
     if (src.channels() == 1) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           int level = src.at<quint8>(i, j);
           dest.setPixel(j, i, qRgb(level, level, level));
         }
       }
     } else if (src.channels() == 3) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
           dest.setPixel(j, i, qRgb(bgr[0], bgr[1], bgr[2]));
         }
       }
     }
   } else if (src.depth() == CV_32F) {
     if (src.channels() == 1) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           int level = scale * src.at<float>(i, j);
           dest.setPixel(j, i, qRgb(level, level, level));
         }
       }
     } else if (src.channels() == 3) {
       for (int i = 0; i < src.rows; ++i) {
         for (int j = 0; j < src.cols; ++j) {
           cv::Vec3f bgr = scale * src.at<cv::Vec3f>(i, j);
           dest.setPixel(j, i, qRgb(bgr[0], bgr[1], bgr[2]));
         }
       }
     }
   }

   return dest;
 }

}  // namespace ebox_gui
