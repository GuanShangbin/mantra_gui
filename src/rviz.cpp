#include "../include/mantra_gui/rviz.hpp"

namespace ebox_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

RvizPlugin::RvizPlugin(QVBoxLayout* ui){
    ui_ = ui;
}

void RvizPlugin::initLidarDisplay(){
    //创建rviz的容器，并将该容器作为组建加入到ui内，其中关键class为VisualizationManager，是个管理类，起到控制创建rviz图层和设置坐标系的作用。
    render_panel_=new rviz::RenderPanel;
    ui_->addWidget(render_panel_);
    manager_=new rviz::VisualizationManager(render_panel_);
    render_panel_->initialize(manager_->getSceneManager(),manager_);

    manager_->initialize();
    manager_->startUpdate();

    // manager_->setFixedFrame("/camera_link");
    manager_->setFixedFrame("/base_link");
    
    rviz::Display *grid_ = manager_->createDisplay("rviz/Grid", "adjustable grid", true );
    ROS_ASSERT( grid_ != NULL );

    // Configure the GridDisplay the way we like it.
    grid_->subProp( "Line Style" )->setValue( "Billboards" );
    grid_->subProp( "Color" )->setValue( QColor( Qt::lightGray ) );
    grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue(0.02);
    grid_->subProp( "Cell Size" )->setValue(10);
    grid_->subProp("Plane Cell Count")->setValue(1000);

    //创建一个类型为rviz/MarkerArray的图层，用于接收topic为rs_obstacle_rviz的障碍物数据
    rviz::Display *map_=manager_->createDisplay("rviz/MarkerArray","obstacle",true);
    ROS_ASSERT(map_!=NULL);
    map_->subProp("Marker Topic")->setValue("/obstacle_cubes");
    map_->subProp("Queue Size")->setValue("1");
    //printf("%s\n", map_->subProp("Namespaces")->getValue().toString());         //Namespaces没有subprob，也没有description

    //添加一个Axes作为推土机的位置表示
    // rviz::Display *base_ = manager_->createDisplay("rviz/Axes", "base_link", true);
    // ROS_ASSERT(base_!=NULL);
    // base_->subProp("Reference Frame")->setValue("base_link");
    // base_->subProp("Length")->setValue(2);
    // base_->subProp("Radius")->setValue(0.6);

    //设置整个地图的展示方式，如视角、距离、偏航等
    viewManager = manager_->getViewManager();
    viewManager->setRenderPanel(render_panel_);
    viewManager->setCurrentViewControllerType("rviz/ThirdPersonFollower");
    viewManager->getCurrent()->subProp("Target Frame")->setValue("/camera");
    viewManager->getCurrent()->subProp("Near Clip Distance")->setValue("0.01");
    viewManager->getCurrent()->subProp("Focal Point")->setValue("0;1;0");
    viewManager->getCurrent()->subProp("Focal Shape Size")->setValue("0.05");
    viewManager->getCurrent()->subProp("Focal Shape Fixed Size")->setValue("true");
    viewManager->getCurrent()->subProp("Distance")->setValue("2");
    viewManager->getCurrent()->subProp("Yaw")->setValue("3.14");
    viewManager->getCurrent()->subProp("Pitch")->setValue("1.57");


    rviz::Display *camera_=manager_->createDisplay("rviz/PointCloud2","pointCloud2",true);
    ROS_ASSERT(map_!=NULL);
    camera_->subProp("Topic")->setValue("/camera/depth/color/points");
    camera_->subProp("Invert Rainbow")->setValue("true");
    camera_->subProp("Style")->setValue("Points");
    camera_->subProp("Size (Pixels)")->setValue("0.01");
    //map_->subProp("Decay Time")->setValue("0");

    camera_->subProp("Position Transformer")->setValue("XYZ");
    camera_->subProp("Color Transformer")->setValue("RGB8");
    camera_->subProp("Queue Size")->setValue("10");
    camera_->subProp("Alpha")->setValue("1");

    //        //这个是创建小车模型的图层，由urdf文件控制
    rviz::Display* robotModel_=manager_->createDisplay("rviz/RobotModel","Qrviz RobotModel",true);
    robotModel_->subProp( "Robot Description" )->setValue("robot_description");
    robotModel_->subProp( "Alpha" )->setValue(1);
    //        rviz::Display *car=manager_->createDisplay("rviz/RobotModel","adjustable robot",true);
    //        ROS_ASSERT(car!=NULL);
    //        car->subProp("Robot Description")->setValue("robot_description");
    //        manager_->startUpdate(); /camera/depth/color/points
}

// Destructor.
RvizPlugin::~RvizPlugin()
{
  delete manager_;
}

}
