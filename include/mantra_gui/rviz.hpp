#ifndef RVIZ_HPP
#define RVIZ_HPP

#include <QWidget>

/********************
 * include of rviz
 *******************/
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/default_plugin/view_controllers/orbit_view_controller.h>
#include <rviz/view_manager.h>
#include <QVBoxLayout>

namespace ebox_gui {

class RvizPlugin: public QObject{
    Q_OBJECT

public:
    RvizPlugin(QVBoxLayout* ui);
    virtual ~RvizPlugin();
    void initLidarDisplay();

private:
    rviz::RenderPanel* render_panel_;// = new rviz::RenderPanel;
    rviz::VisualizationManager *manager_;// =  new rviz::VisualizationManager(pointCloud_panel);
    rviz::ViewManager* viewManager;
    QVBoxLayout* ui_;
};

}

#endif // RVIZ_HPP
