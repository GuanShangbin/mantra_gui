#include "../include/mantra_gui/script_controll.h"
#include<string>

namespace ebox_gui {


ScriptController::ScriptController(){}

void ScriptController::startScripts(){
    //启动程序所需的脚本
    // char* result=new char[1024];
    // executeCMD("roslaunch mantra_description display_gui.launch",result);
    system("gnome-terminal -x bash -c 'roslaunch mantra_description display_gui.launch'&");
   // system("gnome-terminal -x bash -c 'roscore'&");
    //system("gnome-terminal -x bash -c 'nohup roscore'&"); //加上nohup的命令可以在关闭终端后继续执行
    //system("gnome-terminal -x bash -c 'cd src/ebox_gui/py; nohup python pyTranslator.py'&");           //从Qt运行的时候用这行
    //system("gnome-terminal -x bash -c 'cd ../../../src/ebox_gui/py;nohup python pyTranslator.py'&");       //从控制台运行可执行文件时用这行
    //system("gnome-terminal -x bash -c 'killall gnome-terminal-server'&");
}

void ScriptController::endScripts(){
    //结束程序时调用
    system("gnome-terminal -x bash -c 'rosnode kill --all'&");
    system("gnome-terminal -x bash -c 'killall -9 rosmaster'&");
    
    //system("gnome-terminal -x bash -c 'killall python; killall roscore'&");
    //system("gnome-terminal -x bash -c 'killall gnome-terminal-server'&");
}

void ScriptController::cameraScripts(){
    //启动相机时调用
}

void ScriptController::graspScripts(){
    system("gnome-terminal -x bash -c 'roslaunch realsense2_viewer realsense2_viewer.launch'&");
    // system("gnome-terminal -x bash -c 'roslaunch gpd_ros mantra.launch'&");
}

void ScriptController::stopScripts(){
    system("gnome-terminal -x bash -c 'rosnode list|grep camera|xargs rosnode kill'&");
    
    // system("gnome-terminal -x bash -c 'roslaunch gpd_ros mantra.launch'&");
}

void ScriptController::connectScripts(QString robotIP,QString robotPort){
    std::string cmd="gnome-terminal -x bash -c 'roslaunch mantra_application mantra_bringup_config.launch robot_ip:="+robotIP.toStdString()+" robot_port:="+robotPort.toStdString()+"'&";
    system(cmd.c_str());
    
    // system("gnome-terminal -x bash -c 'roslaunch gpd_ros mantra.launch'&");
}

void ScriptController::disconnectScripts(){
    system("gnome-terminal -x bash -c 'rosnode list|grep move_group|xargs rosnode kill'&");
    system("gnome-terminal -x bash -c 'rosnode list|grep mantra|xargs rosnode kill'&");
    // system("gnome-terminal -x bash -c 'roslaunch gpd_ros mantra.launch'&");
}

ScriptController::~ScriptController(){}

void ScriptController::executeCMD(const char *cmd, char *result)
{
    char buf_ps[1024];
    char ps[1024]={0};
    FILE *ptr;
    strcpy(ps, cmd);
    if((ptr=popen(ps, "r"))!=NULL)
    {
        while(fgets(buf_ps, 1024, ptr)!=NULL)
        {
//	       可以通过这行来获取shell命令行中的每一行的输出
//	   	   printf("%s", buf_ps);
           strcat(result, buf_ps);
           if(strlen(result)>1024)
               break;
        }
        pclose(ptr);
        ptr = NULL;
    }
    else
    {
        printf("popen %s error\n", ps);
    }
}
}
