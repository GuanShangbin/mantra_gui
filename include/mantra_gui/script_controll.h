#ifndef SCRIPT_CONTROLL_H
#define SCRIPT_CONTROLL_H

#include <QProcess>
#include <string>

namespace ebox_gui {

class ScriptController : public QObject{
    Q_OBJECT

public:
    ScriptController();
    virtual ~ScriptController();
    static void startScripts();
    static void endScripts();
    static void cameraScripts();
    static void graspScripts();
    static void stopScripts();
    static void connectScripts(QString robotIP,QString robotPort);
    static void disconnectScripts();
    static void executeCMD(const char *cmd, char *result);

};

}

#endif // SCRIPT_CONTROLL_H
