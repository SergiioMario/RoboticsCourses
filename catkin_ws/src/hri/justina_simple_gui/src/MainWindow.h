#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QMainWindow>
#include <ros/package.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include "QtRosNode.h"
#include "justina_tools/JustinaNavigation.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QtRosNode* qtRosNode;
    float robotX;
    float robotY;
    float robotTheta;
    float headPan;
    float headTilt;
    std::vector<float> leftArmPoses;
    std::vector<float> leftArmTorques;
    std::vector<float> rightArmPoses;
    std::vector<float> rightArmTorques;
    bool laIgnoreValueChanged;
    bool raIgnoreValueChanged;
    int laLastRadioButton;
    int raLastRadioButton;
    nav_msgs::Path calculatedPath;
  
    void setRosNode(QtRosNode* qtRosNode);
    void closeEvent(QCloseEvent *event);
    void setPathKnownLoc(const std::string pathKnownLoc);

private:
    bool strToFloatArray(std::string str, std::vector<float>& result);

public slots:
    //Slots for signals emitted in this window (e.g.: pressing buttons)
;
    //Navigation
    void navBtnCalcPath_pressed();
    void navBtnExecPath_pressed();
    void navMoveChanged();

    //Hardware
    void hdPanTiltChanged(double d);
    void laAnglesChanged(double);
    void raAnglesChanged(double);
    void laValuesChanged();
    void raValuesChanged();
    void laOpenGripperChanged(double d);
    void raOpenGripperChanged(double d);
    void laCloseGripperChanged(double d);
    void raCloseGripperChanged(double d);
    void laRadioButtonClicked();
    void raRadioButtonClicked();
    void torsoPoseChanged(double d);
    void torsoLocChanged();
    //Speech synthesis and recog
    void spgSayChanged();
    void sprFakeRecognizedChanged();
    //Vision
    
    //HRI

    //Slots for signals emitted in the QtRosNode (e.g. a topic is received)
    void updateGraphicsReceived();
    
private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
