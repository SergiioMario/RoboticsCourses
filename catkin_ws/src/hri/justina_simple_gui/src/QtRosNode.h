#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GetMap.h"
#include "navig_msgs/CalculatePath.h"
#include <ros/package.h>
#include "tf/transform_listener.h"
#include "justina_tools/JustinaHardware.h"
#include "justina_tools/JustinaNavigation.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
