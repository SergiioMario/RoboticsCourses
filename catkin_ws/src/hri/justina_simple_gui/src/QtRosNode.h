#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "navig_msgs/CalculatePath.h"

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
