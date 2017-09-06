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

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    ros::NodeHandle* n;
    ros::ServiceClient cltGetMap;
    ros::ServiceClient cltCalculatePath;
    tf::TransformListener* tf_listener;
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void getRobotPose(float& robot_x, float& robot_y, float& robot_theta);
    void planPath(float start_x, float start_y, float goal_x, float goal_y, nav_msgs::Path& path);

signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
