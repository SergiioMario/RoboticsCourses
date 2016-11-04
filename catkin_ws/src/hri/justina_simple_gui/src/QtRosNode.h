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
    ros::ServiceClient cltCalculatePath;
    ros::ServiceClient cltGetMap;
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    bool calculatePath(float startX, float startY, float goalX, float goalY);
    bool calculatePath(float startX, float startY, std::string goal_loc);
    bool calculatePath(std::string start_loc, float goalX, float goalY);
    bool calculatePath(std::string start_loc, std::string goal_loc);
    void startGetClose(float goalX, float goalY, float goalTheta);
    void startGetClose(float goalX, float goalY);
    void startGetClose(std::string goal_location);
    void startMoveLateral(float dist);
    void startMoveDistAngle(float dist, float angle);
    void getRobotPose(float& robotX, float& robotY, float& robotTheta);

signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
