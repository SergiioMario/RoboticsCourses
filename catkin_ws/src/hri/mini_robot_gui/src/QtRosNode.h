#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt8MultiArray.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    std::vector<int> sensorDistances;
    float sensorLightL;
    float sensorLightR;
    float sensorTemp;
    float sensorBatt;
    std::vector<float> sensorAccelerometer;
    float accelMvnAvg;
    std::vector<float> accelMvnAvgQueue;
    std::vector<uchar> imgCompressed;
    
    float leftSpeed;
    float rightSpeed;

    ros::NodeHandle* n;
    ros::Subscriber subSensors;
    ros::Subscriber subCompressedImg;
    ros::Publisher  pubSpeeds;          
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void callbackSensors(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callbackCompressedImage(const std_msgs::UInt8MultiArray::ConstPtr& msg);
    
signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
