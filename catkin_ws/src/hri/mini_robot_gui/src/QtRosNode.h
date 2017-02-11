#pragma once
#include <iostream>
#include <cmath>
#include <QThread>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"

class QtRosNode : public QThread
{
Q_OBJECT
public:
    QtRosNode();
    ~QtRosNode();

    std::vector<int> sensorDistances;
    int sensorLightL;
    int sensorLightR;
    int sensorTemp;
    int sensorBatt;
    std::vector<int> sensorAccelerometer;
    int leftSpeed;
    int rightSpeed;

    ros::NodeHandle* n;
    ros::Subscriber subDistanceSensors;
    ros::Subscriber subLightSensorL;
    ros::Subscriber subLightSensorR;
    ros::Subscriber subTempSensor;
    ros::Subscriber subBattSensor;
    ros::Subscriber subAccelerometer;
    ros::Publisher  pubSpeeds;          
    bool gui_closed;
    
    void run();
    void setNodeHandle(ros::NodeHandle* nh);

    void callbackDistanceSensors(const std_msgs::Int16MultiArray::ConstPtr& msg);
    void callbackLightSensorLeft(const std_msgs::Int16::ConstPtr& msg);
    void callbackLightSensorRight(const std_msgs::Int16::ConstPtr& msg);
    void callbackTemperature(const std_msgs::Int16::ConstPtr& msg);
    void callbackBattery(const std_msgs::Int16::ConstPtr& msg);
    void callbackAccelerometer(const std_msgs::Int16MultiArray::ConstPtr& msg);

signals:
    void updateGraphics();
    void onRosNodeFinished();
    
};
