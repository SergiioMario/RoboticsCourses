#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
    sensorDistances.resize(8);
    for(int i=0; i< sensorDistances.size(); i++) sensorDistances[i] = 0;
    sensorLightL = 0;
    sensorLightR = 0;
    sensorTemp = 0;
    sensorBatt = 0;
    sensorAccelerometer.resize(3);
    sensorAccelerometer[0] = 0;
    sensorAccelerometer[1] = 0;
    sensorAccelerometer[2] = 0;
    leftSpeed  = 0;
    rightSpeed = 0;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(20);
    subDistanceSensors = n->subscribe("/minirobot/hardware/distance_sensors", 1, &QtRosNode::callbackDistanceSensors, this);
    subLightSensorL    = n->subscribe("/minirobot/hardware/light_sensor_left", 1, &QtRosNode::callbackLightSensorLeft, this);
    subLightSensorR    = n->subscribe("/minirobot/hardware/light_sensor_right", 1, &QtRosNode::callbackLightSensorRight, this);
    subTempSensor      = n->subscribe("/minirobot/hardware/temperature", 1, &QtRosNode::callbackTemperature, this);
    subBattSensor      = n->subscribe("/minirobot/hardware/battery", 1, &QtRosNode::callbackBattery, this);
    subAccelerometer   = n->subscribe("/minirobot/hardware/accelerometer", 1, &QtRosNode::callbackAccelerometer, this);
    pubSpeeds          = n->advertise<std_msgs::Int16MultiArray>("/minirobot/hardware/speeds", 1);

    std_msgs::Int16MultiArray msgSpeeds;
    msgSpeeds.data.push_back(0);
    msgSpeeds.data.push_back(0);
    bool isZeroSpeedSent = false;
  
    while(ros::ok() && !this->gui_closed)
    {
        //std::cout << "Ros node running..." << std::endl;
        msgSpeeds.data[0] = leftSpeed;
        msgSpeeds.data[1] = rightSpeed;
        if(leftSpeed == 0 && rightSpeed == 0)
        {
            if(!isZeroSpeedSent)
            {
                pubSpeeds.publish(msgSpeeds);
                isZeroSpeedSent = true;
            }
        }
        else
        {
            pubSpeeds.publish(msgSpeeds);
            isZeroSpeedSent = false;
        }
        emit updateGraphics();
        ros::spinOnce();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
    
}

void QtRosNode::callbackDistanceSensors(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 8)
    {
        std::cout << "QtRosNode.->Sensor distance message must be 8-data lenght!!" << std::endl;
        return;
    }
    for(int i=0; i < msg->data.size(); i++)
        sensorDistances[i] = msg->data[i];
}

void QtRosNode::callbackLightSensorLeft(const std_msgs::Int16::ConstPtr& msg)
{
    sensorLightL = msg->data;
}

void QtRosNode::callbackLightSensorRight(const std_msgs::Int16::ConstPtr& msg)
{
    sensorLightR = msg->data;
}

void QtRosNode::callbackTemperature(const std_msgs::Int16::ConstPtr& msg)
{
    sensorTemp = msg->data;
}

void QtRosNode::callbackBattery(const std_msgs::Int16::ConstPtr& msg)
{
    sensorBatt = msg->data;
}

void QtRosNode::callbackAccelerometer(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    if(msg->data.size() !=3)
    {
        std::cout << "QtRosNode.->Accelerometer message must be 3-data lenght!!" << std::endl;
        return;
    }
    for(int i=0; i < msg->data.size(); i++)
        sensorAccelerometer[i] = msg->data[i];
}
