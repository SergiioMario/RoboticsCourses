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
    sensorAccelerometer.resize(9);
    for(int i=0; i < 9; i++)
	sensorAccelerometer[i] = 0;
    leftSpeed  = 0;
    rightSpeed = 0;
    
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(30);
    subDistanceSensors = n->subscribe("/rotombot/hardware/distance_sensors", 10, &QtRosNode::callbackDistanceSensors, this);
    subArduinoSensors  = n->subscribe("/rotombot/hardware/arduino_sensors",  10, &QtRosNode::callbackArduinoSensors, this);
    subCompressedImg   = n->subscribe("/rotombot/hardware/img_compressed", 10, &QtRosNode::callbackCompressedImage, this);
    pubSpeeds  = n->advertise<std_msgs::Float32MultiArray>("/rotombot/hardware/motor_speeds", 10);

    std_msgs::Float32MultiArray msgSpeeds;
    msgSpeeds.data.push_back(0);
    msgSpeeds.data.push_back(0);
    int isZeroSpeedSent = 0;
  
    while(ros::ok() && !this->gui_closed)
    {
        //std::cout << "Ros node running..." << std::endl;
        msgSpeeds.data[0] = leftSpeed;
        msgSpeeds.data[1] = rightSpeed;
        if(leftSpeed == 0 && rightSpeed == 0)
        {
            if(isZeroSpeedSent > 0)
            {
                pubSpeeds.publish(msgSpeeds);
                isZeroSpeedSent--;
            }
        }
        else
        {
            pubSpeeds.publish(msgSpeeds);
            isZeroSpeedSent = 5;
        }
        
        ros::spinOnce();
        emit updateGraphics();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
    
}

void QtRosNode::callbackDistanceSensors(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 8)
    {
        std::cout << "QtRosNode.->Sensor message for distance sensors must be 8-data lenght!!" << std::endl;
        return;
    }
    for(int i=0; i < 8; i++)
        sensorDistances[i] = msg->data[i];
}

void QtRosNode::callbackArduinoSensors(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 13)
    {
        std::cout << "QtRosNode.->Sensor message for arduino sensors must be 13-data lenght!!" << std::endl;
        return;
    }
    sensorLightL = msg->data[0];
    sensorLightR = msg->data[1];
    sensorTemp   = msg->data[2]; //TMP36 gives 10 mV/°C with 750 mV at 25°C. Check Arduino code
    sensorBatt   = msg->data[3]; //Battery is connected to a 1/2 voltage divider. Check arduino code

    for(int i=0; i < 9; i++)
	sensorAccelerometer[i] = msg->data[i + 4];
        
}

void QtRosNode::callbackCompressedImage(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    imgCompressed = msg->data;
    //std::cout << "camera img received" << std::endl;
}
