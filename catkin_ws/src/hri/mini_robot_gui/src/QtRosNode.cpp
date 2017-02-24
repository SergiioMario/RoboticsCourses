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
    for(int i=0; i < 30; i++) accelMvnAvgQueue.push_back(0);
    accelMvnAvg = 0;
    
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(20);
    subSensors = n->subscribe("/minirobot/hardware/sensors", 10, &QtRosNode::callbackSensors, this);
    pubSpeeds  = n->advertise<std_msgs::Float32MultiArray>("/minirobot/hardware/motor_speeds", 10);

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

void QtRosNode::callbackSensors(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if(msg->data.size() != 15)
    {
        std::cout << "QtRosNode.->Sensor message must be 15-data lenght!!" << std::endl;
        return;
    }
    for(int i=0; i < 8; i++)
        sensorDistances[i] = msg->data[i];
    for(int i=8; i < 11; i++)
        sensorAccelerometer[i-8] = msg->data[i];
    sensorLightL = msg->data[11];
    sensorLightR = msg->data[12];
    sensorTemp   = msg->data[13]*500.0/1024.0 - 50; //In TMP36 we have 10 mV/°C, with Vout = 750 mV for T=25°C
    sensorBatt   = msg->data[14]/102.4; //from the 10-bit ADC, 1024 = 10.0 V, since ADC in arduino is powered with 5V
                                        //and signal comes from a 1/2 voltage divisor

    accelMvnAvg = accelMvnAvgQueue[0];
    for(int i=1; i <accelMvnAvgQueue.size(); i++)
    {
        accelMvnAvgQueue[i-1] = accelMvnAvgQueue[i];
        accelMvnAvg += accelMvnAvgQueue[i];
    }
    float accel = sensorAccelerometer[0] * sensorAccelerometer[0];
    accel += sensorAccelerometer[1] * sensorAccelerometer[1];
    accel += sensorAccelerometer[2] * sensorAccelerometer[2];
    accel = sqrt(accel);
    accelMvnAvgQueue[accelMvnAvgQueue.size()-1] = accel;
    accelMvnAvg /= accelMvnAvgQueue.size();
        
}
