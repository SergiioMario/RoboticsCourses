#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Empty.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Path.h"
#include "Controls.h"

nav_msgs::Path goalPath;

void callback_path(const nav_msgs::Path::ConstPtr& msg)
{
    goalPath = *msg;
}

void callback_start(const std_msgs::Empty::ConstPtr& msg)
{
}

int main(int argc, char** argv)
{
    
    std::cout<<"INIT LOW LEVEL CTRL BY MARCOSOFT..."<< std::endl;
    ros::init(argc, argv, "low_level_control");
    ros::NodeHandle n;
    ros::Publisher pubSp = n.advertise<std_msgs::Float32MultiArray>(
        "/hardware/mobile_base/speeds", 1);
    ros::Subscriber subGoalPath = n.subscribe("/navigation/a_star",
                                              1, callback_path);
    ros::Subscriber subStart = n.subscribe("/navigation/start_path",
                                           1, callback_start);
    
    ros::Rate loop(10);
    tf::TransformListener tf;
    tf::StampedTransform t;
    tf.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
    float robotX, robotY, robotTheta;
    std_msgs::Float32MultiArray msgSpeeds;
    float goalX = 0;
    float goalY = 0;
    while(ros::ok())
    {
        tf.lookupTransform("map", "base_link", ros::Time(0), t);
        robotX = t.getOrigin().x();
        robotY = t.getOrigin().y();
        tf::Quaternion q = t.getRotation();
        robotTheta = atan2((float)q.z(), (float)q.w()) * 2;

        msgSpeeds.data = Controls::CalculateSpeeds(robotX, robotY, robotTheta,
                                                   goalX,  goalY, 1);
        pubSp.publish(msgSpeeds);
        
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
