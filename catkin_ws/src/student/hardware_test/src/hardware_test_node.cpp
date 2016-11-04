#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/PointStamped.h"

int main(int argc, char** argv)
{
    std::cout << "Initializing hardware test " << std::endl;
    ros::init(argc, argv, "hardware_test");
    ros::NodeHandle n;
    ros::Publisher pubHeadPose = n.advertise<std_msgs::Float32MultiArray>("/hardware/head/goal_pose", 1);
    ros::Publisher pubPoint = n.advertise<geometry_msgs::PointStamped>("/drawing_test", 1);
    std_msgs::Float32MultiArray msgHeadPose;
    geometry_msgs::PointStamped msgPoint;
    ros::Rate loop(10);

    msgHeadPose.data.push_back(0);
    msgHeadPose.data.push_back(0);
    msgPoint.header.frame_id = "map";
    float t = 0;
    while(ros::ok())
    {
        msgHeadPose.data[0] = sin(t);
        msgHeadPose.data[1] = cos(t);
        msgPoint.point.x = sin(t);
        msgPoint.point.y = cos(t);
        msgPoint.point.z = 0;
        pubHeadPose.publish(msgHeadPose);
        pubPoint.publish(msgPoint);
        ros::spinOnce();
        t += 0.1;
        loop.sleep();
    }
    return 0;
}
