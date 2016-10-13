#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "tf/transform_listener.h"
#include "Controls.h"

int main(int argc, char** argv)
{
    float goalX;
    float goalY;
    for(int i=0; i < argc; i++)
    {
        std::string s(argv[i]);
        if(s.compare("-x") == 0)
        {
            std::stringstream ss(argv[++i]);
            if(!(ss >> goalX))
            {
                std::cout << "Cannot parse X :'(" << std::endl;
                return -1;
            }
        }
        if(s.compare("-y") == 0)
        {
            std::stringstream ss(argv[++i]);
            if(!(ss >> goalY))
            {
                std::cout << "Cannot parse Y :'(" << std::endl;
                return -1;
            }
        }
    }
    std::cout<<"INITIALIZING LOW LEVEL CONTROL NODE BY MARCOSOFT..."<< std::endl;
    ros::init(argc, argv, "low_level_control");
    ros::NodeHandle n;
    ros::Publisher pubSp = n.advertise<std_msgs::Float32MultiArray>(
        "/hardware/mobile_base/speeds", 1);
    std::cout << "Goal: " << goalX << "  " << goalY << std::endl;
    
    ros::Rate loop(10);
    tf::TransformListener tf;
    tf::StampedTransform t;
    tf.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
    float robotX, robotY, robotTheta;
    std_msgs::Float32MultiArray msgSpeeds;
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
        std::cout<< "RobotPos: "<<robotX<<"  "<<robotY << "  " << robotTheta << std::endl;
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
