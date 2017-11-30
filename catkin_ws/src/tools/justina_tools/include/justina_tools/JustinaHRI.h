#pragma once
#include "ros/ros.h"
#include "std_msgs/String.h"

class JustinaHRI
{
private:
    static bool is_node_set;
public:
    static ros::Publisher pubFakeSprRecognized;
    static bool setNodeHandle(ros::NodeHandle* nh);
    static void fakeRecognizedSpeech(std::string sentence);
};
