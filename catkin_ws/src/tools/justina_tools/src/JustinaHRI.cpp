#include "justina_tools/JustinaHRI.h"

bool JustinaHRI::is_node_set = false;
ros::Publisher JustinaHRI::pubFakeSprRecognized;

bool JustinaHRI::setNodeHandle(ros::NodeHandle* nh)
{
    if(JustinaHRI::is_node_set)
	return true;
    if(nh == 0)
	return false;

    pubFakeSprRecognized = nh->advertise<std_msgs::String>("/hri/sp_rec/recognized", 1);
    is_node_set = true;
}

void JustinaHRI::fakeRecognizedSpeech(std::string sentence)
{
    std_msgs::String msg;
    msg.data = sentence;
    pubFakeSprRecognized.publish(msg);
}
