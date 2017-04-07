#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"

void callback_img(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    cv::imshow("Minirobot Image", cv::imdecode(msg->data, 1));
}

int main(int argc, char** argv)
{
    std::cout << "Initializing compressed image visualizer..." << std::endl;
    ros::init(argc, argv, "compres_img_viz");
    ros::NodeHandle n;
    ros::Subscriber subCompressed = n.subscribe("/minirobot/hardware/img_compressed", 1, callback_img);
    ros::Rate loop(30);

    while(ros::ok() && cv::waitKey(10)!= 27)
    {
        ros::spinOnce();
        loop.sleep();
    }
}
