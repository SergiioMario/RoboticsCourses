#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"

int rectXMin = 0;
int rectYMin = 0;
int rectXMax = 1;
int rectYMax = 1;

void callback_click(int event, int x, int y, int flags, void* userdata)
{
    if(event == cv::EVENT_LBUTTONDOWN)
    {
        rectXMin = (x - 5) >= 0 ? x - 5 : 0;
	rectYMin = (y - 5) >= 0 ? y - 5 : 0;
	rectXMax = (x + 5) < 640 ? x + 5 : 639;
	rectYMax = (y + 5) < 480 ? y + 5 : 479;
    }
}

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING KINECT MANAGER ..." << std::endl;
    ros::init(argc, argv, "kinect_man");
    ros::NodeHandle n;

    std::cout << "Kinect Manager.-> Triying to initialize kinect sensor... " << std::endl;
    cv::VideoCapture capture(CV_CAP_OPENNI);
    if(!capture.isOpened())
    {
        std::cout << "Kinect Manager.->Cannot open kinect :'(" << std::endl;
        return 1;
    }
    capture.set(CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION, CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION_ON);
    std::cout << "Kinect Manager.-> Kinect sensor started :D" << std::endl;
    std::cout << "Kinect Manager.-> SYSTEM READY (I think so)" << std::endl;

    cv::namedWindow("KINECT TEST");
    cv::setMouseCallback("KINECT TEST", callback_click, NULL);

    while(ros::ok() && cv::waitKey(15) != 27)
    {
        cv::Mat depthMap;
        cv::Mat bgrImage;
        capture.grab();
        capture.retrieve(depthMap, CV_CAP_OPENNI_POINT_CLOUD_MAP);
        capture.retrieve(bgrImage, CV_CAP_OPENNI_BGR_IMAGE);
	int b=0, g=0, r=0;
	int counter = 0;
	for(int i=rectXMin; i <= rectXMax; i++)
	    for(int j=rectYMin; j <= rectYMax; j++)
	    {
	      b += bgrImage.data[j*bgrImage.step + i*bgrImage.elemSize()];
	      g += bgrImage.data[j*bgrImage.step + i*bgrImage.elemSize() + 1];
	      r += bgrImage.data[j*bgrImage.step + i*bgrImage.elemSize() + 2];
	      counter ++;
	    }
	std::cout << "Mean BGR: B=" << b/counter << "  G=" << g/counter << "  R=" << r/counter << std::endl;

	cv::rectangle(bgrImage, cv::Point(rectXMin, rectYMin), cv::Point(rectXMax, rectYMax), cv::Scalar(0, 255, 0));
        cv::imshow("KINECT TEST", bgrImage);
    }
}
