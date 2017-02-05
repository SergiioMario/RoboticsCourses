#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "ros/ros.h"
#include "Perceptron.h"

int main(int argc, char** argv)
{
    std::string folder = "";
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("-f") == 0)
            folder = argv[++i];
    }
    
    std::cout << "INITIALIZING HANDWRITING RECOGNITION NODE..." << std::endl;
    ros::init(argc, argv, "handwriting");
    ros::NodeHandle n;
    std::cout << "Dataset location: " << folder << std::endl;

    //
    //LOAD TRAINING DATA FROM FILES
    //
    std::vector<std::vector<cv::Mat> > data; //THIS VECTOR OF VECTORS WILL CONTAIN ALL TRAINING IMAGES
    
    for(int i=0; i < 10; i++)
    {
        //Reading bytes from file. Each file contains data for 1000 28x28 bynary images.
        std::vector<char> bytes;
        bytes.resize(784000);
        std::stringstream filename;
        filename << folder << "/data" << i;
        std::cout << "Trying to read file: " << filename.str() << std::endl;
        std::ifstream ifs(filename.str().c_str(), std::ios::binary|std::ios::ate);
        std::streampos size = ifs.tellg();
        if(size != 784000)
        {
            ifs.close();
            std::cout << "Incorrect format in: " << filename << std::endl;
            return 1;
        }
        ifs.seekg(0, std::ios::beg);
        ifs.read(&bytes[0], size);
        ifs.close();
        //

        //Insert a new vector that will contain the new 1000 images
        std::vector<cv::Mat> data_i;
        data.push_back(data_i);
        for(int j=0; j < 1000; j++)
        {
            data[i].push_back(cv::Mat(28,28,CV_8UC1));
            for(int k = 0; k < 784; k++)
                data[i][j].data[k] = bytes[k + j*784];
        }
    }
    std::cout << "Read a total of " << data.size() << " files with " << data[0].size() << " images in each file (Y)" << std::endl;
    
    //
    //THIS IS THE ACTUAL TRAINING
    //
    Perceptron p(data[0][0].rows * data[0][0].cols);
    float tol = 0.001;
    float gradient_mag = tol + 1;
    int attempts = 0;
    float gradient_gain = 0.000005;
    float gradient_k = 0;
    int digit_to_train = 8;
    while(gradient_mag > tol && ++attempts < 300)
    {
        std::vector<float> last_w = p.w;
        float last_threshold = p.threshold;
        for(size_t i=0; i < 10; i++)
        {
            float training_y = i == digit_to_train ? 1.0 : 0;
            float estimated_y = 0;
            for(size_t j=0; j < 500; j++)
            {
                estimated_y = p.eval(data[i][j].data);
                float temp = (training_y - estimated_y)*(estimated_y - estimated_y*estimated_y);
                for(size_t k=0; k < p.w.size(); k++)
                {
                    gradient_k = temp * data[i][j].data[k];
                    p.w[k] += gradient_gain * gradient_k;
                }
                gradient_k = temp; //the (n+1)-th input corresponding to the threshold is always 1
                p.threshold += gradient_gain * gradient_k;
            }
        }
        gradient_mag = 0;
        for(size_t i=0; i < p.w.size(); i++)
            gradient_mag += fabs(p.w[i] - last_w[i]);
        gradient_mag += fabs(p.threshold -last_threshold);
        std::cout << "Current gradient magnitud: " << gradient_mag << std::endl;
    }
    std::cout << "Training finished after " << attempts << std::endl;

    
    //
    //THIS IS JUST FOR DISPLAYING IMAGES IN A NICE WAY
    //
    std::vector<int> img_display_counters;
    for(int i=0; i < 10; i++) img_display_counters.push_back(500);
    char cmd = 0;
    int digit = 0;
    
    while (ros::ok() && (cmd = cv::waitKey(15)) != 27)
    {
        if(cmd >= 0x30 && cmd <= 0x39)
        {
            digit = cmd - 0x30;
            if(++img_display_counters[digit] >= 1000) img_display_counters[digit] = 500;
            std::cout << "Output for this image: = "  << p.eval(data[digit][img_display_counters[digit]].data) << std::endl;
        }
        cv::imshow("Test", data[digit][img_display_counters[digit]]);
    }
    
    return 0;
}
