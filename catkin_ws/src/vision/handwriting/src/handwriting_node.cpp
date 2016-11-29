#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

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
    std::cout << "Dataset location: " << folder << std::endl;

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
    
    std::vector<int> img_display_counters;
    for(int i=0; i < 10; i++) img_display_counters.push_back(0);
    char cmd = 0;
    int digit = 0;
    
    while ((cmd = cv::waitKey(15)) != 27)
    {
        if(cmd >= 0x30 && cmd <= 0x39)
        {
            digit = cmd - 0x30;
            if(++img_display_counters[digit] >= 1000) img_display_counters[digit] = 0;
        }
        cv::imshow("Test", data[digit][img_display_counters[digit]]);
    }
    
    return 0;
}
