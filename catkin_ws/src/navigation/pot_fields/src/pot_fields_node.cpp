#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_listener.h"
#include "std_msgs/Float32MultiArray.h"
//Aguilar Marquez joseluis.amarquez@gmail.com
float robotX;
float robotY;
float robotTheta = 0;
float rejectionX;
float rejectionY;
float attractionX = 0;
float attractionY = 0;
float goalX = 0;
float goalY = 0;

std::vector<float> calculate_rejection(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    float d0 = 0.8;
    float eta = 5.0;
    
    std::cout << msg->ranges.size() <<" "<< msg->angle_min <<" "<< msg->angle_increment << std::endl;
    float sumX = 0;
    float sumY = 0;
    std::vector<float> result;
    result.push_back(0);
    result.push_back(0);
    if(msg->ranges.size() < 1)
    {
        std::cout << "Too few readings..." << std::endl;
        return result;
    }
    for(int i=0;i<msg->ranges.size();i++)
    {
        float dist = msg->ranges[i];
        float angle = msg->angle_min + msg->angle_increment * i;
        float mag= dist<d0? sqrt(1.0/dist - 1.0/ d0) * eta : 0;
        float rej_x = -cos(angle + robotTheta) * mag;
        float rej_y = -sin(angle + robotTheta) * mag;
        sumX += rej_x;
        sumY += rej_y;
    }
    sumX/= msg->ranges.size();
    sumY/= msg->ranges.size();

    result[0] = sumX;
    result[1] = sumY;
    std::cout << "X: " << sumX << "  Y: " << sumY << std::endl;
    rejectionX = sumX;
    rejectionY = sumY;
    return result;
}

void callbackscan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::cout << "Received laser scan... " << std::endl;
    
    calculate_rejection(msg);
}

void callbackGoalPose(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    float goalX = msg->data[0];
    float goalY = msg->data[1];
    //float gamma = 5.0;
    //float mag = sqrt((msg->data[0] - robotX)*(msg->data[0] - robotX) + (msg->data[1] - robotY)*(msg->data[1] - robotY));
    //float attractionX = gamma*(msg->data[0] - robotX)/mag;
    //float attractionY = gamma*(msg->data[1] - robotY)/mag;
}

void calculate_attraction()
{
    float gamma = 1.0;
    float mag = sqrt((goalX - robotX)*(goalX - robotX) + (goalY - robotY)*(goalY - robotY));
    attractionX = (goalX - robotX)/mag*gamma;
    attractionY = (goalY - robotY)/mag*gamma;
}

int main(int argc, char** argv)
{ 
    std::cout << "Initializing potential fields... " << std::endl;
    ros::init (argc,argv,"pot_fils");
    ros::NodeHandle n;
    ros::Subscriber subScan = n.subscribe("/hardware/scan",1,callbackscan);
    ros::Subscriber subGoalPose = n.subscribe("/navigation/goal_pose", 1, callbackGoalPose);
    ros::Publisher pubRej = n.advertise<visualization_msgs::Marker>("/hri/visualization_marker", 3);
    ros::Publisher pubSpeeds = n.advertise<std_msgs::Float32MultiArray>("/hardware/mobile_base/speeds", 1);
    visualization_msgs::Marker msgRej;
    geometry_msgs::Point startRej;
    geometry_msgs::Point endRej;
    endRej.x = 1.0;
    endRej.y = 1.0;
    msgRej.header.frame_id = "map";
    msgRej.type = visualization_msgs::Marker::ARROW;
    msgRej.ns = "pot_fields";
    msgRej.id = 0;
    msgRej.action = visualization_msgs::Marker::ADD;
    msgRej.lifetime = ros::Duration();
    msgRej.points.push_back(startRej);
    msgRej.points.push_back(endRej);
    msgRej.color.r = 1.0;
    msgRej.color.g = 0.0;
    msgRej.color.b = 0.0;
    msgRej.color.a = 1.0;
    msgRej.scale.x = 0.05;
    msgRej.scale.y = 0.05;
    msgRej.scale.z = 0.00;
    visualization_msgs::Marker msgAtt = msgRej;
    visualization_msgs::Marker msgRes = msgRej;
    msgAtt.color.r = 0.0;
    msgAtt.color.g = 1.0;
    msgAtt.id = 1;
    msgRes.color.r = 0.0;
    msgRes.color.b = 1.0;
    msgRes.id = 2;

    tf::TransformListener tf;
    tf::StampedTransform t;
    tf.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(5.0));
    std_msgs::Float32MultiArray msgSpeeds;
    msgSpeeds.data.push_back(0);
    msgSpeeds.data.push_back(0);
    
    ros::Rate loop(10);
    while(ros::ok())
    {
        tf.lookupTransform("map", "base_link", ros::Time(0), t);
        robotX = t.getOrigin().x();
        robotY = t.getOrigin().y();
        tf::Quaternion q = t.getRotation();
        robotTheta = atan2((float)q.z(), (float)q.w()) * 2;

        calculate_attraction();
        msgRej.points[0].x = robotX;
        msgRej.points[0].y = robotY;
        msgRej.points[1].x = robotX + rejectionX;
        msgRej.points[1].y = robotY + rejectionY;
        msgAtt.points[0].x = robotX;
        msgAtt.points[0].y = robotY;
        msgAtt.points[1].x = robotX + attractionX;
        msgAtt.points[1].y = robotY + attractionY;
        msgRes.points[0].x = robotX;
        msgRes.points[0].y = robotY;
        msgRes.points[1].x = robotX + attractionX + rejectionX;
        msgRes.points[1].y = robotY + attractionY + rejectionY;

        msgSpeeds.data[0] = 0.0;
        msgSpeeds.data[1] = 0.0;
        pubRej.publish(msgRej);
        pubRej.publish(msgAtt);
        pubRej.publish(msgRes);
        pubSpeeds.publish(msgSpeeds);
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
