#include "QtRosNode.h"

QtRosNode::QtRosNode()
{
    this->gui_closed = false;
}

QtRosNode::~QtRosNode()
{
}

void QtRosNode::run()
{    
    ros::Rate loop(10);
    this->cltCalculatePath = this->n->serviceClient<navig_msgs::CalculatePath>("/navigation/a_star");
    this->cltGetMap = this->n->serviceClient<nav_msgs::GetMap>("/navigation/static_map");
    this->pubRecogSpeech = this->n->advertise<std_msgs::String>("/recognizedSpeech", 1);
    while(ros::ok() && !this->gui_closed)
    {
        //std::cout << "Ros node running..." << std::endl;
        emit updateGraphics();
        ros::spinOnce();
        loop.sleep();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
    
}

bool QtRosNode::calculatePath(float startX, float startY, float goalX, float goalY)
{
    nav_msgs::GetMap srvGetMap;
    if(!this->cltGetMap.call(srvGetMap))
    {
        std::cout << "JustinaGUI.->Cannot get map :'(" << std::endl;
        return false;
    }
    
    navig_msgs::CalculatePath srv;
    srv.request.start_pose.position.x = startX;
    srv.request.start_pose.position.y = startY;
    srv.request.start_pose.position.z = 0;
    srv.request.goal_pose.position.x = goalX;
    srv.request.goal_pose.position.y = goalY;
    srv.request.goal_pose.position.z = 0;
    srv.request.map = srvGetMap.response.map;
    return this->cltCalculatePath.call(srv);
}

bool QtRosNode::calculatePath(float startX, float startY, std::string goal_loc)
{
}

bool QtRosNode::calculatePath(std::string start_loc, float goalX, float goalY)
{
}

bool QtRosNode::calculatePath(std::string start_loc, std::string goal_loc)
{
}

void QtRosNode::startGetClose(float goalX, float goalY, float goalTheta)
{
}

void QtRosNode::startGetClose(float goalX, float goalY)
{
}

void QtRosNode::startGetClose(std::string goal_location)
{
}

void QtRosNode::startMoveLateral(float dist)
{
}

void QtRosNode::startMoveDistAngle(float dist, float angle)
{
}

void QtRosNode::pubRecognizedSpeech(std::string recogString)
{
    std_msgs::String msg;
    msg.data = recogString;
    this->pubRecogSpeech.publish(msg);
}

void QtRosNode::getRobotPose(float& robotX, float& robotY, float& robotTheta)
{
}
