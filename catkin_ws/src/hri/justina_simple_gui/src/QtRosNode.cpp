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
    tf_listener = new tf::TransformListener();
    tf_listener->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(10.0));
    cltGetMap        = n->serviceClient<nav_msgs::GetMap>("/navigation/static_map");
    cltCalculatePath = n->serviceClient<navig_msgs::CalculatePath>("/navigation/a_star");
    
    ros::Rate loop(10);
    while(ros::ok() && !this->gui_closed)
    {
        //std::cout << "Ros node running..." << std::endl;
        emit updateGraphics();
        loop.sleep();
        ros::spinOnce();
    }
    emit onRosNodeFinished();
}

void QtRosNode::setNodeHandle(ros::NodeHandle* nh)
{
    this->n = nh;
}

void QtRosNode::getRobotPose(float& robot_x, float& robot_y, float& robot_theta)
{
    tf::StampedTransform transform;
    tf::Quaternion q;
    tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
    robot_x = transform.getOrigin().x();
    robot_y = transform.getOrigin().y();
    q = transform.getRotation();
    robot_theta = atan2((float)q.z(), (float)q.w()) * 2;
}

void QtRosNode::planPath(float start_x, float start_y, float goal_x, float goal_y, nav_msgs::Path& path)
{
    nav_msgs::GetMap getMap;
    if(!cltGetMap.call(getMap))
    {
	std::cout << "QtRosNode.->Cannot get map to call a_star service" << std::endl;
	return;
    }

    navig_msgs::CalculatePath calcPath;
    calcPath.request.start_pose.position.x = start_x;
    calcPath.request.start_pose.position.y = start_y;
    calcPath.request.goal_pose.position.x = goal_x;
    calcPath.request.goal_pose.position.y = goal_y;
    calcPath.request.map = getMap.response.map;
    if(!cltCalculatePath.call(calcPath))
    {
	std::cout << "QtRosNode.->Cannot calculate path by calling service" << std::endl;
	return;
    }

    path = calcPath.response.path;
}
