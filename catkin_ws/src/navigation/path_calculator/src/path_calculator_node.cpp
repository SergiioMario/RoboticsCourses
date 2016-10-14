#include "ros/ros.h"
#include "navig_msgs/CalculatePath.h"

nav_msgs::Path global_path;

nav_msgs::OccupancyGrid grow_obs(nav_msgs::OccupancyGrid& map,
                                 int k)
{
    return map;
}

bool a_star(float startX, float startY, float goalX,
            float goalY, nav_msgs::OccupancyGrid& map,
            nav_msgs::Path& optimal_path)
{
    
    optimal_path.header.frame_id = "map";
    optimal_path.poses.clear();
    geometry_msgs::PoseStamped p;
    p.pose.position.x = 1.0;
    p.pose.position.y = 1.0;
    optimal_path.poses.push_back(p);
    p.pose.position.x = 2.0;
    p.pose.position.y = 2.0;
    optimal_path.poses.push_back(p);

    return true;
}

nav_msgs::Path smooth_path(nav_msgs::Path& path, float alpha,
                           float beta)
{
    return path;
}

bool callback_a_star(navig_msgs::CalculatePath::Request& req,
                     navig_msgs::CalculatePath::Response& res)
{
    std::cout << "StartPose: " << req.start_pose.position.x <<
        "  " << req.start_pose.position.y << std::endl;

    req.map = grow_obs(req.map, 6);

    nav_msgs::Path path;
    if(!a_star(req.start_pose.position.x,
               req.start_pose.position.y,
               req.goal_pose.position.x,
               req.goal_pose.position.y, req.map, path))
    {
        std::cout << "Cannot calculate A* :'(" << std::endl;
        return false;
    }
    std::cout << "A* calculated succesfully... " << std::endl;
    res.path = smooth_path(path, 0.1, 0.1);
    global_path = res.path;
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "Init path calculator..." << std::endl;
    ros::init(argc, argv, "path_calculator");
    ros::NodeHandle n;
    ros::ServiceServer srv = n.advertiseService(
        "/navigation/a_star", callback_a_star);
    ros::Publisher pubPath = n.advertise<nav_msgs::Path>(
        "/navigation/a_star_path", 1);
    ros::Rate loop(10);
    while(ros::ok())
    {
        pubPath.publish(global_path);
        ros::spinOnce();
        loop.sleep();
    }
}
