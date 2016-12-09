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
/*    
    bool* isknown=new bool[map.data.size];
    int* gValues= new int[map.data.size];
    int* fValues= new int[map.data.size];
    int* previous= new int[map.data.size];

    bool* isInOpenList=new int[map.data.size]; 
    std::vector<int> openlist;

    int currentCellx=(int)((startx-map.info.origin.position.x)/map.info.resolution));
    int currentCelly=(int)((starty-map.info.origin.position.y)/map.info.resolution));
    int currentCell= currentCelly*map.info.width + currentCellx;
    int goalCell = currentCelly*map.info.width + currentCellx;

    isknow[currentCell]=true;
    gValues[currentCell]=0;
    fvalues[currentCell]= abs(goalCelly-currentCelly)+abs(goalCellx-currentCellx);

    int attemps=map.data.size();
    int neighbors=new int[4];

    for (size_t i=0; i<map.data.size();i++)
    {
        isknow[i]=false;
        gValues[i]=INT_MAX;
        fValues[i]=INT_MAX;
        previous[i]=-1;
        isInOpenList[i]=false;           
    }

    while(currentCell !=goalCell && --attempts >0)
    {
         neighbors[0]= currentCell -1;
         neighbors[1]= currentCell +1;
         neighbors[2]= currentCell + map.info.width;
         neighbors[3]= currentCell - map.info.width;
         int gTemp= gValues[currentCell]+1;
         for(int i=0; i<4 ; i++)
         {
             if(map.data[neighbors[i]]>50 ||isknown[neighbors[i]])
             if(gTemp < gValues[neighbors[i]])
             {
                 gValues[neighbors[i]]=gTemp;
                 fValues[neighbors[i]]=gTemp + abs (goalCell -neighbors[i]%map.info.width) + previous[neighbors[i]]= currentCell;    
             }//end if    
             if(!isInOpenList[neighbors[i]])
             {
                 isInOpenList[neighbirs[i]] = true;
                 openList.push_back(neighbors[i]);
             }    
         }//end for

         int minIdx= -1;
         int minF= INT_MAX;
         for(int i=0; i< openList.size();i++)
         {
             if(fValues[openList[i]] < minF)
             {
                 minF= fValues[openList[i]];
                 min Idx=i;
             }    
         }//en for    

         currentCell = openList[minIdx];
         isKnown[currentCell]=true;
         openList.erase(openList.begin() + minIdx);
         
    }//end of while

    std::cout <<"Current cell=" <<currentCell <<std::end1;
*/      
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
