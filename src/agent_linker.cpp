#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <costmap_2d/costmap_2d.h>

#include "Agent.h"

int main(int argc, char *argv[])
{
	// initialization
	ros::init(argc, argv, "observer");

	ros::NodeHandle nHandle("~");

	Point sLoc(50,60);

	float obsThresh = 50;
	float comThresh = 50;
	int myIndex = 1;
	int numAgents = 1;

	Agent *agent = new Agent(nHandle);
	agent->init(sLoc, myIndex, obsThresh, comThresh, numAgents);

	// return the control to ROS
	ros::spin();

	return 0;
}