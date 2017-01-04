/*
 * Agent.h
 *
 *  Created on: Jun 8, 2016
 *      Author: andy
 */
/*
 * Agent.h
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */

#ifndef SRC_Agent_H_
#define SRC_Agent_H_

#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
 #include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h> // action server
#include <visualization_msgs/Marker.h> // for making marks in rviz
#include <costmap_2d/costmap_2d.h>
#include <vector>
#include <utility>
#include <queue>
#include <fstream>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int8MultiArray.h"


#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#include "World.h" // includes - costmap.h
#include "CostmapCoordination.h" // includes - world.h, frontier.h
#include "CostmapPlanning.h" // includes - world.h, frontier.h

#include "Graph.h" // includes - costmap.h, node.h
#include "GraphCoordination.h" // includes - frontier.h, TreeNode.h, Node.h
#include "GraphPlanning.h" // includes - graph.h, node.h

#include "Inference.h" // includes - frontiers.h, contours.h
#include "Market.h" // for graphCoordination

using namespace std;
using namespace cv;

class Agent{
public:

	ros::Subscriber locSub0, marketSub0, mapUpdatesSub0;
	ros::Publisher markerPub, marketPub0, mapUpdatesPub0;

	void mapUpdatesCallback0( const std_msgs::Int16MultiArray& transmission );
	void locCallback0( const nav_msgs::Odometry& locIn);
	void marketCallback0( const std_msgs::Float32MultiArray& marketOrders);
	
	void publishMapUpdates0( const std_msgs::Int16MultiArray& transmission );
	void publishRvizMarker(Point loc, float radius, int color, int id);
	void publishMarket0(int B);
	ros::Time actTimer, marketTimer;

	int iterCntr, iterPeriod;

	vector<Mat> agentMats;
	void showAgentCostmat( int a );
	vector<Point> agentLocs;
	vector< vector<float> > agentMarkets;
	Mat agentTimes;
	vector<Scalar> agentColors;
	int numAgents;

	bool commoCheck( Point aLoc, Point bLoc, Costmap &costmap, float comThresh );

	// agent stuff
	Agent(ros::NodeHandle nh);
	void init(Point sLoc, int myIndex, float obsThresh, float comThresh, int numAgents);
	Scalar pickMyColor(int a);
	~Agent();
	void showCellsPlot();
	int myIndex;
	Scalar myColor;
	float comThresh;
	float obsThresh;

	Market market;
	bool marketFlag;
	std_msgs::Float32MultiArray marketOrders;

	// working
	void infer(string inferenceMethod, World &World);
	void act(int A);
	void communicate(int A, int B);
	void publishMarkets(int A, int B);
	void publishCostmapUpdates(int A, int B);
	void getCostmapUpdates(Mat &A, Mat &B, vector<short int> &aUpdates, vector<short int> &bUpdates);

	Point cLoc, gLoc, offset; // for map

	// costmap class stuff
	Costmap costmap;

	// inference stuff
	Inference inference;
};

#endif /* SRC_Agent_H_ */
