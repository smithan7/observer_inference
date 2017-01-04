/*
 * Agent.cpp
 *
 *  Created on: Mar 2, 2016
 *      Author: andy
 */


#include "Agent.h"

Agent::Agent(ros::NodeHandle nHandle){
	costmap.init_flag = true;

	locSub0 = nHandle.subscribe("/agent0/loc", 1, &Agent::locCallback0, this);
	marketSub0 = nHandle.subscribe("/agent0/market", 1, &Agent::marketCallback0, this);
	mapUpdatesSub0 = nHandle.subscribe("/agent0/map", 1, &Agent::mapUpdatesCallback0, this);

	markerPub = nHandle.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
	marketPub0 = nHandle.advertise<std_msgs::Float32MultiArray>("/agent0/market", 10);
	mapUpdatesPub0 = nHandle.advertise<std_msgs::Int16MultiArray>("/agent0/map", 10);

	obsThresh = 50;
	actTimer = ros::Time::now();
}

void Agent::init(Point sLoc, int myIndex, float obsThresh, float comThresh, int numAgents){
	this->obsThresh = obsThresh;
	this->comThresh = comThresh;
	
	cLoc= sLoc;
	gLoc = sLoc;

	this->myIndex = myIndex;
	this->numAgents = numAgents;
	pickMyColor(myIndex);

	for(int i=0; i<numAgents; i++){
		agentLocs.push_back(sLoc);
		Mat cm = Mat::ones(96,96, CV_16S)*costmap.unknown;
		agentMats.push_back( cm );

		vector<float> am;
		for(int j=0; j<13; j++){
			am.push_back(0);
		}
		agentMarkets.push_back(am);
		agentColors.push_back( pickMyColor (i) );
	}
	agentTimes = cv::Mat::zeros(numAgents+1, numAgents+1, CV_32FC1);

	market.init(numAgents, myIndex, false);
	costmap.initCostmap(96, 96);
	offset.x = 96 / 2 + 1;
	offset.y = 96 / 2 + 1;

	market.updatecLoc( sLoc );
	market.updateTime( ros::Time::now().toSec() );
	market.updategLoc( sLoc );
	market.updateExploreCost( 0 );
}


void Agent::marketCallback0( const std_msgs::Float32MultiArray& transmission){
	cerr << "into marketCallback0" << endl;
	market.updatecLoc(cLoc);
	market.dissasembleTransmission( transmission );
	agentMarkets[0] = transmission.data;
	market.printMarket();
	cerr << "out of marketCallback0" << endl;
}

void Agent::mapUpdatesCallback0( const std_msgs::Int16MultiArray& transmission ){
	cerr << "into mapUpdatesCallback0: " << transmission.data.size() << endl;
	
	for(size_t i=0; i<transmission.data.size(); i+=3){
		//x,y, val
		Point t(transmission.data[i], transmission.data[i+1]);
		agentMats[0].at<short>(t) = transmission.data[i+2];
	}

	cerr << "out of mapUpdatesCallback0" << endl;
	showAgentCostmat(0);
}

void Agent::showAgentCostmat( int a ){

	Mat t = Mat::zeros(agentMats[a].size(), CV_8UC3);

	for(int i=0; i<agentMats[a].cols; i++){
		for(int j=0; j<agentMats[a].rows; j++){
			Point p(i,j);
			if(agentMats[a].at<short>(p) == costmap.unknown){
				t.at<Vec3b>(p) = costmap.cUnknown;
			}
			else if(agentMats[a].at<short>(p) == costmap.obsWall){
				t.at<Vec3b>(p) = costmap.cObsWall;
			}
			else if(agentMats[a].at<short>(p) == costmap.obsFree){
				t.at<Vec3b>(p) = costmap.cObsFree;
			}
		}
	}

	circle(t, agentLocs[a], 2, agentColors[a], -1 , 8);
	circle(t, cLoc, 2, myColor, -1 , 8);
	char buffer[50];
	sprintf(buffer,"agentMats[%d]", a);
	namedWindow(buffer, WINDOW_NORMAL);
	imshow(buffer, t);
	waitKey(10);
}

void Agent::locCallback0( const nav_msgs::Odometry& locIn){ // this works!
	//cerr << "into locCallback0" << endl;
	agentLocs[0].x = offset.x + 4.8*(locIn.pose.pose.position.y);
	agentLocs[0].y = offset.y + 4.8*(locIn.pose.pose.position.x);

	act(0); // this is check commo with my self and all other agents
	//cerr << "out of locCallback0: " << agentLocs[0] << endl;
}

void Agent::act(int A){
	//cerr << "into  act" << endl;
	float ct = ros::Time::now().toSec();
	Point t(A,myIndex);
	if( ct - agentTimes.at<float>(t) > 1){ // check against me
		//ROS_INFO("checking coms");
		if( commoCheck(agentLocs[A], cLoc, costmap, 5000.0 )){
			market.updateTime(ros::Time::now().toSec() );
			agentTimes.at<float>(t) = ct;
			Point tp(myIndex, A);
			agentTimes.at<float>(tp) = ct;
			communicate(A, myIndex);
		}
	}


	for(int i=0; i<numAgents; i++){ // check against all other agents
		Point t(i, A);
		if( ct - agentTimes.at<float>(t) > 1){
			ROS_INFO("checking coms");
			if( commoCheck(agentLocs[i], agentLocs[A], costmap, 5000.0 )){
				agentTimes.at<float>(t) = ct;
				Point tp(A, i);
				agentTimes.at<float>(tp) = ct;
				communicate(i, myIndex);
			}
		}
	}
	//cerr << "out of act" << endl;
}

void Agent::communicate(int A, int B){
	cerr << "into communicate" << endl;
	ROS_INFO("communicating %i and %i", A, B);
	// exchange markets
	publishMarkets(A, B);
			
	// exchange maps
	publishCostmapUpdates(A,B);
	cerr << "out of communicate" << endl;
}


void Agent::publishMarkets(int A, int B){
	cerr << "into publishMarkets" << endl;
	if(A == myIndex){
		cerr << "A == myIndex" << endl;
		market.dissasembleTransmission( agentMarkets[B] );
		cerr << "A == myIndex" << endl;
	}
	else if( A == 0){
		cerr << "A == 0" << endl;
		publishMarket0( B );
		cerr << "A == 0" << endl;
	}
	/*
	else if( A == 1){
		publishMarket1( B );
	}
	else if( A == 2){
		publishMarket2( B );
	}
	else if( A == 3){
		publishMarket3( B );
	}
	else if( A == 4){
		publishMarket4( B );
	}
	*/

	if(B == myIndex){
		cerr << "B == myIndex" << endl;
		cerr << "agentMarkets.size(): " << agentMarkets.size() << endl;
		market.dissasembleTransmission( agentMarkets[A] );
		cerr << "B == myIndex" << endl;
	}
	else if( B == 0){
		cerr << "B == 0" << endl;
		publishMarket0( A );
		cerr << "B == 0" << endl;
	}
	/*
	else if( B == 1){
		publishMarket1( A );
	}
	else if( B == 2){
		publishMarket2( A );
	}
	else if( B == 3){
		publishMarket3( A );
	}
	else if( B == 4){
		publishMarket4( A );
	}
	*/
	cerr << "out of publishMarkets" << endl;
}

void Agent::publishMarket0( int A ){
	cerr << "into publishMarket0" << endl;
	if(A == myIndex){
		marketPub0.publish( market.assembleTransmission() );
	}
	else{
		std_msgs::Float32MultiArray msg;
		msg.data = agentMarkets[A];
		marketPub0.publish( msg );
	}
	cerr << "out of publishMarket0" << endl;
}

void Agent::publishCostmapUpdates(int A, int B){
	cerr << "into publishCostmapUpdates" << endl;
	vector<short int> aUpdates, bUpdates;

	if( A == myIndex ){
		getCostmapUpdates(costmap.cells, agentMats[B], aUpdates, bUpdates );
	}
	else if( B == myIndex ){
		getCostmapUpdates(agentMats[A], costmap.cells, aUpdates, bUpdates );
	}
	else{
		getCostmapUpdates(agentMats[A], agentMats[B], aUpdates, bUpdates );
	}

	if( A == myIndex ){
		costmap.updateCostmap( bUpdates );
	}
	else if(A == 0){
		std_msgs::Int16MultiArray bd;
		bd.data = bUpdates;
		mapUpdatesPub0.publish( bd );
	}
	/*
	else if(A == 1){
		mapUpdatesPub1( bUpdates );
	}
	else if(A == 2){
		mapUpdatesPub2( bUpdates );
	}
	else if(A == 3){
		mapUpdatesPub3( bUpdates );
	}
	else if(A == 4){
		costmap.updateCostmap( bUpdates );
	}
	*/

	if( B == myIndex ){
		costmap.updateCostmap( aUpdates );
	}
	else if(B == 0){
		std_msgs::Int16MultiArray ad;
		ad.data = bUpdates;
		mapUpdatesPub0.publish( ad );
	}
	/*
	else if(B == 1){
		mapUpdatesPub1( aUpdates );	
	}
	else if(B == 2){
		mapUpdatesPub2( aUpdates );
	}
	else if(B == 3){
		mapUpdatesPub3( aUpdates );
	}
	*/

	cerr << "out of publishCostmapUpdates" << endl;
}

void Agent::getCostmapUpdates(Mat &A, Mat &B, vector<short int> &aUpdates, vector<short int> &bUpdates){
	cerr << "into getCostmapUpdates" << endl;
	aUpdates.clear();
	bUpdates.clear();

	cerr << A.cols << ", " << B.cols << endl;
	for(int i=0; i<A.cols; i++){
		for(int j=0; j<A.rows; j++){
			Point p(i,j);

			if(A.at<short>(p) != B.at<short>(p) ){ // do we think the same thing?
				if(A.at<short>(p) == costmap.obsFree || costmap.obsWall){
					bUpdates.push_back(i);
					bUpdates.push_back(j);
					bUpdates.push_back(A.at<short>(p));
					B.at<short>(p) = A.at<short>(p);
				}
				else if(B.at<short>(p) == costmap.obsFree || B.at<short>(p) == costmap.obsWall){
					aUpdates.push_back(i);
					aUpdates.push_back(j);
					aUpdates.push_back(B.at<short>(p));
					A.at<short>(p) = B.at<short>(p);
				}
			}
		}
	}
	cerr << "out of getCostmapUpdates" << endl;
}

bool Agent::commoCheck( Point aLoc, Point bLoc, Costmap &costmap, float comThresh ){
	//cerr << "into commoCheck" << endl;
	if( pow(aLoc.x-bLoc.x,2)+pow(aLoc.y-bLoc.y,2) > pow(comThresh,2)){
		return false;
	}

	Mat ta = Mat::zeros(costmap.cells.size(), CV_8UC1);
	LineIterator it(ta, aLoc, bLoc, 4, false);
	bool flag = false;
	for(int i=0; i<it.count; i++, ++it){
		Point pp  = it.pos();
		//circle(ta, pp, 1, Scalar(255), -1, 8);
		if(pp.x >= costmap.cells.cols || pp.y >= costmap.cells.rows || costmap.cells.at<short>(pp) != costmap.obsFree){
			if( flag ){
				return false;
			}
			else{
				flag = true;
			}
		}
	}
	return true;
}


void Agent::showCellsPlot(){
	cerr << "into showCellsPlot" << endl;
	costmap.buildCellsPlot();

	circle(costmap.displayPlot,cLoc,2, myColor,-1, 8);
	circle(costmap.displayPlot,gLoc,2, Scalar(0,0,255),-1, 8);

	char buffer[50];
	sprintf(buffer,"Agent[%d]::costMap", myIndex);

	namedWindow(buffer, WINDOW_NORMAL);
	imshow(buffer, costmap.displayPlot);
	waitKey(1);
	cerr << "out of showCellsPlot" << endl;
}

Scalar Agent::pickMyColor(int index){
	Scalar a(0,0,0);

	a = a;

	if(index == 0){
		a[0] = 255;
	}
	else if(index == 1){
		a[1] = 255;
	}
	else if(index == 2){
		a[2] = 255;
	}
	else if(index == 3){
		a[0] = 255;
		a[1] = 153;
		a[2] = 51;
	}
	else if(index == 4){
		a[0] = 255;
		a[1] = 255;
		a[2] = 51;
	}
	else if(index == 5){
		a[0] = 255;
		a[1] = 51;
		a[2] = 255;
	}
	else if(index == 6){
		a[0] = 51;
		a[1] = 255;
		a[2] = 255;
	}
	else if(index == 7){
		a[0] = 153;
		a[1] = 255;
		a[2] = 51;
	}
	else if(index == 8){
		a[0] = 255;
		a[1] = 255;
		a[2] = 255;
	}
	else if(index == 9){
		// white
	}

	if(index == myIndex){
		myColor = a;
	}

	return a;
}

void Agent::publishRvizMarker(Point loc, float radius, int color, int id){
	visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = id;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = (loc.y - offset.y)/4.8;
    marker.pose.position.y = (loc.x - offset.x)/4.8;
    marker.pose.position.z = 0.5;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;

    // Set the color -- be sure to set alpha to something non-zero!

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    if(color == 0){
    	marker.color.r = 1.0f;
    }
    else if(color == 1){
    	marker.color.g = 1.0f;
    }
    else if(color == 2){
    	marker.color.b = 1.0f;
    }

    marker.lifetime = ros::Duration(5);
    markerPub.publish(marker);
}


Agent::~Agent(){

}

