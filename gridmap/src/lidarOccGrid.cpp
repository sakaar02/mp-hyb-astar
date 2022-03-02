#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Int8MultiArray.h"
#include <iostream>
#include <vector>
#include <utility>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<typeinfo>
#include <cstdlib>
//#include "matplotlib-cpp/matplotlibcpp.h" // use if needed to plot points on a graph instead of rviz
#include <cmath>


using namespace std;
using namespace cv;
//namespace plt = matplotlibcpp; 

const float ResAngLidar = 0.498;                            //lidar angular resolution in degrees (basically increment in angle between consecutive LIDAR rays
const int  NumLidarRays = 723;                            //number of angular lidar beams (equal to number of elements in ranges array of LaserScan)

const float MapRes     = 0.1;                              //map resolution(m/cell)
const float FinMapRes  = 0.1;                              //map resolution(m/cell)
const float ResConv    = FinMapRes/MapRes;                 //for converting map to final map

const int MapHeightM   = 20;  //720 //72                            //map height in meters
const int MapWidthM    = 20; //1280                              //map width in meters

const int MapHeight    = MapHeightM/MapRes;                //map height in pixel
const int MapWidth     = MapWidthM/MapRes;                 //map width in pixel

const int CARCEN_X = MapWidth/2;				//center of car with respect to map
const int CARCEN_Y = MapWidth/2;				//center of car with respect to map

const int ObstCost = 100;                                  //ID of obstacle

const int NumRayToExclude = 180;

const  float INF_REPLACE = 2000;
const float PI = 3.1415926;


std_msgs::Int8MultiArray LaneMap, RoadMap;
ros::Publisher pub_local_map, pub_blown_local_map;

pair<float,float> convToCart(int i,float r) //convert from polar to cartesian co-ordinates
{	bool test = ((int(r))<(214) && (int(r))>(-214)); // 214 is a random number, we basically want 'test' to be true when r is finite, false when infinite 

	if (test)
	{
			float ang = (i*ResAngLidar)*(M_PI/180.0); // angle at which the point is present
			float x   = r*cos(ang); // conversion to cartesian coordinates
			float y   = r*sin(ang);

			//APPLYING TRANSFORM
			float theta = PI/2 ;
			float x1 = x * cos(theta) - y * sin(theta);
			float y1 = x * sin(theta) + y * cos(theta);
			return make_pair(x1,-y1);
	}
	else{
		return make_pair(INF_REPLACE, INF_REPLACE); // INF_REPLACE corresponds to a finite large number which acts as an identifiable placeholder for infinity
	}
}

//vector<int> XCO; //used while plotting on matplotlib
//vector<int> YCO;


void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{	//cout << "LIDAR CALLBACK WORKING\t";
	// cout << msg->ranges.size() << "\n";
	if(true){
		//cout << "Entering IF\n";

		vector< pair < float, float > > LidarXY(NumLidarRays,make_pair(0,0));
		
		for(int i=0; i<NumLidarRays; i++)
		{
		
		LidarXY[i] = convToCart(i,msg->ranges[i]);
		}	
		//cout << "DONE WITH CONV TO CART FUNCTION\n";
		
		
		//declaring and initialising gridmap
		nav_msgs::OccupancyGrid GridMap;
		GridMap.header.stamp = ros::Time::now();
		GridMap.header.frame_id = "MPOccGrid";
		GridMap.info.resolution = MapRes;
		GridMap.info.origin.position.x = 0.0;
		GridMap.info.origin.position.y = 0.0;
		GridMap.info.origin.position.z = 0.0;
		GridMap.info.origin.orientation.x = 0.0;
		GridMap.info.origin.orientation.y = 0.0;
		GridMap.info.origin.orientation.z = 0.0;
		GridMap.info.origin.orientation.w = 1.0;
		GridMap.info.width = MapWidth;
		GridMap.info.height = MapHeight;
		GridMap.info.map_load_time = ros::Time::now();

		//cout << "DECLARED AN OCC GRID\n";


		for(int i=0;i<MapHeight;i++)
		{
			for(int j=0;j<MapWidth;j++)
			{
				GridMap.data.push_back(0);
			}
		}
		//cout << "INITIALIZED AN OCC GRID\n";

		//vector<int> XCO_filter; //filter out infinite X and Y values (used while plotting on matplotlib)
		//vector<int> YCO_filter;
		
		// OBSTACLE ADD TO GRID MAP
		for(int i=0; i<LidarXY.size() ;i++)
		{	
			int Xco = round(LidarXY[i].first/MapRes); // conversion into meters
			int Yco = round(LidarXY[i].second/MapRes); //conversion into meters
			
			//XCO.push_back(Xco);
			//YCO.push_back(Yco);
			 
			if( ! ( Xco==(INF_REPLACE/MapRes) || Yco==(INF_REPLACE/MapRes) ) )
			{
				//XCO_filter.push_back(Xco);
				//YCO_filter.push_back(Yco);
				
				GridMap.data[(Xco+CARCEN_X)*MapWidth+(Yco+CARCEN_Y)] = ObstCost;
			}
		}

		//plotting on a graph
		// // Set the size of output image to 1200x780 pixels
		// // plt::figure_size(1400, 1400);
		// // Plot line from given x and y data. Color is selected automatically.
		// plt::plot(XCO_filter, YCO_filter, "ro");
		// plt::xlim(-40, 40);
		// plt::ylim(-300, 75);
		
		// plt::show();

		//cout << "DONE DATA IN GRID MAP\n";

		pub_local_map.publish(GridMap);
		//cout << "PUBLISHED GRID MAP\n";

		GridMap.data.clear();
		//cout << "GRID MAP CLEARED\n";
	}
}


int main(int argc,char** argv)
{	cout << "Start with this node\n";
	ros::init(argc,argv,"local_map");

	ros::NodeHandle LRSubNode, LidarSubNode, MapPubNode, BlownPubNode, PathSubNode;

	// ros::Subscriber SubLane = LRSubNode.subscribe<nav_msgs::OccupancyGrid>("/cv/laneoccgrid",1,LaneCallback);
	ros::Subscriber SubLidar = LidarSubNode.subscribe<sensor_msgs::LaserScan>("/laserscan",1,LidarCallback);

	pub_local_map = MapPubNode.advertise<nav_msgs::OccupancyGrid>("/scan/local_map",1);


	ros::Rate RosLoopRate(15.0);
	while(ros::ok())
	{
		ros::spinOnce();//check for incoming messages
		RosLoopRate.sleep();

	}
	return 0;
}

// CONFIRM IF LIDAR KA RANGES GIVES DATA IN CLOCKWISE DIRECTION
// -2147483648	│ 2147483647
