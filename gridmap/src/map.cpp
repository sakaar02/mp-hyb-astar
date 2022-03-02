//check variable redundancies
//check commented out code
//check Subscriber
//check laserscan parameters

//ros libraries
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Int8MultiArray.h"
//c++ libraries
#include <iostream>
#include <vector>
#include <utility>
#include <math.h>
//opencv libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

const float ResAngLidar = 0.498;                            //lidar angular resolution in degrees
const int  NumLidarRays = 723;                            //number of angular lidar beams

int LaneMapW, LaneMapH;                                    //lane map size


float LaneMapRes       = 0.01;                              //lane occupancy grid map resolution in (m/cell)

const float MapRes     = 0.1;                              //map resolution(m/cell)
const float FinMapRes  = 0.1;                              //map resolution(m/cell)
const float ResConv    = FinMapRes/MapRes;                 //for converting map to final map

const int MapHeightM   = 128;  //720 //72                            //map height in meters
const int MapWidthM    = 128; //1280                              //map width in meters

const int MapHeight    = MapHeightM/MapRes;                //map height in pixel
const int MapWidth     = MapWidthM/MapRes;                 //map width in pixel
const int FinMapHeight = MapHeightM/FinMapRes;             //final map height in pixel
const int FinMapWidth  = MapWidthM/FinMapRes;              //final map width in pixel
const int MapRangeM    = 7.2;                               //range for mapping in meters
const int MapRange     = MapRangeM/MapRes;                 //range for mapping in pixel

const int ObstSafeDistCM   = 30;                           //safe distance about obstacle when it is on one side in cm
const int ObstSafeDist     = ObstSafeDistCM*0.01/MapRes;   //safe distance about obstacle when it is on one side in pixel
const int ObstSafeRadiiCM  = 20;                           //radius around obstacle for path planning in cm
const int ObstSafeRadii    = ObstSafeRadiiCM*0.01/MapRes;  //radius around obstacle for path planning in cm
const int LaneSafeRadiiCM  = 20;                           //radius around lane for path planning in cm
const int LaneSafeRadii    = LaneSafeRadiiCM*0.01/MapRes;  //radius around lane for path planning in pixel
const int RoadSafeRadiiCM  = 20;                           //radius around road for path planning in cm
const int RoadSafeRadii    = RoadSafeRadiiCM*0.01/MapRes;  //radius around road for path planning in pixel

const int LidarDistCM      = 0;                            //distance of lidar from center of car in cm
const int LidarDist        = LidarDistCM*0.01/MapRes;      //distance of lidar from center of car in pixel
const int CamLidarDistCM   = 0.0;                          //distance between camera and lidar in cm
const int CamDistCM        = LidarDistCM - CamLidarDistCM; //distance of cam from center of car in cm
const int CamDist          = CamDistCM*0.01/MapRes;        //distance of cam from center of car in pixel
const int LaneShiftCM      = 0;                            //width error in lane in cm
const int LaneShift        = LaneShiftCM*0.01/MapRes;      //width error in lane in pixel

const int CarCenX  = MapWidth/2;                           //center of car with respect to map
const int CarCenY  = 0;                                    //center of car with respect to map

const int ObstCost = 100;                                  //ID of obstacle
const int LaneCost = 80;                                   //ID of lane
const int FinObstCost = 18;                               //final cost of obstacle
const int FinLaneCost = 110;                               //final cost of lane
const float ObstWtg = FinObstCost/100.0;
const float LaneWtg = FinLaneCost/100.0;
// ye bakchodi kyu hai mujhe bhi nahi pata

std_msgs::Int8MultiArray LaneMap, RoadMap;
ros::Publisher pub_local_map, pub_blown_local_map;

pair<float,float> convToCart(int i,float r) //convert from polar to cartesian co-ordinates
{
	float ang = ((i*ResAngLidar)-90)*(M_PI/180.0);
	float x   = r*cos(ang);
	float y   = r*sin(ang);
	return make_pair(x,y);
}

//WORKING GREAT!
void LaneCallback(nav_msgs::OccupancyGrid msg)
{	cout << "LANECALL BAC WORKING\n";
	if(!msg.data.empty())
	{
		// cout << "In Lane callback" << endl;
		int i,j;
		LaneMapW = msg.info.width;
		LaneMapH = msg.info.height;
		LaneMapRes = msg.info.resolution;
		if(LaneMap.data.empty()){
			for(i=0; i<LaneMapH; i++)//initialising map
			{
				for(j=0;j<LaneMapW; j++)
				{
					LaneMap.data.push_back(0);
				}
			}
		}

		for(i=LaneMapH-1;i>=0;i--)//putting values
		{
			for(j=0;j<LaneMapW;j++)
			{
				int i1;
				i1=LaneMapH-1-i;
				LaneMap.data[i1*LaneMapW+j] = msg.data[i*LaneMapW+j];
				// cout << double(LaneMap.data[i1*LaneMapW+j]) << "\t";

			}
		}
	}
	// cout << "DONE WITH LANE\n";

}

void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{	cout << "LIDAR CALLBACK WORKING\t";
	// cout << msg->ranges.size() << "\n";
	if(true){
		cout << "Entering IF\n";

		vector< pair < float, float > > LidarXY(NumLidarRays,make_pair(0,0));
		for(int i=179;i<NumLidarRays-180;i++)
		{
			{
				// LidarXY[i] = convToCart((i-NumLidarRays/2),msg->ranges[NumLidarRays - (i)]);
				LidarXY[i] = convToCart((i),msg->ranges[(i)]);
				// LidarXY[i].first += LidarDistCM*0.01;
				// cout << "LidarXY" << LidarXY[i].first <<" , " << LidarXY[i].second << "\n";
			}
		}
		cout << "DONE WITH CONV TO CART FUNCTION\n";

		nav_msgs::OccupancyGrid GridMap;
		GridMap.header.stamp = ros::Time::now();
		GridMap.header.frame_id = "base_link";
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

		cout << "DECLARED AN OCC GRID\n";

		// nav_msgs::OccupancyGrid BlownGridMap;
		// BlownGridMap.header.stamp = ros::Time::now();
		// BlownGridMap.header.frame_id = "base_link";
		// BlownGridMap.info.resolution = MapRes;
		// BlownGridMap.info.origin.position.x = 0.0;
		// BlownGridMap.info.origin.position.y = 0.0;
		// BlownGridMap.info.origin.position.z = 0.0;
		// BlownGridMap.info.origin.orientation.x = 0.0;
		// BlownGridMap.info.origin.orientation.y = 0.0;
		// BlownGridMap.info.origin.orientation.z = 0.0;
		// BlownGridMap.info.origin.orientation.w = 1.0;
		// BlownGridMap.info.width  = MapWidth;
		// BlownGridMap.info.height = MapHeight;
		// BlownGridMap.info.map_load_time = ros::Time::now();

		// nav_msgs::OccupancyGrid FinGridMap;
		// FinGridMap.header.stamp = ros::Time::now();
		// FinGridMap.header.frame_id = "base_link";
		// FinGridMap.info.resolution = FinMapRes;
		// FinGridMap.info.origin.position.x = 0.0;
		// FinGridMap.info.origin.position.y = 0.0;
		// FinGridMap.info.origin.position.z = 0.0;
		// FinGridMap.info.origin.orientation.x = 0.0;
		// FinGridMap.info.origin.orientation.y = 0.0;
		// FinGridMap.info.origin.orientation.z = 0.0;
		// FinGridMap.info.origin.orientation.w = 1.0;
		// FinGridMap.info.width  = FinMapWidth;
		// FinGridMap.info.height = FinMapHeight;
		// FinGridMap.info.map_load_time = ros::Time::now();

		for(int i=0;i<MapHeight;i++)
		{
			for(int j=0;j<MapWidth;j++)
			{
				GridMap.data.push_back(0);
				// BlownGridMap.data.push_back(0);
			}
		}
		cout << "INITIALIZED AN OCC GRID\n";
		// for(int i=0;i<FinMapHeight;i++)
		// {
		// 	for(int j=0;j<FinMapWidth;j++)
		// 	{
		// 		FinGridMap.data.push_back(0);
		// 	}
		// }

		// LANE ADDING
		// int LaneMapShift = (((MapWidthM) - (LaneMapW*LaneMapRes))/2)/MapRes;
		// for(int i=0;i<LaneMapH;i++)//adding lane data
		// {
		// 	for(int j=0;j<LaneMapW;j++)
		// 	{
		// 		int Xco = j*LaneMapRes/MapRes + LaneMapShift;
		// 		int Yco = i*LaneMapRes/MapRes + CamDist;

		// 		if(LaneMap.data[(i)*LaneMapW+j]==2)
		// 		{
		// 			GridMap.data[(Yco+CarCenY)*MapWidth+Xco+CarCenX-LaneShift-MapWidth/2] = LaneCost;
		// 		}
		// 	}
		// }

		// doubtful usage
		// lame lane blocks HARD CODED STUFF
		// for(int i=0;i<MapWidth;i++)
		// 	for(int j=0;j<20;j++)
		// 		if(((i<MapWidth/2+60) &&(i>MapWidth/2+40)) || ((i<MapWidth/2-40)&&(i>MapWidth/2-60)))
		// 			GridMap.data[i+j*MapWidth] = LaneCost;

		// OBSTACLE ADD
		for(int i=0;i<LidarXY.size();i++)
		{
			int Xco = round(LidarXY[i].first/MapRes);
			int Yco = round(LidarXY[i].second/MapRes);
			// cout << "LidarXY" << LidarXY[i].first <<" , " << LidarXY[i].second << "\n";
			cout << "Xco:" << Xco << "\n";
			cout << "Yco:" << Yco << "\n";	
			cout << "final:" << (Xco+LidarDist)*MapWidth+Yco+CarCenX << "\n" ;
			// if(!(Xco==LidarDist && Yco==(CarCenY)))
			// 	GridMap.data[(Xco+LidarDist)*MapWidth+Yco+CarCenX] = ObstCost;

		}
		cout << "DONE DATA IN GRID MAP\n";

		// Mat FinLocalMap     = Mat::zeros(FinMapHeight,FinMapWidth,CV_8UC1);
		// Mat LocalMap        = Mat::zeros(MapHeight,MapWidth,CV_8UC1);
		// Mat BlownLocalMap   = Mat::zeros(MapHeight,MapWidth,CV_8UC1);
		// Mat ObstMat         = Mat::zeros(MapHeight,MapWidth,CV_8UC1); //obstacle matrix
		// Mat LaneMat         = Mat::zeros(MapHeight,MapWidth,CV_8UC1); //lane matrix
		//
		// for(int i=0;i<MapHeight;i++)
		// {
		// 	for(int j=0;j<MapWidth;j++)
		// 	{
		
		// 		int i_= MapHeight - i;
		// 		LocalMap.at<uchar>(i,j) = GridMap.data[i_*MapWidth+j]*2.55;
		// 		// BlownLocalMap.at<uchar>(i,j) = BlownGridMap.data[i_*MapWidth+j]*2.55;
		
		// 		// if (BlownGridMap.data[i_*MapWidth+j] == ObstCost)
		// 		// 	ObstMat.at<uchar>(i,j) = 0;
		// 		// else
		// 		// 	ObstMat.at<uchar>(i,j) = 200;
		
		// 		// if(BlownGridMap.data[i_*MapWidth+j] == LaneCost)
		// 		// 	LaneMat.at<uchar>(i,j) = 0;
		// 		// else
		// 		// 	LaneMat.at<uchar>(i,j) = 200;
		// 	}
		// }
		//
		// distanceTransform (ObstMat, ObstMat, CV_DIST_L2, 5);
		// distanceTransform (LaneMat, LaneMat, CV_DIST_L2, 5);
		// normalize (ObstMat, ObstMat, 0.0, 1.0, NORM_MINMAX);
		// normalize (LaneMat, LaneMat, 0.0, 1.0, NORM_MINMAX);
		// subtract(1,ObstMat,ObstMat);
		// subtract(1,LaneMat,LaneMat);


		// for(int i=0;i<MapHeight;i++)//initialising map
		// {
		// 	for(int j=0;j<MapWidth;j++)
		// 	{
		// 			FinGridMap.data[(int)(i/ResConv)*FinMapWidth+(int)(j/ResConv)] = BlownGridMap.data[i*MapWidth+j];
		// 	}
		// }

		pub_local_map.publish(GridMap);
		cout << "PUBLISHED GRID MAP\n";
		// pub_blown_local_map.publish(BlownGridMap);

		// if(!LaneMap.data.empty())
		// 	LaneMap.data.clear();

		// FinGridMap.data.clear();
		GridMap.data.clear();
		cout << "GRID MAP CLEARED\n";
		// BlownGridMap.data.clear();
	}
}

int main(int argc,char** argv)
{	cout << "Start with this node\n";
	ros::init(argc,argv,"local_map");

	ros::NodeHandle LRSubNode, LidarSubNode, MapPubNode, BlownPubNode, PathSubNode;

	ros::Subscriber SubLane = LRSubNode.subscribe<nav_msgs::OccupancyGrid>("/cv/laneoccgrid",1,LaneCallback);
	ros::Subscriber SubLidar = LidarSubNode.subscribe<sensor_msgs::LaserScan>("/laserscan",1,LidarCallback);

	pub_local_map = MapPubNode.advertise<nav_msgs::OccupancyGrid>("/scan/local_map",1);
	// pub_blown_local_map = BlownPubNode.advertise<nav_msgs::OccupancyGrid>("/scan/blown_local_map",1);

	ros::Rate RosLoopRate(15.0);
	while(ros::ok())
	{
		ros::spinOnce();//check for incoming messages
		RosLoopRate.sleep();

	}
	return 0;
}