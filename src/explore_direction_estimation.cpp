
/*	

	EXPLORE DIRECTION ESTIMATOR for ROBOTIC EXPLORATION
	---------------------------------------------------
	
	Author - Junaid Ahmed Ansari
	email  - ansariahmedjunaid@gmail.com
	Inst.  - IIIT Hyderabad
	
	Description - This node estimates all possible safe driveable directions for a robot given a disparity image. The safety is governed by
				  the availability of enough gap width which can be set dynamically (see ExploreDirectionEstimator::setupParameters() ).
				  
				  Important features:
					  - Road plane segmentation based on RANSAC (pcl plane fitting used) and hence obstacle segmentation  
					  - genrates possible driveable directions for the vehicle based on available gap
					  - generates grid map (not probabilistic)
				  
				  Subscribes To:
				  	  - /(namespace)/left/camera_info
				  	  - /(namespace)/right/camera_info
				  	  - /(namespace)/disparity
				  	  - /(namespace)/left/image_rect
				  	  
				  publishes
				  	  - /explore/ground_points
				  	  - /explore/obstacle_points
				  	  - /explore/grid_view 
				  	  - /explore/incomming_point_cloud (if coloring set...it is colored based on segmentation)
				  	  - /explore/drive_directions	(markers for drive direction)
				  	  - /explore/gap_marks (gaps are drawn as lines in green color)		
				  	  - /explore/direction_as_poses (drive directions as PoseArray msg - position - gapcenterpos, orientation - direction)
				  	  
				  	  
	Note - More info on parameters to be here soon....				  								  
	
	To Do - publish a range of directions for ever gap so that the PLANNER node can decide upon which is most suitable		
		  - Write callback for param change
		  - when there is no plane fo fitting, keep the previous values of parameters
		  - parameter for overriding marker height 
		  - provide parameter for angle rejection
	
*/

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <vector>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <stereo_msgs/DisparityImage.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>

#include <image_geometry/stereo_camera_model.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GridCells.h>
#include <tf/transform_datatypes.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


using namespace ros;

using namespace std;
using namespace sensor_msgs;
using namespace stereo_msgs;
using namespace message_filters;
using namespace geometry_msgs;


long int pub_counter = 0;

ros::Publisher pub_point_cloud;			// to publish the incoming point cloud (colored/not colored)
ros::Publisher pub_ground_points;		// 			  ground points
ros::Publisher pub_grid_view;			//		      grid view with possible drive directions		 
ros::Publisher pubDriveDirectionGlobal;	//			  possible drive directions as marker array in global frame
ros::Publisher pub_obstacle_points;
ros::Publisher pubGapMarkersGlobal;		// publish gap markers
ros::Publisher pubDriveDirectionAsPoses;
ros::Publisher pubFreeSpaceMarker;
int inputMode = 0;	// 0-disparity 1-point cloud

ros::NodeHandle *n;

vector<float> dirs;
vector<cv::Point2f> locs;		
vector<cv::Vec4f> gapEndPoints;					// gap end points.... (0,1) - (x1, y1) (2,3) - (x2,y2)

class ExploreDirectionEstimator{

	public:
	
		// constructor
		ExploreDirectionEstimator(ros::NodeHandle* n){
					
			nh = n;
			sizeOfGrid = 101;
			centerOfGridInX = 50;
			centerOfGridInZ = 0;
			gridUnit = 0.1;										//10cm
			angularOccupancyResolution = 1;						// degrees
			angularOccupancyMaxRho = centerOfGridInX*gridUnit;	// full X/2 grids
			angleOffset = 37;
			colorPointCloud = false;
			minGapForDriving = 1.5;
			cameraHeightFromRoad = 0.9;
			obstacleSegMethod = 0;
			minHeightOfObstacle = 0.6;
			
			grid = cv::Mat::zeros(sizeOfGrid,sizeOfGrid,CV_8UC1);				
		
			// make sure it is even in number
			numOfAngularOccupancyBins = 180/angularOccupancyResolution;	
			angularOccupancy.assign(numOfAngularOccupancyBins, 0);
			angularOccupancyRho.assign(numOfAngularOccupancyBins, angularOccupancyMaxRho);					
			
			//init plane parameters...four parameters
			roadPlaneParameters[0] = roadPlaneParameters[1] = roadPlaneParameters[2] = roadPlaneParameters[3] = 0;			
			
			planeFittingDistThres = 0.06;
			// setup the parameters in the ros parameter server	-- loaded with the same default values as above
			setObstacleTolerance(0.3);
			setGroundTolerance(0.1);

			setupParameters();
		}
		
		inline void setGridCenter(int inX, int inZ){
			centerOfGridInX = inX;
			centerOfGridInZ = inZ;			
			nh->setParam("grid_center_x", inX);
			nh->setParam("grid_center_z", inZ);			
		}
		inline void setGridSize(int size){

			sizeOfGrid = size;						
			nh->setParam("grid_size", size);
			makeGrid();
		}					
		
		
		// sets up the shared pointer to the input cloud
		inline void setPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
			inputCloud = cloud;
		}
		
		
		// get plane parameters...later we will have plane segmentatino in the class itself..
		inline void setRoadPlaneParameters(cv::Scalar planeParams){
			roadPlaneParameters = planeParams;
		}
		
		// height of the camera from the road plane. used to segment points in a naive fasion as, points < height are obstacles
		inline void setCameraHeightFromRoad(float height){
			nh->setParam("cam_height", height);
			cameraHeightFromRoad=height;
		}		

		// set the maximum distance for the obstacle to consider it for angular occupancy bins
		inline void setAngularOccupancyRhoMax(float rho){
			nh->setParam("angular_occupancy_max_rho", rho);
			angularOccupancyMaxRho = rho;
		}
		
		// angle offset fromboth sides in angular occupancy
		inline void setAngleOffset(float offset){
			nh->setParam("angular_occupancy_angle_offset", offset);		
			angleOffset = offset;
		}
		
		inline void setPlaneFittingDistThres(float thres){
			planeFittingDistThres = thres;
			nh->setParam("plane_fitting_dist_thres", thres);
		}
		
		//gap width 
		inline void setMinimumGapForSafeDriving(float safeGap){
			nh->setParam("min_safe_driving_gap", safeGap);
			minGapForDriving = safeGap;
		}
		
		// when True passed, the point cloud is colored with RED for Obstacles and GREEN for road plane, else nothing
		inline void setPointCloudColoring(bool en){
			colorPointCloud = en;
		}
		
		inline void setAngularOccupancyResolution(float res){
			nh->setParam("angular_occupancy_resolution", res);
			angularOccupancyResolution = res;
		}

		inline void setMinHeighOfObstacle(float height){

			minHeightOfObstacle = height;
			nh->setParam("min_height_of_obstacle", height);
			
		}
		
		inline void setObstacleSegMethod(int method){
			obstacleSegMethod = method;
			if(method > 0){
				obstacleSegMethod = 1;				
			}
			
			nh->setParam("obstacle_segmentation_method", obstacleSegMethod);	
		}

		inline void setGroundTolerance(float groundTol){
		
			groundTolerance = groundTol;
			nh->setParam("ground_tol", groundTol);				
		}
		
		inline void setObstacleTolerance(float obstacleTol){
		
			obstacleTolerance = obstacleTol;		
			nh->setParam("obstacle_tol", obstacleTol);				
		}
		
		inline float getCameraHeightFromRoad(){
			
			return cameraHeightFromRoad;
		}
			
		inline float getObstacleTolerance(){
			return obstacleTolerance;
		}
		
		inline float getMinHeightOfObstacle(){
			return minHeightOfObstacle;
		}
		
		
		inline int getObstacleSegMethod(){
			return obstacleSegMethod;
		}

		inline float getGroundTolerance(){
			return groundTolerance;
		}
		
		inline int getGridSize(){
		
			return sizeOfGrid;
		}

		inline float getAngularOccupancyMaxRho(){
			return angularOccupancyMaxRho;
		}
		
		inline float getAngularOccupancyResolution(){
			return angularOccupancyResolution;
		}
		
		inline float getMinGapForDriving(){
			return minGapForDriving;
		}
		
		inline float getPlaneFittingDistThres(){
			return planeFittingDistThres;
		}
		
		inline cv::Point2i getGridCenters(){
			return cv::Point2i(centerOfGridInX, centerOfGridInZ);
		}
		
		inline int getObstacleSegmentationMethod(){
			return obstacleSegMethod;
		}
			
		inline bool getPointColorFlag(){
			return colorPointCloud;
		}	
		
		inline cv::Scalar getRoadPlaneParameters(){
			return roadPlaneParameters;
		}
		// returns if the point is an obstacle or not based on plane and minObstacle Height parameters
		inline bool isObstacle(cv::Point3f p){
		
		    float e = (roadPlaneParameters[0]*p.x + roadPlaneParameters[1]*p.y + roadPlaneParameters[2]*p.z + roadPlaneParameters[3]);
			return (p.y <roadPlaneParameters[3] && e>obstacleTolerance)?true:false;
 			
		}
		
		inline bool isGround(cv::Point3f p){
			
		    float e = (roadPlaneParameters[0]*p.x + roadPlaneParameters[1]*p.y + roadPlaneParameters[2]*p.z + roadPlaneParameters[3]);
			return (p.y <roadPlaneParameters[3] && e<groundTolerance)?true:false;
		}
		
		inline bool isGroundByHeight(cv::Point3f p, float height){
			
		    return ((p.y > height))?true:false;
		}
		
		// this functin can be used when we want qualify obstacles based on the height of from camera
		inline bool isObstacleByHeight(cv::Point3f p, float height){
			
			return ((p.y <= height))?true:false;
		}				
						
		inline float toDegree(float rad){
			return rad*57.29580;
		}
		inline float toRadian(float deg){
			return deg/57.29580;
		}						
							
		// updates the passed allDriveDir (as reference) vector with all possible drive directions
		// this interface does every thing, process pnt cld, get occupancies, and compute drive directions and gap locations
		// possibleDriveDirections   - vector of all possible drivable directions in degrees
		// possibleDriveGapLocations - center location of the driveable gaps
		// possiblegap endpoints - gap endpoints
		bool getDriveDirectionsBasedOnGapWidth(vector<float>& possibleDriveDirections, vector<cv::Point2f>& possibleDriveGapLocations, vector<cv::Vec4f>& possibleGapEndPoints){
		
		
			if(inputCloud->size() >0){		
			
				processCloud();
				getGaps();	
	
				for(int i = 0; i<gapsInAngularOccupancyStartInd.size(); ++i){								
				
					int s = gapsInAngularOccupancyStartInd[i];
					int e = gapsInAngularOccupancyEndInd[i];

					// compute the co-ordinates of the obstacles before and after the start and end positions of the gap respectively
					float theta1 = s*angularOccupancyResolution;
					float theta2 = e*angularOccupancyResolution;

					float x1 = angularOccupancyRho[s-1]*cos(toRadian(theta1));
					float y1 = angularOccupancyRho[s-1]*sin(toRadian(theta1));
				
					float x2 = angularOccupancyRho[e]*cos(toRadian(theta2));
					float y2 = angularOccupancyRho[e]*sin(toRadian(theta2));
				
					float gapWidth = sqrt( pow(y2-y1, 2) + pow(x2-x1, 2) );
					
					//cerr<<"++"<<x1<<","<<y1<<","<<x2<<","<<y2<<endl;
					// check if gap is drivable
					if(gapWidth >= minGapForDriving){
				
						// compute the cooridinates of gap center 
						cv::Point2f gapCenter;
						gapCenter.x = min(x1,x2) + abs(x2-x1)/2;
						gapCenter.y = min(y1,y2) + abs(y2-y1)/2;					
					
						float thetaGapCenter = toDegree(atan(gapCenter.y/gapCenter.x));
					
						if(thetaGapCenter < 0){						
							thetaGapCenter += 180;
						}
						
						// perpendicular direction the gap opening
						
						//cerr<<"+"<<gapCenter<<"*"<<gapWidth<<endl;
						possibleDriveDirections.push_back(thetaGapCenter);
						possibleDriveGapLocations.push_back(gapCenter);
						possibleGapEndPoints.push_back(cv::Vec4f(x1,y1, x2,y2));
					}//endif				
				
				}//endfor
				
			}//endif
			else{
				return false;	//cloud empty
			}
		
			return true;	//success			
		}//end func	
		
		// updates the passed allDriveDir (as reference) vector with all possible drive directions
		// this interface does every thing, process pnt cld, get occupancies, and compute drive directions and gap locations
		// possibleDriveDirections   - vector of all possible drivable directions in degrees
		// possibleDriveGapLocations - center location of the driveable gaps
		bool getDriveDirectionsBasedOnGapWidth(vector<float>& possibleDriveDirections, vector<cv::Point2f>& possibleDriveGapLocations){
		
		
			if(inputCloud->size() >0){		
			
				processCloud();
				getGaps();	
	
				for(int i = 0; i<gapsInAngularOccupancyStartInd.size(); ++i){								
				
					int s = gapsInAngularOccupancyStartInd[i];
					int e = gapsInAngularOccupancyEndInd[i];

					// compute the co-ordinates of the obstacles before and after the start and end positions of the gap respectively
					float theta1 = s*angularOccupancyResolution;
					float theta2 = e*angularOccupancyResolution;

					float x1 = angularOccupancyRho[s-1]*cos(toRadian(theta1));
					float y1 = angularOccupancyRho[s-1]*sin(toRadian(theta1));
				
					float x2 = angularOccupancyRho[e]*cos(toRadian(theta2));
					float y2 = angularOccupancyRho[e]*sin(toRadian(theta2));
				
					float gapWidth = sqrt( pow(y2-y1, 2) + pow(x2-x1, 2) );
					
					//cerr<<"++"<<x1<<","<<y1<<","<<x2<<","<<y2<<endl;
					// check if gap is drivable
					if(gapWidth >= minGapForDriving){
				
						// compute the cooridinates of gap center 
						cv::Point2f gapCenter;
						gapCenter.x = min(x1,x2) + abs(x2-x1)/2;
						gapCenter.y = min(y1,y2) + abs(y2-y1)/2;					
					
						float thetaGapCenter = toDegree(atan(gapCenter.y/gapCenter.x));
					
						if(thetaGapCenter < 0){						
							thetaGapCenter += 180;
						}
						//cerr<<"+"<<gapCenter<<"*"<<gapWidth<<endl;
					
						possibleDriveDirections.push_back(thetaGapCenter);
						possibleDriveGapLocations.push_back(gapCenter);
					}//endif				
				
				}//endfor
				
			}//endif
			else{
				return false;	//cloud empty
			}
		
			return true;	//success			
		}//end func		
		

		// puts the grid in point cloud form into the passed pcl ptr... if only_obstacle flag is True then ony obstacles are passed 
		// as grid else full grid is made into point cloud
		void getGridAsPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, bool onlyObstacle = true){
			
			pcl::PointXYZRGB pt;
			// make a vector from grid[][] for now to display the grid as the PCL message in RVIZ
			for(int i = 0; i<sizeOfGrid; ++i){		//z
				for(int j = 0; j<sizeOfGrid; ++j){	//x
							
					if(grid.at<uchar>(i,j) == 1){
						pt.x = (j-centerOfGridInX)*0.1;						
						pt.z = (i-centerOfGridInZ)*0.1;
						
						if(obstacleSegMethod == 0)
							pt.y = minHeightOfObstacle;	// only height based
						else
							pt.y = roadPlaneParameters[3]-groundTolerance;  // plane based
							
						pt.b = 0;
						pt.r = 250;
						pt.g = 50;							
						cloud->push_back(pt);	//scale it	        	
					}
					if(grid.at<uchar>(i,j) == 0 && !onlyObstacle){
						pt.x = (j-centerOfGridInX)*0.1;
						pt.z = (i-centerOfGridInZ)*0.1;

						if(obstacleSegMethod == 0)
							pt.y = minHeightOfObstacle;
						else
							pt.y = roadPlaneParameters[3]-groundTolerance;  

						pt.b = 0;
						pt.r = 0;
						pt.g = 90;							
						cloud->push_back(pt);	//scale it

					}

				}
			}
		
		}
		
		
		// puts the grid in nav_gridCells msg format  form into the passed pcl ptr... if only_obstacle flag is True then ony obstacles are passed 
		void getGridAsOccupencyGrid(nav_msgs::GridCells& asCells){
			
			geometry_msgs::Point pt;
			// make a vector from grid[][] for now to display the grid as the PCL message in RVIZ
			for(int i = 0; i<sizeOfGrid; ++i){		//z
				for(int j = 0; j<sizeOfGrid; ++j){	//x
							
					if(grid.at<uchar>(i,j) == 1){
						pt.x = (j-centerOfGridInX);						
						pt.z = (i-centerOfGridInZ);
						
						if(obstacleSegMethod == 0)
							pt.y = minHeightOfObstacle;	// only height based
						else
							pt.y = roadPlaneParameters[3]-groundTolerance;  // plane based
							
						asCells.cells.push_back(pt);	//scale it	        	

					}
					if(grid.at<uchar>(i,j) == 0){
						pt.x = (j-centerOfGridInX);
						pt.z = (i-centerOfGridInZ);

						if(obstacleSegMethod == 0)
							pt.y = minHeightOfObstacle;
						else
							pt.y = roadPlaneParameters[3]-groundTolerance;  
												
						asCells.cells.push_back(pt);	//scale it

					}

				}
			}
			
		}
		
		
		// render directions in the grid point cloud
		void renderPossibleDriveDirectionsOnGrid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const vector<float>& dir, const vector<cv::Point2f>& loc){
		
			pcl::PointXYZRGB pt;
			for(int i= 0; i<dir.size(); ++i){
	
			 //if(dir[i] >30){
				pt.x = loc[i].x*10;
				pt.z = loc[i].y*10;
				pt.y = 0;
				
				pt.b=250;
				pt.g = 250;
				pt.r = 250;
				
				cloud->push_back(pt);
			
				
				if(dir[i] >= 10 && dir[i] <= 170){ 
			
					for(int j = 0; j<65; j+=1){
									
						pt.x = int(j* cos(toRadian(dir[i])) );			// use to degree /toRadian here later...
						pt.z = int(j* sin(toRadian(dir[i])) );
						pt.y = -1;
		
						pt.b = 150;
						pt.r = 10;
						pt.g = 150;		
						cloud->push_back(pt);
					}
					
					
				}
			   //}	
			}
			
			
		}
		
		// prints out the angular Occupancy...good for debugging
		void printAngularOccupancy(){
		
			for(int i = 0; i<angularOccupancy.size(); ++i){
					
				cerr<<angularOccupancy[i]<<" ";
			}
			for(int i = 0; i<angularOccupancy.size(); ++i){
					
				cerr<<angularOccupancyRho[i]<<" ";
			}
		}
		
		
		
		// loads "cloud" with only ground points of the inputCloud
		void getGroundPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){ //In, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut
		
			long int cntr = 0;
			for(int i=0; i<inputCloud->size(); ++i){

				cv::Point3f p;
				
				p.x = inputCloud->points[i].x;
				p.y = inputCloud->points[i].y;
				p.z = inputCloud->points[i].z;								
				
				if( (obstacleSegMethod == 1 && isGround(p)) || (obstacleSegMethod == 0 && isGroundByHeight(p,minHeightOfObstacle)) ){						
				
					cloud->push_back(inputCloud->points[i]);
				}
			}
			
		}

		// loads "cloud" with only obstacle points of the inputCloud		
		void getObstaclePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){ //In, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut
		
			long int cntr = 0;
			for(int i=0; i<inputCloud->size(); ++i){

				cv::Point3f p;
				
				p.x = inputCloud->points[i].x;
				p.y = inputCloud->points[i].y;
				p.z = inputCloud->points[i].z;

				if( (obstacleSegMethod == 1 && isObstacle(p)) || (obstacleSegMethod == 0 && isObstacleByHeight(p,minHeightOfObstacle)) ){						
					cloud->push_back(inputCloud->points[i]);
				}
			}
			
		}
		
		
		
		// fit plane to the passed point cloud and return the parameters as cv::Scalar i.e. 4 parameters
		cv::Scalar fitRoadPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr p){
		
			  if(p->size() > 40){

				pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
				pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

				// Create the segmentation object
				pcl::SACSegmentation<pcl::PointXYZRGB> seg;
				// Optional
				seg.setOptimizeCoefficients (true);
				// Mandatory
				seg.setModelType (pcl::SACMODEL_PLANE);
				seg.setMethodType (pcl::SAC_RANSAC);
				seg.setDistanceThreshold (planeFittingDistThres);

				seg.setInputCloud (p);
				seg.segment (*inliers, *coefficients);  
				  
				cv::Scalar planeParameters(coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3]);
	
				std::cerr << "-Model coefficients: " << coefficients->values[0] << " " 
								                    << coefficients->values[1] << " "
								                    << coefficients->values[2] << " " 
								                    << coefficients->values[3] << std::endl;
								                    
				//set and return			                        
				setRoadPlaneParameters(planeParameters);
				return planeParameters;	                                
			  }
			  else{
			  
	  			setRoadPlaneParameters(cv::Scalar(0,0,0,0));
			  	return cv::Scalar(0,0,0,0);
			  	
			  }
			  		
		}
		
		// loads the parameters changed via "rosparam set" command -- make sure of calling it in your callback so that 
		// the parameters values are changed
		void loadParameters(){
			bool load;
			nh->getParam("load_params", load);
			
			if(load){
				nh->getParam("ground_tol", groundTolerance);			// tolerance for extracting ground
				nh->getParam("obstacle_tol", obstacleTolerance);		// tolerance for extracting obstacle
				nh->getParam("grid_size", sizeOfGrid);			// grid size
				nh->getParam("cam_height", cameraHeightFromRoad);
				nh->getParam("angular_occupancy_max_rho", angularOccupancyMaxRho);
				nh->getParam("angular_occupancy_angle_offset", angleOffset);
				nh->getParam("angular_occupancy_resolution", angularOccupancyResolution);						
				nh->getParam("min_safe_driving_gap", minGapForDriving);
				nh->getParam("plane_fitting_distance_thres", planeFittingDistThres);
				nh->getParam("color_input_cloud", colorPointCloud);
				nh->getParam("grid_center_x", centerOfGridInX);
				nh->getParam("grid_center_z", centerOfGridInZ);
				nh->getParam("obstacle_segmentation_method", obstacleSegMethod);				
				nh->getParam("min_height_for_obstacles",minHeightOfObstacle);
				
				// make it false again
				nh->setParam("load_params", false);
			}
			
		}				
		
		
		// it prints out all the parameters
		void printAllParameters(){
			
				printf("%32s - %f\n","ground_tol", groundTolerance);			
				printf("%32s - %f\n","obstacle_tol", obstacleTolerance);		
				printf("%32s - %d\n","grid_size", sizeOfGrid);			
				printf("%32s - %f\n","cam_height", cameraHeightFromRoad);
				printf("%32s - %f\n","angular_occupancy_max_rho", angularOccupancyMaxRho);
				printf("%32s - %f\n","angular_occupancy_angle_offset", angleOffset);
				printf("%32s - %i\n","angular_occupancy_resolution", angularOccupancyResolution);						
				printf("%32s - %f\n","min_safe_driving_gap", minGapForDriving);
				printf("%32s - %f\n","plane_fitting_distance_thres", planeFittingDistThres);
				printf("%32s - %d\n","color_input_cloud", colorPointCloud);
				printf("%32s - %d\n","grid_center_x", centerOfGridInX);
				printf("%32s - %d\n","grid_center_z", centerOfGridInZ);
				printf("%32s - %d\n","obstacle_segmentation_method", obstacleSegMethod);				
				printf("%32s - %f\n","min_height_for_obstacles",minHeightOfObstacle);
		}
		
	private:
	
		// sets up parameters for dynamic change using "rosparam set" command
		void setupParameters(){
		
			nh->setParam("load_params",true);		// flag to load parameters from server in to the node	
			nh->setParam("ground_tol", 0.06);			// tolerance for extracting ground
			nh->setParam("obstacle_tol", 0.215);		// tolerance for extracting obstacle
			nh->setParam("grid_size", 101);			// grid size
			nh->setParam("cam_height", 0.9);
			nh->setParam("angular_occupancy_max_rho", 5.5);
			nh->setParam("angular_occupancy_angle_offset", 35);
			nh->setParam("angular_occupancy_resolution", 1);						
			nh->setParam("min_safe_driving_gap", 1.3);
			nh->setParam("plane_fitting_distance_thres", 0.06);
			nh->setParam("color_input_cloud", false);
			nh->setParam("grid_center_x", 50);
			nh->setParam("grid_center_z", 0);
			nh->setParam("obstacle_segmentation_method", 0);	//0 - height based, 1- plane based
			nh->setParam("min_height_for_obstacles",0.75);		// anthing above this is an obstacle - used only for height based segment.
			
			printAllParameters();
		}


		// make grid 
		inline void makeGrid(){
			if(grid.rows != sizeOfGrid){
				grid.create(sizeOfGrid, sizeOfGrid, CV_8UC1);
				grid.setTo(cv::Scalar(0));							
			}
		}

		
		// this  ses the cloud and generates OccupancyGrid (not probabilistic as of now) and angular Occupancy 
		// bins all in one loop
		void processCloud(){
		
			//on every iteration make+inititalize grid to make sure that grid is cleared and if size has changed then it is allocated accordingly
			makeGrid();
			grid.setTo(cv::Scalar(0));
			
			for(int i=0; i<angularOccupancy.size(); ++i){
				angularOccupancy[i] = 0;
				angularOccupancyRho[i] = angularOccupancyMaxRho;
			} 
				
			for(size_t i = 0; i<inputCloud->size(); ++i){								
			
				float x = inputCloud->points[i].x;
				float z = inputCloud->points[i].z;	
				float y = inputCloud->points[i].y;
				cv::Point3f p(x,y,z);
				// process the points i.e. make grid/make angular occupancies if and only if they are obstacles 
				if( (obstacleSegMethod == 1 && isObstacle(p)) || (obstacleSegMethod == 0 && isObstacleByHeight(p,minHeightOfObstacle)) ){

					// coloring enabled
				    if(colorPointCloud){
				    	inputCloud->points[i].r = 200;
				    	inputCloud->points[i].g = 0;
				    	inputCloud->points[i].b = 0;
				    }
					//translate the points
					double tr_x = x + centerOfGridInX*gridUnit;	
					double tr_z = z + centerOfGridInZ*gridUnit;	;//-=------------
			
					// mark grid to be occupied i.e. 1;				
					int gridXLoc = 0, gridZLoc = 0;
				
					// scale the location of points to the grid location
					if(tr_x >0)
						gridXLoc = int(tr_x / gridUnit);				
					if(tr_z >0)
						gridZLoc = int(tr_z / gridUnit);
								
					// update occupancy and angular occupancy in one go
					if(gridXLoc <sizeOfGrid && gridZLoc < sizeOfGrid) {	
					
						//rowxcol i.e. ZxX							
						grid.at<uchar>(gridZLoc,gridXLoc) = 1;			//uchar is openCV defined for CV_8UC1		
				
						// also update the angular occupancy bins--								
				
						// tralslate the grid locations wrt center
						float z = gridZLoc - centerOfGridInZ;
						float x = gridXLoc - centerOfGridInX;
				
						// converting points to meter units as the grid is in units of 10cm
						x = x*0.1;
						z = z*0.1;

						// distance to the grid location from center
						float dist = sqrt(x*x + z*z);
						int angleBin = 0;
						 
						if(x!=0.0){	
							angleBin = toDegree(atan(z/x))/angularOccupancyResolution;
							// cerr<<x<<","<<z<<"," << "*"<<(atan(z/x)/0.017)<<endl;
						}
						else{

							angleBin = numOfAngularOccupancyBins/2;	// which means 90 degrees i.e. center of the bins
							
						}
					
						if(angleBin < 0)
							angleBin = numOfAngularOccupancyBins+angleBin;	

				
						if(dist <= angularOccupancyMaxRho && dist < angularOccupancyRho[angleBin]){
							angularOccupancy[angleBin] = 1;
							angularOccupancyRho[angleBin] = dist;	// put new Min for that angle value
							
						}
				
						
				
					}//end update_occupancy if
					
				}//end isObstacle cond
				else{
				    if(colorPointCloud){
				    	inputCloud->points[i].r = 0;
				    	inputCloud->points[i].g = 200;
				    	inputCloud->points[i].b = 0;
				    }
				}
				
			}//end for loop
			
		}//end func
		
	
		// generates the start and end indices of the gaps in the angular occupancy bins and stores them in the 
		// vectors gapsInAngularOccupancyStartInd and gapsInAngularOccupancyEndInd
		void getGaps()
		{
			gapsInAngularOccupancyStartInd.clear();
			gapsInAngularOccupancyEndInd.clear();
			int binsToLeave = int(angleOffset/angularOccupancyResolution);	
			int binCntr = 0 + binsToLeave;
			int st = -1, end = -1;
			while(binCntr < (angularOccupancy.size()-binsToLeave)){
		
				if(angularOccupancy[binCntr] == 0 && st == -1)
					st = binCntr;
				if(angularOccupancy[binCntr] == 1 && st != -1){
					end = binCntr;
					//cout<<st<<","<<end<<","<< (st+(end-st)/2)*5<<endl;
					gapsInAngularOccupancyStartInd.push_back(st);
					gapsInAngularOccupancyEndInd.push_back(end);
					st = -1;
					end= -1;
							
				}
		
				binCntr++;
			}
			if(st !=-1){
					end = binCntr-1;
					//cout<<st<<","<<end<<","<< (st+(end-st)/2)*5<<endl;
					gapsInAngularOccupancyStartInd.push_back(st);
					gapsInAngularOccupancyEndInd.push_back(end);
					st = -1;
					end= -1;
			}
			
			//fixing the issue of not having a valid rho in the angularOccupancyrho vector when the 
			
			int sizeOfIndices = gapsInAngularOccupancyEndInd.size(); 			
			
			if(angularOccupancy[gapsInAngularOccupancyStartInd[0]] == 0){
				angularOccupancyRho[gapsInAngularOccupancyStartInd[0]-1] = angularOccupancyRho[gapsInAngularOccupancyEndInd[0]];
			}
			if(angularOccupancy[gapsInAngularOccupancyEndInd[sizeOfIndices-1]] == 0){
				angularOccupancyRho[gapsInAngularOccupancyEndInd[sizeOfIndices-1]] = angularOccupancyRho[gapsInAngularOccupancyStartInd[sizeOfIndices-1]-1];
			}
						
	
		}	


	public:		

	private:			
	
		cv::Mat grid;										// to store the grid CV_8UC1
		int sizeOfGrid	;
		int centerOfGridInX;
		int centerOfGridInZ;
		float gridUnit;									// size of each grid block in meters
		
		int angularOccupancyResolution;					// resolution of angular occupancy bins
		int numOfAngularOccupancyBins;				
		vector<float> angularOccupancyRho;				// stores the distance to the obstacle corresponding to the angularOccupancy bin
		vector<int> angularOccupancy;					// encodes the presence of obstacle corresponding to the angle bin
		float angularOccupancyMaxRho;					// maximum distance to be looked for in Rho direction while making angular occupancy bins
		float angleOffset;								// angles to leave from both ends of the bin
		
		vector<int> gapsInAngularOccupancyStartInd;		// stores the indices of start of gap  
		vector<int> gapsInAngularOccupancyEndInd;		// end indices, again index of the gap
														// if bins - 10001	startInd = 1, endInd = 3, gap = endInd - startInd + 1;		
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud;			// shared pointer to the point cloud
		bool colorPointCloud;										// flag for coloring the point cloud to show segmentation 
		
		cv::Scalar roadPlaneParameters;					// stores A,B,C,D of the plane eq. Ax + By + Cz + D = 0;
		float cameraHeightFromRoad;						// used in qualifying points as road or obstacle
				
		float minGapForDriving;							// the minimum grap for the vehicle to pass safely	
		
		ros::NodeHandle* nh;
		
		float groundTolerance;		
		float obstacleTolerance;
		float planeFittingDistThres;
		int obstacleSegMethod;						// segment obstacle via Plane fitting on road (1) or just by height (0);
		float minHeightOfObstacle;						// used in just height based obstacle segmentation

		
};

	


/*end of class ----- start of GLOBAL VARIABLES AND FUNCTIONS ---------------------*/


ExploreDirectionEstimator* estimateDir;

float maxZforPlaneFit;
float maxXforPlaneFit;
float maxYforPlaneFit;

float minZ,maxY;
float markerPosOffset;

// publish drive direction markers--BadDirs mean those direction which might not be appropriate inspite of availability of safe drivable
// gap in that directions..mostly because of to sharp turning required...
// gapMark means a line being drawn in the gap of drivable gap
// connectGap means connect the center and gap center while publishing the marker
void publishDirections(const vector<float>& dirs, const vector<cv::Point2f>& locs, const vector<cv::Vec4f>& gapEndPoints, std_msgs::Header h, bool publishGapMark = false, bool connectToGap = false, bool rejectBadDirs = false){		
		
		visualization_msgs::Marker marker;
		visualization_msgs::Marker gapMarker;	
		visualization_msgs::Marker freeSpace;
		
		geometry_msgs::PoseArray markerDirectinosAsPoses;
		geometry_msgs::Pose poseOfMarker;
		
		//h.frame_id="my_zed_optical_frame";			// for debuggin only -- to be removed;
		
		markerDirectinosAsPoses.header = h;
		
		marker.header = h;		
		marker.ns = "drive_directions";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::LINE_LIST;
		marker.action = visualization_msgs::Marker::ADD;

		float markerHeight;
		cv::Scalar roadPlaneParams;
		
		if(estimateDir->getObstacleSegmentationMethod() == 0){
			markerHeight = estimateDir->getMinHeightOfObstacle();
		}
		else{
			roadPlaneParams = estimateDir->getRoadPlaneParameters();
			markerHeight= roadPlaneParams[3]-estimateDir->getObstacleTolerance();	// right on the base of obstacles
		}
		
		markerHeight = markerPosOffset + markerHeight;
		
		if(publishGapMark){
			gapMarker.header = h;
			gapMarker.ns="gap_markers";
			gapMarker.id = 0;
			gapMarker.type = visualization_msgs::Marker::LINE_LIST;
			gapMarker.action = visualization_msgs::Marker::ADD;
		}
		
	
			freeSpace.header = h;
			freeSpace.ns="gap_markers";
			freeSpace.id = 0;
			freeSpace.type = visualization_msgs::Marker::TRIANGLE_LIST;
			freeSpace.action = visualization_msgs::Marker::ADD;
		

		// construct the directions with line_list 		
	    geometry_msgs::Point p;	    
	
	    for(int i = 0; i<dirs.size(); ++i){										
			
			//check for inappropriate direction and if required reject them
			if(rejectBadDirs){
					
					if(dirs[i] >50 && dirs[i] < 140){										
						
						// avoid divide by zero issue .. OR rather avoid for 90 deg dir
						if( (gapEndPoints[i])[0] != (gapEndPoints[i])[2] ){
						
							 float perpGapDir= estimateDir->toDegree(atan( abs((gapEndPoints[i])[1]-(gapEndPoints[i])[3]) / abs((gapEndPoints[i])[0]-(gapEndPoints[i])[2]) ) );      							 
							 //reject perpendicular dir below certain threshold 
							 if(90-perpGapDir < 40) 				// note: a vertical gap in Z has 90-perpGapDir = 0
							 	continue;	//skip this drive direction 
						}
					}
					
			}
			
			//make free space using triangle list
			p.x = p.z = 0;
			p.y = markerHeight;
			freeSpace.points.push_back(p);
			
			// make gap markers 			
			p.x = (gapEndPoints[i])[0];
			p.z = (gapEndPoints[i])[1];
			p.y = markerHeight;
			gapMarker.points.push_back(p);
			freeSpace.points.push_back(p);
			// make gap markers
			p.x = (gapEndPoints[i])[2];
			p.z = (gapEndPoints[i])[3];
			p.y = markerHeight;
			gapMarker.points.push_back(p);
			freeSpace.points.push_back(p);												
			
			freeSpace.scale.x = 1;
			freeSpace.scale.y = 1;
			freeSpace.scale.z = 1;
			freeSpace.color.a = 0.5;
			freeSpace.color.g = 200;
			
			// from origin
			p.x = 0;
			p.y = markerHeight;
			p.z = 0;
			marker.points.push_back(p);
			
			if(!connectToGap){
				// to direction				    	
				p.x = 2 * cos(estimateDir->toRadian(dirs[i])) ;
				p.y = markerHeight;
				p.z = 2 * sin(estimateDir->toRadian(dirs[i])) ;
				marker.points.push_back(p);
	    	}
	    	else{
	    		cerr<<"locs"<<locs[i].x<<endl;
	    		// to direction				    	
				p.x = locs[i].x;// locs is in grid coordinates in which each unit is
				p.y = markerHeight;//rpParams[3]-estimateDir->getObstacleTolerance();
				p.z = locs[i].y;//locs * sin(estimateDir->toRadian(dirs[i])) ;
				marker.points.push_back(p);
	    	}

	    	geometry_msgs::Point p;
	    	p.x = 0;// locs is in grid coordinates in which each unit is
			p.y = markerHeight;//rpParams[3]-estimateDir->getObstacleTolerance();
			p.z = 0;//locs * sin(estimateDir->toRadian(dirs[i])) ;

	    	poseOfMarker.position = p;
	    	poseOfMarker.orientation =tf::createQuaternionMsgFromRollPitchYaw(dirs[i],0,0);		/// assuming the function takes in degree units
	    	markerDirectinosAsPoses.poses.push_back(poseOfMarker);
	    }
		  
		
		marker.scale.x = 0.1;
		marker.scale.y = 0.1;
		marker.scale.z = 1;
		
		marker.color.b = 205;
		marker.color.a = 1;
		marker.lifetime = ros::Duration();
		pubDriveDirectionGlobal.publish(marker);
		
		if(publishGapMark){
			gapMarker.scale.x = 0.15;
			gapMarker.scale.z = 1;
			gapMarker.color.r = 53;
			gapMarker.color.g = 50;
			gapMarker.color.b = 200;
			
			gapMarker.color.a = 0.6;
			gapMarker.lifetime = ros::Duration();
			pubGapMarkersGlobal.publish(gapMarker);
		}
						
		pubDriveDirectionAsPoses.publish(markerDirectinosAsPoses);
		pubFreeSpaceMarker.publish(freeSpace);
}



/* callback for disparity, l_image, both info messages.
   This function operates on disparity images. It first computes the point cloud from disparity and then
   computes directins, grid, ground, obstacles, etc.. and publish them.
*/

void computeDirectionsFromDisparity( const ImageConstPtr& l_image_msg,
                                 	 const CameraInfoConstPtr& l_info_msg,
                                 	 const CameraInfoConstPtr& r_info_msg,
                                 	 const DisparityImageConstPtr& disp_msg) {

	int inputMode;
	n->getParam("input_mode", inputMode);
	if(inputMode == 0){	// disparity mode
	
	
		bool loadParam;

		// load the parameters .. parameters only change if "load_params" is true;

		// this sequence of loading these parameters and then calling loadParameters() is important...dont change the sequence as
		// if we call loadParameters() before then the load_params value will be set to FALSE and hence max_Z...these parameters will not change
		n->getParam("load_params",loadParam);
		if(loadParam){
			n->getParam("max_Z_for_plane_fitting",maxZforPlaneFit);
			n->getParam("max_X_for_plane_fitting",maxXforPlaneFit);
			n->getParam("max_Y_for_plane_fitting",maxYforPlaneFit);
			n->getParam("marker_z_position_offset",markerPosOffset);
			n->getParam("min_Z",minZ);	//for trimming incoming point cloud or point cloud from disparty
			n->getParam("max_Y",maxY);	// "
		
		}			
		estimateDir->loadParameters();
		
		
		// look for and change grid size from ros param value - "grid_size"
		//int grSize;		
		//if(n->getParam("grid_size",grSize))
			//estimateDir->setGridSize(grSize);
			
		cv_bridge::CvImageConstPtr cv_ptr, cv_ptr_d;
		cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
		cv_ptr_d = cv_bridge::toCvCopy(disp_msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
		  
		image_geometry::StereoCameraModel model;
		model.fromCameraInfo(*l_info_msg, *r_info_msg);

		// to store only the ground points	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		point_cloud->header.frame_id = "my_zed_optical_frame";//l_image_msg->header.frame_id;
		point_cloud->header.stamp = pcl_conversions::toPCL(l_info_msg->header).stamp;
		point_cloud->width = 1;	

	 	// to store the color coded point cloud	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ground(new pcl::PointCloud<pcl::PointXYZRGB>);			
		point_cloud_ground->header.frame_id = point_cloud->header.frame_id;//l_image_msg->header.frame_id;
		point_cloud_ground->header.stamp = pcl_conversions::toPCL(l_info_msg->header).stamp;
		point_cloud_ground->width = 1;
	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_obstacle(new pcl::PointCloud<pcl::PointXYZRGB>);				
		point_cloud_obstacle->header.frame_id = point_cloud->header.frame_id;//l_image_msg->header.frame_id;
		point_cloud_obstacle->header.stamp = pcl_conversions::toPCL(l_info_msg->header).stamp;
		point_cloud_obstacle->width = 1;	

	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_grid(new pcl::PointCloud<pcl::PointXYZRGB>);				
		point_cloud_grid->header.frame_id = point_cloud->header.frame_id;//l_image_msg->header.frame_id;
		point_cloud_grid->header.stamp = pcl_conversions::toPCL(l_info_msg->header).stamp;
		point_cloud_grid->width = 1;	
		
		for(size_t i =cv_ptr_d->image.rows/2; i <=cv_ptr_d->image.rows; ++i){	//cv_ptr_d->image.rows/2
			for(size_t j =0; j <=cv_ptr_d->image.cols; ++j){
			
				
					cv::Point3d point;	//
					cv::Point2d px(j,i);
					float disp = cv_ptr_d->image.at<float>(i,j);
					model.projectDisparityTo3d(px, disp, point);
					pcl::PointXYZRGB p;
					// != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(point.z) 
					
						
					if(point.y >maxY && point.z > minZ && point.z<=estimateDir->getGridSize()*0.1 &&  point.z!= image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(point.z) && !isnan(point.z)){

						p.x = point.x;
						p.y = point.y;
						p.z = point.z;				       
						p.r = cv_ptr->image.at<float>(i,j);
						p.g = cv_ptr->image.at<float>(i,j);
						p.b = cv_ptr->image.at<float>(i,j);
					
						// points to be passed for plane fitting
						if( p.x >=-maxXforPlaneFit && p.x <= maxXforPlaneFit && p.z<maxZforPlaneFit && p.y>maxYforPlaneFit){  //p.y>1.4 only for kitti_set
											
							point_cloud_ground->push_back(p);
		
						}	
					
						// all points
						point_cloud->push_back(p);
					}
				
				
			}
		}

		estimateDir->setPointCloud(point_cloud);
	
	    //fit plane only when obstacle segmentation method = 1 i.e. road plane fitting based		
		if(estimateDir->getObstacleSegMethod() == 1)
			cv::Scalar rpParams = estimateDir->fitRoadPlane(point_cloud_ground);
			
		dirs.clear();
		locs.clear();
	    gapEndPoints.clear();
	    
		estimateDir->getDriveDirectionsBasedOnGapWidth(dirs, locs, gapEndPoints);	
		point_cloud_ground->clear();
	
		estimateDir->getGridAsPointCloud(point_cloud_grid, false);		// with free space rendered
						
	//  GROUND POINTS --- point_cloud_new
		
		float tol=0.1;
		n->getParam("ground_tol", tol);
		estimateDir->getGroundPoints(point_cloud_ground);
		estimateDir->getObstaclePoints(point_cloud_obstacle);	
		
		pub_point_cloud.publish(point_cloud);
		pub_grid_view.publish(point_cloud_grid);
		pub_ground_points.publish(point_cloud_ground);
		pub_obstacle_points.publish(point_cloud_obstacle);
		
		publishDirections(dirs, locs, gapEndPoints, l_info_msg->header, true, false  , true);

	}//if disparity

}



/* callback for point cloud of type PointCloud2.
   This function directly operates on the incoming point cloud and then
   computes directins, grid, ground, obstacles, etc.. and publish them.
*/

void computeDirectionsFromPointCloud(const sensor_msgs::PointCloud2ConstPtr &cloud){
				
	int inputMode;
	n->getParam("input_mode", inputMode);
	if(inputMode == 1){	//point clloud mode

	
		bool loadParam;

		// load the parameters .. parameters only change if "load_params" is true;

		// this sequence of loading these parameters and then calling loadParameters() is important...dont change the sequence as
		// if we call loadParameters() before then the load_params value will be set to FALSE and hence max_Z...these parameters will not change
		n->getParam("load_params",loadParam);
		if(loadParam){
			n->getParam("max_Z_for_plane_fitting",maxZforPlaneFit);
			n->getParam("max_X_for_plane_fitting",maxXforPlaneFit);
			n->getParam("max_Y_for_plane_fitting",maxYforPlaneFit);
			n->getParam("marker_z_position_offset",markerPosOffset);
			n->getParam("min_Z",minZ);	//for trimming incoming point cloud or point cloud from disparty
			n->getParam("max_Y",maxY);	// "
		
		}			
		estimateDir->loadParameters();

		// to store only the ground points	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_conv(new pcl::PointCloud<pcl::PointXYZRGB>);
		
		pcl::fromROSMsg(*cloud, *point_cloud_conv);
		point_cloud_conv->width = 1;	
		
		// to store the color coded point cloud	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);			
		point_cloud->header = point_cloud_conv->header;
		point_cloud->width = 1;
		
	 	// to store the color coded point cloud	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ground(new pcl::PointCloud<pcl::PointXYZRGB>);			
		point_cloud_ground->header = point_cloud->header;
		point_cloud_ground->width = 1;
	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_obstacle(new pcl::PointCloud<pcl::PointXYZRGB>);				
		point_cloud_obstacle->header = point_cloud->header;
		point_cloud_obstacle->width = 1;	

	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_grid(new pcl::PointCloud<pcl::PointXYZRGB>);				
		point_cloud_grid->header = point_cloud->header;
		point_cloud_grid->width = 1;	
				
		cv::Point3f p;
		// make points for road to be fit to be fit..
		for(size_t i = 0; i<point_cloud_conv->size(); ++i){
			
			
			p.x = point_cloud_conv->points[i].x;
			p.y = point_cloud_conv->points[i].y;
			p.z = point_cloud_conv->points[i].z;				       
			if(p.y >maxY && p.z > minZ && p.z<=estimateDir->getGridSize()*0.1 && p.z!= image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(p.z) && !isnan(p.z)){
				
				// points to be passed for plane fitting
				if( p.x >=-maxXforPlaneFit && p.x <= maxXforPlaneFit && p.z<maxZforPlaneFit && p.y>maxYforPlaneFit){  //p.y>1.4 only for kitti_set
					point_cloud_ground->push_back(point_cloud_conv->points[i]);

				}
				
				point_cloud->push_back(point_cloud_conv->points[i]);
			}
					
		}
		
		estimateDir->setPointCloud(point_cloud);
		
		//fit plane only when obstacle segmentation method = 1 i.e. road plane fitting based		
		if(estimateDir->getObstacleSegMethod() == 1)
			cv::Scalar rpParams = estimateDir->fitRoadPlane(point_cloud_ground);
			
		dirs.clear();
		locs.clear();
	    gapEndPoints.clear();
	    
		estimateDir->getDriveDirectionsBasedOnGapWidth(dirs, locs, gapEndPoints);	
		point_cloud_ground->clear();
	
		estimateDir->getGridAsPointCloud(point_cloud_grid, false);		// with free space rendered
						
	//  GROUND POINTS --- point_cloud_new
		
		estimateDir->getGroundPoints(point_cloud_ground);
		estimateDir->getObstaclePoints(point_cloud_obstacle);	
		
		pub_point_cloud.publish(point_cloud);
		pub_grid_view.publish(point_cloud_grid);
		pub_ground_points.publish(point_cloud_ground);
		pub_obstacle_points.publish(point_cloud_obstacle);
		
		publishDirections(dirs, locs, gapEndPoints, cloud->header, true, false, true);
		
	}
}


int main(int argc, char **argv){		


	printf("\n\n%32s - %s\n","Parameters","Values");
	printf("%32s - %s\n","--------------------------------","----------");
			
    ros::init(argc, argv, "explore");
	n = new ros::NodeHandle("~");

	estimateDir = new ExploreDirectionEstimator(n);

	n->setParam("max_Z_for_plane_fitting", 10);
	n->setParam("max_X_for_plane_fitting", 1);
	n->setParam("max_Y_for_plane_fitting", estimateDir->getCameraHeightFromRoad()-0.3);
	n->setParam("min_Z", 1.5);
	n->setParam("max_Y", -1.0);
	n->setParam("marker_z_position_offset",-0.1);
	n->setParam("input_mode", 1);	//0-means use disparity, 1- use point cloud
	
	
	printf("%32s - %f\n","marker_z_position_offset",-0.1);
	printf("%32s - %f\n","max_Z_for_plane_fitting", 10.0);			
	printf("%32s - %f\n","max_X_for_plane_fitting", 1.0);		
	printf("%32s - %f\n","max_Y_for_plane_fitting", estimateDir->getCameraHeightFromRoad()-0.3);			
	printf("%32s - %f\n","min_Z", 1.5);		
	printf("%32s - %f\n","max_Y", -1.0);
	printf("%32s - %i\n","input_method", 1);	

	message_filters::Subscriber<sensor_msgs::Image> sub_l_img(*n, "/stereo_camera/left/image_rect", 10);
	message_filters::Subscriber<DisparityImage> sub_disp_img(*n, "/stereo_camera/disparity", 10);
	message_filters::Subscriber<sensor_msgs::CameraInfo> sub_l_info(*n, "/stereo_camera/left/camera_info_throttle", 10);
	message_filters::Subscriber<CameraInfo> sub_r_info(*n, "/stereo_camera/right/camera_info_throttle", 10);

	typedef sync_policies::ApproximateTime<Image, CameraInfo, CameraInfo, DisparityImage> myApprxSyncPolicy;
	
	Synchronizer<myApprxSyncPolicy> sync(myApprxSyncPolicy(10), sub_l_img, sub_l_info,sub_r_info, sub_disp_img);
	sync.registerCallback(boost::bind(computeDirectionsFromDisparity, _1, _2,_3,_4));

	ros::Subscriber sub_point_cloud = n->subscribe("/stereo_camera/points2", 10, computeDirectionsFromPointCloud);

    //sub = nh.subscribe("/camera/points2", 100, pointCloudFromDisparity);  
	pub_point_cloud = n->advertise<pcl::PointCloud<pcl::PointXYZRGB> >("incoming_point_cloud", 1);
	pub_grid_view = n->advertise<pcl::PointCloud<pcl::PointXYZRGB> >("grid_view", 1);
	pub_ground_points = n->advertise<pcl::PointCloud<pcl::PointXYZRGB> >("ground_points", 1);
	pub_obstacle_points = n->advertise<pcl::PointCloud<pcl::PointXYZRGB> >("obstacle_points", 1);
		
    pubDriveDirectionGlobal = n->advertise<visualization_msgs::Marker>("drive_directions",1);	
    pubGapMarkersGlobal = n->advertise<visualization_msgs::Marker>("gap_marks",1);	
    pubDriveDirectionAsPoses = n->advertise<geometry_msgs::PoseArray>("directions_as_poses",1);    
    pubFreeSpaceMarker = n->advertise<visualization_msgs::Marker>("free_space_marker", 1);
    
	ros::spin();
	
	delete n;			// delete node handle pointer memory block
	delete estimateDir;	
}





