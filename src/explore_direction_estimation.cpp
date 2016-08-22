
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
				  	  - ~/ground_points
				  	  - ~/obstacle_points
				  	  - ~/grid_view 
				  	  - ~/incomming_point_cloud (if coloring set...it is colored based on segmentation)
				  	  - ~/drive_directions	(markers for drive direction)
				  	  - ~/gap_marks (gaps are drawn as lines in green color)		
				  	  - ~/direction_as_poses (drive directions as PoseArray msg - position - gapcenterpos, orientation - direction)
				  	  
				  	  
	Note - More info on parameters to be here soon....				  								  
	
	ToDo - publish a range of directions for ever gap so that the PLANNER node can decide upon which is most suitable
	
	kind of a problem - not a problem but sometimes I find that this node does not get the subscribed messages... dont know why...
	
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

ros::Subscriber sub;
ros::Subscriber imageSub_l;
ros::Subscriber imageSub_r;
ros::Subscriber sub_point_cloud;

ros::Publisher pub_point_cloud;			// to publish the incoming point cloud (colored/not colored)
ros::Publisher pub_ground_points;		// 			  ground points
ros::Publisher pub_grid_view;			//		      grid view with possible drive directions		 
ros::Publisher pubDriveDirectionGlobal;	//			  possible drive directions as marker array in global frame
ros::Publisher pub_obstacle_points;
ros::Publisher pubGapMarkersGlobal;		// publish gap markers
ros::Publisher pubDriveDirectionAsPoses;
bool useDisparity = true;

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
			
			grid = cv::Mat::zeros(sizeOfGrid,sizeOfGrid,CV_8UC1);				
		
			// make sure it is even in number
			numOfAngularOccupancyBins = 180/angularOccupancyResolution;	
			angularOccupancy.assign(numOfAngularOccupancyBins, 0);
			angularOccupancyRho.assign(numOfAngularOccupancyBins, angularOccupancyMaxRho);					
			
			//init plane parameters...four parameters
			roadPlaneParameters[0] = roadPlaneParameters[1] = roadPlaneParameters[2] = roadPlaneParameters[3] = 0;			
			
			planeFittingDistThres = 0.06;
			// setup the parameters in the ros parameter server	-- loaded with the same default values as above
			setupParameters();
			setObstacleTolerance(0.3);
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
		
		inline float getCameraHeightFromRoad(){
			
			return cameraHeightFromRoad;
		}
		// height of the camera from the road plane. used to segment points in a naive fasion as, points < height are obstacles
		inline void setCameraHeightFromRoad(float height){
			nh->setParam("cam_height", height);
			cameraHeightFromRoad=height;
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
		
		
		// this functin can be used when we want qualify obstacles based on the height of from camera
		inline bool isObstacleByHeight(cv::Point3f p, float height){
			
			return ((p.y <= height))?true:false;
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
		inline float toDegree(float rad){
			return rad*57.29580;
		}
		inline float toRadian(float deg){
			return deg/57.29580;
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

		inline float getGroundTolerance(){
			return groundTolerance;
		}
		
		
		inline float getObstacleTolerance(){
			return obstacleTolerance;
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
						
						cerr<<"+"<<gapCenter<<"*"<<gapWidth<<endl;
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
						cerr<<"+"<<gapCenter<<"*"<<gapWidth<<endl;
					
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
						pt.y = 0;
						pt.z = (i-centerOfGridInZ)*0.1;
						pt.b = 0;
						pt.r = 250;
						pt.g = 50;							
						cloud->push_back(pt);	//scale it	        	
					}
					if(grid.at<uchar>(i,j) == 0 && !onlyObstacle){
						pt.x = (j-centerOfGridInX)*0.1;
						pt.y = 0;
						pt.z = (i-centerOfGridInZ)*0.1;
						pt.b = 0;
						pt.r = 0;
						pt.g = 90;							
						cloud->push_back(pt);	//scale it

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
			cerr<<"------------"<<endl;
		}
		
		
		
		// loads "cloud" with only ground points of the inputCloud
		void getGroundPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){ //In, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut
		
			long int cntr = 0;
			for(int i=0; i<inputCloud->size(); ++i){

				cv::Point3f p;
				
				p.x = inputCloud->points[i].x;
				p.y = inputCloud->points[i].y;
				p.z = inputCloud->points[i].z;								
				if(isGround(p)){
				
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
				if(isObstacle(p)){
				
					cloud->push_back(inputCloud->points[i]);
				}
			}
			
		}
		
		
		// fit plane to the passed point cloud and return the parameters as cv::Scalar i.e. 4 parameters
		cv::Scalar getPlaneParameters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr p){
		
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
	
				std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
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
				// make it false again
				nh->setParam("load_params", false);
			}
			
		}
		
		inline void setGroundTolerance(float groundTol){
		
			groundTolerance = groundTol;
			nh->setParam("ground_tol", groundTol);				
		}
		
		inline void setObstacleTolerance(float obstacleTol){
		
			obstacleTolerance = obstacleTol;		
			nh->setParam("obstacle_tol", obstacleTol);				
		}
		
		
	private:
	
		// sets up parameters for dynamic change using "rosparam set" command
		void setupParameters(){
		
			nh->setParam("load_params", false);		// flag to load parameters from server in to the node	
			nh->setParam("ground_tol", 0.1);			// tolerance for extracting ground
			nh->setParam("obstacle_tol", 0.3);		// tolerance for extracting obstacle
			nh->setParam("grid_size", 101);			// grid size
			nh->setParam("cam_height", 0.9);
			nh->setParam("angular_occupancy_max_rho", 8);
			nh->setParam("angular_occupancy_angle_offset", 35);
			nh->setParam("angular_occupancy_resolution", 1);						
			nh->setParam("min_safe_driving_gap", 1.5);
			nh->setParam("plane_fitting_distance_thres", 0.06);
			nh->setParam("color_input_cloud", false);
			nh->setParam("grid_center_x", 50);
			nh->setParam("grid_center_z", 0);
			
			
		}


		// make grid 
		inline void makeGrid(){
			if(grid.rows != sizeOfGrid){
				grid.create(sizeOfGrid, sizeOfGrid, CV_8UC1);
				grid.setTo(cv::Scalar(0));							
			}
		}

		
		// this processes the cloud and generates OccupancyGrid (not probabilistic as of now) and angular Occupancy 
		// bins all in one loop
		void processCloud(){
		
			//on every iteration make+inititalize grid to make sure that grid is cleared and if size has changed then it is allocated accordingly
			grid.setTo(cv::Scalar(0));

			for(int i=0; i<angularOccupancy.size(); ++i){
				angularOccupancy[i] = 0;
				angularOccupancyRho[i] = angularOccupancyMaxRho;
			} 
				
			for(size_t i = 0; i<inputCloud->size(); ++i){								
			
				float x = inputCloud->points[i].x;
				float z = inputCloud->points[i].z;	
				float y = inputCloud->points[i].y;
				
				// process the points i.e. make grid/make angular occupancies if and only if they are obstacles 
				if( isObstacle(cv::Point3f(x,y,z)) ){
				
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
			/*if(sizeOfInd != 0)
			{
				int firstStartInd = gapsInAngularOccupancyStartInd[0] 
				int lastEndInd = gapsInAngularOccupancyStartInd[sizeOfInd-1];
			
				if(gapsInAngularOccupancyStartInd.size() == 1){
				
					if(angularOccupancy[firstStartInd] == 0 )
				}	
			
				if(fistStartInd != 0){
			
				if(angularOccupancy[firstStartInd-1] != 1){
					
					angularOccupancy[firstStartInd-1] = 1;
					angualrOccupancyRho[[firstStartInd-1] =  angularOccupancyRho[firstStartInd+]
				}
			}*/
			if(angularOccupancy[gapsInAngularOccupancyStartInd[0]] == 0){
				angularOccupancyRho[gapsInAngularOccupancyStartInd[0]-1] = angularOccupancyRho[gapsInAngularOccupancyEndInd[0]];
			}
			if(angularOccupancy[gapsInAngularOccupancyEndInd[sizeOfIndices-1]] == 0){
				angularOccupancyRho[gapsInAngularOccupancyEndInd[sizeOfIndices-1]] = angularOccupancyRho[gapsInAngularOccupancyStartInd[sizeOfIndices-1]-1];
			}
			
			//cerr<<"getGaps-"<<
			//if(gapsInAngularOccupancyEndInd[sizeOfIndices-1] == numOfAngularOccupancyBins-1 && angularOccupancy[numOfAngularOccupancyBins-1] == 0){
			//	angularOccupancyRho[numOfAngularOccupancyBins-1] = angularOccupancyRho[gapsInAngularOccupancyStartInd[sizeOfIndices-1]-1];
			//}
	
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
		
};

	


/*end of class ----- start of GLOBAL VARIABLES AND FUNCTIONS ---------------------*/


ExploreDirectionEstimator* estimateDir;
cv::Scalar rpParams; 	//store the road plane parameters


// publish drive direction markers--BadDirs mean those direction which might not be appropriate inspite of availability of safe drivable
// gap in that directions..mostly because of to sharp turning required...
// gapMark means a line being drawn in the gap of drivable gap
// connectGap means connect the center and gap center while publishing the marker
void publishDirections(const vector<float>& dirs, const vector<cv::Point2f>& locs, const vector<cv::Vec4f>& gapEndPoints, std_msgs::Header h, bool publishGapMark = false, bool connectToGap = false, bool rejectBadDirs = false){
	
		visualization_msgs::Marker marker;
		visualization_msgs::Marker gapMarker;	
		geometry_msgs::PoseArray markerDirectinosAsPoses;
		geometry_msgs::Pose poseOfMarker;
		
		markerDirectinosAsPoses.header = h;

		marker.header = h;		
		marker.ns = "drive_directions";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::LINE_LIST;
		marker.action = visualization_msgs::Marker::ADD;

		if(publishGapMark){
			gapMarker.header = h;
			gapMarker.ns="gap_markers";
			gapMarker.id = 0;
			gapMarker.type = visualization_msgs::Marker::LINE_LIST;
			gapMarker.action = visualization_msgs::Marker::ADD;
		}

		// construct the directions with line_list 		
	    geometry_msgs::Point p;	    
	
	    for(int i = 0; i<dirs.size(); ++i){										
			
			//check for inappropriate direction and if required reject them
			if(rejectBadDirs){
					
					if(dirs[i] >50 && dirs[i] < 140){										
						
						// avoid divide by zero issue .. OR rather avoid for 90 deg dir
						if( (gapEndPoints[i])[0] != (gapEndPoints[i])[2] ){
						
							 float perpGapDir= estimateDir->toDegree(atan( abs((gapEndPoints[i])[1]-(gapEndPoints[i])[3]) / abs((gapEndPoints[i])[0]-(gapEndPoints[i])[2]) ) );      							 
							 cerr<<90-perpGapDir<<endl;
							 //reject perpendicular dir below certain threshold 
							 if(90-perpGapDir < 40) 				// note: a vertical gap in Z has 90-perpGapDir = 0
							 	continue;	//skip this drive direction 
						}
					}
					
			}
			
			// make gap markers - WE MIGHT HAVE TO SCALE THESE MARKERS AS MAY BE THEY ARE IN GRID COORDINATES..
			p.x = (gapEndPoints[i])[0];
			p.z = (gapEndPoints[i])[1];
			p.y = rpParams[3]-estimateDir->getObstacleTolerance();						// the height at which we want to show the marker...
			gapMarker.points.push_back(p);

			// make gap markers
			p.x = (gapEndPoints[i])[2];
			p.z = (gapEndPoints[i])[3];
			p.y = rpParams[3]-estimateDir->getObstacleTolerance();						// the height at which we want to show the marker...
			gapMarker.points.push_back(p);			
			
			// from origin
			p.x = 0;
			p.y = 0;
			p.z = 0;
			marker.points.push_back(p);
			
			if(!connectToGap){
				// to direction				    	
				p.x = 2 * cos(estimateDir->toRadian(dirs[i])) ;
				p.y = 0;
				p.z = 2 * sin(estimateDir->toRadian(dirs[i])) ;
				marker.points.push_back(p);
	    	}
	    	else{
	    		cerr<<"locs"<<locs[i].x<<endl;
	    		// to direction				    	
				p.x = locs[i].x;// locs is in grid coordinates in which each unit is
				p.y = rpParams[3]-estimateDir->getObstacleTolerance();
				p.z = locs[i].y;//locs * sin(estimateDir->toRadian(dirs[i])) ;
				marker.points.push_back(p);
	    	}

	    	geometry_msgs::Point p;
	    	p.x = locs[i].x;// locs is in grid coordinates in which each unit is
			p.y = rpParams[3]-estimateDir->getObstacleTolerance();
			p.z = locs[i].y;//locs * sin(estimateDir->toRadian(dirs[i])) ;

	    	poseOfMarker.position = p;
	    	poseOfMarker.orientation =tf::createQuaternionMsgFromYaw(dirs[i]);		/// assuming the function takes in degree units
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
			gapMarker.scale.x = 0.1;
			
			gapMarker.color.r = 53;
			gapMarker.color.g = 220;
			gapMarker.color.b = 0;
			
			gapMarker.color.a = 0.7;
			gapMarker.lifetime = ros::Duration();
			pubGapMarkersGlobal.publish(gapMarker);
		}
						
		pubDriveDirectionAsPoses.publish(markerDirectinosAsPoses);
}




void pointCloudFromDisparity(	 const ImageConstPtr& l_image_msg,
                                 const CameraInfoConstPtr& l_info_msg,
                                 const CameraInfoConstPtr& r_info_msg,
                                 const DisparityImageConstPtr& disp_msg) {

	if(useDisparity){
		float maxZforPlaneFit;
		float maxXforPlaneFit;
		n->getParam("maxZforPlaneFitting",maxZforPlaneFit);
		n->getParam("maxXforPlaneFitting",maxXforPlaneFit);
		
		// load the parameters .. parameters only change if "load_params" is true;
		estimateDir->loadParameters();
		
		// look for and change grid size from ros param value - "grid_size"
		int grSize;		
		if(n->getParam("grid_size",grSize))
			estimateDir->setGridSize(grSize);
			
		cv_bridge::CvImageConstPtr cv_ptr, cv_ptr_d;
		cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
		cv_ptr_d = cv_bridge::toCvCopy(disp_msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
		  
		image_geometry::StereoCameraModel model;
		model.fromCameraInfo(*l_info_msg, *r_info_msg);

		// to store only the ground points	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		point_cloud->header.frame_id = "zed_optical_frame";//l_image_msg->header.frame_id;
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
		
		for(size_t i =cv_ptr_d->image.rows/2; i <=cv_ptr_d->image.rows; ++i){
			for(size_t j =0; j <=cv_ptr_d->image.cols; ++j){
			
				
					cv::Point3d point;	//
					cv::Point2d px(j,i);
					float disp = cv_ptr_d->image.at<float>(i,j);
					model.projectDisparityTo3d(px, disp, point);
					pcl::PointXYZRGB p;
					// != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(point.z) 
					if(point.y >0 && point.z > 0 && point.z<20 &&  point.z!= image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(point.z)){

						p.x = point.x;
						p.y = point.y;
						p.z = point.z;				       
						p.r = cv_ptr->image.at<float>(i,j);
						p.g = cv_ptr->image.at<float>(i,j);
						p.b = cv_ptr->image.at<float>(i,j);
					
						// points to be passed for plane fitting
						if( p.x >=-maxXforPlaneFit && p.x <= maxXforPlaneFit && p.z<maxZforPlaneFit && p.y>estimateDir->getCameraHeightFromRoad() - 0.3){  //p.y>1.4 only for kitti_set
											
							point_cloud_ground->push_back(p);
		
						}	
					
						// all points
						point_cloud->push_back(p);
					}
				
				
			}
		}

		estimateDir->setPointCloud(point_cloud);
	
		//cerr<<point_cloud_seg->size()<<endl;
		rpParams = estimateDir->getPlaneParameters(point_cloud_ground);
			
		dirs.clear();
		locs.clear();
	    gapEndPoints.clear();
	    
		estimateDir->getDriveDirectionsBasedOnGapWidth(dirs, locs, gapEndPoints);	
		point_cloud_ground->clear();
	
		estimateDir->getGridAsPointCloud(point_cloud_grid, false);		// with free space rendered
		estimateDir->renderPossibleDriveDirectionsOnGrid(point_cloud_grid, dirs, locs);
	
	//  GROUND POINTS --- point_cloud_new
	
		float tol=0.1;
		n->getParam("ground_tol", tol);
		estimateDir->getGroundPoints(point_cloud_ground);
		estimateDir->getObstaclePoints(point_cloud_obstacle);	
		
		pub_point_cloud.publish(point_cloud);
		pub_grid_view.publish(point_cloud_grid);
		pub_ground_points.publish(point_cloud_ground);
		pub_obstacle_points.publish(point_cloud_obstacle);
		
		publishDirections(dirs, locs, gapEndPoints, l_info_msg->header, true, true, true);
		
	}//if disparity

}


int main(int argc, char **argv){		
		
	
    ros::init(argc, argv, "explore");
	n = new ros::NodeHandle("~");
	n->setParam("maxZforPlaneFitting", 10);
	n->setParam("maxXforPlaneFitting", 1);
	
	estimateDir = new ExploreDirectionEstimator(n);	
	
		
	message_filters::Subscriber<sensor_msgs::Image> sub_l_img(*n, "/stereo_camera/left/image_rect", 10);
	message_filters::Subscriber<DisparityImage> sub_disp_img(*n, "/stereo_camera/disparity", 10);
	message_filters::Subscriber<sensor_msgs::CameraInfo> sub_l_info(*n, "/stereo_camera/left/camera_info_throttle", 10);
	message_filters::Subscriber<CameraInfo> sub_r_info(*n, "/stereo_camera/right/camera_info_throttle", 10);
	
	typedef sync_policies::ApproximateTime<Image, CameraInfo, CameraInfo, DisparityImage> myApprxSyncPolicy;
	
	Synchronizer<myApprxSyncPolicy> sync(myApprxSyncPolicy(10), sub_l_img, sub_l_info,sub_r_info, sub_disp_img);
	sync.registerCallback(boost::bind(pointCloudFromDisparity, _1, _2,_3,_4));
		
	
    //sub = nh.subscribe("/camera/points2", 100, pointCloudFromDisparity);  
	pub_point_cloud = n->advertise<pcl::PointCloud<pcl::PointXYZRGB> >("incoming_point_cloud", 1);
	pub_grid_view = n->advertise<pcl::PointCloud<pcl::PointXYZRGB> >("grid_view", 1);
	pub_ground_points = n->advertise<pcl::PointCloud<pcl::PointXYZRGB> >("ground_points", 1);
	pub_obstacle_points = n->advertise<pcl::PointCloud<pcl::PointXYZRGB> >("obstacle_points", 1);
		
    pubDriveDirectionGlobal = n->advertise<visualization_msgs::Marker>("drive_directions",1);	
    pubGapMarkersGlobal = n->advertise<visualization_msgs::Marker>("gap_marks",1);	
    pubDriveDirectionAsPoses = n->advertise<geometry_msgs::PoseArray>("directions_as_poses",1);
	ros::spin();
	
	delete n;			// delete node handle pointer memory block
	delete estimateDir;	
}
	





/*
// call back for the point cloud topic consisiting of only obstacles
void onlyObstaclePointCloudCallback(const sensor_msgs::PointCloud2ConstPtr points){


		
	if(usePointCloud){	
		
			
		
		// to store only the ground points	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_grid(new pcl::PointCloud<pcl::PointXYZRGB>);				
		point_cloud_grid->header = point_cloud->header;
		point_cloud_grid->header.frame_id = "kitti_stereo";//l_image_msg->header.frame_id;
		point_cloud_grid->width = 1;				
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_trimmed(new pcl::PointCloud<pcl::PointXYZRGB>);				
		point_cloud_trimmed->header = point_cloud->header;
		point_cloud_trimmed->header.frame_id = "kitti_stereo";//l_image_msg->header.frame_id;
		point_cloud_trimmed->width = 1;				
			
			
		// message to point cloud conversion		
		pcl::fromROSMsg(*points, *point_cloud);
		
		float camHeight = estimateDir->getCameraHeightFromRoad();
		for(size_t i=0; i<point_cloud->size(); ++i){
		
			if(point_cloud->points[i].y < camHeight*0.5 && point_cloud->points[i].y >camHeight*0.5 - 0.5){
				point_cloud_trimmed->push_back(point_cloud->points[i]);
			}
		}
		
		estimateDir->setPointCloud(point_cloud_trimmed);	
			
		point_cloud_grid->header = point_cloud->header;			
	    point_cloud_grid->width = 1;	    		
	
		dirs.clear();
		locs.clear();
		gapEndPoints.clear();
		
		estimateDir->getDriveDirectionsBasedOnGapWidth(dirs, locs, gapEndPoints);							
	
    	estimateDir->getGridAsPointCloud(point_cloud_grid, false);		// with free space rendered
	    estimateDir->renderPossibleDriveDirectionsOnGrid(point_cloud_grid, dirs, locs);
	    
    	
		pub_point_cloud.publish(point_cloud_trimmed);
		pub_grid_view.publish(point_cloud_grid);
		publishDirections(dirs, locs, gapEndPoints, points->header, true, false, true);
		// marker to be published here....
	}
}


*/

/*

Listener listener;
  ros::Subscriber sub = n.subscribe("chatter", 1000, &Listener::callback, &listener);

 rosbag play ~/Downloads/kitti_00.bag /kitti_stereo/left/image_rect:=/kitti_stereo/left/image_raw /kitti_stereo/right/image_rect:=/kitti_stereo/right/image_raw
 
  ROS_NAMESPACE=kitti_stereo rosrun stereo_image_proc stereo_image_proc stereo:=kitti_stereo image:=image_rect


 
 */





/* REMOVED CODES */

/* updates the passed allDriveDir (as reference) vector with all possible
// drive directions; it depends upon the availability of area> areaThres 
void getDriveDirectionsBasedOnArea(const vector<int>& s, const vector<int>& e, float areaThres, float rmin, float rmax, vector<float>& dir){
	
	for(int i = 0; i<s.size(); ++i){
	
		float area = (((e[i] -s[i])*5)/float(360))*PI*(rmax*rmax - rmin*rmin);
		//cout<<"+"<<i<<","<< (((e[i] -s[i])*5)/float(360))	<<endl;
		if(area>=areaThres){
			
			dir.push_back( ((e[i] -s[i])*5) + s[i] );
			cerr<<"++"<<((e[i] -s[i])*5) + s[i] <<endl;
		}
	}
}




float r = 0;
		if(i!=0)
			r = (binsMin[e[i]]+ binsMin[s[i]-1])/2;//? binsMin[e[i]] : binsMin[s[i]]; 
		else
			r = binsMin[e[i]];
			
			
			
			
			
			
			
*/





/*// filter point cloud using PCL .... Dont break your head a lot...
void publishSmoothedPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointsInPCLForm){

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsInPCLForm(new pcl::PointCloud<pcl::PointXYZRGB>);	
	//pcl::fromROSMsg(*cloud, *pointsInPCLForm);

	
	// Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  
  mls.setComputeNormals (false);
  
  
  //remove NAN values or u might get run time problems like:
  
  //build/pcl-1.7-K_Z193/pcl-1.7-1.7.1/kdtree/include/pcl/kdtree/impl/kdtree_flann.hpp:172: int pcl::KdTreeFLANN<PointT, Dist>::radiusSearch(const PointT&, double, std::vector<int>&, std::vector<float>&, unsigned int) const [with PointT = pcl::PointXYZRGB; Dist = flann::L2_Simple<float>]: Assertion `point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!"' failed.
  
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pointsInPCLForm, *pointsInPCLForm, indices);
  
  // Set parameters
  mls.setInputCloud (pointsInPCLForm);
  mls.setPolynomialFit (true);
  mls.setPolynomialOrder(1);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.05);

  // Reconstruct
  mls.process (mls_points);
  
  		
		pub.publish(mls_points);
}
*/

/*
// returns the normal of the most dominant plane. In this particular example, the most
// dominant plane means ROAD plane
pcl::PointXYZ getDominantPlaneNormal(const pcl::PointCloud<pcl::PointXYZRGB> pc){

	float distBWPoints = 0.1;			//the points should atleast be 'distBWPoints' distant to each other
	
	int64_t sizeOfPointCloud = pc.size();
	vector<pcl::PointXYZ> normals;		// to store the normals of the planar models 
	
	// to hold the indices of points for random sampling
	vector<int64_t> randomIndices1, randomIndices2, randomIndices3;
	for (int i=0;i<sizeOfPointCloud; ++i){
		randomIndices1.push_back(i);
		randomIndices2.push_back(i);
		randomIndices3.push_back(i);				
	} 
	
	// generate a random series of indices	
	random_shuffle(randomIndices1.begin(), randomIndices1.end());
	random_shuffle(randomIndices2.begin(), randomIndices2.end());
	random_shuffle(randomIndices3.begin(), randomIndices3.end());
		
	Eigen::Vector3d p1, p2, p3,v1,v2, normal;

	for(int i=0; i<sizeOfPointCloud; ++i){
		
		p1.x = pc[randomIndices1[i]].x; p1.y = pc[randomIndices1[i]].y; p1.z = pc[randomIndices1[i]].z;
		p2.x = pc[randomIndices1[i]].x; p2.y = pc[randomIndices1[i]].y; p2.z = pc[randomIndices1[i]].z;
		p3.x = pc[randomIndices1[i]].x; p3.y = pc[randomIndices1[i]].y; p3.z = pc[randomIndices1[i]].z;
		
		//check if the points aren't the same... later we will test for a minimum distance between the points to be selected
		if ( sqrt( pow((p1.x - p2.x),2) + pow((p1.y - p2.y),2) + pow((p1.z - p2.z),2) ) > 0.0
		          && sqrt( pow((p1.x - p3.x),2) + pow((p1.y - p3.y),2) + pow((p1.z - p3.z),2) ) >0.0 ){
		               
			v1.x = p1.x - p2.x;
			v1.y = p1.y - p2.y;
			v1.z = p1.z - p2.z;
			
			v2.x = p1.x - p3.x;
			v2.y = p1.y - p3.y;
			v2.z = p1.z - p3.z;
			
Eigen::Vector3f a;
			//normals.push_back(v1^v2);	// taking cross product
		}
	}
	
}*/



	/* publish the direction markers for possible drive directions 
	
		visualization_msgs::Marker marker;
		marker.header.frame_id = pointsInPCLForm.header.frame_id;
		marker.header.stamp = ros::Time::now();
		marker.ns = "basics";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;

		// construct the directions with line_strips 
		
	    geometry_msgs::Point p;	    
	
		p.x = 0;
		p.y = 0;
		p.z = 40;
		marker.points.push_back(p);
	    
	    for(int i = 0; i<dir.size(); ++i){	
	    	
			p.x = 2*(cos(dir[i])/0.017);
			p.y = 0;
			p.z = 3*(sin(dir[i])/0.017);
			marker.points.push_back(p);

			p.x = 0;
			p.y = 0;
			p.z = 40;
			marker.points.push_back(p);
	    
	    }
		  
		
		marker.scale.x = 2;
		marker.scale.y = 1;
		marker.scale.z = 1;
		
		marker.color.b = 205;
		marker.color.a = 1;
		marker.lifetime = ros::Duration();
		pubDriveDirection.publish(marker);
	*/
	





/*

// OLD DISPARITY TO POINT CLOUD FUNCTION

void pointCloudFromDisparity(	 const ImageConstPtr& l_image_msg,
                                 const CameraInfoConstPtr& l_info_msg,
                                 const CameraInfoConstPtr& r_info_msg,
                                 const DisparityImageConstPtr& disp_msg){
	
	cv_bridge::CvImageConstPtr cv_ptr, cv_ptr_d;
    cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8);
    cv_ptr_d = cv_bridge::toCvCopy(disp_msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
      
	image_geometry::StereoCameraModel model;
	model.fromCameraInfo(*l_info_msg, *r_info_msg);
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	
	point_cloud->header.frame_id = l_image_msg->header.frame_id;
    point_cloud->header.stamp = pcl_conversions::toPCL(l_info_msg->header).stamp;
    point_cloud->width = 1;//point_cloud->frame_id = l_image_msg->frame_id;
    
	for(size_t i =cv_ptr_d->image.rows/2; i <=cv_ptr_d->image.rows; ++i){
		for(size_t j =0; j <=cv_ptr_d->image.cols; ++j){
			
				
				cv::Point3d point;	//
				cv::Point2d px(j,i);
				float disp = cv_ptr_d->image.at<float>(i,j);
				model.projectDisparityTo3d(px, disp, point);
				pcl::PointXYZ p;
				// != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(point.z) 
				if(point.y<0.65 && point.y>0 && point.z > 0 && point.z <10 &&  point.z!= image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(point.z)){
					p.x = point.x;
					p.y = point.y;
					p.z = point.z;													
					point_cloud->points.push_back(p);
				}
				else{
					float bad_point = std::numeric_limits<float>::quiet_NaN ();
					p.x = p.y = p.z = bad_point;
					
				}
				
		}
	}
	
    point_cloud->height = point_cloud->size();
}

*/

















/*


//FROM SERVER


void pointCloudFromDisparity(	 const ImageConstPtr& l_image_msg,
                                 const CameraInfoConstPtr& l_info_msg,
                                 const CameraInfoConstPtr& r_info_msg,
                                 const DisparityImageConstPtr& disp_msg){
	
	cv_bridge::CvImageConstPtr cv_ptr, cv_ptr_d;
    cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8);
    cv_ptr_d = cv_bridge::toCvCopy(disp_msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
      
	image_geometry::StereoCameraModel model;
	model.fromCameraInfo(*l_info_msg, *r_info_msg);

	// to store only the ground points	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

 	// to store the color coded point cloud	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_seg(new pcl::PointCloud<pcl::PointXYZRGB>);	
	
	point_cloud_seg->header.frame_id = "pcl";//l_image_msg->header.frame_id;
    point_cloud_seg->header.stamp = pcl_conversions::toPCL(l_info_msg->header).stamp;
    point_cloud_seg->width = 1;

	point_cloud->header.frame_id = "pcl";//l_image_msg->header.frame_id;
    point_cloud->header.stamp = pcl_conversions::toPCL(l_info_msg->header).stamp;
    point_cloud->width = 1;//point_cloud->frame_id = l_image_msg->frame_id;
    
	for(size_t i =cv_ptr_d->image.rows/2; i <=cv_ptr_d->image.rows; ++i){
		for(size_t j =0; j <=cv_ptr_d->image.cols; ++j){
			
				
				cv::Point3d point;	//
				cv::Point2d px(j,i);
				float disp = cv_ptr_d->image.at<float>(i,j);
				model.projectDisparityTo3d(px, disp, point);
				pcl::PointXYZRGB p;
				// != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(point.z) 
				if(point.y >0 && point.z > 0 && point.z<15 &&  point.z!= image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(point.z)){

					p.x = point.x;
					p.y = point.y;
					p.z = point.z;				       
					
					point_cloud->push_back(p);
				}
				
				
		}
	}

    for(size_t i = 0; i<point_cloud->size(); ++i){
		if(point_cloud->points[i].y>1.45 &&  point_cloud->points[i].x>-1.2 && point_cloud->points[i].x<1.2 && point_cloud->points[i].z<5){
			point_cloud_seg->push_back(point_cloud->points[i]);
		
		}	
    }	
//cerr<<"herer"<<__LINE__<<endl;

    // rememeber when you mess up with the size of cloud buffer: u get exception like:
   //     terminate called after throwing an instance of 'ros::serialization::StreamOverrunException'... 	
//    point_cloud_seg->height = point_cloud_seg->size();
//    point_cloud->height = point_cloud->size();
//cerr<<"herer"<<__LINE__<<endl;

    getPlaneParameters(point_cloud_seg);

     // - clear the points in point_cloud_seg as we dont need it any more.. use this to load the above the ground ponts
   point_cloud_seg->clear();		

     // change the color of plane and obstacle points

// cerr<<"herer"<<__LINE__<<" "<<point_cloud->size()<<"seg "<<endl;//point_cloud_seg<<endl;

    for(size_t i=0; i<point_cloud->size(); ++i){
		
        pcl::PointXYZRGB p = point_cloud->points[i];
	float planeFitErr = PNa*p.x + PNb*p.y + PNc*p.z + PNd;
	//cerr<<"Plane FItt error"<<planeFitErr<<endl;
	if(planeFitErr >0.25){	//0.15, plan - 0.04, z<5, y>1.45
		point_cloud->points[i].r = 200;
		
		//also make a cloud of obstacle points
		if(point_cloud->points[i].z > 2 && point_cloud->points[i].y>0.4)
			point_cloud_seg->push_back(point_cloud->points[i]);
	}
	else
		point_cloud->points[i].g = 200;
	
    }		

    //pub.publish(point_cloud);
    //occupancy_above_ground(point_cloud_seg);

}

*/



//point_cloud_seg->clear();
    //estimateDir.getObstaclesPoints(point_cloud_seg);
    /*
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  	sor.setInputCloud (point_cloud_new);
  	sor.setMeanK (100);
  	sor.setStddevMulThresh (1.5);
  	sor.filter (*point_cloud_seg);
	*/
    
    
    //occupancy_above_ground(point_cloud_seg);
    //estimateDir.renderPossibleDriveDirectionsOnGrid(point_cloud_seg, dirs, locs);
    //estimateDir.printAngularOccupancy();
	//cerr<<"^";
    //for(int i=0; i<dirs.size(); ++i)
    	//cerr<<dirs[i]<<",";
    //publish   
    //cerr<<endl;
    //pub_counter++;
    
    //if(pub_counter%10 == 0)
    
    
    //************************************GENREATING MARKERS *************************************//
    
    
    /*	visualization_msgs::Marker marker;
    	visualization_msgs::MarkerArray marker_array_msg;
    	
    	marker_array_msg.markers.resize(dirs.size());

		// construct the arrow markers and push it in the marker array for all drivable directions 		
		
		for(int i=0; i<dirs.size(); ++i){
		
			marker_array_msg.markers[i].header.frame_id = point_cloud->header.frame_id;
			marker_array_msg.markers[i].header.stamp = l_info_msg->header.stamp;
			marker_array_msg.markers[i].ns = "kitti_stereo";
			marker_array_msg.markers[i].id = 0;
			marker_array_msg.markers[i].type = visualization_msgs::Marker::ARROW;
			marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
		
			geometry_msgs::Point p;	    
			
			p.x = 0;
			p.y = -2;
			p.z = 0;
			marker_array_msg.markers[i].points.push_back(p);
	    	    	
			p.x = 2 * cos( estimateDir.toRadian(dirs[i]) );
			p.y = -2;
			p.z = 2 * sin( estimateDir.toRadian(dirs[i]) ) ;
			marker_array_msg.markers[i].points.push_back(p);

			marker_array_msg.markers[i].color.b = 205;
			marker_array_msg.markers[i].color.a = 1;
			marker_array_msg.markers[i].lifetime = ros::Duration();
	    	marker_array_msg.markers[i].scale.x = 12;
			marker_array_msg.markers[i].scale.y = 11;
	    	marker_array_msg.markers[i].scale.z = 11;
		
	    }
		  
		
	//	marker.scale.x = 2;
	//	marker.scale.y = 1;
	//	marker.scale.z = 1;
		*/
		

//		marker.header.stamp.nsec = (point_cloud->header.stamp)*1000; //ros::Time::now();
/***************			
		visualization_msgs::Marker marker;
		marker.header.frame_id = point_cloud->header.frame_id;
		marker.header.stamp = l_info_msg->header.stamp;
	
		marker.id = 0;
		marker.type = visualization_msgs::Marker::LINE_STRIP;
		marker.action = visualization_msgs::Marker::ADD;

		// construct the directions with line_strips 
		
	    geometry_msgs::Point p;	    
	
		p.x = 0;
		p.y = 0;
		p.z = 0;
		marker.points.push_back(p);
	    
	    for(int i = 0; i<dirs.size(); ++i){	
	    	
			p.x = 2 * cos(estimateDir.toRadian(dirs[i])) ;
			p.y = 0;
			p.z = 2 * sin(estimateDir.toRadian(dirs[i]));
			marker.points.push_back(p);

			p.x = 0;
			p.y = 0;
			p.z = 0;
			marker.points.push_back(p);
	    
	    }
		  
		
		marker.scale.x = 0.1;
		marker.scale.y = 0.2;
		marker.scale.z = 4;
		
		marker.color.b = 205;
		marker.color.a = 1;
		marker.lifetime = ros::Duration();
		
		
		
    //*********************************************************************************************/	




