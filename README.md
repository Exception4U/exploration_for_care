# exploration_for_care
## EXPLORE DIRECTION ESTIMATOR for ROBOTIC EXPLORATION
### Description - 
This node estimates all possible safe driveable directions for a robot given a disparity image. The safety is governed by the availability of enough gap width which can be set dynamically 
'''
see ExploreDirectionEstimator::setupParameters() .
'''				  
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
				  	  
				  	  
