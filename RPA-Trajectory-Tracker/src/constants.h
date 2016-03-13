//
//  Created by Samantha Chen on 3/20/15.
//
//

#ifndef __constants__
#define __constants__


#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Default values for people detector
#define DEFAULT_SVM_PATH "data/trainedLinearSVMForPeopleDetectionWithHOG.yaml"
#define DEFAULT_MIN_CONFIDENCE -3.5f //-0.6f 
#define DEFAULT_MIN_HEIGHT 0.4f //1.3;
#define DEFAULT_MAX_HEIGHT 2.7f //2.3;
#define DEFAULT_MIN_WIDTH 0.2f
#define DEFAULT_MAX_WIDTH 2.0f
#define DEFAULT_VOXEL_SIZE 0.1f //0.06f

// Kinect for Windows Specs
#define HORIZONTAL_FOV 57.0f
#define VERTICAL_FOV 43.0f
#define KINECT_MIN_Z 0.8f
#define KINECT_MAX_Z 4.0f

// Histogram
#define NUM_HUE_BINS 180//60
#define NUM_SAT_BINS 255 //80

// People Tracker
#define MIN_PERSON_CONFIDENCE -0.165f
#define MIN_CLUSTER_DIST 0.2f

// People Detector
#define MIN_CLUSTER_SIZE 70
#define MIN_SUBCLUSTER_SIZE 30
#define MAX_SUBCLUSTER_SIZE 100000
#define MIN_PERSON_HEIGHT 1.1
#define MAX_PERSON_HEIGHT 2.3

#endif