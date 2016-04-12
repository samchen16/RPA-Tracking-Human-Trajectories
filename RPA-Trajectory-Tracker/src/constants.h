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


// People Tracker
#define MAX_FRAMES_INACTIVE 5
#define MIN_PERSON_CONFIDENCE -200.8f
#define MIN_CLUSTER_DIST 0.2f
//#define MAX_FRAMES_INACTIVE 6

// People Detector
#define SVM_PATH "data/trainedLinearSVMForPeopleDetectionWithHOG.yaml"
// thresholds for plane removal
#define PLANE_REMOVAL_DIST_THRESH 0.2
// thresholds for statistical outlier removal
#define OUTLIER_REMOVAL_NUM_POINTS 60
#define OUTLIER_REMOVAL_STD_DEV 2.0
// thresholds for euclidean clustering 
#define EUCLIDEAN_CLUSTERING_DIST_THRESH 2
#define EUCLIDEAN_CLUSTERING_MIN_CLUSTER 1250
#define EUCLIDEAN_CLUSTERING_MAX_CLUSTER 8000
// thresholds for person subclustering
#define MIN_SUBCLUSTER 2
#define MAX_SUBCLUSTER 500000 
#define MIN_PERSON_HEIGHT 0.1f 
#define MAX_PERSON_HEIGHT 3.5f 
#define MIN_HEAD_DIST 0.1f

// Kinect for Windows Specs
#define HORIZONTAL_FOV 57.0f
#define VERTICAL_FOV 43.0f
#define KINECT_MIN_Z 0.8f
#define KINECT_MAX_Z 4.0f

// Histogram
#define NUM_HUE_BINS 180
#define NUM_SAT_BINS 255


#endif
