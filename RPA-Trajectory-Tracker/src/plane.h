#ifndef __plane__
#define __plane__

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include "constants.h"

   
void ransac (PointCloudT::Ptr in, PointCloudT::Ptr plane, double dist_threshold, int num_iter);
void ransacRemove (PointCloudT::Ptr in, PointCloudT::Ptr no_plane, double dist_threshold, int num_iter);
Eigen::Vector4f* ransacCoeff (PointCloudT::Ptr in, double dist_threshold, int num_iter);
void ransac (PointCloudT::Ptr in, pcl::PointIndices::Ptr seg_inliers, pcl::ModelCoefficients::Ptr seg_coefficients, double dist_threshold, int num_iter);
void ransacMultiple (PointCloudT::Ptr cloud, std::vector<PointCloudT::Ptr> planes, double dist_threshold, int num_iter, int points_threshold);    
void ransacRemoveMultiple (PointCloudT::Ptr cloud, double dist_threshold, int num_iter, int points_threshold);    

/*

static PointCloudT::Ptr organizedMultiplane (PointCloudT::Ptr in, double radius);
static void organizedMultiplaneCoeff (PointCloudT::Ptr in, PointCloudT::Ptr out, double radius);
*/
    

#endif /* defined(__plane__) */
