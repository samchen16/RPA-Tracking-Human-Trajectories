#ifndef __Downsample__
#define __Downsample__

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/keypoints/uniform_sampling.h>


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Downsample {
private:


public:
	
    static void organized_downsample (PointCloudT::Ptr input_cloud, PointCloudT::Ptr output_cloud, int incr_x, int incr_y);
   
    static PointCloudT::Ptr voxelGrid (PointCloudT::Ptr in, float leaf_size);
    static void voxelGrid (PointCloudT::Ptr in, PointCloudT::Ptr out, float leaf_size);

    static PointCloudT::Ptr uniformSample (PointCloudT::Ptr in, double radius);
    static void uniformSample (PointCloudT::Ptr in, PointCloudT::Ptr out, double radius);
    
    static pcl::PointCloud<int>::Ptr uniformSampleIndices (PointCloudT::Ptr in, double radius);    
    static void uniformSampleIndices (PointCloudT::Ptr in, pcl::PointCloud<int>::Ptr indices, double radius);
    
    static PointCloudT::Ptr randomSample (PointCloudT::Ptr in, int num);
    static void randomSample (PointCloudT::Ptr in, PointCloudT::Ptr out, int num);
    
};

#endif /* defined(__Downsample__) */
