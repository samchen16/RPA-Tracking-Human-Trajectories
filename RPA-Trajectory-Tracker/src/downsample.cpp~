#include "downsample.h"



void Downsample::organized_downsample (PointCloudT::Ptr input_cloud, PointCloudT::Ptr output_cloud, int incr_x, int incr_y)
{
    output_cloud->points.resize((input_cloud->height/incr_y) * (input_cloud->width/incr_x));
    output_cloud->width = input_cloud->width / incr_x;
    output_cloud->height = input_cloud->height / incr_y;
    output_cloud->is_dense = input_cloud->is_dense;
    output_cloud->sensor_orientation_ = input_cloud->sensor_orientation_;
    output_cloud->sensor_origin_ = input_cloud->sensor_origin_;

    for (int j = 0; j < input_cloud->width; j+=incr_x) {
        for (int i = 0; i < input_cloud->height; i+=incr_y) {  
            pcl::copyPoint ((*input_cloud)(j,i), (*output_cloud)(j/incr_x,i/incr_y));
        }
    } 
}

PointCloudT::Ptr Downsample::voxelGrid(PointCloudT::Ptr in, float leaf_size) {
    PointCloudT::Ptr out (new PointCloudT);
    voxelGrid(in, out, leaf_size);
    return out;
}

void Downsample::voxelGrid(PointCloudT::Ptr in, PointCloudT::Ptr out, float leaf_size) {
	pcl::VoxelGrid<PointT> vg;               
    vg.setInputCloud (in);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*out);
}

PointCloudT::Ptr Downsample::uniformSample(PointCloudT::Ptr in, double radius) {
    PointCloudT::Ptr out (new PointCloudT);    	
    uniformSample(in, out, radius);
    return out;
}

void Downsample::uniformSample(PointCloudT::Ptr in, PointCloudT::Ptr out, double radius) {
	pcl::PointCloud<int>::Ptr indices = uniformSampleIndices(in, radius);    
    pcl::copyPointCloud (*in, indices->points, *out);
}
 
pcl::PointCloud<int>::Ptr Downsample::uniformSampleIndices(PointCloudT::Ptr in, double radius) {
    pcl::PointCloud<int>::Ptr indices (new pcl::PointCloud<int>);
    uniformSampleIndices(in, indices, radius);
    return indices;
}

void Downsample::uniformSampleIndices(PointCloudT::Ptr in, pcl::PointCloud<int>::Ptr indices, double radius) {	
    pcl::UniformSampling<PointT> uniform_sampling;
    uniform_sampling.setInputCloud (in);
    uniform_sampling.setRadiusSearch (radius);
    uniform_sampling.compute (*indices);
}

PointCloudT::Ptr Downsample::randomSample(PointCloudT::Ptr in, int num) {
    PointCloudT::Ptr out (new PointCloudT);
    randomSample(in, out, num);
    return out;    	
}

void Downsample::randomSample(PointCloudT::Ptr in, PointCloudT::Ptr out, int num) {
    pcl::RandomSample<PointT> random_sample;
    random_sample.setInputCloud(in);
    random_sample.setSample(num);
    random_sample.setSeed(rand());
    random_sample.filter(*out); 
}

