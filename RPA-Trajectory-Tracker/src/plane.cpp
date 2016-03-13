#include "plane.h"


void ransac(PointCloudT::Ptr in, PointCloudT::Ptr plane, double dist_threshold, int num_iter) {
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    ransac(in, inliers, coefficients, dist_threshold, num_iter);
    
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (in);
    extract.setIndices (inliers);
    // Extract the planar inliers
    extract.setNegative (false);
    extract.filter (*plane);
}

void ransacRemove(PointCloudT::Ptr in, PointCloudT::Ptr no_plane, double dist_threshold, int num_iter) {
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    ransac(in, inliers, coefficients, dist_threshold, num_iter);
    
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (in);
    extract.setIndices (inliers);
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*no_plane);
}
 
Eigen::Vector4f* ransacCoeff(PointCloudT::Ptr in, double dist_threshold, int num_iter) {
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    ransac(in, inliers, coefficients, dist_threshold, num_iter);
    std::vector<float> c = coefficients->values;    
    Eigen::Vector4f* coeff_vector = new Eigen::Vector4f(c[0], c[1], c[2], c[3]);
    return coeff_vector;
}

void ransac(PointCloudT::Ptr in, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, double dist_threshold, int num_iter) {
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (num_iter);
    seg.setDistanceThreshold (dist_threshold);

    seg.setInputCloud (in);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }    
}
    
       
void ransacMultiple (PointCloudT::Ptr cloud, std::vector<PointCloudT::Ptr> planes, double dist_threshold, int num_iter, int points_threshold) {
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    PointCloudT::Ptr cloud_filtered (new PointCloudT);    

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (num_iter);
    seg.setDistanceThreshold (dist_threshold);

    while (cloud->points.size () > points_threshold) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);        
        // Extract the planar inliers        
        extract.setNegative (false);
        PointCloudT::Ptr plane (new PointCloudT);            
        extract.filter (*plane);
        planes.push_back(plane);
        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_filtered);
        *cloud = *cloud_filtered;
    }
}

void ransacRemoveMultiple (PointCloudT::Ptr cloud, double dist_threshold, int num_iter, int points_threshold) {
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    PointCloudT::Ptr cloud_filtered (new PointCloudT);    

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (num_iter);
    seg.setDistanceThreshold (dist_threshold);

    int i = 0;
    while (i < points_threshold) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0) {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        else if (inliers->indices.size() < points_threshold) {
            break;
        }

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        // Remove the planar inliers, extract the rest        
        extract.setNegative (true);
        extract.filter (*cloud_filtered);
        *cloud = *cloud_filtered;
        i += 1;
    }
}

