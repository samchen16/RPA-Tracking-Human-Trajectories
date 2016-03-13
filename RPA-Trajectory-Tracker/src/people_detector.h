//
//  people_detector.h
//  PeopleDetector
//

#ifndef __people_detector__
#define __people_detector__

#include <vector>
#include <pcl/point_types.h>

#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/people/head_based_subcluster.h>
#include <pcl/people/person_classifier.h>
#include "cluster_data.h"


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PeopleDetector {

private:
        
    PointCloudT::Ptr cloud;
    PointCloudT::Ptr cloud_main;    
    PointCloudT::Ptr cloud_filtered;
    PointCloudT::Ptr cloud_with_no_planes;
    PointCloudT::Ptr cloud_with_no_outliers;
    
    pcl::PointCloud<pcl::RGB>::Ptr rgb_image;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    pcl::PointCloud<pcl::Label>::Ptr labels;    
    pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator;
    pcl::people::PersonClassifier<pcl::RGB>* person_classifier;
    

    std::vector<pcl::PointIndices>* all_indices;        
    std::vector<PointCloudT>* all;
    std::vector<ClusterData*>* clusters;

    void extractRGBFromPointCloud (PointCloudT::Ptr input_cloud, pcl::PointCloud<pcl::RGB>::Ptr& output_cloud);
    void clear();

    int name_num;

    // thresholds for plane removal
    float _plane_dist_thresh;
    // thresholds for statistical outlier removal
    int _num_points;
    float _std_dev ;
    // thresholds for euclidean clustering    
    float _euclidean_dist_thresh;  
    int _min_cluster;
    int _max_cluster;
    // thresholds for person subclustering
    int _min_subcluster;
    int _max_subcluster;
    float _min_person_height;
    float _max_person_height;
    float _min_head_dist;


public:
    PeopleDetector();
    ~PeopleDetector();

    // cloud must be set before detect is called
    void setInputCloud(PointCloudT::Ptr input_cloud) { cloud = input_cloud; }    
    // main method for detection
    void unorganizedDetect(pcl::visualization::PCLVisualizer::Ptr, Eigen::VectorXf ground_coeffs);
    void organizedDetect(pcl::visualization::PCLVisualizer::Ptr, Eigen::VectorXf ground_coeffs);
        

    // getters
    std::vector<pcl::PointIndices>* getAllIndices() { return all_indices; }
    std::vector<PointCloudT>* getAll() { return all; }
    PointCloudT::Ptr getCloudMain() { return cloud_main; }    
    PointCloudT::Ptr getCloudFiltered() { return cloud_filtered; }
    PointCloudT::Ptr getCloudWithNoPlanes() { return cloud_with_no_planes; }
    PointCloudT::Ptr getCloudWithNoOutliers() { return cloud_with_no_outliers; }        
    std::vector<ClusterData*>* getClusterData() { return clusters; }
    
    // setters
    void set_euclidean_dist_thresh(float v) { _euclidean_dist_thresh = v; }        
    void set_min_cluster(int v) { _min_cluster = v; }  
    void set_min_subcluster(int v) { _min_subcluster = v; }
    void set_max_subcluster(int v) { _max_subcluster = v; }
    void set_min_person_height(float v) { _min_person_height = v; }  
    void set_max_person_height(float v) { _max_person_height = v; }  
    void set_min_head_dist(float v) { _min_head_dist = v; }  
        

};


#endif /* defined(__people_detector__) */
