//
//  people_detector.cpp
//  PeopleDetector
//

#include "people_detector.h"
#include "constants.h"
#include "downsample.h"
#include "util.h"
#include "draw.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/filter.h>
#include "plane.h"

PeopleDetector::PeopleDetector() {
    
    
    // Initialize vectors
    all_indices = new std::vector<pcl::PointIndices>();    
    all = new std::vector<PointCloudT>();
    
    // For rgb image
    rgb_image = pcl::PointCloud<pcl::RGB>::Ptr (new pcl::PointCloud<pcl::RGB>);
    // For filtering
    cloud_main = PointCloudT::Ptr (new PointCloudT);
    cloud_filtered = PointCloudT::Ptr (new PointCloudT); 
    cloud_with_no_planes = PointCloudT::Ptr (new PointCloudT); 
    cloud_with_no_outliers = PointCloudT::Ptr (new PointCloudT);  
    // For estimating normals
    cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);    
    // For plane segmentation
    labels = pcl::PointCloud<pcl::Label>::Ptr (new pcl::PointCloud<pcl::Label>);    
    // For cluster segmentation
    euclidean_cluster_comparator = pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label>::Ptr (new pcl::EuclideanClusterComparator<PointT, pcl::Normal, pcl::Label> ());


    // Create person classifier
    person_classifier = new pcl::people::PersonClassifier<pcl::RGB>();
    person_classifier->loadSVMFromFile(SVM_PATH);
 
    name_num = 0;

    clusters = new std::vector<ClusterData*>(); 

    // thresholds for plane removal
    _plane_dist_thresh = PLANE_REMOVAL_DIST_THRESH;
    // thresholds for statistical outlier removal
    _num_points = OUTLIER_REMOVAL_NUM_POINTS;
    _std_dev = OUTLIER_REMOVAL_STD_DEV;
    // thresholds for euclidean clustering 
    _euclidean_dist_thresh = EUCLIDEAN_CLUSTERING_DIST_THRESH;
    _min_cluster = EUCLIDEAN_CLUSTERING_MIN_CLUSTER;  
    _max_cluster = EUCLIDEAN_CLUSTERING_MAX_CLUSTER;
    // thresholds for person subclustering
    _min_subcluster = MIN_SUBCLUSTER; 
    _max_subcluster = MAX_SUBCLUSTER; 
    _min_person_height = MIN_PERSON_HEIGHT; 
    _max_person_height = MAX_PERSON_HEIGHT; 
    _min_head_dist = MIN_HEAD_DIST;


}

PeopleDetector::~PeopleDetector() {
    delete[] all_indices;
}

void PeopleDetector::clear() {
    // Clear vectors
    all->clear();
    all_indices->clear();
    
    // Clear pointclouds
    rgb_image->clear();
    cloud_main->clear();
    cloud_filtered->clear();
    cloud_with_no_planes->clear();
    cloud_with_no_outliers->clear();
    cloud_normals->clear();
    labels->clear();
}


void PeopleDetector::unorganizedDetect(pcl::visualization::PCLVisualizer::Ptr viewer, Eigen::VectorXf ground_coeffs) {
    // Clear data
    clear();    

    // Get rbg image            
    extractRGBFromPointCloud(cloud, rgb_image);

    // Downsample (cloud_filtered)
    //Downsample::organized_downsample(cloud, cloud_main, 2, 2);
    //Downsample::organized_downsample(cloud, cloud_filtered, 2, 2);
    float LEAF_SIZE = 0.05;
    Downsample::voxelGrid(cloud, cloud_main, LEAF_SIZE);  
    Downsample::voxelGrid(cloud, cloud_filtered, LEAF_SIZE);  
    //*cloud_main = *cloud_filtered; 

    // Deal with ground 
    Eigen::Vector4f gc;   
    gc[0] = ground_coeffs[0];
    gc[1] = ground_coeffs[1];
    gc[2] = ground_coeffs[2];
    gc[3] = ground_coeffs[3];
    pcl::SampleConsensusModelPlane<PointT>::Ptr dit (new pcl::SampleConsensusModelPlane<PointT> (cloud_main));
    std::vector<int> ground_inliers;
    dit->selectWithinDistance (gc, _plane_dist_thresh, ground_inliers);
    pcl::PointIndices::Ptr ground_ptr (new pcl::PointIndices);
    ground_ptr->indices = ground_inliers;   

    // Remove planes (cloud_with_no_planes)
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_main);  
    extract.setIndices (ground_ptr);  
    extract.setNegative (true);
    extract.filter (*cloud_main);
    extract.filter (*cloud_with_no_planes);
    
    //ransacRemoveMultiple (cloud_main, _plane_dist_thresh, 100, 500); 
    
    // Statistical Outlier Removal (cloud_with_no_outliers)
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloud_main);
    sor.setMeanK (_num_points);
    sor.setStddevMulThresh (_std_dev);
    sor.filter (*cloud_main);    
    sor.filter (*cloud_with_no_outliers);
  
    // Euclidean Clustering
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud_main, *cloud_main, index); 	

    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_main);
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(_euclidean_dist_thresh);
    ec.setMinClusterSize(_min_cluster);
    ec.setMaxClusterSize(_max_cluster);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_main);
    ec.extract(*all_indices);
    displayClusters (all_indices, viewer, cloud_main);
    
    // Head based sub-clustering 
    std::vector<pcl::people::PersonCluster<PointT> > headbased_clusters;
    pcl::people::HeadBasedSubclustering<PointT> subclustering;
    subclustering.setInputCloud(cloud_main);
    subclustering.setGround(ground_coeffs);
    subclustering.setInitialClusters(*all_indices);
    subclustering.setHeightLimits(_min_person_height, _max_person_height); 
    subclustering.setMinimumDistanceBetweenHeads(_min_head_dist);
    subclustering.setSensorPortraitOrientation(false);
    subclustering.setDimensionLimits(_min_subcluster, _max_subcluster);
    subclustering.subcluster(headbased_clusters);
    
    std::cout << "Clusters: " << all_indices->size() << "    Subclusters: " << headbased_clusters.size() << std::endl;
    Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics
    
    //Evaluate confidence for the current PersonCluster
    int k = 0;              
    for (std::vector<pcl::people::PersonCluster<PointT> >::iterator it = headbased_clusters.begin(); it != headbased_clusters.end(); ++it) {
        Eigen::Vector3f centroid = rgb_intrinsics_matrix * (it->getTCenter());
        centroid /= centroid(2);
        Eigen::Vector3f top = rgb_intrinsics_matrix * (it->getTTop());
        top /= top(2);
        Eigen::Vector3f bottom = rgb_intrinsics_matrix * (it->getTBottom());
        bottom /= bottom(2);
        it->setPersonConfidence(person_classifier->evaluate(rgb_image, bottom, top, centroid, false));
        
        ClusterData* cd = new ClusterData(&(*it), cloud_main);
        clusters->push_back(cd);    

        if (it->getPersonConfidence() > MIN_PERSON_CONFIDENCE) {
            std::cout << "Found a person! Drawing bounding box..." << std::endl;
            it->drawTBoundingBox(*viewer, k);
            k++;      
        }
     }
}

void PeopleDetector::organizedDetect(pcl::visualization::PCLVisualizer::Ptr viewer, Eigen::VectorXf ground_coeffs) {
    // Clear data
    clear();    

    // Get rbg image            
    extractRGBFromPointCloud(cloud, rgb_image);

    // Downsample
    Downsample::organized_downsample(cloud, cloud_filtered, 2, 2);
 
    // Calculate normals    
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.05f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud_filtered);
    ne.compute(*cloud_normals);
    
    // Deal with ground
    Eigen::Vector4f gc;   
    gc[0] = ground_coeffs[0];
    gc[1] = ground_coeffs[1];
    gc[2] = ground_coeffs[2];
    gc[3] = ground_coeffs[3];
    pcl::SampleConsensusModelPlane<PointT>::Ptr dit (new pcl::SampleConsensusModelPlane<PointT> (cloud_filtered));
    std::vector<int> ground_inliers;
    dit->selectWithinDistance (gc, 0.1, ground_inliers);
    pcl::PointIndices ground;
    ground.indices = ground_inliers;
    //

    // Segment planes  
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;  
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;
    
    pcl::OrganizedMultiPlaneSegmentation< PointT, pcl::Normal, pcl::Label > mps;
    mps.setMinInliers (500);
    mps.setAngularThreshold (0.017453 * 4.0); // 2 degrees
    mps.setDistanceThreshold (0.5); // 2cm
    mps.setInputCloud (cloud_filtered);
    mps.setInputNormals (cloud_normals);
    mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
    
    std::vector<pcl::PointIndices> plane_indices;
    
    for (int i = 0; i < model_coefficients.size(); i++) {
        pcl::ModelCoefficients model_coeffs = model_coefficients.at(i);        
        Eigen::Vector4f gc;   
        gc[0] = model_coeffs.values.at(0);
        gc[1] = model_coeffs.values.at(1);
        gc[2] = model_coeffs.values.at(2);
        gc[3] = model_coeffs.values.at(3);
        pcl::SampleConsensusModelPlane<PointT>::Ptr dit (new pcl::SampleConsensusModelPlane<PointT> (cloud_filtered));
        std::vector<int> model_inliers;
        dit->selectWithinDistance (gc, 0.1, model_inliers);
        pcl::PointIndices model;
        model.indices = model_inliers;
        plane_indices.push_back(model);

    }

    /*for (int i = 0; i < label_indices.size(); i++) {
        if (label_indices[i].indices.size() > 100) {
            pcl::PointIndices plane;
            for (int j = 0; j < label_indices[i].indices.size(); j++) {
                plane.indices.push_back(label_indices[i].indices.at(j));
            }
            plane_indices.push_back(plane);
        }
    }*/
    plane_indices.push_back(ground);

    std::vector<bool> plane_labels;
    plane_labels.resize (plane_indices.size()+1, false);
    for (int i = 0; i < labels->size(); i++) {
        labels->at(i).label = plane_indices.size();
    }

    for (int k = 0; k < plane_indices.size(); k++) {
        pcl::PointIndices plane = plane_indices.at(k);
        for (int l = 0; l < plane.indices.size(); l++) {
            int index = plane.indices.at(l);
            labels->at(index).label = k;
        }
        plane_labels.at(k) = true;
    }

    // for display
    pcl::PointIndices::Ptr li_ptr (new pcl::PointIndices);     
    for (int i = 0; i < labels->size(); i++) {
        if (labels->at(i).label < plane_indices.size()) {
            li_ptr->indices.push_back(i);
        }
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_filtered);    
    extract.setIndices (li_ptr);
    extract.setNegative (true);
    extract.filter (*cloud_with_no_planes);
    //

    //displayPlanarRegions (regions, viewer);
    
    // Segment more
    euclidean_cluster_comparator->setInputCloud (cloud_filtered);
    euclidean_cluster_comparator->setLabels (labels);
    euclidean_cluster_comparator->setExcludeLabels (plane_labels);
    euclidean_cluster_comparator->setDistanceThreshold (_euclidean_dist_thresh, true);

    pcl::PointCloud<pcl::Label> euclidean_labels;
    std::vector<pcl::PointIndices> euclidean_label_indices;
    pcl::OrganizedConnectedComponentSegmentation<PointT,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator);
    euclidean_segmentation.setInputCloud (cloud_filtered);
    euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

    for (size_t i = 0; i < euclidean_label_indices.size (); i++) {
        std::cout << "Evaluating cluster of size " << euclidean_label_indices[i].indices.size () << std::endl;
        if (euclidean_label_indices[i].indices.size () > _min_cluster) {
            all_indices->push_back(euclidean_label_indices[i]);
        }    
    }
    displayClusters (all_indices, viewer, cloud_filtered);
    
    // Head based sub-clustering 
    std::vector<pcl::people::PersonCluster<PointT> > headbased_clusters;
    pcl::people::HeadBasedSubclustering<PointT> subclustering;
    subclustering.setInputCloud(cloud_filtered);
    subclustering.setGround(ground_coeffs);
    subclustering.setInitialClusters(*all_indices);
    subclustering.setHeightLimits(_min_person_height, _max_person_height); 
    subclustering.setMinimumDistanceBetweenHeads(_min_head_dist);
    subclustering.setSensorPortraitOrientation(false);
    subclustering.setDimensionLimits(_min_subcluster, _max_subcluster);
    subclustering.subcluster(headbased_clusters);
    
    std::cout << "Region : " << regions.size() << "    Clusters: " << all_indices->size() << "    Subclusters: " << headbased_clusters.size() << std::endl;
    Eigen::Matrix3f rgb_intrinsics_matrix;
    rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics
   
    //Evaluate confidence for the current PersonCluster
    int k = 0;              
    for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = headbased_clusters.begin(); it != headbased_clusters.end(); ++it) {
        Eigen::Vector3f centroid = rgb_intrinsics_matrix * (it->getTCenter());
        centroid /= centroid(2);
        Eigen::Vector3f top = rgb_intrinsics_matrix * (it->getTTop());
        top /= top(2);
        Eigen::Vector3f bottom = rgb_intrinsics_matrix * (it->getTBottom());
        bottom /= bottom(2);
        it->setPersonConfidence(person_classifier->evaluate(rgb_image, bottom, top, centroid, false));
        
        ClusterData* cd = new ClusterData(&(*it), cloud_filtered);
        clusters->push_back(cd);    
            
        if (it->getPersonConfidence() > MIN_PERSON_CONFIDENCE) {                
            std::cout << "Found a person! Drawing bounding box..." << std::endl;
            it->drawTBoundingBox(*viewer, k);
            k++;      
        }
            /*PointCloudT::Ptr out (new PointCloudT);
            pcl::copyPointCloud (*cloud_filtered, it->getIndices().indices, *out);
            std::ostringstream ss;
            ss << "data2/" << name_num << ".pcd";
            std::string name = ss.str();
            save_pointcloud(name, out);
            name_num = name_num + 1;*/
     }
    
}



void PeopleDetector::extractRGBFromPointCloud (PointCloudT::Ptr input_cloud, pcl::PointCloud<pcl::RGB>::Ptr& output_cloud)
{
  // Extract RGB information from a point cloud and output the corresponding RGB point cloud  
  output_cloud->points.resize(input_cloud->height*input_cloud->width);
  output_cloud->width = input_cloud->width;
  output_cloud->height = input_cloud->height;

  pcl::RGB rgb_point;
  for (uint32_t j = 0; j < input_cloud->width; j++)
  {
    for (uint32_t i = 0; i < input_cloud->height; i++)
    { 
      rgb_point.r = (*input_cloud)(j,i).r;
      rgb_point.g = (*input_cloud)(j,i).g;
      rgb_point.b = (*input_cloud)(j,i).b;    
      (*output_cloud)(j,i) = rgb_point; 
    }
  }
}

