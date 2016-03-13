#include "cluster_data.h"
#include "constants.h"

ClusterData::ClusterData(Point pos, cv::Mat* col, double conf, PointCloudT::Ptr cld) {
    position = pos;
    color = col;
    person_confidence = conf;
    cloud = cld;
}


ClusterData::ClusterData(pcl::people::PersonCluster<PointT>* pc, PointCloudT::Ptr main_cloud) {    
    position = Point();                    
    position.x = pc->getTCenter()(0);
    position.y = pc->getTCenter()(1);
    position.z = pc->getTCenter()(2);   
    person_confidence = pc->getPersonConfidence();

    cloud = PointCloudT::Ptr (new PointCloudT);
    pcl::copyPointCloud (*main_cloud, pc->getIndices().indices, *cloud);
    color = get_histogram(convert_pc2mat(cloud));        
}

ClusterData::~ClusterData() {
   //delete cloud;
}


