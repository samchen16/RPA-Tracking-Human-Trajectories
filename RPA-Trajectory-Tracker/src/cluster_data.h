#ifndef __cluster_data__
#define __cluster_data__

#include "point.h"
#include "util.h"
#include "opencv2/core/core.hpp"
#include <pcl/people/person_cluster.h>

class ClusterData {
private:

    PointCloudT::Ptr cloud;
    cv::Mat* color;          
    Point position;
    double person_confidence;
    float vel_score;
    float col_score; 
    float pos_score;

public:
    ClusterData(Point pos, cv::Mat* col, double conf, PointCloudT::Ptr cld);    
    ClusterData(pcl::people::PersonCluster<PointT>* pc, PointCloudT::Ptr main_cloud);
    ~ClusterData();

    PointCloudT::Ptr getCloud() { return cloud; }
    cv::Mat* getColor() { return color; }          
    Point getPosition() { return position; }
    double getConfidence() { return person_confidence; }

    float getVelocityScore() { return vel_score; }
    void setVelocityScore(float v) { vel_score = v; } 
    float getColorScore() { return col_score; }
    void setColorScore(float v) { col_score = v; } 
    float getPositionScore() { return pos_score; }
    void setPositionScore(float v) { pos_score = v; } 

};

#endif /* defined(__cluster_data__) */
