#ifndef __draw__
#define __draw__

#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/segmentation/planar_region.h>
#include <pcl/filters/boost.h>
#include "trajectory.h"
#include "point.h"
#include <vector>
#include <pcl/point_types.h>

void displayPlanarRegions (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions, 
                      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    
void displayClusters (std::vector<PointCloudT> clusters, 
                      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer);
    
void displayClusters (std::vector<PointCloudT> clusters, 
                      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, int r, int g, int b);

void displayClusters (std::vector<pcl::PointIndices>* cluster_indices, 
                          boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, PointCloudT::Ptr main_cloud);
 
void drawLine (pcl::visualization::PCLVisualizer::Ptr viewer, float x1, float y1, float z1, float x2, float y2, float z2, std::string color, std::string name);
void drawLine (pcl::visualization::PCLVisualizer::Ptr viewer, Point p1, Point p2, std::string color, std::string name);

void drawTrajectory (pcl::visualization::PCLVisualizer::Ptr viewer, Trajectory* traj, std::string color, std::string name);

void drawTrajectories(pcl::visualization::PCLVisualizer::Ptr viewer, std::vector<Trajectory*>* trajs);

#endif /* defined(__draw__) */
