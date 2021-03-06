#ifndef __util__
#define __util__

#include <pcl/io/pcd_io.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <pcl/point_types_conversion.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include "constants.h"


class Trajectory;
class ClusterData;

float cluster_distance(ClusterData* cd1, ClusterData* cd2);
void save_pointcloud(std::string name, PointCloudT::Ptr point_cloud);
void save_trajectories(std::vector<Trajectory*>* trajs, const char* filename);
cv::Mat* convert_pc2RGBmat (PointCloudT::Ptr cloud);
cv::Mat* convert_pc2mat (PointCloudT::Ptr cloud, bool hsv=true);
cv::Mat* get_histogram(cv::Mat* mat);
double compare_histograms (cv::Mat hist1, cv::Mat hist2);
cv::Mat get_full_HSV(int height, int width, int hue_num_bins, int sat_num_bins);
void display_histogram_2D(cv::Mat hist);


#endif /* defined(__util__) */
