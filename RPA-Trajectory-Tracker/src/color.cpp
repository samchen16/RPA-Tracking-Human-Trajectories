#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>    

#include "util.h"
#include "constants.h"
#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


int main (int argc, char** argv) {
    //std::cout << "argc = " << argc << std::endl;
    //for(int i = 0; i < argc; i++)
    //  std::cout << "argv[" << i << "] = " << argv[i] << std::endl;

    std::string cmd = argv[1];    

    if (cmd.compare("d") == 0)
    {
        std::string path = argv[2];
        PointCloudT::Ptr cloud (new PointCloudT);
        pcl::io::loadPCDFile (path, *cloud); 
        
        // display point cloud
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("PCL Viewer"));     
        viewer->setCameraPosition(0,0,-3,0,-0.5,0,0);
        pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);                       
        viewer->addPointCloud<PointT> (cloud, rgb, "cloud");
        viewer->spinOnce();
       
        // display histogram
        cv::Mat* mat = convert_pc2mat (cloud); 
        cv::Mat* hist = get_histogram(mat);
        display_histogram_2D(*hist);
    }
    else if (cmd.compare("c") == 0)
    {
        std::string path1 = argv[2];
        std::string path2 = argv[3];
        
        PointCloudT::Ptr cloud1 (new PointCloudT);
        pcl::io::loadPCDFile (path1, *cloud1); 
        PointCloudT::Ptr cloud2 (new PointCloudT);
        pcl::io::loadPCDFile (path2, *cloud2);
        cv::Mat* mat1 = convert_pc2mat (cloud1); 
        cv::Mat* hist1 = get_histogram(mat1);
        cv::Mat* mat2 = convert_pc2mat (cloud2); 
        cv::Mat* hist2 = get_histogram(mat2);
        
        double similarity = compare_histograms(*hist1, *hist2);
        std::cout << "Similarity: " << similarity << std::endl;
                 
    }

    return 0;
}
