#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"

#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


// OCCAM CONSTANTS
#define BASELINE 0.12f //in meters 
#define FOCAL 2.8f //in mm
#define FOCAL_PIXELS 10.583 //in pixels


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void save_pointcloud(PointCloudT::Ptr point_cloud) {
    std::string name = "data/pointcloud.pcd";
    pcl::PCDWriter writer;
    writer.write<PointT> (name, *point_cloud); 
}


int main( int argc, char** argv )
{
    std::string test_pic = "data/img1446588820968.jpg";
    cv::Mat img;

    // Read the file
    img = cv::imread(test_pic, CV_LOAD_IMAGE_COLOR);   
    
    // Check for invalid input
    if(!img.data) {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    // Create a window for display.
    //cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );

    // Show our image inside it.
    //cv::imshow( "Display window", img );                   

    // Wait for a keystroke in the window
    //cv::waitKey(0);                                          

    // Convert to pointcloud
    cv::Mat hsv_img;    
    cv::cvtColor(img, hsv_img, CV_RGB2HSV);

    PointCloudT::Ptr pc (new PointCloudT);
    int width = img.cols;
    int height = img.rows/2; 
    
   
    PointT point;         
    for (int h = 0; h < height; h++) {
        for (int w = 0; w < width; w++) {
            PointT* point = new PointT();                     
            
            float disparity = hsv_img.at<cv::Vec3b>(h+height, w)[0];
            float depth = BASELINE * FOCAL_PIXELS / disparity;            
            // get RGBA            
            point->r = img.at<cv::Vec3b>(h+height,w)[2];
	          point->g = img.at<cv::Vec3b>(h+height,w)[1];
	          point->b = img.at<cv::Vec3b>(h+height,w)[0];        
            point->a = 255;       
         
            // Get XYZ
            point->x = w * depth / FOCAL_PIXELS;
	          point->y = h * depth / FOCAL_PIXELS;
	          point->z = depth;        
            pc->push_back(*point);
            //std::cout << "disparity: " << disparity << "   depth: " << depth << std::endl;
            
        }
    }

    pc->width = width;
    pc->height = height; 
    
    std::cout << "width: " << pc->width << "    height: " << pc->height << std::endl;
    save_pointcloud(pc);
    // Display created pointcloud
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("PCL Viewer"));     
    
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(pc);                       
    viewer->addPointCloud<PointT> (pc, rgb, "cloud");
    while (!viewer->wasStopped()) {
        viewer->spinOnce();    
    }

    return 0;
}
