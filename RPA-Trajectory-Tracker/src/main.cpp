#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <iostream>
#include <signal.h>
#include <ctime>
#include <pcl/io/pcd_io.h>

#include "trajectory.h"
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/centroid.h>

#include "people_detector.h"
#include "constants.h"
#include "people_tracker.h"
#include "util.h"
#include "draw.h"
#include <stdlib.h>
#include <stdio.h>

PeopleDetector* detector;
PeopleTracker* tracker;

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer::Ptr viewerPtr;
};    

void pp_callback (const pcl::visualization::PointPickingEvent& event, void* args) {
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

// Define the function to be called when ctrl-c (SIGINT) signal is sent to process
void signal_callback_handler(int sig)
{
    std::string filepath = "data/input.txt";
    save_trajectories(tracker->getFinished(), filepath.c_str());        
    save_trajectories(tracker->getTrajectories(), filepath.c_str());
     
    printf("\nExited successfully \n");
    exit(sig);
}

int main (int argc, char** argv) {
    
    // the first pointcloud loaded is data/<pc_folder> + <pc_start> + .pcd
    // the last pointcloud loaded is data/<pc_folder> + <pc_finish> + .pcd
    std::string pc_folder = "twoperson";
    int pc_start = 415;;
    int pc_finish = 450;
    bool visualizeOn = false;

    if (argc > 1) {
      pc_folder = argv[1];
    } 
    if (argc > 2) {   
        pc_start = atoi(argv[2]);
    }
    if (argc > 3) {   
        pc_finish = atoi(argv[3]);
    }
  

    PointCloudT::Ptr cloud (new PointCloudT);
    std::stringstream ss;
    ss << pc_start;
    std::string name = "data/" + pc_folder+ "/pointcloud" + ss.str() + ".pcd";
    
    if (pcl::io::loadPCDFile<PointT> (name, *cloud) == -1) {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
  
    // Display pointcloud
    pcl::visualization::PCLVisualizer::Ptr temp_viewer (new pcl::visualization::PCLVisualizer("PCL Viewer"));     
    temp_viewer->setCameraPosition(0,0,-2,0,-1,0,0);
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);          
    temp_viewer->addPointCloud<PointT> (cloud, rgb, "input_cloud");
    temp_viewer->spinOnce();

    // Add point picking callback to viewer:
    struct callback_args cb_args;
    PointCloudT::Ptr clicked_points_3d (new PointCloudT);
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = pcl::visualization::PCLVisualizer::Ptr(temp_viewer);
    temp_viewer->registerPointPickingCallback (pp_callback, (void*)&cb_args);
    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

    // Spin until 'Q' is pressed:
    temp_viewer->spin();
    std::cout << "done." << std::endl;
        

    // Ground plane estimation:
    Eigen::VectorXf ground_coeffs;
    ground_coeffs.resize(4);
    std::vector<std::vector<int> > lists_of_3_points;
    std::vector<int> clicked_points_indices; 
    for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++) {
        if (i % 3 == 0) {
            std::vector<int> clicked_points_indices;    
        }
        clicked_points_indices.push_back(i);
        if (i % 3 == 2) {
            lists_of_3_points.push_back(clicked_points_indices);
        }
    }
    pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
    //model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
        
    model_plane.computeModelCoefficients(lists_of_3_points.at(0),ground_coeffs);
    std::cout << "Ground plane: " << ground_coeffs(0) << " " << ground_coeffs(1) << " " << ground_coeffs(2) << " " << ground_coeffs(3) << std::endl;
  
    
    //pcl::PointCloud<PointT>::Ptr ground (new pcl::PointCloud<PointT> ());
    //pcl::copyPointCloud<PointT>(*cloud, inliers, *ground); 
    //save_pointcloud("ground.pcd", ground);
    

    // Initialize viewer
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("PCL Viewer"));     
    viewer->setCameraPosition(0,0,-2,0,-1,0,0);
    

    // Initialize variables for displaying framerate
    double last = pcl::getTime();
    double now = pcl::getTime();
    double start_time = now;

    // Initialize people detector
    detector = new PeopleDetector();
    tracker = new PeopleTracker();
    tracker->setTime(now-start_time);
    
    // Main loop:
    for (int pc_num = pc_start; pc_num <= pc_finish; pc_num++) {
        
        signal(SIGINT, signal_callback_handler);

        std::stringstream ss;
        ss << pc_num;
        std::string name = "data/" + pc_folder + "/pointcloud" + ss.str() + ".pcd";
        
        if (pcl::io::loadPCDFile<PointT> (name, *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
            return (-1);
        }
        now = pcl::getTime();
        // Reset viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        detector->getClusterData()->clear();            

        // detect clusters
        detector->setInputCloud(cloud);
        detector->unorganizedDetect(viewer, ground_coeffs);
        // set parameters
        
        tracker->setTime(now-start_time);
        tracker->setClusterData(detector->getClusterData());
        // modify trajectories            
        tracker->track();

        std::cout << "Processing pointcloud: " << pc_num << std::endl;                                          
        // draw lines from previous position to current position
        if (visualizeOn) {
          drawTrajectories(viewer, tracker->getTrajectories());
          // Update display
          pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(detector->getCloudMain());                       
          viewer->addPointCloud<PointT> (detector->getCloudMain(), rgb, "cloud_filtered");
          viewer->spinOnce();        
        }
        /*std::string input = "n";
        cin >> input;
        if (input == "m") {
            viewer->spin();    

        }*/
        
    }

    std::string filepath_finished = "data/finished.txt";
    std::string filepath_trajectories = "data/trajectories.txt";
    std::string filepath_all = "data/all.txt";    
    save_trajectories(tracker->getFinished(), filepath_finished.c_str());        
    save_trajectories(tracker->getTrajectories(), filepath_trajectories.c_str());
    save_trajectories(tracker->getTrajectories(), filepath_all.c_str());
    save_trajectories(tracker->getFinished(), filepath_all.c_str());
    return 0;
}
