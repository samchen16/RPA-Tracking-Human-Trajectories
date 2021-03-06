#include "draw.h"
#include "constants.h"

void displayPlanarRegions (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > &regions, 
                      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
    char name[1024];
    unsigned char red [6] = {255,   0,   0, 255, 255,   0};
    unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
    unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

    pcl::PointCloud<PointT>::Ptr contour (new pcl::PointCloud<PointT>);

    for (size_t i = 0; i < regions.size (); i++) {
        Eigen::Vector3f centroid = regions[i].getCentroid ();
        Eigen::Vector4f model = regions[i].getCoefficients ();
        pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
        pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                                           centroid[1] + (0.5f * model[1]),
                                           centroid[2] + (0.5f * model[2]));
        sprintf (name, "normal_%d", unsigned (i));
        viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);

        contour->points = regions[i].getContour ();
        sprintf (name, "plane_%02d", int (i));
        pcl::visualization::PointCloudColorHandlerCustom <PointT> color (contour, red[i%6], grn[i%6], blu[i%6]);
        if(!viewer->updatePointCloud(contour, color, name))
            viewer->addPointCloud (contour, color, name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
    }
}


void displayClusters (std::vector<PointCloudT> clusters, 
                          boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, int r, int g, int b) {
    char name[1024];
    for (size_t i = 0; i < clusters.size (); i++) {
        sprintf (name, "cluster_%d" , int (i));
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color0(boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),r,g,b);
        if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),color0,name))
            viewer->addPointCloud (boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),color0,name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
    }
}


void displayClusters (std::vector<PointCloudT> clusters, 
                          boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
    char name[1024];
    unsigned char red [6] = {255,   0,   0, 255, 255,   0};
    unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
    unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

    for (size_t i = 0; i < clusters.size (); i++) {
        sprintf (name, "cluster_%d" , int (i));
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color0(boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),red[i%6],grn[i%6],blu[i%6]);
        if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),color0,name))
            viewer->addPointCloud (boost::make_shared<pcl::PointCloud<PointT> >(clusters[i]),color0,name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
    }
}


void displayClusters (std::vector<pcl::PointIndices>* cluster_indices, 
                          boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, PointCloudT::Ptr main_cloud) {
    char name[1024];
    unsigned char red [6] = {255,   0,   0, 255, 255,   0};
    unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
    unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

    for (size_t i = 0; i < cluster_indices->size (); i++) {
        pcl::PointIndices point_indices = cluster_indices->at(i);
        PointCloudT cluster;
        pcl::copyPointCloud (*main_cloud, point_indices.indices, cluster);

        sprintf (name, "cluster_%d" , int (i));
        pcl::visualization::PointCloudColorHandlerCustom<PointT> color0(boost::make_shared<pcl::PointCloud<PointT> >(cluster),red[i%6],grn[i%6],blu[i%6]);
        if (!viewer->updatePointCloud (boost::make_shared<pcl::PointCloud<PointT> >(cluster),color0,name))
            viewer->addPointCloud (boost::make_shared<pcl::PointCloud<PointT> >(cluster),color0,name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, name);
    }
}

void drawLine (pcl::visualization::PCLVisualizer::Ptr viewer, float x1, float y1, float z1, float x2, float y2, float z2, std::string color, std::string name)
{
    pcl::PointXYZ p1; pcl::PointXYZ p2;
    p1.x = x1; p1.y = y1; p1.z = z1;
    p2.x = x2; p2.y = y2; p2.z = z2;
    viewer->addLine(p1, p2, name);
    if (color.compare("r") == 0) 
    {
        viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, name);
    }
    else if (color.compare("b") == 0)
    {
        viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, name);
    }
    else
    {
        viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, name);
    }
    viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, name);

}


void drawLine (pcl::visualization::PCLVisualizer::Ptr viewer, Point p1, Point p2, std::string color, std::string name)
{
    drawLine(viewer, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, color, name);
}

void drawTrajectory (pcl::visualization::PCLVisualizer::Ptr viewer, Trajectory* traj, std::string color, std::string name)
{
    int size = traj->getSize();
    if (size < 2)
    {
        return;
    }
    Point* p1 = traj->getPosition(size-1);
    Point* p2 = traj->getPosition(size-2);
    drawLine(viewer, *p1, *p2, color, name);
}

void drawTrajectories(pcl::visualization::PCLVisualizer::Ptr viewer, std::vector<Trajectory*>* trajs)
{   

    for (int i = 0; i < trajs->size(); i++) 
    {
        Trajectory* traj = trajs->at(i);
        if (traj->isActive()) {
            std::ostringstream ss;
            ss << "line" << i ;
            std::string name = ss.str();

            if (traj->isPerson())
            {   
                drawTrajectory(viewer, traj, "r", name);
            }
            else 
            {
                //drawTrajectory(viewer, traj, "b", name);
            }
        }
    }
}

