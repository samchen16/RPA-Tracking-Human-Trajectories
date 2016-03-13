#include "util.h"
#include "trajectory.h"
#include <fstream>
#include "cluster_data.h"

float cluster_distance(ClusterData* cd1, ClusterData* cd2)
{
    Point p = cd1->getPosition();
    Point q = cd2->getPosition();
    float x = p.x - q.x;
    float y = p.y - q.y;
    float z = p.z - q.z;
    float dist = x*x + y*y + z*z;
    return dist;
}

void save_pointcloud(std::string name, PointCloudT::Ptr point_cloud) {
    pcl::PCDWriter writer;
    writer.write<PointT> (name, *point_cloud, false); 
}

void save_trajectories(std::vector<Trajectory*>* trajs, const char* filename)
{
    std::ofstream myfile;
    myfile.open(filename, std::ios_base::app);  

    for (int i = 0; i < trajs->size(); i++)
    { 
        Trajectory* traj = trajs->at(i);
        if (traj->getPositions()->size() < 3) {
            continue;
        }

        myfile << "traj " << traj->getID() << endl;

        if (traj->isPerson()) {
            myfile << "human" << endl;
        }
        else {
            myfile << "obstacle" << endl;
        }
        
        std::vector<Point*>* positions = traj->getPositions();
        std::vector<Point*>* velocities = traj->getVelocities();
        std::vector<float>* times = traj->getTimes();

        for(std::vector<Point*>::iterator itr = positions->begin(); itr != positions->end(); itr++)
        {
            Point* pos = *itr;
            myfile << "p " << pos->x << " " << pos->z << " " << pos->y <<std::endl;
        }
        for(std::vector<Point*>::iterator itr = velocities->begin(); itr != velocities->end(); itr++)
        {
            Point* vel = *itr;
            myfile << "v " << vel->x << " " << vel->z << " " << vel->y <<std::endl;
        }
        myfile << "v " << 0.0f << " " << 0.0f << " " << 0.0f <<std::endl;
        
        for(std::vector<float>::iterator itr = times->begin(); itr != times->end(); itr++)
        {
            float time = *itr;
            myfile << "t " << time <<std::endl;
        }
        myfile << "end" << std::endl;
    }

    myfile.close();
}


// Converts point cloud to RGB matrix
// RGB (0-255)
cv::Mat* convert_pc2RGBmat (PointCloudT::Ptr cloud)
{
    cv::Mat* mat = new cv::Mat(cloud->height, cloud->width, CV_8UC3); 
    if (cloud->isOrganized()) {
        for (int h = 0; h < mat->rows; h++) {
            for (int w = 0; w < mat->cols; w++) {
                Eigen::Vector3i rgb = cloud->at(w, h).getRGBVector3i();
                mat->at<cv::Vec3b>(h,w)[0] = rgb[2];
                mat->at<cv::Vec3b>(h,w)[1] = rgb[1];
                mat->at<cv::Vec3b>(h,w)[2] = rgb[0];
            }
        }
    }
    else {
       for (int w = 0; w < mat->cols; w++) {
            Eigen::Vector3i rgb = cloud->at(w).getRGBVector3i();
            mat->at<cv::Vec3b>(0,w)[0] = rgb[2];
            mat->at<cv::Vec3b>(0,w)[1] = rgb[1];
            mat->at<cv::Vec3b>(0,w)[2] = rgb[0];
        }
    }       
    return mat;
}


// Converts point cloud to either RGB or HSV matric
// RGB (0-255)
// HSV (0-180, 0-255, 0-255)
cv::Mat* convert_pc2mat (PointCloudT::Ptr cloud, bool hsv)
{
    cv::Mat* mat = convert_pc2RGBmat(cloud);
    if (hsv)
    {    
        cv::cvtColor(*mat, *mat, CV_BGR2HSV);
    }    
    return mat;
}


// Expects HSV image
cv::Mat* get_histogram(cv::Mat* mat) 
{
    int h_bins = NUM_HUE_BINS; int s_bins = NUM_SAT_BINS;
    int histSize[] = { h_bins, s_bins };
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 255 };
    const float* ranges[] = { h_ranges, s_ranges };
    int channels[] = { 0, 1 };

    cv::Mat* hist = new cv::Mat();
    calcHist( mat, 1, channels, cv::Mat(), *hist, 2, histSize, ranges, true, false );
    //normalize( *hist, *hist, 0, 1, cv::NORM_MINMAX, -1);

    return hist;
}

double compare_histograms (cv::Mat hist1, cv::Mat hist2)
{
    cv::Mat norm_hist1;
    cv::Mat norm_hist2;
    normalize( hist1, norm_hist1, 0, 1, cv::NORM_MINMAX, -1);
    normalize( hist2, norm_hist2, 0, 1, cv::NORM_MINMAX, -1);
    int compare_method = CV_COMP_CORREL; //CV_COMP_CHISQR;
    double similarity = compareHist( norm_hist1, norm_hist2, compare_method );

    //display_histogram_2D(hist1);
    //display_histogram_2D(hist2);

    return similarity;
}

// hue is row, varies with height
// sat in column, varies with width    
cv::Mat get_full_HSV(int height, int width, int hue_num_bins, int sat_num_bins)
{
    cv::Mat image( height, width, CV_8UC3, cv::Scalar( 0,0,0) );
    int bin_height = cvFloor( (double) height/hue_num_bins );
    int bin_width = cvFloor( (double) width/sat_num_bins );
    
    for(int i = 0; i < hue_num_bins; i++ )
    {   
        for (int j = 0; j < sat_num_bins; j++) 
        {
            cv::Vec3b color;            
            color[0] = i * cvRound(180.0/hue_num_bins); //hue
            color[1] = j * cvRound(255.0/sat_num_bins); //sat
            color[2] = 255; //val

            for (int r = i*bin_height; r < (i+1)*bin_height; r++)
            {
                for (int c = j*bin_width; c < (j+1)*bin_width; c++)
                { 
                    cv::Vec3b pixel_color = image.at<cv::Vec3b>(r,c);
                    pixel_color[0] = color[0];
                    pixel_color[1] = color[1];
                    pixel_color[2] = color[2];
                    image.at<cv::Vec3b>(r,c) = pixel_color;
                    
                }
            }
        }      
    }
    return image;
}

// takes hsv mat
void display_histogram_2D(cv::Mat hist) 
{
    // size of histogram image
    int height = 360; int width = 255;
    int hue_num_bins = NUM_HUE_BINS; 
    int sat_num_bins = NUM_SAT_BINS; 
    int bin_height = cvFloor( (double) height/hue_num_bins );
    int bin_width = cvFloor( (double) width/sat_num_bins );    
    cv::Mat hist_image( height, width, CV_8UC3, cv::Scalar( 0,0,0) ); 
    cv::Mat full_hsv = get_full_HSV(height, width, hue_num_bins, sat_num_bins);
    cv::cvtColor(full_hsv, full_hsv, CV_HSV2BGR);

    // create mask   
    cv::Mat mask( height, width, CV_8UC1, cv::Scalar(0) ); 
    for(int i = 0; i < hue_num_bins; i++ )
    {   
        for (int j = 0; j < sat_num_bins; j++) 
        {
            for (int r = i*bin_height; r < (i+1)*bin_height; r++)
            {
                for (int c = j*bin_width; c < (j+1)*bin_width; c++)
                { 
                    if (hist.at<float>(i,j) != 0)
                        mask.at<unsigned char>(r,c) = 1.0;
                }
            }
        }      
    }
    full_hsv.copyTo(hist_image, mask);
    
    // Display
    cv::namedWindow("histogram", cv::WINDOW_AUTOSIZE );
    cv::imshow("histogram", hist_image);

    cv::waitKey(0);
}
