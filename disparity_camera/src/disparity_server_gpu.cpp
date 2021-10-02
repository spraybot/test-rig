// ===================== c++ Headers ====================
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <iomanip>
#include <fstream>
#include <vector>
#include <time.h>
#include <omp.h>
// ===================== ROS Headers ====================
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
// ===================== PCL Headers ====================
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>


#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>


// ===================== OPENCV Headers ====================
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc.hpp"
#include <iostream>
#include <string>
// ===================== OPENCV Headers ====================
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>

// ===================== CUSTOM Headers ====================
#include "disparity_camera/disparityActionAction.h"
#include <pcl/visualization/cloud_viewer.h>
//================= SGBM STEREO Headers ===================
#include "disparity_camera/common.h"

#define z_lower 0    // in meter
#define z_higher 1.0 // in meter

using namespace std;
using namespace cv;
using namespace pcl;
using namespace cv::ximgproc;

//=========== Globals for CUDA ==============
void free_gpu_mem();
cv::Mat compute_disparity(cv::Mat* left_img, cv::Mat* right_img, float* cost_time);
void cuda_init(SGM_PARAMS* params);

class disparityAction{

public:
  disparityAction(std::string name) : as_(nh_, name, false), action_name_(name), it_(nh_), cv_ptr(new cv_bridge::CvImage){

    img_resize_flag_param = false;  // initial value
    img_resize_val_param = 0.5f;    // initial value

    as_.registerGoalCallback(boost::bind(&disparityAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&disparityAction::preemptCB, this));

    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud_inhand",10); // publish PCL to this topic
    as_.start();
    image_pub_ = it_.advertise("/disparity_map", 1);
  }

  void goalCB(){
    goal_ = as_.acceptNewGoal()->input_image;
    sgbm_compute();
  }

  void preemptCB(){
    ROS_ERROR("%s:Preempted:", action_name_.c_str());
    as_.setPreempted();
  }

  void display_pcl(PointCloud<PointXYZ>::Ptr c){
    pcl::visualization::PCLVisualizer viewer("visualize");
    viewer.addPointCloud<pcl::PointXYZ>(c, "cloud");

    while (!viewer.wasStopped())
      viewer.spinOnce();
  }


  void sgbm_compute(){
    //ROS_INFO("Inside sgbm_compute callback...");

    if (!as_.isActive()){
      ROS_WARN("Action_Server not active...... rosrun the action server first....");
      return;
    }

    left = cv_bridge::toCvCopy(goal_[0],  "mono8")->image; // this is rectified image
    right = cv_bridge::toCvCopy(goal_[1], "mono8")->image; // this is rectified image

    nh_.getParam("/disparity_camera_server_gpu/sgbm/resize_img_flag", img_resize_flag_param);
    cout<<"img_resize_flag_param: "<<img_resize_flag_param<<endl;

    nh_.getParam("/disparity_camera_server_gpu/sgbm/resize_img_val", img_resize_val_param);
    cout<<"img_resize_val_param: "<<img_resize_val_param<<endl;

    if (img_resize_flag_param == true){
      //nh_.getParam("/disparity_camera_server_gpu/sgbm/resize_img_val", img_resize_val_param);
      std::cout<<"resizing image...."<<std::endl;
      cv::resize(left, left, cv::Size(), img_resize_val_param, img_resize_val_param);
      cv::resize(right, right, cv::Size(), img_resize_val_param, img_resize_val_param);
    }

    //============== stereo calibraion param ========================
  /*
    base_line = -0.059390168;
    X = 0, Y = 0, Z = 0;
    f_norm = 1712.332973 * img_resize_val_param; // multuiplication to match the
                                               // size of the resized image
    Cx = 691.089867 * img_resize_val_param;
    Cy = 519.971565 * img_resize_val_param;
*/
    base_line = -0.059840622f;
    X = 0, Y = 0, Z = 0;
    f_norm = 1726.732396420546 * img_resize_val_param; // multuiplication to match the
                                               // size of the resized image
    Cx = 633.65170288 * img_resize_val_param;
    Cy = 494.84524917 * img_resize_val_param;

    Mat disp8(left.rows, left.cols, CV_32F);

/*
    int winSize = 3;
    Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,256,winSize);
    sgbm->setP1(72);
    sgbm->setP2(288);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(256);
    sgbm->setUniquenessRatio(10);
    sgbm->setPreFilterCap(11);
    sgbm->setSpeckleWindowSize(15);
    sgbm->setSpeckleRange(7);   // lower the better
    sgbm->setDisp12MaxDiff(50);
    sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);
    sgbm->compute(left,right,sgbm_disp);
    sgbm_disp.convertTo(disp8, CV_32F, 1.0/16.0);
*/
    params.preFilterCap = 63;
    params.BlockSize = 3;
    params.P1 = 24 * params.BlockSize * params.BlockSize;
    params.P2 = 96 * params.BlockSize * params.BlockSize;
    params.uniquenessRatio = 0;
    params.disp12MaxDiff = 1000;
    cuda_init(&params);

    //start = cv::getTickCount();
    disp8 = compute_disparity(&left, &right, NULL);
    Mat filtered_disp_vis;
    getDisparityVis(disp8,filtered_disp_vis,1.0);
    namedWindow("filtered disparity GPU (no filtering)", WINDOW_AUTOSIZE);
    imshow("filtered disparity GPU (no filtering)", filtered_disp_vis);
    cv::waitKey(1);    
    

    // ===================== ATTEMPT TO USE THE WLS FILTER    
    /*
    double lambda = 8000.0;
    double sigma  = 1.5;
    double fbs_spatial = 16.0;
    double fbs_luma = 8.0;
    double fbs_chroma = 8.0;
    double fbs_lambda = 128.0;
    double vis_mult = 1.0;

    Mat left_flip;
    Mat right_flip;
    flip(left,left_flip,1);
    flip(right,right_flip,1);
    //start = cv::getTickCount();
    disp8_right_flipped = compute_disparity(&right_flip, &left_flip, NULL);
    
    flip(disp8_right_flipped,disp8_right,1);
    

    Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(0,160,params.BlockSize);
    left_matcher->setP1(24*params.BlockSize*params.BlockSize);
    left_matcher->setP2(96*params.BlockSize*params.BlockSize);
    left_matcher->setPreFilterCap(63);
    left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);

    Ptr<DisparityWLSFilter> wls_filter = createDisparityWLSFilter(left_matcher);
    Mat filtered_disp;

    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);
    wls_filter->filter(disp8_left,left,filtered_disp,disp8_right);
    cout << filtered_disp.size() << endl;

    //Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher); 
    
    Mat filtered_disp_vis;
    getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
    namedWindow("filtered disparity", WINDOW_AUTOSIZE);
    imshow("filtered disparity", filtered_disp_vis);


    Mat right_disp_vis;
    getDisparityVis(disp8_right,right_disp_vis,1.0);
    namedWindow("right_disparity", WINDOW_AUTOSIZE);
    imshow("right_disparity", right_disp_vis);
    cv::waitKey(1);      
    */
    // ===================== END ATTEMPT TO USE THE WLS FILTER   

    
    //disp8.convertTo(disp8, CV_32F, 1.0/16.0);
    //end = cv::getTickCount();
    //duration = (end - start)* 1000/cv::getTickFrequency();// in ms
    //cout<<"stereo duration: "<<duration<<" ms"<<endl;


    //cv::namedWindow("cam1", 0);
    //imshow("cam1", disp8);
    //cv::waitKey(1);
    
    pcl::PointXYZRGB cloud_xyz;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr CLOUD( new pcl::PointCloud<pcl::PointXYZ>); // this is a smart pointer
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr CLOUD = pcl::PointCloud<pcl::PointXYZRGB>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGB>);
    
    sensor_msgs::PointCloud2 point_cloud_result;

    //start = cv::getTickCount();

    //#pragma omp parallel for
    for (int i = 0; i < left.rows; i++){
      for (int j = 0; j < left.cols; j++){

        Z = (float)((-1 * f_norm * base_line) / ((int) filtered_disp_vis.at<uchar>(i, j)));
        X = (float)(Z / f_norm) * (j - Cx); // cols
        Y = (float)(Z / f_norm) * (i - Cy); // rows

        cloud_xyz.x = X;
        cloud_xyz.y = Y;
        cloud_xyz.z = Z;

        //cloud_xyz.r = 255;
        //cloud_xyz.g = 0;
        //cloud_xyz.b = 0;

        if ((Z >= 0.1 && Z < 1)){
          //Z = 1.009037*Z - 0.0281793;
          CLOUD->points.push_back(cloud_xyz);
        }
      }
    }

    //end = cv::getTickCount();

    //duration = (end - start)* 1000/cv::getTickFrequency();// in ms

    //cout<<"duration: "<<duration<<" ms"<<endl;

    cout<<"cloud size: "<<CLOUD->points.size()<<endl;
    //display_pcl(CLOUD);

//================================= VOXEL FILTERING =======================================

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr no_nan_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    vector<int> no_nan_index;
    pcl::removeNaNFromPointCloud(*CLOUD,*no_nan_cloud,no_nan_index);

    pcl::PointIndices::Ptr removed_indices (new pcl::PointIndices);//, removed_indices_cane (new pcl::PointIndices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr VF_CLOUD(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr IN_CLOUD(new pcl::PointCloud<pcl::PointXYZRGB>);
    *IN_CLOUD = *no_nan_cloud;

    //=========================== VOX FILTERING ===============================================

    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(IN_CLOUD);
    sor.setLeafSize (0.005f, 0.005f, 0.01f); // 0.005 0.005 0.005
    sor.filter (*VF_CLOUD);

    cout<<"VF_CLOUD size: "<<VF_CLOUD->points.size()<<endl;

    //=========================== SOR FILTERING ===============================================
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter (true);
    filter.setInputCloud (VF_CLOUD);
    filter.setMeanK (10); //5,10
    filter.setStddevMulThresh (0.5); //0.95  0.01  // low STD = more filtering
    filter.setNegative (0);
    filter.setKeepOrganized (false);
    filter.filter (*xyz_cloud_filtered);
    filter.getRemovedIndices (*removed_indices);

    //======================= CONVERTING TO ROS MESSAGE =======================================

    pcl::toROSMsg(*xyz_cloud_filtered, point_cloud_result);
    point_cloud_result.header.frame_id = "inhand_link";
    ros::Time time = ros::Time::now();
    //point_cloud_result.header.stamp = time;
    point_cloud_result.header.stamp = goal_[0].header.stamp;
    pub_.publish(point_cloud_result);
    cout << goal_[0].header.stamp << endl;
    cv_ptr->encoding = "mono8";
    //cv_ptr->header.stamp = time;
    //cout << goal_[0].header.stamp << endl;
    cv_ptr->header.stamp = goal_[0].header.stamp;
    cv_ptr->header.frame_id = "inhand_link";

    cv_ptr->image = filtered_disp_vis;
    image_pub_.publish(cv_ptr->toImageMsg());
    cout << goal_[0].header.stamp << endl;
    as_.setSucceeded(result_.result);
    ROS_INFO("Action successfully executed ...............................");
  }

  ~disparityAction() {}

  cv::Mat left, right;
  float base_line;

  float X, Y, Z;
  double f_norm; // =  CamLProj.at<double> (0,0);
  double Cx;     // =  CamLMatrix.at<double>(0,2);
  double Cy;     // =  CamLMatrix.at<double>(1,2);
  bool img_resize_flag_param;
  double img_resize_val_param;
  double start, end, duration;
  cv::Mat sgbm_disp;
  ros::NodeHandle nh_;

protected:
  
  actionlib::SimpleActionServer<disparity_camera::disparityActionAction> as_;
  std::string action_name_;
  disparity_camera::disparityActionActionFeedback feedback_;
  disparity_camera::disparityActionActionResult result_;
  std::vector<sensor_msgs::Image> goal_;
  ros::Publisher pub_;
  SGM_PARAMS params;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;

  cv_bridge::CvImagePtr cv_ptr;
};

int main(int argc, char** argv){

  ros::init(argc, argv, "disparity_camera_server_gpu");
  disparityAction action_bottom(ros::this_node::getName());

  ros::spin();
  return 0;
}
