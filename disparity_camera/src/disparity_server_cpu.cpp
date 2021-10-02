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
//================= SGBM GPU STEREO Headers ===================
//#include "disparity_camera/common.h"

#define z_lower 0    // in meter
#define z_higher 1.0 // in meter

using namespace std;
using namespace cv;
using namespace pcl;
using namespace cv::ximgproc;





class disparityAction{

public:
  disparityAction(std::string name) : as_(nh_, name, false), action_name_(name), it_(nh_), cv_ptr(new cv_bridge::CvImage){

    CLOUD = pcl::PointCloud<pcl::PointXYZRGB>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGB>); // this is a smart pointer
    as_.registerGoalCallback(boost::bind(&disparityAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&disparityAction::preemptCB, this));
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/point_cloud_inhand",10); // publish PCL to this topic
    as_.start();
    image_pub_ = it_.advertise("/disparity_map", 1);
  }

  void init(){

    //default values

    ParamResizeImgFlag = false;
    ParamResizeImgVal = 1;//0.5;

    base_line = -0.059840622f; // tis comes from the Stereo calibration process
    X = 0, Y = 0, Z = 0;
    f_norm = 1726.732396420546 * ParamResizeImgVal; // multuiplication to match the
                                                    // size of the resized image
    Cx = 633.65170288 * ParamResizeImgVal;
    Cy = 494.84524917 * ParamResizeImgVal;

    ParamWinSize = 3;
    ParamMinDisparity = 0;
    ParamMaxDisparity = 416;
    ParamUniqueRatio = 30;
    ParamPreFilterCap = 11;
    ParamSpeckleWinSize = 30;
    ParamSpeckleRange = 3; // lower the better
    ParamDisp12MaxDisff = 0;

  }

  void goalCB(){
    goal_ = as_.acceptNewGoal()->input_image;    
    init();
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
      ROS_WARN("Action_Server CPU not active...... rosrun the action server first....");
      return;
    }

    left =  cv_bridge::toCvCopy(goal_[0], "mono8")->image; // this is rectified image
    right = cv_bridge::toCvCopy(goal_[1], "mono8")->image; // this is rectified image

    nh_.getParam("/disparity_camera_server_cpu/sgbm/ResizeImgFlag", ParamResizeImgFlag);
    nh_.getParam("/disparity_camera_server_cpu/sgbm/ResizeImgVal", ParamResizeImgVal);
    nh_.getParam("/disparity_camera_server_cpu/sgbm/WinSize", ParamWinSize);
    nh_.getParam("/disparity_camera_server_cpu/sgbm/MinDisparity", ParamMinDisparity);
    nh_.getParam("/disparity_camera_server_cpu/sgbm/MaxDisparity", ParamMaxDisparity);
    nh_.getParam("/disparity_camera_server_cpu/sgbm/UniqueRatio", ParamUniqueRatio);
    nh_.getParam("/disparity_camera_server_cpu/sgbm/PreFilterCap", ParamPreFilterCap);
    nh_.getParam("/disparity_camera_server_cpu/sgbm/SpeckleWinSize", ParamSpeckleWinSize);
    nh_.getParam("/disparity_camera_server_cpu/sgbm/SpeckleRange", ParamSpeckleRange);
    nh_.getParam("/disparity_camera_server_cpu/sgbm/Disp12MaxDisff", ParamDisp12MaxDisff);

    nh_.getParam("/disparity_camera_server_cpu/calib/Cx", Cx);
    nh_.getParam("/disparity_camera_server_cpu/calib/Cy", Cy);
    nh_.getParam("/disparity_camera_server_cpu/calib/StereoBaseline", base_line);
    nh_.getParam("/disparity_camera_server_cpu/calib/FNorm", f_norm);

    double lambda = 8000.0;
    double sigma  = 1.5;
    double fbs_spatial = 16.0;
    double fbs_luma = 8.0;
    double fbs_chroma = 8.0;
    double fbs_lambda = 128.0;
    double vis_mult = 1.0;

    /*
    cout<<"======================= PARAM PRINT OUT ==============================="<<endl;
    cout<<"/disparity_camera_server_cpu/calib/Cx"<< Cx<<endl;
    cout<<"/disparity_camera_server_cpu/calib/Cy"<< Cy<<endl;
    cout<<"/disparity_camera_server_cpu/calib/StereoBaseline"<< base_line<<endl;
    cout<<"/disparity_camera_server_cpu/calib/FNorm"<< f_norm<<endl;
    cout<<"/disparity_camera_server_cpu/sgbm/ResizeImgFlag: "<< ParamResizeImgFlag<<endl;
    cout<<"/disparity_camera_server_cpu/sgbm/ResizeImgVal: "<< ParamResizeImgVal<<endl;
    cout<<"/disparity_camera_server_cpu/sgbm/WinSize: "<< ParamWinSize<<endl;
    cout<<"/disparity_camera_server_cpu/sgbm/MinDisparity: "<< ParamMinDisparity<<endl;
    cout<<"/disparity_camera_server_cpu/sgbm/MaxDisparity: "<< ParamMaxDisparity<<endl;
    cout<<"/disparity_camera_server_cpu/sgbm/UniqueRatio: "<< ParamUniqueRatio<<endl;
    cout<<"/disparity_camera_server_cpu/sgbm/PreFilterCap: "<< ParamPreFilterCap<<endl;
    cout<<"/disparity_camera_server_cpu/sgbm/SpeckleWinSize: "<< ParamSpeckleWinSize<<endl;
    cout<<"/disparity_camera_server_cpu/sgbm/SpeckleRange: "<< ParamSpeckleRange<<endl;
    cout<<"/disparity_camera_server_cpu/sgbm/Disp12MaxDisff:"<< ParamDisp12MaxDisff<<endl;

    */

    if (ParamResizeImgFlag == true){
      nh_.getParam("/disparity_camera_server_cpu/sgbm/ResizeImgVal", ParamResizeImgVal);
      cv::resize(left, left, cv::Size(), ParamResizeImgVal, ParamResizeImgVal);
      cv::resize(right, right, cv::Size(), ParamResizeImgVal, ParamResizeImgVal);
    }

    //============== stereo calibraion param ========================


    Cx = Cx * static_cast<double>(ParamResizeImgVal);
    Cy = Cy * static_cast<double>(ParamResizeImgVal);
    f_norm = f_norm * static_cast<double>(ParamResizeImgVal);
    
    
    // ================ Disparity + Filtering =======================
    
    Mat left_for_matcher, right_for_matcher;
    Mat left_disp,right_disp;
    Mat filtered_disp,solved_disp,solved_filtered_disp;
    Mat conf_map = Mat(left.rows,left.cols,CV_8U);
    conf_map = Scalar(255);
    Rect ROI;
    Ptr<DisparityWLSFilter> wls_filter;
    double matching_time, filtering_time;
    double solving_time = 0; 

    
    Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(ParamMinDisparity,ParamMaxDisparity,ParamWinSize);
    left_matcher->setP1(24*ParamWinSize*ParamWinSize);
    left_matcher->setP2(96*ParamWinSize*ParamWinSize);
    left_matcher->setPreFilterCap(63);
    left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
    
    //wls_filter = createDisparityWLSFilter(left_matcher);
    
    //Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

    //matching_time = (double)getTickCount();
    left_matcher->compute(left, right, left_disp);
    
    /*
    right_matcher->compute(right, left, right_disp);
  

    matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();
    

    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);
    filtering_time = (double)getTickCount();
    wls_filter->filter(left_disp,left,filtered_disp,right_disp);
    filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
     */

    //cv::namedWindow("left", WINDOW_AUTOSIZE);
    //imshow("left", left);

    //cv::namedWindow("right", WINDOW_AUTOSIZE);
    //imshow("right", right);

    Mat filtered_disp_vis;
    //getDisparityVis(filtered_disp,filtered_disp_vis,vis_mult);
    getDisparityVis(left_disp,filtered_disp_vis,vis_mult);
    namedWindow("filtered disparity CPU (+filtering)", WINDOW_AUTOSIZE);
    imshow("filtered disparity CPU (+filtering)", filtered_disp_vis);
    cv::waitKey(1);
    
    
    // ================= END ==========================================

    //#pragma omp parallel for
    CLOUD->points.clear();
    for (int i = 0; i < left.rows; i++){
      for (int j = 0; j < left.cols; j++){
        int disp = (int) filtered_disp_vis.at<uchar>(i, j);
        if (disp == 0) continue;
        
        Z = (float)((-f_norm * base_line) / (filtered_disp_vis.at<uchar>(i, j)));
        X = (float)(Z / f_norm) * (j - Cx); // cols
        Y = (float)(Z / f_norm) * (i - Cy); // rows

        cloud_xyz.x = X;
        cloud_xyz.y = Y;
        cloud_xyz.z = Z;

        cloud_xyz.r = 0;
        cloud_xyz.g = 255;
        cloud_xyz.b = 0;
        //cout << X << "," << Y << "," << Z << endl;
        //CLOUD->points.push_back(cloud_xyz);
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
    /*
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

    */
    //pcl::io::savePCDFile("/home/abhi/ros/catkin_ws/src/rovin_move/src/FOLDER/inhand_cloud.pcd",
    //                     *xyz_cloud_filtered, true);
    //======================= CONVERTING TO ROS MESSAGE =======================================

    //pcl::toROSMsg(*xyz_cloud_filtered, point_cloud_result);
    pcl::toROSMsg(*CLOUD, point_cloud_result);
    point_cloud_result.header.frame_id = "inhand_link";
    ros::Time time = ros::Time::now();
    //point_cloud_result.header.stamp = time;
    point_cloud_result.header.stamp = goal_[0].header.stamp;
    pub_.publish(point_cloud_result);

    cv_ptr->encoding = "mono8";
    //cv_ptr->header.stamp = time;
    //cout << goal_[0].header.stamp << endl;
    cv_ptr->header.stamp = goal_[0].header.stamp;
    cv_ptr->header.frame_id = "inhand_link";

    cv_ptr->image = filtered_disp_vis;
    image_pub_.publish(cv_ptr->toImageMsg());

    as_.setSucceeded(result_.result);
    ROS_INFO("Action successfully executed ...............................");
  }

  ~disparityAction() {}

  cv::Mat left, right;

  double base_line;
  double X, Y, Z;
  double f_norm; // =  CamLProj.at<double> (0,0);
  double Cx;     // =  CamLMatrix.at<double>(0,2);
  double Cy;     // =  CamLMatrix.at<double>(1,2);
  bool  ParamResizeImgFlag;
  double ParamResizeImgVal;
  int ParamWinSize;
  int ParamMinDisparity;
  int ParamMaxDisparity;
  int ParamUniqueRatio;
  int ParamPreFilterCap;
  int ParamSpeckleWinSize;
  int ParamSpeckleRange;
  int ParamDisp12MaxDisff;

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

  pcl::PointXYZRGB cloud_xyz;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr CLOUD;//( new pcl::PointCloud<pcl::PointXYZ>); // this is a smart pointer
  sensor_msgs::PointCloud2 point_cloud_result;

  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;

  cv_bridge::CvImagePtr cv_ptr;

};

int main(int argc, char** argv){

  ros::init(argc, argv, "disparity_camera_server_cpu");
  disparityAction action_bottom(ros::this_node::getName());

  ros::spin();
  return 0;
}
