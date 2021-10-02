#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>

#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

#include <boost/thread.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <disparity_camera/disparityActionAction.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

using namespace std;
using namespace pcl;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

cv::Mat cam0, cam1;

typedef actionlib::SimpleActionClient<disparity_camera::disparityActionAction> Client;

Client *clientPointer;
disparity_camera::disparityActionActionGoal goal;
ros::Publisher pub_;

void threadSpinner(){

    ROS_INFO("Calling ros::spin() Spinning.......");
    ros::spin();
}

void spinThread(){

    Client &temp = *clientPointer;

    bool finished_before_timeout= temp.waitForResult(ros::Duration(3.0)); //wait max 30 sec

    if (finished_before_timeout){
        actionlib::SimpleClientGoalState state = temp.getState();
        cout<<"REPLY: "<<state.toString().c_str()<<endl;
    }

    else
        ROS_WARN("'%s' Action did not complete in the allocated time....",ros::this_node::getName().c_str());
}

void stereo_callback(const  sensor_msgs::ImageConstPtr& imageLeft, const sensor_msgs::ImageConstPtr& imageRight){

    //cout<<"Inside the client subscriber..........."<<endl;

    Client &can = *clientPointer;

    sensor_msgs::Image goal_image[2];

    cam0 = cv_bridge::toCvShare(imageLeft,"mono8")->image; //bgr8
    cam1 = cv_bridge::toCvShare(imageRight, "mono8")->image;

    //cout<<"row: "<<cam0.rows<<" cols: "<<cam0.cols<<" ch: "<<cam0.channels()<<endl;
    //cout<<"row: "<<cam1.rows<<" cols: "<<cam1.cols<<" ch: "<<cam1.channels()<<endl;

    cv_bridge::CvImage img_bridge_left, img_bridge_right, img_bridge_gan;

    std_msgs::Header header; // empty header
    //header.stamp = ros::Time::now(); // time
    header.stamp = imageLeft->header.stamp;

    img_bridge_left = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, cam0);
    img_bridge_left.toImageMsg(goal_image[0]);

    img_bridge_right = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, cam1);
    img_bridge_right.toImageMsg(goal_image[1]);

    goal.goal.input_image.clear();
    goal.goal.input_image.push_back(goal_image[0]);
    goal.goal.input_image.push_back(goal_image[1]);

    can.sendGoal(goal.goal);

    cout<<"Goal Sent ...................   "<<endl;

    boost::thread spin_thread(&spinThread);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "disparity_camera_client_cpu");

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> *img1_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,"/mapping/left/image_rect",1);
    message_filters::Subscriber<sensor_msgs::Image> *img2_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,"/mapping/right/image_rect",1);
    message_filters::sync_policies::ApproximateTime<Image, Image> MySyncPolicy(1000);
    MySyncPolicy.setAgePenalty(1);
    MySyncPolicy.setInterMessageLowerBound(0,ros::Duration{1});
    MySyncPolicy.setInterMessageLowerBound(1,ros::Duration{1});

    const message_filters::sync_policies::ApproximateTime<Image, Image> MyConstSyncPolicy = MySyncPolicy;
    message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<Image, Image> > sync(MyConstSyncPolicy, *img1_sub, *img2_sub);
    sync.registerCallback(boost::bind(&stereo_callback, _1, _2));

    std::string serverName = "disparity_camera_server_cpu";
    Client ac_(serverName);
    clientPointer = &ac_;

    ROS_WARN("Waiting for '%s' cpu server to start....",serverName.c_str());
    ac_.waitForServer();
    ROS_INFO("'%s' server started.....",serverName.c_str());

    ros::spin();
    return 0;
}
