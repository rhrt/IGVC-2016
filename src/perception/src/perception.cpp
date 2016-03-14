#include <iostream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include <tf/tf.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <sensor_msgs/PointCloud2.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "perception.h"
#include <math.h>
// #include "depth_to_3d.h"
// #include "depth_to_3d.cpp"
using namespace std;
using namespace cv;
#define PI 3.14159265

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace vision
{

PerceptionNode::PerceptionNode(const ros::NodeHandle& nh, uint threads = 3) : m_nh(nh), spinner_( threads ), ac("move_base", true)
{
  ROS_INFO("are we created?");
  m_loop_rate = 30;
  ros::NodeHandle ns;
  depthInitialized = false;
}

void PerceptionNode::rgbImageCallback(const sensor_msgs::ImageConstPtr& msg){
  ROS_INFO("Image Callback called");
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //img = cv_bridge::toCvShare(msg, "bgr8");
  //imshow("view", cv_ptr->image);
  Mat img;
  img = cv_ptr->image;
  ROS_INFO("I heard an image");
  //find any pedestrians
  HOGDescriptor hog;
  vector<Rect> found, found_filtered;
  hog.detectMultiScale(img, found, 0, Size(8, 8), Size(32, 32), 1.05, 2);


  //Drawing
  ROS_INFO("Found [%s] people", std::to_string(found.size()).c_str() );
  size_t i, j;
  for (i = 0; i < found.size(); i++)
  {
    Rect r = found[i];
    for (j = 0; j < found.size(); j++)
      if (j != i && (r & found[j]) == r)
        break;
    if (j == found.size())
      found_filtered.push_back(r);
  }
  for (i = 0; i < found_filtered.size(); i++)
  {
    Rect r = found_filtered[i];
    r.x += cvRound(r.width * 0.1);
    r.width = cvRound(r.width * 0.8);
    r.y += cvRound(r.height * 0.06);
    r.height = cvRound(r.height * 0.9);
    rectangle(img, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
  }
  imshow("view", img);



  //calculate actual position relative to robot
  int maxArea = 0;
  Rect tracking;
  for (i = 0; i < found.size(); i++)
  {
    Rect r = found[i];
    for (j = 0; j < found.size(); j++)
      if (j != i && (r & found[j]) == r)
        break;
    if (j == found.size())
      found_filtered.push_back(r);
  }
  for (i = 0; i < found_filtered.size(); i++)
  {
    Rect r = found_filtered[i];
    if (r.width * r.height > maxArea)
    {
      maxArea = r.width * r.height;
      tracking = Rect(r);
    }
    //r.x += cvRound(r.width*0.1);
    //r.width = cvRound(r.width*0.8);
    //r.y += cvRound(r.height*0.06);
    //r.height = cvRound(r.height*0.9);
    //rectangle(img, r.tl(), r.br(), cv::Scalar(0,255,0), 2);
  }
  ROS_INFO("Calculating Depth");
  if(!depthInitialized){
    ROS_INFO("No depth image yet");
    return;
  }

  Point center((tracking.tl().x + tracking.br().x) / 2.0, (tracking.tl().y + tracking.br().y) / 2.0);
  double depth =  currentDepthImage.at<short>(int(center.x), int(center.y)) / 1000.0  - 1.0;
  ROS_INFO("Person is at depth: [%s]", to_string(depth).c_str());


  cv::Point3f center3d(0, 0, 0);

  /*
    Mat KMatrix = Mat(3, 4, CV_32FC1);
    KMatrix[0] = cam_model_.cameraInfo().K[0];
    cv::rgbd::depthTo3dSparse(currentDepthImage, KMatrix, center, center3d);
  */
  if (found.size() >= 1) {

    ros::Rate rate(10.0);
    tf::StampedTransform transform;

    try {
      listener.lookupTransform("/base_link", "/odom",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    //geometry_msgs::Twist vel_msg;
    /*
    vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                transform.getOrigin().x());
    vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                              pow(transform.getOrigin().y(), 2));
    */
    int width = img.cols;
    double angle = (center.y - width / 2) * 55.0 / (1.0 * width);
    double finalAngle = (transform.getOrigin().w() - angle);
    double finalx = cos(finalAngle * PI / 180.0) * depth + transform.getOrigin().x();
    double finaly = sin(finalAngle * PI / 180.0) * depth + transform.getOrigin().y();
    ROS_INFO("Current pose: x,y,w [%s , %s , %s]", to_string(transform.getOrigin().x()).c_str(),
             to_string(transform.getOrigin().y()).c_str(),
             to_string(transform.getOrigin().w()).c_str());




    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = finalx;
    goal.target_pose.pose.position.y = finaly;
    //goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal x,y: [%s, %s]", to_string(finalx).c_str(), to_string(finaly).c_str());
    ac.sendGoal(goal);
  }

 
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved 1 meter forward");
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");
  
}


void PerceptionNode::depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("Depth Image Callback called");

  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  currentDepthImage = cv_ptr->image;
  depthInitialized = true;
  //ROS_INFO("I heard a depth image");
}
void PerceptionNode::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  cam_model_.fromCameraInfo(msg);
  //Kmatrix = msg->K;
}

}

int main (int argc, char** argv)
{
  ROS_INFO("Initializing perception node");
  ros::init(argc, argv, "human_finder");
  ros::NodeHandle n;
  //ros::Publisher persongoals_pub = n.advertise<std_msgs::String>("chatter", 1000);
  vision::PerceptionNode node(n);
  ros::Subscriber subd = n.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_color", 10, &vision::PerceptionNode::depthImageCallback, &node);
  ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color", 10, &vision::PerceptionNode::rgbImageCallback, &node);
  namedWindow("view");
  startWindowThread();
  // node.spin();
  ros::spin();
  destroyWindow("view");


}
