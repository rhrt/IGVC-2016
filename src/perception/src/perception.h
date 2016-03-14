//  Copyright 1234 blah
#ifndef PERCEPTION
#define PERCEPTION
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/image_encodings.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
//  Blah
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
namespace vision {
class PerceptionNode {
  int m_loop_rate;
  cv::Mat currentDepthImage;
  tf::TransformListener listener;
  MoveBaseClient ac;
  ros::NodeHandle m_nh;
  ros::MultiThreadedSpinner spinner_;
  image_geometry::PinholeCameraModel cam_model_;
  bool depthInitialized;
public:
  PerceptionNode(const ros::NodeHandle& nh, uint threads);
  void rgbImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void depthImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
  void spin(){spinner_.spin();};
};
}

using namespace std;
namespace std
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}
#endif