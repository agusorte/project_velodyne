/*
 * ProjectVelodyne.hpp
 *
 *  Created on: Feb, 2018
 *  Author: Agustin Ortega
 *  Email: aortega.jim@gmail.com
 */

#pragma once
// c++
#include <math.h>
#include <string>
#include <vector>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h> 


// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

//PCL

#include <pcl/point_types.h>                
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
///BOOST
#include <boost/foreach.hpp>
#include <boost/assign/list_of.hpp>

// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>



namespace project_velodyne {

using namespace message_filters;
using namespace pcl;
using namespace sensor_msgs;
namespace enc = sensor_msgs::image_encodings;
// aproximated policy deffinitions
// and messages filters
typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
typedef message_filters::Subscriber<sensor_msgs::Image > ImageSubscriber;
typedef message_filters::Subscriber<sensor_msgs::PointCloud2 > CloudSubscriber;
typedef message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes > BoundingBoxSubscriber;

typedef sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::PointCloud2,darknet_ros_msgs::BoundingBoxes> MySyncPolicy_Aprox;
typedef boost::shared_ptr<darknet_ros_msgs::BoundingBoxes const > BoundingBoxesConstPtr;


class ProjectVelodyne
{
 public:
    /*!
   * Constructor.
   */
  explicit ProjectVelodyne(ros::NodeHandle nh);

  //!desttructor
  ~ProjectVelodyne();

    /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  void readParameters();

  /*!
  * Initialize the ROS connections.
  */
  void init();

  /*!
  * project point cloud to image coord system
  @param[in] cloud pointcloud to project
  @param[out] cloudProj projected pointcloud
  @param[out] indexes indexes of the original pointcloud
  */

  void projectPointCloud2Image( const PointCloud<PointXYZ>::Ptr &  cloud,
    PointCloud<PointXYZ>::Ptr &  cloudProj,
    std::vector<int >& indexes);

  /*!
  * segment point cloud eliminate and select 
  one part for the i meand some degrees for the field of view
  @param[in] cloud
  @param[out] cloudSeg cloud filtered or segmented
  NOT USED
  */

  void segmentPointCloud( const PointCloud<PointXYZ>::Ptr &  cloud,
    PointCloud<PointXYZ>::Ptr &  cloudSeg);

  /*!
  * Callback of camera, velodyne and Bounding boxes.
  * @param[in] msg image message.
  * @param[in] cloudI pointcloud message
  * @param[in] Boudingbox message YOLO
  */
  void callback(const sensor_msgs::ImageConstPtr& msg,
  const sensor_msgs::PointCloud2ConstPtr& cloudI,
  const BoundingBoxesConstPtr& bb_msg);

    /*!
  * creatre bounding box of a subcloud
  * @param[in] cloud.
  * @param[out] min_point_AABB  min point bounding box
  * @param[out] max_point_AABB max point bounding box
  * @param[out] min_point_OBB min point for the oriented BB
  * @param[out] max_point_OBB max point for the oriented BB
  * @param[out] position_OBB position of BB
  * @param[out] major_value // axis in x
  * @param[out] middle_value // axis in y
  * @param[out] minor_value // ayis in z
  * @param[out] marker // message to publish bases in cube list
  * 
  */
  
  void computeBoundingBox(const PointCloud<PointXYZ>::Ptr &  cloud,
  pcl::PointXYZ& min_point_AABB,
  pcl::PointXYZ& max_point_AABB,
  pcl::PointXYZ& min_point_OBB,
  pcl::PointXYZ& max_point_OBB,
  pcl::PointXYZ& position_OBB,
  float& angle_y,
  float& major_value, float& middle_value, float& minor_value,
  visualization_msgs::Marker& marker);

private:

  //! ROS node handle.
  ros::NodeHandle nodeHandle_;

  //! camera parameters
  Eigen::Matrix3f K;
  Eigen::Matrix3f R;
  Eigen::Vector3f t;

  //! radial distortion
  float k1,k2,k3,p1,p2;

  //! index projected points
  std::vector<int> cloudIndexes;

  //! vector of pointclouds objects
  std::vector<PointCloud<PointXYZ>::Ptr> cloudObjs;

  //! pointclouds projected
  PointCloud<PointXYZ>::Ptr cloudProj;

  //! pointcloud Original
  PointCloud<PointXYZ>::Ptr cloudOrig;

  //! pointcloud Original
  PointCloud<PointXYZ>::Ptr cloudSeg;

  //!image current
  cv::Mat imCurr;

  //!image width and height;
  int w,h;

  //!  kitti file format
  std::ofstream kittiFile; 

  //! topic for markers
  std::string bb_marker_topic;

  //! debbuging projectios
  bool image_view;

  //! kitti file
  std::string kittiFileName;


protected:


  //! Subscribers
  CloudSubscriber cloud_sub;
  ImageSubscriber image_sub;
  BoundingBoxSubscriber boundingBox_sub;

  //! syncronize messages based
  // in approximated policy
  Synchronizer<MySyncPolicy_Aprox>* sync;



  //! bounding box subscribe
  ros::Subscriber bb_sub;

  //! publishers
  ros::Publisher bb_marker_pub; // bounding box 
  ros::Publisher cloud_seg_pub; // pointc cloud
  ros::Publisher im_out_pub; // image with points projected




};
}

