
#include "project_velodyne/ProjectVelodyne.hpp"

/*
 * ProjectVelodyne.cpp
 *
 *  Created on: Feb, 2018
 *  Author: Agustin Ortega
 *  Email: aortega.jim@gmail.com
 */

namespace project_velodyne {

ProjectVelodyne::ProjectVelodyne(ros::NodeHandle nh)
    : nodeHandle_(nh){
      
  ROS_INFO("[ProjectVelodyne] Node started.");


  init();
}

ProjectVelodyne::~ProjectVelodyne(){

  kittiFile.close();
}
void ProjectVelodyne::init(){


 ROS_INFO ("[init] project_velodyne_node");
 // read parameters
 readParameters();
   
}

void ProjectVelodyne::readParameters(){

  //camera parameters
  K = Eigen::Matrix3f::Zero();
  R = Eigen::Matrix3f::Zero(3,3); 
  t = Eigen::Vector3f::Zero();

  // Set vector sizes.
  std::vector<float> r,t_,k,d;

  nodeHandle_.param("camera0/rotation", r, std::vector<float>(0));
  nodeHandle_.param("camera0/translation", t_, std::vector<float>(0));
  nodeHandle_.param("camera0/intrisics", k, std::vector<float>(0));
  nodeHandle_.param("camera0/distortion", d, std::vector<float>(0));


  R = Eigen::Matrix3f(r.data()).transpose();
  K = Eigen::Matrix3f(k.data()).transpose();
  t = Eigen::Vector3f(t_.data());

  // distortion
  k1 = d[0];
  k2 = d[1];
  k3 = d[4];
  p1 = d[2];
  p2 = d[3];

  // read parameters
  // subscribe topics
  std::string im_topic;
  std::string pc_topic;
  std::string bb_topic;
  std::string bb2_topic;
  std::string pc2_topic;
  std::string imOut_topic;


  // topics subscribers and publishers
  nodeHandle_.param("subscribers_/camera_reading/topic", im_topic, std::string("/camera/image_raw"));
  nodeHandle_.param("subscribers_/pointcloud_reading/topic", pc_topic, std::string("velodyne_in"));
  nodeHandle_.param("subscribers_/bounding_box_reading/topic", bb_topic,  std::string("bounding_boxes_in"));

  nodeHandle_.param("publishers_/pointcloud_writting/topic", pc2_topic, std::string("velodyne_out"));
  nodeHandle_.param("publishers_/bounding_box_writting/topic", bb2_topic,  std::string("bounding_boxes_out"));
  nodeHandle_.param("publishers_/image_writting/topic", imOut_topic,  std::string("image_out"));


  // subscribers
  image_sub.subscribe(nodeHandle_ ,im_topic ,1);
  cloud_sub.subscribe(nodeHandle_,pc_topic,1);
  boundingBox_sub.subscribe(nodeHandle_,bb_topic,1);

  // callback based in sync policy
  sync = new Synchronizer<MySyncPolicy_Aprox>(MySyncPolicy_Aprox(700),image_sub,cloud_sub,boundingBox_sub);
  sync->registerCallback(boost::bind(&ProjectVelodyne::callback,this,_1,_2,_3));

  //publishers
  bb_marker_pub = nodeHandle_.advertise<visualization_msgs::Marker>(bb2_topic.c_str(), 1);
  cloud_seg_pub = nodeHandle_.advertise<sensor_msgs::PointCloud2>(pc2_topic.c_str(), 1);
  im_out_pub    = nodeHandle_.advertise<sensor_msgs::Image>(imOut_topic.c_str(), 1);

  // image out 
  nodeHandle_.param("image_output", image_view, true);

  // outr kitty file
  nodeHandle_.param("kittifile", kittiFileName, std::string("/default"));

  // file kitti creation
  kittiFile.open(kittiFileName);

  if (!kittiFile.is_open())
   ROS_ERROR("Error creating kitti file");
    
}



void ProjectVelodyne::callback(const sensor_msgs::ImageConstPtr& msg,
  const sensor_msgs::PointCloud2ConstPtr& cloudI,
  const BoundingBoxesConstPtr& bb_msg){

  // point cloud
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
   
   
  if ((cloudI->width * cloudI->height) == 0)
     return ;
   
   pcl::fromROSMsg(*cloudI, *cloud);
   
  // images
  cv_bridge::CvImageConstPtr cv_ptr;
  
  try{
   
   if (enc::isColor(msg->encoding)) 
     cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
   else
     cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
  }
  catch (cv_bridge::Exception& e){
   
   ROS_ERROR("cv_bridge exception: %s", e.what());
   
   return ;
   
  }

  imCurr = cv_ptr->image;
  cv::Mat im = imCurr.clone();

  w = im.cols;
  h = im.rows;

  cloudProj =  PointCloud<pcl::PointXYZ>::Ptr(new PointCloud<pcl::PointXYZ>);
  cloudIndexes.clear();

  projectPointCloud2Image(cloud,cloudProj,cloudIndexes);
  cloudOrig =  PointCloud<pcl::PointXYZ>::Ptr(cloud);


  for (auto it:cloudProj->points) {
    cv::Point pt = cv::Point(it.y/it.z,it.x/it.z);
    cv::circle(im,pt, 1, cv::Scalar(0, 255, 0), CV_FILLED, CV_AA);


  }

  cloudSeg =  PointCloud<pcl::PointXYZ>::Ptr(new PointCloud<pcl::PointXYZ>);
  for (auto i:bb_msg->boundingBoxes){
   
    PointCloud<PointXYZ>::Ptr subcloud(new PointCloud<PointXYZ>);

    for (size_t j=0; j < cloudProj->points.size(); j++) {

      PointXYZ pp = cloudProj->points[j];
      cv::Point pt = cv::Point(pp.y/pp.z,pp.x/pp.z);

      if( i.xmin<pt.x && i.ymin < pt.y && pt.x < i.xmax && pt.y <  i.ymax){
        
        cv::circle(im,pt, 1, cv::Scalar(0, 0, 255), CV_FILLED, CV_AA);
        subcloud->push_back(cloudOrig->points[cloudIndexes[j]]);
      
      }
      
    }

    // find closest point to avoid noisy points
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (subcloud);

    int K = 600;

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*subcloud, minPt, maxPt);
    PointCloud<PointXYZ>::Ptr subclouda(new PointCloud<PointXYZ>);


    if ( kdtree.nearestKSearch (minPt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      for (size_t ii = 0; ii < pointIdxNKNSearch.size (); ++ii){
        subclouda->points.push_back(subcloud->points[ pointIdxNKNSearch[ii] ]);
        cloudSeg->push_back(subcloud->points[ pointIdxNKNSearch[ii] ]);
      }

    }

    cloudObjs.push_back(subclouda);



  }

  // see if point is inside of BB

  pcl::PointXYZ min_point_AABB;
  pcl::PointXYZ max_point_AABB;
  pcl::PointXYZ min_point_OBB;
  pcl::PointXYZ max_point_OBB;
  pcl::PointXYZ position_OBB;
  float major_value, middle_value, minor_value, angle_y;

  int i= 0;
  //visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker;

  for (auto cloudi:cloudObjs){

   
    computeBoundingBox(cloudi,
    min_point_AABB,max_point_AABB,min_point_OBB,max_point_OBB,
    position_OBB, angle_y, major_value, middle_value, minor_value,
    marker);
    //KITTI file format
    //   1 type Describes the type of object: 'Car', 'Van', 'Truck',
    // 'Pedestrian', 'Person_sitting', 'Cyclist', 'Tram',
    // 'Misc' or 'DontCare'
    // 1 truncated Float from 0 (non-truncated) to 1 (truncated), where
    // truncated refers to the object leaving image boundaries
    // 1 occluded Integer (0,1,2,3) indicating occlusion state:
    // 0 = fully visible, 1 = partly occluded
    // 2 = largely occluded, 3 = unknown
    // 1 alpha Observation angle of object, ranging [-pi..pi]
    // 4 bbox 2D bounding box of object in the image (0-based index):
    // contains left, top, right, bottom pixel coordinates
    // 3 dimensions 3D object dimensions: height, width, length (in meters)
    // 3 location 3D object location x,y,z in camera coordinates (in meters)
    // 1 rotation_y Rotation ry around Y-axis in camera coordinates [-pi..pi]
    // 1 score Only for results: Float, indicating confidence in
    // detection, needed for p/r curves, higher is better.


    kittiFile<<"\'"+bb_msg->boundingBoxes[i].Class+"\'"<<" "<<1<<" "
    <<3<<" "<<angle_y<<" " // check value
    << bb_msg->boundingBoxes[i].xmin<<" "<<bb_msg->boundingBoxes[i].ymin<<" "
    << bb_msg->boundingBoxes[i].xmax<<" "<<bb_msg->boundingBoxes[i].ymax<<" "
    << minor_value<<" "<<major_value<<" "<<middle_value<<" " // check values
    << position_OBB.x<<" "<<position_OBB.y<<" "<<position_OBB.z<<" "
    << angle_y<<" "<< bb_msg->boundingBoxes[i].probability<<" \n";

  }

  // publish markers bounding boxes
  bb_marker_pub.publish(marker);

  // compute bounding box of regions or cubes
  cloudObjs.clear();


  // publish objects found
  sensor_msgs::PointCloud2 outCloudseg;
  pcl::toROSMsg(*cloudSeg,outCloudseg);
  outCloudseg.header.frame_id ="/velodyne";
  cloud_seg_pub.publish(outCloudseg);

  // publish image pointcloud projected
  sensor_msgs::ImagePtr im_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", im).toImageMsg();
  im_out_pub.publish(im_msg);

}

void ProjectVelodyne::projectPointCloud2Image( const PointCloud<PointXYZ>::Ptr &  cloud,
     PointCloud<PointXYZ>::Ptr &  cloudProj,
     std::vector<int >& indexes)
{

  
  float cx = K(0,2);
  float cy = K(1,2);
  float fx = K(0,0);
  float fy = K(1,1);


  // index counters
  int i = 0;
 
  for (auto it:cloud->points) {

    Eigen::Vector3f pt = Eigen::Vector3f(it.x, it.y, it.z);

    // transform laser to camera coord
    Eigen::Vector3f ptrans = K*(R*pt+t); 

    // point distorted
    cv::Point2f pdis = cv::Point2f(ptrans(0)/ptrans(2), ptrans(1)/ptrans(2));
    float x = (pdis.x - cx)/fx;
    float y = (pdis.y - cy)/fy;
    float r2 = x*x + y*y;

    // model of distortion
    float xdist = x * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2);
    float ydist = y * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2);


    xdist = xdist + (2. * p1 * x * y + p2 * (r2 + 2. * x * x));
    ydist = ydist + (p1 * (r2 + 2. * y * y) + 2. * p2 * x * y);

     // Back to absolute coordinates.
    xdist = xdist * fx + cx;
    ydist = ydist * fy + cy;


    // just consider point in that are in front on the focal axis
    if(ptrans(2) > 0){
      
      if(0<ydist && ydist<w && 0<xdist && xdist<h){

        PointXYZ ptrans_ = PointXYZ(   xdist * ptrans(2),  ydist * ptrans(2), ptrans(2));
        cloudProj->points.push_back(ptrans_);
        indexes.push_back(i);

      }
    }
  
    i++;
              
  }
  
}


void ProjectVelodyne::segmentPointCloud( const PointCloud<PointXYZ>::Ptr &  cloud,
    PointCloud<PointXYZ>::Ptr &  cloudSeg){

  /// remove principal plane
  PassThrough<PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");// here we have to see where x and z
  pass.setFilterLimits (-1.5, 1.5);
  pass.filter (*cloudSeg);


}

void ProjectVelodyne::computeBoundingBox(const PointCloud<PointXYZ>::Ptr &  cloud,
    pcl::PointXYZ& min_point_AABB,
    pcl::PointXYZ& max_point_AABB,
    pcl::PointXYZ& min_point_OBB,
    pcl::PointXYZ& max_point_OBB,
    pcl::PointXYZ& position_OBB,
    float& angle_y,
    float& major_value, float& middle_value, float& minor_value,
    visualization_msgs::Marker& marker){



  pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud (cloud);
  feature_extractor.compute ();

  std::vector <float> moment_of_inertia;
  std::vector <float> eccentricity;
  
  Eigen::Matrix3f rotational_matrix_OBB;
  Eigen::Vector3f major_vector, middle_vector, minor_vector;
  Eigen::Vector3f mass_center;

  // compute moments and 
  // extract bounding box from pointcloud
  feature_extractor.getMomentOfInertia (moment_of_inertia);
  feature_extractor.getEccentricity (eccentricity);
  feature_extractor.getAABB (min_point_AABB, max_point_AABB);
  feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
  feature_extractor.getEigenValues (major_value, middle_value, minor_value);
  feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
  feature_extractor.getMassCenter (mass_center);

 
  Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
  Eigen::Quaternionf quat (rotational_matrix_OBB);

  Eigen::Vector3f ea = rotational_matrix_OBB.eulerAngles(0, 1, 2);

  angle_y = ea[1]; 

  pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
  pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
  pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
  pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
 
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "velodyne";
  marker.header.stamp = ros::Time::now();
  marker.ns = "BB";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point temp;
  
  temp.x =mass_center (0);
  temp.y =mass_center (1);
  temp.z =mass_center (2);

  marker.points.push_back(temp);

  marker.pose.orientation.x = quat.x();
  marker.pose.orientation.y = quat.y();
  marker.pose.orientation.z = quat.z();
  marker.pose.orientation.w = quat.w();
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;

  marker.scale.x = max_point_AABB.x - min_point_AABB.x;
  marker.scale.y = max_point_AABB.y - min_point_AABB.y;
  marker.scale.z = max_point_AABB.z - min_point_AABB.z;

  //colors
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5f;
  marker.lifetime = ros::Duration();

   
}



}
