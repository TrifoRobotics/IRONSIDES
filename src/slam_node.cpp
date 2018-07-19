/**
 * Copyright 2017 PerceptIn
 *
 * This End-User License Agreement (EULA) is a legal agreement between you
 * (the purchaser) and PerceptIn regarding your use of
 * PerceptIn Robotics Vision System (PIRVS), including PIRVS SDK and
 * associated documentation (the "Software").
 *
 * IF YOU DO NOT AGREE TO ALL OF THE TERMS OF THIS EULA, DO NOT INSTALL,
 * USE OR COPY THE SOFTWARE.
 *
 *  :+++++;                                                  ++'
 *  ++++++++;                                                ++'
 *  ++`   .++ .'''''` '''',     :':   `''''', ;''':  '''''''`++' '`   ''
 *  ++`    ++ ++''''.;+'''++, ;++'++, ++'''',,+'''++:+''++''.++',+'   ++
 *  ++`  :+++ ++     ;+.   ++ ++   ++ ++     ,+,   ++   ++   ++',+++  ++
 *  +++++++`  ++     ;+.  .++ ++      ++     ,+,  .++   ++   ++',+++. ++
 *  +++:`     +++++  ;+.;++'  ++      +++++  ,+++++;    ++   ++',+,++ ++
 *  ++`       ++```  ;+`++    ++      ++```  ,+'.       ++   ++',+,,+'++
 *  ++`       ++     ;+.,++   ++      ++     ,+,        ++   ++',+, ++++
 *  ++`       ++     ;+. :++  ++. ,++ ++     ,+,        ++   ++',+,  +++
 *  '+`       ++''''.:+.  ;++  '+'+'  ++'''',,+,        '+   ++',+,  ;+'
 *
 */

#include <camera_calibration_parsers/parse.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <pirvs_ironsides.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <opencv2/highgui.hpp>

#define Q_SIZE_STEREOIMAGE 1
#define Q_SIZE_POSESTAMPED 30
#define Q_SIZE_ODOMETRY 50
#define Q_SIZE_POINTCLOUD 5

std::shared_ptr<PIRVS::PerceptInDevice> gDevice = NULL;

/**
 * Gracefully exit when CTRL-C is hit
 */
void exit_handler(int s) {
  if (gDevice != NULL) {
    gDevice->StopDevice();
  }
  exit(1);
}
/**
 * Convert from Ironsides coordinate to rviz corrdinate.
 * Rotation Matrix = [-1, 0, 0]
 *                   [ 0, 0,-1]
 *                   [ 0,-1, 0]
 * Quaternion = [ 0, -0.7071068, 0.7071068, 0 ]
 */
void CreateRvizTranslateToInitFrame(geometry_msgs::TransformStamped *trans,
                                    std::string child_frame_id) {
  trans->header.frame_id = "init_frame";
  trans->child_frame_id = child_frame_id;
  trans->transform.translation.x = 0;
  trans->transform.translation.y = 0;
  trans->transform.translation.z = 0;
  trans->transform.rotation.x = 0;
  trans->transform.rotation.y = -0.7071068;
  trans->transform.rotation.z = 0.7071068;
  trans->transform.rotation.w = 0;
}
/**
 * Create Map Point Cloud Message From MAP
 */
void CreateMapPointCloudMsg(sensor_msgs::PointCloud *pointCloud,
                            std::shared_ptr<PIRVS::Map> map,
                            ros::Time timestamp) {
  if (!map) {
    return;
  }
  std::vector<PIRVS::Point3d> map_points;
  map->GetPoints(&map_points);
  size_t t = map_points.size();
  int idx = 0;
  for (auto point : map_points) {
    geometry_msgs::Point32 point32;
    point32.x = point.x;
    point32.y = point.y;
    point32.z = point.z;
    pointCloud->points.push_back(point32);
  }
  pointCloud->header.stamp = timestamp;
  pointCloud->header.frame_id = "pointcloud";
}
/**
 * Create Pose Message
 */
void CreatePoseMsg(geometry_msgs::PoseStamped *msg_poseStamped,
                   PIRVS::PoseInQuaternion pose_in_quaternion,
                   ros::Time timestamp) {
  msg_poseStamped->header.stamp = timestamp;
  msg_poseStamped->header.frame_id = "pose";
  msg_poseStamped->pose.position.x = pose_in_quaternion.tx;
  msg_poseStamped->pose.position.y = pose_in_quaternion.ty;
  msg_poseStamped->pose.position.z = pose_in_quaternion.tz;
  msg_poseStamped->pose.orientation.x = pose_in_quaternion.qx;
  msg_poseStamped->pose.orientation.y = pose_in_quaternion.qy;
  msg_poseStamped->pose.orientation.z = pose_in_quaternion.qz;
  msg_poseStamped->pose.orientation.w = pose_in_quaternion.qw;
}
/**
 * Create Odometry Message
 */
void CreateOdometryMsg(nav_msgs::Odometry *odom,
                       PIRVS::PoseInQuaternion pose_in_quaternion,
                       ros::Time timestamp) {
  odom->header.stamp = timestamp;
  odom->header.frame_id = "odom";
  // set odom the position
  odom->pose.pose.position.x = pose_in_quaternion.tx;
  odom->pose.pose.position.y = pose_in_quaternion.ty;
  odom->pose.pose.position.z = pose_in_quaternion.tz;
  odom->pose.pose.orientation.x = pose_in_quaternion.qx;
  odom->pose.pose.orientation.y = pose_in_quaternion.qy;
  odom->pose.pose.orientation.z = pose_in_quaternion.qz;
  odom->pose.pose.orientation.w = pose_in_quaternion.qw;
  // set odom the velocity
  odom->child_frame_id = "base_link";
  odom->twist.twist.linear.x = 0;
  odom->twist.twist.linear.y = 0;
  odom->twist.twist.angular.z = 0;
}
/**
 * Create Image Message
 */
void CreateImageMsg(sensor_msgs::ImagePtr *image_msg, cv::Mat cv_image,
                    std::string frame_id, ros::Time timestamp) {
  std_msgs::Header image_header;
  image_header.stamp = timestamp;
  image_header.frame_id = frame_id;
  *image_msg = cv_bridge::CvImage(image_header,
                                  sensor_msgs::image_encodings::MONO8, cv_image)
                   .toImageMsg();
}

int main(int argc, char **argv) {
  if (argc < 3) {
    ROS_ERROR(
        "Not enough input argument.\n"
        "Usage:\n%s [calib JSON] [voc JSON]\n",
        argv[0]);
    return -1;
  }
  const std::string file_calib(argv[1]);
  const std::string file_voc(argv[2]);
  // install SIGNAL handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  std::shared_ptr<PIRVS::SlamState> slam_state;
  if (!PIRVS::InitState(file_calib, &slam_state)) {
    ROS_ERROR("Failed to InitState.\n");
    return -1;
  }
  std::shared_ptr<PIRVS::Map> map;
  if (!PIRVS::InitMap(file_calib, file_voc, &map)) {
    ROS_ERROR("Failed to InitMap.\n");
    return -1;
  }
  PIRVS::TrajectoryDrawer drawer;
  PIRVS::Mat img_draw;
  // Create an interface to stream the PerceptIn V1 device.
  if (!PIRVS::CreatePerceptInDevice(&gDevice, PIRVS::PerceptInDevice::RAW_MODE,
                                    true) ||
      !gDevice) {
    ROS_ERROR("Failed to create device.\n");
    return -1;
  }
  // Start streaming from the device.
  if (!gDevice->StartDevice()) {
    ROS_ERROR("Failed to start device.\n");
    return -1;
  }
  // create Publisher and set topic names.
  ros::init(argc, argv, "SLAMPublisher", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_left =
      it.advertise("StereoImage/left", Q_SIZE_STEREOIMAGE);
  image_transport::Publisher pub_right =
      it.advertise("StereoImage/right", Q_SIZE_STEREOIMAGE);
  ros::Publisher pub_pose =
      nh.advertise<geometry_msgs::PoseStamped>("Pose", Q_SIZE_POSESTAMPED);
  ros::Publisher odom_pub =
      nh.advertise<nav_msgs::Odometry>("odom", Q_SIZE_ODOMETRY);
  ros::Publisher pub_pointcloud =
      nh.advertise<sensor_msgs::PointCloud>("PointCloud", Q_SIZE_POINTCLOUD);

  tf2_ros::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  tf2_ros::TransformBroadcaster pointcloud_broadcaster;
  geometry_msgs::TransformStamped pointcloud_trans;

  CreateRvizTranslateToInitFrame(&odom_trans, "odom");
  CreateRvizTranslateToInitFrame(&pointcloud_trans, "pointcloud");

  bool stereo_data_available = false;
  while (ros::ok) {
    std::shared_ptr<const PIRVS::Data> data;
    if (!gDevice->GetData(&data)) {
      continue;
    }
    std::shared_ptr<const PIRVS::StereoData> stereo_data =
        std::dynamic_pointer_cast<const PIRVS::StereoData>(data);
    if (stereo_data) {
      stereo_data_available = true;
    }
    // wait for the first image
    if (!stereo_data_available) {
      continue;
    }
    if (!PIRVS::RunSlam(data, map, slam_state)) {
      ROS_ERROR("SLAM failed.\n");
      break;
    }
    if (stereo_data) {
      ros::Time timestamp((double)stereo_data->timestamp / 1000.0f);
      // publish odom tranform
      odom_trans.header.stamp = timestamp;
      odom_broadcaster.sendTransform(odom_trans);
      if (map) {
        sensor_msgs::PointCloud pointCloud;
        CreateMapPointCloudMsg(&pointCloud, map, timestamp);
        pub_pointcloud.publish(pointCloud);
        pointcloud_trans.header.stamp = timestamp;
        pointcloud_broadcaster.sendTransform(pointcloud_trans);
      }
      // publish pose
      PIRVS::PoseInQuaternion pose_in_quaternion;
      if (slam_state->GetPoseInQuaternion(&pose_in_quaternion)) {
        geometry_msgs::PoseStamped msg_poseStamped;
        CreatePoseMsg(&msg_poseStamped, pose_in_quaternion, timestamp);
        pub_pose.publish(msg_poseStamped);

        nav_msgs::Odometry msg_odom;
        CreateOdometryMsg(&msg_odom, pose_in_quaternion, timestamp);
        odom_pub.publish(msg_odom);
      }
      // Decode and publish stereo images
      cv::Mat cv_img_left =
          cv::Mat(stereo_data->img_l.height, stereo_data->img_l.width, CV_8UC1);
      memcpy(cv_img_left.data, stereo_data->img_l.data,
             cv_img_left.total() * cv_img_left.elemSize());
      cv::Mat cv_img_right =
          cv::Mat(stereo_data->img_r.height, stereo_data->img_r.width, CV_8UC1);
      memcpy(cv_img_right.data, stereo_data->img_r.data,
             cv_img_right.total() * cv_img_right.elemSize());

      sensor_msgs::ImagePtr image_msg_left;
      sensor_msgs::ImagePtr image_msg_right;
      CreateImageMsg(&image_msg_left, cv_img_left, "StereoLeft", timestamp);
      CreateImageMsg(&image_msg_right, cv_img_right, "StereoRight", timestamp);
      pub_left.publish(image_msg_left);
      pub_right.publish(image_msg_right);
    }
  }
  if (gDevice != NULL) {
    gDevice->StopDevice();
  }
  if (ros::ok) {
    ros::shutdown();
  }
  return 0;
}
