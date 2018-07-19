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

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point32.h>
#include <image_transport/image_transport.h>
#include <pirvs_ironsides.h>
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
#define Q_SIZE_POINTCLOUD 5
#define Q_SIZE_IMU 1000

std::shared_ptr<PIRVS::PerceptInDevice> gDevice = NULL;

/**
 * Gracefully exit when CTRL-C is hit
 */

void exit_handler(int s) {
  if (gDevice != NULL) {
    gDevice->StopDevice();
  }
  ros::shutdown;
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
 * Create IMU Message
 */
void CreateIMUMsg(sensor_msgs::Imu *imu_msg,
                  std::shared_ptr<const PIRVS::ImuData> imu_data,
                  ros::Time timestamp) {
  if (!imu_data) {
    return;
  }
  imu_msg->header.stamp = timestamp;
  imu_msg->header.frame_id = "IMU";
  imu_msg->angular_velocity.x = imu_data->ang_v.x;
  imu_msg->angular_velocity.y = imu_data->ang_v.y;
  imu_msg->angular_velocity.z = imu_data->ang_v.z;
  imu_msg->linear_acceleration.x = imu_data->accel.x;
  imu_msg->linear_acceleration.y = imu_data->accel.y;
  imu_msg->linear_acceleration.z = imu_data->accel.z;
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
  // install SIGNAL handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  if (!PIRVS::CreatePerceptInDevice(&gDevice,
                                    PIRVS::PerceptInDevice::DEPTH_MODE) ||
      !gDevice) {
    ROS_ERROR("Failed to create device.\n");
    return -1;
  }
  if (!gDevice->StartDevice()) {
    ROS_ERROR("Failed to start device.\n");
    return -1;
  }

  // create Publisher and set topic names.
  ros::init(argc, argv, "DepthImagePublisher",
            ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_left =
      it.advertise("StereoImage/rect_left", Q_SIZE_STEREOIMAGE);
  image_transport::Publisher pub_right =
      it.advertise("StereoImage/rect_right", Q_SIZE_STEREOIMAGE);
  image_transport::Publisher pub_depth =
      it.advertise("StereoImage/depth", Q_SIZE_STEREOIMAGE);
  ros::Publisher pub_imu =
      nh.advertise<sensor_msgs::Imu>("IMU/data_raw", Q_SIZE_IMU);

  bool stereo_data_available = false;
  while (ros::ok) {
    std::shared_ptr<const PIRVS::Data> data;
    if (!gDevice->GetData(&data)) {
      continue;
    }
    std::shared_ptr<const PIRVS::StereoDepthData> stereo_depth_data =
        std::dynamic_pointer_cast<const PIRVS::StereoDepthData>(data);
    if (stereo_depth_data) {
      ros::Time timestamp((double)stereo_depth_data->timestamp / 1000.0f);
      cv::Mat cv_img_left = cv::Mat(stereo_depth_data->img_l.height,
                                    stereo_depth_data->img_l.width, CV_8UC1);
      memcpy(cv_img_left.data, stereo_depth_data->img_l.data,
             cv_img_left.total() * cv_img_left.elemSize());
      cv::Mat cv_img_right = cv::Mat(stereo_depth_data->img_r.height,
                                     stereo_depth_data->img_r.width, CV_8UC1);
      memcpy(cv_img_right.data, stereo_depth_data->img_r.data,
             cv_img_right.total() * cv_img_right.elemSize());
      cv::Mat cv_img_depth =
          cv::Mat_<uint16_t>(stereo_depth_data->img_depth.height,
                             stereo_depth_data->img_depth.width);
      memcpy(cv_img_depth.data, stereo_depth_data->img_depth.data,
             cv_img_depth.total() * cv_img_depth.elemSize());

      sensor_msgs::ImagePtr image_msg_left;
      sensor_msgs::ImagePtr image_msg_right;
      CreateImageMsg(&image_msg_left, cv_img_left, "StereoRectLeft", timestamp);
      CreateImageMsg(&image_msg_right, cv_img_right, "StereoRectRight",
                     timestamp);
      pub_left.publish(image_msg_left);
      pub_right.publish(image_msg_right);
      // publish depth image
      std_msgs::Header image_header_depth;
      image_header_depth.stamp = timestamp;
      image_header_depth.frame_id = "StereoDepth";
      sensor_msgs::ImagePtr msg_depth =
          cv_bridge::CvImage(image_header_depth,
                             sensor_msgs::image_encodings::MONO16, cv_img_depth)
              .toImageMsg();
      pub_depth.publish(msg_depth);
    }
    std::shared_ptr<const PIRVS::ImuData> imu_data =
        std::dynamic_pointer_cast<const PIRVS::ImuData>(data);
    if (imu_data) {
      sensor_msgs::Imu imu_msg;
      ros::Time t((double)imu_data->timestamp / 1000.0f);
      CreateIMUMsg(&imu_msg, imu_data, t);
      pub_imu.publish(imu_msg);
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
