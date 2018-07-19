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
#include <image_transport/image_transport.h>
#include <pirvs_ironsides.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>
#include <memory>
#include <opencv2/highgui.hpp>

#define Q_SIZE_STEREOIMAGE 1
#define Q_SIZE_CAMERAINFO 1
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
                                  sensor_msgs::image_encodings::BGR8, cv_image)
                   .toImageMsg();
}
int main(int argc, char **argv) {
  // install SIGNAL handler

  if (argc < 3) {
    ROS_ERROR(
        "Not enough input argument.\n"
        "Usage:\n%s [calib_left YAML] [calib_right YAML]\n",
        argv[0]);
    return -1;
  }
  const std::string file_calib_left(argv[1]);
  const std::string file_calib_right(argv[2]);

  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = exit_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  if (!PIRVS::CreatePerceptInDevice(&gDevice, PIRVS::PerceptInDevice::RAW_MODE,
                                    false) ||
      !gDevice) {
    ROS_ERROR("Failed to create device.\n");
    return -1;
  }
  if (!gDevice->StartDevice()) {
    ROS_ERROR("Failed to start device.\n");
    return -1;
  }
  // create Publisher and set topic names.
  ros::init(argc, argv, "ImagePublisher", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub_left =
      it.advertise("StereoImage/left", Q_SIZE_STEREOIMAGE);
  image_transport::Publisher pub_right =
      it.advertise("StereoImage/right", Q_SIZE_STEREOIMAGE);
  ros::Publisher pub_imu =
      nh.advertise<sensor_msgs::Imu>("IMU/data_raw", Q_SIZE_IMU);
  ros::Publisher pub_left_cam_info;
  pub_left_cam_info = nh.advertise<sensor_msgs::CameraInfo>(
      "StereoImage/left/camera_info", Q_SIZE_CAMERAINFO);
  sensor_msgs::CameraInfo camera_calib_msg_left;
  std::string cam_name_left;
  if (!camera_calibration_parsers::readCalibration(
          file_calib_left, cam_name_left, camera_calib_msg_left)) {
    ROS_ERROR("Error while loading left camera calib infor.");
    return -1;
  }

  ros::Publisher pub_right_cam_info;
  pub_right_cam_info = nh.advertise<sensor_msgs::CameraInfo>(
      "StereoImage/right/camera_info", Q_SIZE_CAMERAINFO);
  sensor_msgs::CameraInfo camera_calib_msg_right;
  std::string cam_name_right;
  if (!camera_calibration_parsers::readCalibration(
          file_calib_right, cam_name_right, camera_calib_msg_right)) {
    ROS_ERROR("Error while loading right camera calib infor.");
    return -1;
  }

  bool stereo_data_available = false;
  while (ros::ok) {
    std::shared_ptr<const PIRVS::Data> data;
    if (!gDevice->GetData(&data)) {
      continue;
    }
    std::shared_ptr<const PIRVS::StereoData> stereo_data =
        std::dynamic_pointer_cast<const PIRVS::StereoData>(data);
    if (stereo_data) {
      ros::Time timestamp((double)stereo_data->timestamp / 1000.0f);
      cv::Mat cv_img_left =
          cv::Mat(stereo_data->img_l.height, stereo_data->img_l.width, CV_8UC3);
      memcpy(cv_img_left.data, stereo_data->img_l.data,
             cv_img_left.total() * cv_img_left.elemSize());
      cv::Mat cv_img_right =
          cv::Mat(stereo_data->img_r.height, stereo_data->img_r.width, CV_8UC3);
      memcpy(cv_img_right.data, stereo_data->img_r.data,
             cv_img_right.total() * cv_img_right.elemSize());

      sensor_msgs::ImagePtr image_msg_left;
      sensor_msgs::ImagePtr image_msg_right;
      CreateImageMsg(&image_msg_left, cv_img_left, "StereoRectLeft", timestamp);
      CreateImageMsg(&image_msg_right, cv_img_right, "StereoRectRight",
                     timestamp);
      pub_left.publish(image_msg_left);
      pub_right.publish(image_msg_right);
      pub_left_cam_info.publish(camera_calib_msg_left);
      pub_right_cam_info.publish(camera_calib_msg_right);
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
