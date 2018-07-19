# Ironsides - ROS Integration

This package lets you use the Ironsides visual inertial computing module with ROS.

## Getting started

- Install ROS (Kinetic)
    - http://wiki.ros.org/kinetic/Installation/Ubuntu
- Create a ROS Workspace
    - mkdir -p ~/catkin_ws/src
    - cd ~/catkin_ws/
    - catkin_make
    - source ~/catkin_ws/devel/setup.bash
- Import and make our code
    - place pirvs_ros folder in ~/catkin_ws/src
    - run init python script to import PIRVS SDK
        - cd ~/catkin_ws/src/pirvs_ros/init
        - python pirvs_ros_init.py [PIRVS SDK folder] [calib_raw JSON] [calib_rectified JSON]
    - cd ~/catkin_ws
    - catkin_make
    
## Quick Launch
- Try Stereo_camera_node
    - switch Ironsides to Raw Mode
    - roslaunch pirvs_ros raw_camera.launch
- Try Depth_node
    - switch Ironsides to Depth Mode
    - roslaunch pirvs_ros depth_camera.launch
- Try Slam_node
    - switch Ironsides to Raw Mode
    - roslaunch pirvs_ros display.launch
    
### Test Stereo_camera_node
- Published topic:
    - StereoImage/left: left image
    - StereoImage/right: right image
    - IMU/data_raw: IMU
    - StereoImage/left/cameraInfo: left cam calibration
    - StereoImage/right/cameraInfo: right cam calibration
- Run Stereo_camera_node:
    - rosrun pirvs_ros stereo_camera_node [cam_calib_left.yaml] [cam_calib_right.yaml]
- Check image:
    - rosrun image_view image_view image:=[topic name]
    - e.g., rosrun image_view image_view image:=/pirvs_cam_raw/StereoImage/left
- Check IMU,cameraInfo:
    - rostopic echo [topic name]
    - e.g., rostopic echo IMU/data_raw
    
### Test Depth_camera_node
- Published topic:
    - StereoImage/rect_left: rectified left image
    - StereoImage/rect_right: rectified right image
    - StereoImage/detph: depth image
- Run Depth_camera_node:
    - rosrun pirvs_ros depth_camera_node 
- Check image:
    - rosrun image_view image_view image:=[topic name]
    - e.g., rosrun image_view image_view image:=/pirvs_cam_depth/StereoImage/rect_left
      
### Test Slam_node
- Published topic:
    - published topics:
    - StereoImage/left: left image
    - StereoImage/right: right image
    - Pose: camera pose
    - odom: odom information for kviz
- Run slam_node:
    - rosrun pirvs_ros slam_node [calib.json] [vocab.json] 
- Check image:
    - rosrun image_view image_view image:=[topic name]
    - e.g., rosrun image_view image_view image:=/pirvs_slam/StereoImage/left
- Check Pose:
    - rostopic echo [topic name]
    - e.g., rostopic echo Pose
- Check Pose using RVIZ:
    - rosrun rviz rviz
    - add odometry, listen topic "odom"
    - set fixed frame to my frame

For more information visit [PerceptIn ROS Wiki](https://wiki.ros.org/perceptin)
