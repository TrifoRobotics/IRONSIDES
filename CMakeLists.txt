cmake_minimum_required(VERSION 2.8.3)
project(pirvs_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        tf2_ros
        image_transport
        cv_bridge
        camera_calibration_parsers
        )

### 3rd party ###
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBS})

# LIBUSB
find_package(PkgConfig)
pkg_check_modules(LIBUSB libusb-1.0)
include_directories(${LIBUSB_INCLUDE_DIRS})
link_directories(${LIBUSB_LIBRARY_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK2 REQUIRED gtk+-2.0)
include_directories(${GTK2_INCLUDE_DIRS})
link_directories(${GTK2_INCLUDE_DIRS})
# Add other flags to the compiler
add_definitions(${GTK2_CFLAGS_OTHER})

set(EXT_LIBS
        ${LIBUSB_LIBRARIES}
        ${GTK2_LIBRARIES}
        -lpthread
        -lpng
        -lz
        -ldl
        -fopenmp
        -fPIC
	-ljpeg
        )

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES pirvs_ros
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_library(pirvs INTERFACE)
target_include_directories(pirvs INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/pirvs/include)
target_link_libraries(pirvs INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/pirvs/lib/libPerceptInPIRVS.a ${EXT_LIBS})

add_executable(stereo_camera_node src/stereo_camera_node.cpp)
target_link_libraries(stereo_camera_node ${catkin_LIBRARIES} pirvs)

add_executable(depth_camera_node src/depth_camera_node.cpp)
target_link_libraries(depth_camera_node ${catkin_LIBRARIES} pirvs)

add_executable(slam_node src/slam_node.cpp)
target_link_libraries(slam_node ${catkin_LIBRARIES} pirvs)
