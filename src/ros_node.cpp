#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "../include/utility/visualization.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int n_delay_times = 2;
std::shared_ptr<System> pSystem;

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg) {
  cv_bridge::CvImageConstPtr ptr;
  if (img_msg->encoding == "8UC1") {
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  }
  else 
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  cv::Mat img = ptr->image.clone();
  return img;
}


void img_callback(const sensor_msgs::ImageConstPtr &img_msg) {
  double dStampSec = img_msg->header.stamp.toSec();
  Mat img = getImageFromMsg(img_msg);
  pSystem->PubImageData(dStampSec, img);
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
  double dStampSec = imu_msg->header.stamp.toSec();
	Vector3d vAcc;
	Vector3d vGyr;
  vAcc<<imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z;
  vGyr<<imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z;
  pSystem->PubImuData(dStampSec, vGyr, vAcc);
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "VIO");
  ros::NodeHandle n("~");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  if (argc != 2) {
    printf("please input config filename");
    return 1;
  }

  string config_path = argv[1];
  printf("config_file: %s\n", argv[1]);
  pSystem.reset(new System(config_path));

  // readParameters(config_path);
  ROS_WARN("waiting for image and imu...");
  registerPub(n);

  ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

  std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);
  
  ros::spin();
  return 0;
}
