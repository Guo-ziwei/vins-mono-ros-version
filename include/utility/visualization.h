#pragma once

#include <fstream>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>

#include "../estimator.h"

extern ros::Publisher pub_odometry;
extern ros::Publisher pub_path;
extern nav_msgs::Path path;

extern int IMAGE_ROW, IMAGE_COL;

void registerPub(ros::NodeHandle &n);

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header);

void pubTF(const Estimator &estimator, const std_msgs::Header &header);