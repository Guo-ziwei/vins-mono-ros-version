#include "../../include/utility/visualization.h"

using namespace Eigen;
using namespace ros;

ros::Publisher pub_odometry;
ros::Publisher pub_path;
nav_msgs::Path path;

int IMAGE_ROW, IMAGE_COL;

static double sum_of_path = 0;
static Vector3d last_path(0.0, 0.0, 0.0);

size_t pub_conter = 0;

void registerPub(ros::NodeHandle &n) {
  pub_odometry = n.advertise<nav_msgs::Odometry>("Odometry", 1000);
  pub_path = n.advertise<nav_msgs::Path>("path", 1000);
}

void pubOdometry(const Estimator &estimator, const std_msgs::Header &header) {
  nav_msgs::Odometry odometry;
  odometry.header = header;
  odometry.header.frame_id = "map";
  odometry.child_frame_id = "map";
  Quaterniond tmp_Q;
  tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);
  odometry.pose.pose.position.x = estimator.Ps[WINDOW_SIZE].x();
  odometry.pose.pose.position.y = estimator.Ps[WINDOW_SIZE].y();
  odometry.pose.pose.position.z = estimator.Ps[WINDOW_SIZE].z();
  odometry.pose.pose.orientation.x = tmp_Q.x();
  odometry.pose.pose.orientation.y = tmp_Q.y();
  odometry.pose.pose.orientation.z = tmp_Q.z();
  odometry.pose.pose.orientation.w = tmp_Q.w();
  odometry.twist.twist.linear.x = estimator.Vs[WINDOW_SIZE].x();
  odometry.twist.twist.linear.y = estimator.Vs[WINDOW_SIZE].y();
  odometry.twist.twist.linear.z = estimator.Vs[WINDOW_SIZE].z();
  pub_odometry.publish(odometry);

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = header;
  pose_stamped.header.frame_id = "map";
  pose_stamped.pose = odometry.pose.pose;
  path.header = header;
  path.header.frame_id = "map";
  path.poses.push_back(pose_stamped);
  pub_path.publish(path);

}

void pubTF(const Estimator &estimator, const std_msgs::Header &header) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  // body frame
  Vector3d correct_t;
  Quaterniond correct_q;
  correct_t = estimator.Ps[WINDOW_SIZE];
  correct_q = estimator.Rs[WINDOW_SIZE];

  transform.setOrigin(tf::Vector3(correct_t(0),
                                  correct_t(1),
                                  correct_t(2)));
  q.setW(correct_q.w());
  q.setX(correct_q.x());
  q.setY(correct_q.y());
  q.setZ(correct_q.z());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, header.stamp, "map", "body"));

  // camera frame
  transform.setOrigin(tf::Vector3(estimator.tic[0].x(),
                                  estimator.tic[0].y(),
                                  estimator.tic[0].z()));
  q.setW(Quaterniond(estimator.ric[0]).w());
  q.setX(Quaterniond(estimator.ric[0]).x());
  q.setY(Quaterniond(estimator.ric[0]).y());
  q.setZ(Quaterniond(estimator.ric[0]).z());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, header.stamp, "body", "camera"));
  // nav_msgs::Odometry odometry;
  // odometry.header = header;
  // odometry.header.frame_id = "world";
  // odometry.pose.pose.position.x = estimator.tic[0].x();
  // odometry.pose.pose.position.y = estimator.tic[0].y();
  // odometry.pose.pose.position.z = estimator.tic[0].z();
  // Quaterniond tmp_q{estimator.ric[0]};
  // odometry.pose.pose.orientation.x = tmp_q.x();
  // odometry.pose.pose.orientation.y = tmp_q.y();
  // odometry.pose.pose.orientation.z = tmp_q.z();
  // odometry.pose.pose.orientation.w = tmp_q.w();
  // pub_extrinsic.publish(odometry);
}