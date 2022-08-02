#include "location/ekf_pose_extrapolator.h"
#include <laser_geometry/laser_geometry.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/NavSatFix.h"
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>

std::string odom_topic;
std::string fix_topic;
std::string imu_topic;
using namespace neptune;
using namespace location;
PoseExtrapolatorInterface* pose_extraplotor;
constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);
common::Time FromRos(const ::ros::Time& time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return common::FromUniversal(
      (time.sec + ::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (time.nsec + 50) / 100);  // + 50 to get the rounding correct.
}
//

tf::TransformBroadcaster* tf_broadcaster;
nav_msgs::Path path;
ros::Publisher  path_publisher;
void PubFusionData(const transform::Rigid3d& pose) {
  const Eigen::Vector3d& translation = pose.translation();
  const Eigen::Quaterniond& rotaion = pose.rotation();
  geometry_msgs::TransformStamped tf_trans;
  tf_trans.header.stamp = ros::Time::now();
  tf_trans.header.frame_id = "map";
  tf_trans.child_frame_id = "base_link";
  tf_trans.transform.translation.x = translation.x();
  tf_trans.transform.translation.y = translation.y();
  tf_trans.transform.translation.z = translation.z();
  tf_trans.transform.rotation.x = rotaion.x();
  tf_trans.transform.rotation.y = rotaion.y();
  tf_trans.transform.rotation.z = rotaion.z();
  tf_trans.transform.rotation.w = rotaion.w();
  tf_broadcaster->sendTransform(tf_trans);

  geometry_msgs::PoseStamped this_pose_stamped;
  static int index = 0;
  path.header.frame_id  = "map";
  this_pose_stamped.pose = geometry_msgs::Pose();
  this_pose_stamped.pose.position.x = translation.x();
  this_pose_stamped.pose.position.y = translation.y();
  this_pose_stamped.pose.position.z = translation.z();
  this_pose_stamped.pose.orientation = tf_trans.transform.rotation;
  this_pose_stamped.header.seq = ++index;
  this_pose_stamped.header.stamp = ros::Time::now();
  this_pose_stamped.header.frame_id = "map";
  path.poses.push_back(this_pose_stamped);
  path_publisher.publish(path);
}
void Run(const std::string& inputbag_name) {
  rosbag::Bag in_bag;
  in_bag.open(inputbag_name, rosbag::bagmode::Read);
  rosbag::View view(in_bag);
  rosbag::View::const_iterator view_iterator = view.begin();
  for (auto view_iterator = view.begin(); view_iterator != view.end();
       view_iterator++) {
    rosbag::MessageInstance msg = *view_iterator;
    if (msg.isType<nav_msgs::Odometry>()) {
      if (msg.getTopic() == odom_topic) {
        const nav_msgs::Odometry &odom = *msg.instantiate<nav_msgs::Odometry>();
      }
    }
    if (msg.isType<sensor_msgs::Imu>()) {
      const sensor_msgs::Imu& imu = *msg.instantiate<sensor_msgs::Imu>();
      pose_extraplotor->AddImuData(sensor::ImuData{
          FromRos(imu.header.stamp),
          Eigen::Vector3d{imu.linear_acceleration.x,imu.linear_acceleration.y,
                          imu.linear_acceleration.z},
          Eigen::Vector3d{imu.angular_velocity.x, imu.angular_velocity.y,
                          imu.angular_velocity.z}});
    }
    if (msg.isType<sensor_msgs::NavSatFix>()) {
      const sensor_msgs::NavSatFix &gps =
          *msg.instantiate<sensor_msgs::NavSatFix>();
      if (gps.status.status == 2) {
        sensor::FixedFramePoseData fix_data{
            FromRos(gps.header.stamp),
            transform::Rigid3d::Translation(
                {gps.latitude, gps.longitude, gps.altitude}),
            Eigen::Map<const Eigen::Matrix3d>(gps.position_covariance.data())};
        pose_extraplotor->AddFixedFramePoseData(fix_data);
        auto pose = pose_extraplotor->ExtrapolatePose(fix_data.time);
        PubFusionData(pose);
      }
    }
  };
}
int main(int argc,char** argv) {
   
  ros::init(argc, argv, "location_main");
  ros::NodeHandle nh;
  std::string bag_file(argv[1]);
  path_publisher = nh.advertise<nav_msgs::Path>("pose_path", 1);
  tf_broadcaster = new tf::TransformBroadcaster();
  pose_extraplotor = new PoseExtrapolatorEkf(PoseExtrapolatorEkfOption{{}});
  ros::Rate rate(100);
  Run(bag_file);
  ros::spin();
  return 0;
}