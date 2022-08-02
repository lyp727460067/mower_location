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
#include "sensor_msgs/Imu.h"
std::string odom_topic;
std::string fix_topic;
std::string imu_topic;
using namespace neptune;
using namespace location;
PoseExtrapolatorInterface* pose_extraplotor;

//
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
        nav_msgs::Odometry odom = *msg.instantiate<nav_msgs::Odometry>();
      }
    }
    if (msg.isType<sensor_msgs::Imu>()) {
      sensor_msgs::Imu::Ptr msg_ptr = msg.instantiate<sensor_msgs::Imu>();
    }
  }
}
int main() {
  pose_extraplotor = new PoseExtrapolatorEkf(PoseExtrapolatorEkfOption{{}});

  return 0;
}