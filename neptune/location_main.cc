#include "glog/logging.h"
#include "location/ekf_pose_extrapolator.h"
#include "neptune/location/fusion_interface.h"
#include "neptune/neptune_options.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/NavSatFix.h"
#include <Eigen/Core>
#include <condition_variable>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <laser_geometry/laser_geometry.h>
#include <memory>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>
std::string odom_topic;
std::string fix_topic;
std::string imu_topic;
using namespace neptune;
using namespace location;
// PoseExtrapolatorInterface* pose_extraplotor;
NeptuneOptions options;
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }
Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude) {
  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates
  constexpr double a = 6378137.; // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f); // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(DegToRad(latitude));
  const double cos_phi = std::cos(DegToRad(latitude));
  const double sin_lambda = std::sin(DegToRad(longitude));
  const double cos_lambda = std::cos(DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + altitude) * cos_phi * cos_lambda;
  const double y = (N + altitude) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + altitude) * sin_phi;

  return Eigen::Vector3d(x, y, z);
}

std::unique_ptr<Eigen::Affine3d> ecef_to_local_frame = nullptr;
Eigen::Affine3d ComputeLocalFrameFromLatLong(const double latitude,
                                             const double longitude) {
  const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, 0.);
  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(DegToRad(latitude - 90.), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(DegToRad(-longitude), Eigen::Vector3d::UnitZ());

  Eigen::Translation3d translatioin{rotation * -translation};
  return translatioin * rotation;
}

std::unique_ptr<FustionInterface> pose_extraplotor;
constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);
common::Time FromRos(const ::ros::Time &time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return common::FromUniversal(
      (time.sec + ::common::kUtsEpochOffsetFromUnixEpochInSeconds) *
          10000000ll +
      (time.nsec + 50) / 100); // + 50 to get the rounding correct.
}
//

tf::TransformBroadcaster *tf_broadcaster;
nav_msgs::Path path;
ros::Publisher path_publisher;
void PubFusionData(const transform::Rigid3d &pose) {
  const Eigen::Vector3d &translation = pose.translation();
  const Eigen::Quaterniond &rotaion = pose.rotation();
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
  path.header.frame_id = "map";
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
void Run(const std::string &inputbag_name) {
  int cnt = 0;
  rosbag::Bag in_bag;
  in_bag.open(inputbag_name, rosbag::bagmode::Read);
  rosbag::View view(in_bag);
  rosbag::View::const_iterator view_iterator = view.begin();
  for (auto view_iterator = view.begin(); view_iterator != view.end();
       view_iterator++) {
    rosbag::MessageInstance msg = *view_iterator;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (msg.isType<nav_msgs::Odometry>()) {
      // if (msg.getTopic() == odom_topic) {
      const nav_msgs::Odometry &odom = *msg.instantiate<nav_msgs::Odometry>();
      const Eigen::Vector3d translation{odom.pose.pose.position.x,
                                        odom.pose.pose.position.y,
                                        odom.pose.pose.position.z};
      // LOG(INFO)<<translation;
      const Eigen::Quaterniond rotation{
          odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
          odom.pose.pose.orientation.y, odom.pose.pose.orientation.x};
      pose_extraplotor->AddOdometryData(
          {FromRos(odom.header.stamp),
           transform::Rigid3d(translation, rotation)});
      // }
    }
    if (msg.isType<sensor_msgs::Imu>()) {
      const sensor_msgs::Imu& imu = *msg.instantiate<sensor_msgs::Imu>();
      pose_extraplotor->AddImuData(sensor::ImuData{
          FromRos(imu.header.stamp),
          Eigen::Vector3d{imu.linear_acceleration.x, imu.linear_acceleration.y,
                          imu.linear_acceleration.z} +
              options.rigid_param.imu_instrinsci.ba,
          Eigen::Vector3d{imu.angular_velocity.x, imu.angular_velocity.y,
                          imu.angular_velocity.z} +
              options.rigid_param.imu_instrinsci.bg});
      auto pose = pose_extraplotor->ExtrapolatePose(FromRos(imu.header.stamp));
      LOG(INFO) << pose;
      PubFusionData(pose);
    }
    if (msg.isType<sensor_msgs::NavSatFix>()) {
      const sensor_msgs::NavSatFix &gps =
          *msg.instantiate<sensor_msgs::NavSatFix>();
      // if (gps.status.status == 2) {
      // sensor::FixedFramePoseData fix_data{
      //     FromRos(gps.header.stamp),
      //     transform::Rigid3d::Translation(
      //         {gps.latitude, gps.longitude, gps.altitude}),
      //     Eigen::Map<const
      //     Eigen::Matrix3d>(gps.position_covariance.data())};
      //  pose_extraplotor->AddFixedFramePoseData(fix_data);

<<<<<<< HEAD
      if (ecef_to_local_frame == nullptr) {
        ecef_to_local_frame = std::make_unique<Eigen::Affine3d>(
            ComputeLocalFrameFromLatLong(gps.latitude, gps.longitude));
      }
      Eigen::Vector3d lat_pose =
          *ecef_to_local_frame *
          LatLongAltToEcef(gps.latitude, gps.longitude, gps.altitude);
      lat_pose.z() = 0;
      sensor::FixedFramePoseData fix_data{
          FromRos(gps.header.stamp), transform::Rigid3d::Translation(lat_pose),
          Eigen::Map<const Eigen::Matrix3d>(gps.position_covariance.data())};
      LOG(INFO) << lat_pose;
      pose_extraplotor->AddFixedFramePoseData(fix_data);
      //   auto pose = pose_extraplotor->ExtrapolatePose(fix_data.time);
      //   LOG(INFO) << pose;
      //   PubFusionData(pose);
=======

    if (msg.isType<geometry_msgs::Vector3Stamped>()) {
      const geometry_msgs::Vector3Stamped encoder_msg =
          *msg.instantiate<geometry_msgs::Vector3Stamped>();
      sensor::EncoderData encoder_data;
      encoder_data.time = FromRos(encoder_msg.header.stamp);
      encoder_data.encoder.x() = encoder_msg.vector.x;
      encoder_data.encoder.y() = encoder_msg.vector.y;
      pose_extraplotor->AddEncoderData(encoder_data);
    }
    cnt++;
    // if (cnt > 500) {
    //   break;
    // }
  };
int main(int argc, char **argv) {

  ros::init(argc, argv, "location_main");

  google::InitGoogleLogging("location_main");
  //   google::SetLogDestination(google::INFO,
  //   "/tmp/mower_localization/log/");
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  LOG(INFO) << "start fusion";
  ros::NodeHandle nh;
  std::string bag_file(argv[1]);
  path_publisher = nh.advertise<nav_msgs::Path>("pose_path", 1);
  tf_broadcaster = new tf::TransformBroadcaster();
  options =
      neptune::LodeOptions("/home/cl/mower/mower_localization/src/ros_wrapper/"
                           "src/mower_location/neptune/configuration_files",
                           "config.lua");
  // pose_extraplotor = new
  // PoseExtrapolatorEkf(PoseExtrapolatorEkfOption{{}});

  FusionOption fusion_opt;
  fusion_opt.use_fustion_type = options.fustion_options.location_use_type;
  fusion_opt.local_pose_option = options.fustion_options.local_pose_option;
  fusion_opt.ekf_option.ekf_option.imu_to_gps =
      options.rigid_param.sensor_extrinsic.imu_to_gps;
  fusion_opt.ekf_option.ekf_option.imu_to_odom =
      options.rigid_param.sensor_extrinsic.imu_to_odom;
  fusion_opt.ekf_option.ekf_option.body_to_imu =
      options.rigid_param.sensor_extrinsic.body_to_imu;
  fusion_opt.ekf_option.kinamics_option.b =
      options.rigid_param.kinamics_params.b;
  fusion_opt.ekf_option.kinamics_option.noise_v =
      options.rigid_param.kinamics_params.nv;
  fusion_opt.ekf_option.kinamics_option.noise_w =
      options.rigid_param.kinamics_params.nw;
  fusion_opt.ekf_option.kinamics_option.r =
      options.rigid_param.kinamics_params.r;

  pose_extraplotor = FustionInterface::CreatFusion(fusion_opt);

  LOG(INFO) << "start fusion";
  ros::Rate rate(100);
  Run(bag_file);
  ros::shutdown();
  return 0;
}