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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <random>
#include <fstream>

using namespace neptune;
using namespace location;
namespace{

std::string odom_topic;
std::string fix_topic;
std::string imu_topic;
NeptuneOptions options;
}


 FusionOption fusion_opt;
// PoseExtrapolatorInterface* pose_extraplotor;
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
                                             const double longitude,
                                             const double alt) {
  std::ifstream gps_file("/home/yuxu/gps.txt");
  if (gps_file.good()) {//
    std::string line;
    if (std::getline(gps_file, line)) {
      std::istringstream iss(line);
      double lat;
      double lon;
      double alt1;
      iss >> lat >> lon >> alt1;
      std::cout << std::setprecision(20) << lat << "," << lon << "," << alt1<<std::endl;
      const Eigen::Vector3d translation = LatLongAltToEcef(lat, lon, alt1);
      const Eigen::Quaterniond rotation =
          Eigen::AngleAxisd(DegToRad(lat - 90.), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(DegToRad(-lon), Eigen::Vector3d::UnitZ());
      Eigen::Translation3d translatioin{rotation * -translation};
      gps_file.close();
      return translatioin * rotation;
    }
  } else {
    std::ofstream out("/home/yuxu/gps.txt");
    out << std::setprecision(20) << latitude << " " << longitude << " " << alt
        << std::endl;
    out.close();
  }
  const Eigen::Vector3d translation =
      LatLongAltToEcef(latitude, longitude, alt);
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

  // if(path_publisher.getNumSubscribers()!=0){
  const Eigen::Vector3d &translation = pose.translation();
  const Eigen::Quaterniond &rotaion = pose.rotation();
  geometry_msgs::TransformStamped tf_trans;
  tf_trans.header.stamp = ros::Time::now();
  tf_trans.header.frame_id = "map";
  tf_trans.child_frame_id = "base_link";
  tf_trans.transform.translation.x = translation.x();
  tf_trans.transform.translation.y = translation.y();
  tf_trans.transform.translation.z = 0;//translation.z();
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
  this_pose_stamped.pose.position.z = 0;
  this_pose_stamped.pose.orientation = tf_trans.transform.rotation;
  this_pose_stamped.header.seq = ++index;
  this_pose_stamped.header.stamp = ros::Time::now();
  this_pose_stamped.header.frame_id = "map";
  path.poses.push_back(this_pose_stamped);
  path_publisher.publish(path);
  
}


nav_msgs::Path rtk_path;
ros::Publisher rtk_path_publisher;
void PubRtkData(const transform::Rigid3d &pose) {

  if(rtk_path_publisher.getNumSubscribers()!=0){
  const Eigen::Vector3d &translation = pose.translation();
  const Eigen::Quaterniond &rotaion = pose.rotation();
  //
  geometry_msgs::PoseStamped this_pose_stamped;
  static int index = 0;
  path.header.frame_id = "map";
  this_pose_stamped.pose = geometry_msgs::Pose();
  this_pose_stamped.pose.position.x = translation.x();
  this_pose_stamped.pose.position.y = translation.y();
  this_pose_stamped.pose.position.z = translation.z();
  this_pose_stamped.pose.orientation.x = rotaion.x();
  this_pose_stamped.pose.orientation.y = rotaion.y();
  this_pose_stamped.pose.orientation.z = rotaion.z();
  this_pose_stamped.pose.orientation.w = rotaion.w();
  this_pose_stamped.header.seq = ++index;
  this_pose_stamped.header.stamp = ros::Time::now();
  this_pose_stamped.header.frame_id = "map";
  rtk_path.poses.push_back(this_pose_stamped);
  rtk_path_publisher.publish(path);
  }
}
ros::Publisher markpub;
visualization_msgs::Marker mark;
void PubRtkDataMark(const transform::Rigid3d &pose) {
  if(markpub.getNumSubscribers()!=0){
  visualization_msgs::MarkerArray marks;
  std::default_random_engine e;
  int mark_id = 0;
  mark.header.frame_id = "map";

  mark.ns = "rtk_mark";
  mark.header.stamp = ::ros::Time::now();
  mark.id = mark_id++;
  mark.action = visualization_msgs::Marker::ADD;
  mark.type = visualization_msgs::Marker::POINTS;
  // mark.type = visualization_msgs::Marker::ARROW;
  mark.lifetime = ros::Duration(0);
  mark.scale.x = 0.05;
  mark.scale.y = 0.05;
  mark.scale.z = 0.05;
  std::uniform_real_distribution<float> ran(0, 1);
  mark.color.r = 1;       // ran(e);//1.0;
  mark.color.a = 1;       // ran(e);
  mark.color.g = ran(e);  //(mark_id / sizeofils);
  mark.color.b = ran(e);  //(sizeofils- mark_id) / sizeofils;
  // LOG(INFO)<<mark.color.g<<mark.color.b;
  int cnt = 0;
  geometry_msgs::Point point;
  point.x = pose.translation().x();
  point.y = pose.translation().y();
  point.z = 0;
  mark.points.push_back(point);
  marks.markers.push_back(mark);
  LOG_EVERY_N(INFO, 10) << "still pub path in:";
  markpub.publish(marks);
  }
}

std::mutex mute;
void HandleImuMessage(const sensor_msgs::Imu::ConstPtr &msg1) {
  std::lock_guard<std::mutex> lock(mute);
  const sensor_msgs::Imu &imu = *msg1;
  pose_extraplotor->AddImuData(sensor::ImuData{
      FromRos(imu.header.stamp),
      Eigen::Vector3d{imu.linear_acceleration.x, imu.linear_acceleration.y,
                      imu.linear_acceleration.z},
      Eigen::Vector3d{imu.angular_velocity.x, -imu.angular_velocity.y,
                      imu.angular_velocity.z}});

}
sensor_msgs::NavSatFix gnav_fix;

void HandlClean(const std_msgs::Empty::ConstPtr msg) {
  if (remove("/home/yuxu/gps.txt") == 0) {
    ecef_to_local_frame =
        std::make_unique<Eigen::Affine3d>(ComputeLocalFrameFromLatLong(
            gnav_fix.latitude, gnav_fix.longitude, gnav_fix.altitude));
  std::lock_guard<std::mutex> lock(mute);
  path.poses.clear(); 
  rtk_path.poses.clear();
  mark.points.clear();
  pose_extraplotor = FustionInterface::CreatFusion(fusion_opt);
  }
};
void HandleRtkMessage(const sensor_msgs::NavSatFix::ConstPtr &msg1) {

  std::lock_guard<std::mutex> lock(mute);
  const sensor_msgs::NavSatFix &gps = *msg1;
  if (isnan(gps.latitude) || isnan(gps.longitude) || isnan(gps.altitude)) {
    return;
  }
  gnav_fix = gps;
  if (ecef_to_local_frame == nullptr) {
    ecef_to_local_frame =
        std::make_unique<Eigen::Affine3d>(ComputeLocalFrameFromLatLong(
            gps.latitude, gps.longitude, gps.altitude));
    std::cout << std::setprecision(12)
              << ecef_to_local_frame->translation().transpose();
  }
  Eigen::Vector3d lat_pose =
      *ecef_to_local_frame *
      LatLongAltToEcef(gps.latitude, gps.longitude, gps.altitude);
  sensor::FixedFramePoseData fix_data{
      FromRos(gps.header.stamp), transform::Rigid3d::Translation(lat_pose),
      Eigen::Map<const Eigen::Matrix3d>(gps.position_covariance.data())};
//   PubRtkData(fix_data.pose);
  PubRtkDataMark(fix_data.pose);

  pose_extraplotor->AddFixedFramePoseData(fix_data);
  auto pose = pose_extraplotor->ExtrapolatePose(FromRos(gps.header.stamp));
  PubFusionData(pose);
}

void HandleOdometryMessage(const nav_msgs::Odometry::ConstPtr &msg) {

  std::lock_guard<std::mutex> lock(mute);
  const nav_msgs::Odometry &odom = *msg;
//   const Eigen::Vector3d translation{odom.pose.pose.position.x,
//                                     odom.pose.pose.position.y,
//                                     odom.pose.pose.position.z};
  const Eigen::Vector3d translation{odom.pose.pose.position.x,
                                    odom.pose.pose.position.y,
                                    0};
  // LOG(INFO)<<translation;
  const Eigen::Quaterniond rotation{
      odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y, odom.pose.pose.orientation.z};
  pose_extraplotor->AddOdometryData(
      {FromRos(odom.header.stamp), transform::Rigid3d(translation, rotation)});
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "location_main");

  google::InitGoogleLogging("location_main");
  //   google::SetLogDestination(google::INFO,
  //   "/tmp/mower_localization/log/");
  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  LOG(INFO) << "start fusion";
  ros::NodeHandle nh;
  path_publisher = nh.advertise<nav_msgs::Path>("pose_path", 1);
  rtk_path_publisher = nh.advertise<nav_msgs::Path>("rtk_path", 1);
  markpub =
      nh.advertise<visualization_msgs::MarkerArray>("mark_rtk_path", 10);
  tf_broadcaster = new tf::TransformBroadcaster();
  options = neptune::LodeOptions(
      "/home/yuxu/mower_ws/src/mower/src/mower_location/neptune/"
      "configuration_files",
      "config.lua");
  // pose_extraplotor = new
  // PoseExtrapolatorEkf(PoseExtrapolatorEkfOption{{}});
  ros::Subscriber odom_subscrib =
      nh.subscribe<nav_msgs::Odometry>("odom", 10, HandleOdometryMessage);
  ros::Subscriber imu_subscrib =
      nh.subscribe<sensor_msgs::Imu>("/imu", 10, HandleImuMessage);
  ros::Subscriber rtk_subscrib =
      nh.subscribe<sensor_msgs::NavSatFix>("fix", 10, HandleRtkMessage);
  ros::Subscriber clean_subscrib =
      nh.subscribe<std_msgs::Empty>("gps_file_clean", 10, HandlClean);

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
  ros::spin();
  LOG(INFO) << "start fusion";
  ros::shutdown();
  return 0;
  }
