#include "neptune_options.h"

#include <memory>
#include <string>

#include "common/configuration_file_resolver.h"
#include "common/lua_parameter_dictionary.h"
namespace neptune {

NeptuneOptions CreateNodeOptions(
    common::LuaParameterDictionary* const lua_parameter_dictionary) {
  NeptuneOptions options;
  // auto neptune_option = lua_parameter_dictionary->GetDictionary("neptune");
  auto imu_intrinsci_option =
      lua_parameter_dictionary->GetDictionary("imu_instrinsic");
  options.rigid_param.imu_instrinsci = NeptuneOptions::RigidParm::ImuInstrinsic{
      {imu_intrinsci_option->GetDouble("bax"),
       imu_intrinsci_option->GetDouble("bay"),
       imu_intrinsci_option->GetDouble("baz")},
      {imu_intrinsci_option->GetDouble("nbax"),
       imu_intrinsci_option->GetDouble("nbay"),
       imu_intrinsci_option->GetDouble("nbaz")},
      {imu_intrinsci_option->GetDouble("kax"),
       imu_intrinsci_option->GetDouble("kay"),
       imu_intrinsci_option->GetDouble("kaz")},
      {imu_intrinsci_option->GetDouble("tax"),
       imu_intrinsci_option->GetDouble("tay"),
       imu_intrinsci_option->GetDouble("taz")},
      {imu_intrinsci_option->GetDouble("bgx"),
       imu_intrinsci_option->GetDouble("bgy"),
       imu_intrinsci_option->GetDouble("bgz")},     
      {imu_intrinsci_option->GetDouble("nbgx"),
       imu_intrinsci_option->GetDouble("nbgy"),
       imu_intrinsci_option->GetDouble("nbgz")},
      {imu_intrinsci_option->GetDouble("kgx"),
       imu_intrinsci_option->GetDouble("kgy"),
       imu_intrinsci_option->GetDouble("kgz")},
      {imu_intrinsci_option->GetDouble("tgx"),
       imu_intrinsci_option->GetDouble("tgy"),
       imu_intrinsci_option->GetDouble("tgz")},
      {imu_intrinsci_option->GetDouble("tgx1"),
       imu_intrinsci_option->GetDouble("tgy1"),
       imu_intrinsci_option->GetDouble("tgz1")},
  };

  auto sensor_extrinsic_options =
      lua_parameter_dictionary->GetDictionary("sensor_extrinsic");

  auto imu_to_gps = sensor_extrinsic_options->GetDictionary("imu_to_gps");
  auto imu_to_odom = sensor_extrinsic_options->GetDictionary("imu_to_odom");
  auto bady_to_imu = sensor_extrinsic_options->GetDictionary("bady_to_imu");
  options.rigid_param.sensor_extrinsic =
      NeptuneOptions::RigidParm::SensorExtrinsic{
          transform::Rigid3d(
              Eigen::Vector3d{imu_to_gps->GetDouble("x"),
                              imu_to_gps->GetDouble("y"),
                              imu_to_gps->GetDouble("z")},
              transform::RollPitchYaw(imu_to_gps->GetDouble("r"),
                                      imu_to_gps->GetDouble("p"),
                                      imu_to_gps->GetDouble("yaw"))),

          transform::Rigid3d(
              Eigen::Vector3d{imu_to_odom->GetDouble("x"),
                              imu_to_odom->GetDouble("y"),
                              imu_to_odom->GetDouble("z")},
              transform::RollPitchYaw(imu_to_odom->GetDouble("r"),
                                      imu_to_odom->GetDouble("p"),
                                      imu_to_odom->GetDouble("yaw"))),

          transform::Rigid3d(
              Eigen::Vector3d{bady_to_imu->GetDouble("x"),
                              bady_to_imu->GetDouble("y"),
                              bady_to_imu->GetDouble("z")},
              transform::RollPitchYaw(bady_to_imu->GetDouble("r"),
                                      bady_to_imu->GetDouble("p"),
                                      bady_to_imu->GetDouble("yaw")))};
  return options;
}

NeptuneOptions LodeOptions(const std::string& configuration_directory,
                           const std::string& configuration_basename) {
  auto file_resolver = std::make_unique<common::ConfigurationFileResolver>(
      std::vector<std::string>{configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return CreateNodeOptions(&lua_parameter_dictionary);
}
}  // namespace neptune