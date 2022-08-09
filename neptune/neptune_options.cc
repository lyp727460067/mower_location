#include "neptune/neptune_options.h"

#include <memory>
#include <string>

#include "neptune/common/configuration_file_resolver.h"
#include "neptune/common/lua_parameter_dictionary.h"
namespace neptune {

NeptuneOptions CreateNodeOptions(
    common::LuaParameterDictionary *const lua_parameter_dictionary) {
  NeptuneOptions options;
  // auto neptune_option = lua_parameter_dictionary->GetDictionary("neptune");
  auto imu_intrinsci_option =
      lua_parameter_dictionary->GetDictionary("imu_instrinsic");

  options.rigid_param.imu_instrinsci.ba =
      Eigen::Vector3d{imu_intrinsci_option->GetDouble("bax"),
                      imu_intrinsci_option->GetDouble("bay"),
                      imu_intrinsci_option->GetDouble("baz")};
  options.rigid_param.imu_instrinsci.nba =
      Eigen::Vector3d{imu_intrinsci_option->GetDouble("nbax"),
                      imu_intrinsci_option->GetDouble("nbay"),
                      imu_intrinsci_option->GetDouble("nbaz")};
  options.rigid_param.imu_instrinsci.ka =
      Eigen::Vector3d{imu_intrinsci_option->GetDouble("kax"),
                      imu_intrinsci_option->GetDouble("kay"),
                      imu_intrinsci_option->GetDouble("kaz")};
  options.rigid_param.imu_instrinsci.ta.setIdentity();
  options.rigid_param.imu_instrinsci.ta(0, 1) =
      imu_intrinsci_option->GetDouble("tayz");
  options.rigid_param.imu_instrinsci.ta(0, 2) =
      imu_intrinsci_option->GetDouble("tazy");
  options.rigid_param.imu_instrinsci.ta(1, 2) =
      imu_intrinsci_option->GetDouble("tazx");

  options.rigid_param.imu_instrinsci.bg =
      Eigen::Vector3d{imu_intrinsci_option->GetDouble("bgx"),
                      imu_intrinsci_option->GetDouble("bgy"),
                      imu_intrinsci_option->GetDouble("bgz")};
  options.rigid_param.imu_instrinsci.nbg =
      Eigen::Vector3d{imu_intrinsci_option->GetDouble("nbgx"),
                      imu_intrinsci_option->GetDouble("nbgy"),
                      imu_intrinsci_option->GetDouble("nbgz")};
  options.rigid_param.imu_instrinsci.kg =
      Eigen::Vector3d{imu_intrinsci_option->GetDouble("kgx"),
                      imu_intrinsci_option->GetDouble("kgy"),
                      imu_intrinsci_option->GetDouble("kgz")};

  options.rigid_param.imu_instrinsci.tg.setIdentity();
  options.rigid_param.imu_instrinsci.tg(0, 1) =
      imu_intrinsci_option->GetDouble("tgyz");
  options.rigid_param.imu_instrinsci.tg(0, 2) =
      imu_intrinsci_option->GetDouble("tgzy");
  options.rigid_param.imu_instrinsci.tg(1, 0) =
      imu_intrinsci_option->GetDouble("tgxz");
  options.rigid_param.imu_instrinsci.tg(1, 2) =
      imu_intrinsci_option->GetDouble("tgzx");
  options.rigid_param.imu_instrinsci.tg(2, 0) =
      imu_intrinsci_option->GetDouble("tgxy");
  options.rigid_param.imu_instrinsci.tg(2, 1) =
      imu_intrinsci_option->GetDouble("tgyx");

  auto sensor_extrinsic_options =
      lua_parameter_dictionary->GetDictionary("sensor_extrinsic");
  auto fusion_optoin = lua_parameter_dictionary->GetDictionary("fusion_option");
  options.fustion_options.location_use_type =
      fusion_optoin->GetInt("location_use_type");

  auto local_pose_option = fusion_optoin->GetDictionary("local_pose_option");
  options.fustion_options.local_pose_option = {
      local_pose_option->GetInt("fustion_type"),
      local_pose_option->GetDouble("fix_weitht"),
      local_pose_option->GetDouble("extraplaton_weitht")};
  auto imu_to_gps = sensor_extrinsic_options->GetDictionary("imu_to_gps");
  auto imu_to_odom = sensor_extrinsic_options->GetDictionary("imu_to_odom");
  auto body_to_imu = sensor_extrinsic_options->GetDictionary("body_to_imu");
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
              Eigen::Vector3d{body_to_imu->GetDouble("x"),
                              body_to_imu->GetDouble("y"),
                              body_to_imu->GetDouble("z")},
              transform::RollPitchYaw(body_to_imu->GetDouble("r"),
                                      body_to_imu->GetDouble("p"),
                                      body_to_imu->GetDouble("yaw")))};
  auto kinamics_opt =
      lua_parameter_dictionary->GetDictionary("kinamics_params");
  auto kinamins_nv = kinamics_opt->GetDictionary("nv");
  auto kinamins_nw = kinamics_opt->GetDictionary("nw");

  options.rigid_param.kinamics_params =
      NeptuneOptions::RigidParm::KinamicsParams{
          kinamics_opt->GetDouble("b"),
          Eigen::Vector3d{kinamins_nv->GetDouble("x"),
                          kinamins_nv->GetDouble("y"),
                          kinamins_nv->GetDouble("z")},
          Eigen::Vector3d{kinamins_nw->GetDouble("x"),
                          kinamins_nw->GetDouble("y"),
                          kinamins_nw->GetDouble("z")},
          kinamics_opt->GetDouble("r")};
  return options;
}

NeptuneOptions LodeOptions(const std::string &configuration_directory,
                           const std::string &configuration_basename) {
  auto file_resolver = std::make_unique<common::ConfigurationFileResolver>(
      std::vector<std::string>{configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return CreateNodeOptions(&lua_parameter_dictionary);
}
} // namespace neptune