#include "neptune/location/fusion_interface.h"
#include "neptune/location/local_pose_fusion.h"

namespace neptune {
namespace location {

std::unique_ptr<FustionInterface>
FustionInterface::CreatFusion(const FusionOption option) {
  if (option.use_fustion_type == 0) {
    return std::make_unique<PureOdomImuFusion>();
  } else if (option.use_fustion_type == 1) {
    return std::make_unique<LocalPoseFusionWithEskf>(option.ekf_option);
  } else if (option.use_fustion_type == 2) {
    return std::make_unique<LocalPoseFusion>(option.local_pose_option);
  } else if (option.use_fustion_type == 3) {
    CHECK(option.use_fustion_type != 3) << "optimazation type not implent";
  }
}
} // namespace location
} // namespace neptune