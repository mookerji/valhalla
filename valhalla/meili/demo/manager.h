#ifndef MMP_DEMO_MANAGER_H_
#define MMP_DEMO_MANAGER_H_

#include <memory>

#include <glog/logging.h>

#include <valhalla/meili/demo/configuration.h>
#include <valhalla/meili/demo/graphical_models.h>
#include <valhalla/meili/demo/macros.h>
#include <valhalla/meili/demo/measurement.h>
#include <valhalla/meili/demo/spatial.h>

namespace valhalla {

namespace matching {

class MapMatchingManager {
public:
  MapMatchingManager(const Config& config, std::shared_ptr<RoadNetworkIndex> roads)
      : config_(config), roads_(roads) {
  }

  bool IsInitialized() const {
    return roads_ != nullptr;
  }

  // TODO(mookerji): Look into return types, and having this be a statically allocated working area.
  StateSequence Match(const ObservationSet& trajectory) {
    CHECK(IsInitialized());
    HiddenMarkovModel trellis(config_, roads_);
    trellis.InitModel(trajectory);
    return trellis.Decode();
  }

private:
  VL_DISALLOW_COPY_AND_ASSIGN(MapMatchingManager);

  Config config_;
  std::shared_ptr<RoadNetworkIndex> roads_;
};
} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_MANAGER_H_
