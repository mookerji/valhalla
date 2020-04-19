#ifndef MMP_DEMO_MANAGER_H_
#define MMP_DEMO_MANAGER_H_

#include <valhalla/meili/demo/configuration.h>
#include <valhalla/meili/demo/graphical_models.h>
#include <valhalla/meili/demo/macros.h>
#include <valhalla/meili/demo/serialization.h>
#include <valhalla/meili/demo/spatial.h>

namespace valhalla {

namespace matching {

// Map matching manager

class MapMatchingManager {
public:
  MapMatchingManager() {
  }

private:
  VL_DISALLOW_COPY_AND_ASSIGN(MapMatchingManager);

  Configuration configuration_;
  RoadNetworkIndex road_index_;
  TrajectoryMeasurements measurements_;
  HiddenMarkovModel trellis_;
};
} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_MANAGER_H_
