#ifndef MMP_DEMO_SPATIAL_H_
#define MMP_DEMO_SPATIAL_H_

#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <glog/logging.h>

#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/meili/geometry_helpers.h>
#include <valhalla/meili/grid_range_query.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/linesegment2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/tiles.h>
#include <valhalla/sif/dynamiccost.h>

#include <valhalla/meili/demo/configuration.h>
#include <valhalla/meili/demo/macros.h>

using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace valhalla {

namespace matching {

// Internal Trajectory measurements

struct Measurement {
  struct Data {
    PointLL lnglat;
  };

  Data data{};

  // Parse measurement from a line
  explicit Measurement(std::string line) {
    DLOG(INFO) << "measurement line: " << line;
    // TODO: Parsing needs to be pulled out to another function
    std::vector<float> records;
    std::istringstream ss(line);
    size_t num_cols = 0;
    for (std::string col; std::getline(ss, col, ','); ++num_cols) {
      CHECK(num_cols < 2) << "Only 'lng,lat' per line.";
      records.emplace_back(std::stof(col));
    }
    data = Data{PointLL(records[0], records[1])};
  }
};

struct Tracepoint {
  PointLL location;
  float distance;
  std::string name;
  unsigned waypoint_index;
  unsigned matchings_index;
  unsigned alternatives_count;
};

class TrajectoryMeasurements {

public:
  TrajectoryMeasurements() = default;

  TrajectoryMeasurements(std::vector<Measurement> meas) : measurements_(meas) {
  }

  Measurement& operator[](unsigned i) {
    return measurements_.at(i);
  }

  const Measurement& operator[](unsigned i) const {
    return measurements_.at(i);
  }

  const void Add(const Measurement& meas) {
    measurements_.emplace_back(meas);
  }

  size_t size() const {
    return measurements_.size();
  }

  bool empty() const {
    return measurements_.empty();
  }

private:
  VL_DISALLOW_COPY_AND_ASSIGN(TrajectoryMeasurements);
  std::vector<Measurement> measurements_;
};

std::vector<Measurement> LoadMeasurements(std::string filename) {
  DLOG(INFO) << "Loading from ..." << filename;
  std::vector<Measurement> measurements;
  std::fstream fs(filename, std::fstream::in);
  std::string line;
  while (!fs.eof()) {
    std::getline(fs, line);
    if (line.empty()) {
      continue;
    }
    measurements.emplace_back(Measurement(line));
  }
  return measurements;
}

// Seaching the road network

class RoadNetworkIndex {

public:
  RoadNetworkIndex() = default;

  RoadNetworkIndex(const std::shared_ptr<GraphReader>& graph) : graph_(graph) {
  }

  bool IsInitialized() {
    CHECK(false) << "Not implemented";
  }

  void GetNearestEdges(PointLL point, float radius) {
    CHECK(false) << "Not implemented";
  }

  // TODO: src, src_edge, dst, dst_edge
  float GetNetworkDistanceMeters() {
    CHECK(false) << "Not implemented";
    return 0;
  }

  // TODO:
  // Get edge by ID
  //

private:
  VL_DISALLOW_COPY_AND_ASSIGN(RoadNetworkIndex);
  std::shared_ptr<GraphReader> graph_;
};

} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_SPATIAL_H_
