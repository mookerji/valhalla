#ifndef MMP_DEMO_H_
#define MMP_DEMO_H_

#include <cmath>
#include <memory>
#include <vector>

#include <boost/property_tree/ptree.hpp>
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

// Macros

#define VL_DISALLOW_COPY_AND_ASSIGN(TypeName)                                                        \
  TypeName(const TypeName&) = delete;                                                                \
  TypeName(TypeName&&) = delete;                                                                     \
  TypeName& operator=(const TypeName&) = delete;                                                     \
  TypeName& operator=(TypeName&&) = delete;

using namespace valhalla::midgard;

namespace valhalla {

namespace matching {

// Constants

constexpr double kDefaultSearchRadiusMeters = 50;
constexpr double kSigmaZ = 5;
constexpr double kBeta = 3;
const std::string kDefaultCostModel = "auto";

// Configuration

// TODO(mookerji): Should this be captured in a protobuf somewhere?
struct Configuration {

  struct CandidateSearch {
    double search_radius_meters = kDefaultSearchRadiusMeters;
  };

  struct TransitionLikelihood {
    double beta = kBeta;
  };

  struct EmissionLikelihood {
    double sigma_z = kSigmaZ;
  };

  struct CostModel {
    std::string mode = kDefaultCostModel;
  };

  struct Grid {
    size_t cache_size = 100240;
    size_t grid_size = 500;
  };

  explicit Configuration() = default;

  explicit Configuration(const boost::property_tree::ptree& config)
      : candidate_search(CandidateSearch{config.get<float>("default.search_radius")}),
        transition(TransitionLikelihood{config.get<float>("default.beta")}),
        emission(EmissionLikelihood{config.get<float>("default.sigma_z")}),
        costing(CostModel{config.get<std::string>("mode")}),
        grid(Grid{config.get<size_t>("grid.cache_size"), config.get<size_t>("grid.size")})

  {
  }

  CandidateSearch candidate_search{};
  TransitionLikelihood transition{};
  EmissionLikelihood emission{};
  CostModel costing{};
  Grid grid{};
};

// Graphical Models: Likelihood models

class RoadNetworkIndex;

class EmissionLikelihood {

public:
  EmissionLikelihood(double sigma_z = kSigmaZ) : sigma_z_(sigma_z) {
  }

  double operator()(const PointLL& point) const {
    CHECK(sigma_z_ > 0);
    // TODO:
    // project
    // distance
    const double l2_distance = 0;

    return -0.5 * std::pow(l2_distance / sigma_z_, 2) - 0.5;
  }

private:
  double sigma_z_;
};

class TransmissionLikelihood {

public:
  TransmissionLikelihood(double beta = kBeta) : beta_(beta) {
  }

  double operator()() const {
    CHECK(beta_ > 0);
    return 0;
  }

private:
  double beta_;
  std::shared_ptr<RoadNetworkIndex> network_;
};

// Graphical Models

struct StateNode {
  size_t measurement_id = 1;
  size_t edge_id = 2;
};

class ViterbiPath {

public:
  ViterbiPath() = default;

  double GetScore() const {
    return 0;
  }

  double GetWeight() const {
    return 0;
  }

  double GetDistanceMeters() const {
    return 0;
  }

  double GetDurationSec() const {
    return 0;
  }

  StateNode& operator[](unsigned i) {
    return nodes_.at(i);
  }

  const StateNode& operator[](unsigned i) const {
    return nodes_.at(i);
  }

  size_t size() const {
    return nodes_.size();
  }

private:
  VL_DISALLOW_COPY_AND_ASSIGN(ViterbiPath);
  std::vector<StateNode> nodes_;
};

class HiddenMarkovModel {
public:
  HiddenMarkovModel() {
  }

  bool IsInitialized() {
    return false;
  }

  // ViterbiPath Decode() {
  //   return;
  // }

  float GetEdgeLikeliHood() {
    CHECK(IsInitialized());
    return 0.;
  }

private:
  VL_DISALLOW_COPY_AND_ASSIGN(HiddenMarkovModel);
};

// Internal Trajectory measurements

struct Measurement {
  struct Data {
    midgard::PointLL lnglat;
  };

  Data data;
};

class TrajectoryMeasurements {

public:
  TrajectoryMeasurements() {
  }

  Measurement& operator[](unsigned i) {
    return measurements_.at(i);
  }

  const Measurement& operator[](unsigned i) const {
    return measurements_.at(i);
  }

  size_t size() const {
    return measurements_.size();
  }

private:
  VL_DISALLOW_COPY_AND_ASSIGN(TrajectoryMeasurements);
  std::vector<Measurement> measurements_;
};

// Seaching the road network

class RoadNetworkIndex {

public:
  RoadNetworkIndex() {
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
};

// Serialization to Valhalla and OSRM-style responses

class OSRMResponse {

  struct Matching {};

public:
  OSRMResponse() {
  }
};

class ValhallaResponse {

public:
  ValhallaResponse() {
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

// TODO(mookerji): serialize to GeoJSON

template <typename T> std::string ToGeoJSON(const T& geo) {
  return "";
}

template <> std::string ToGeoJSON(const Measurement& geo) {
  return "Measurement";
}

template <> std::string ToGeoJSON(const Tracepoint& geo) {
  return "Tracepoint";
}

template <> std::string ToGeoJSON(const TrajectoryMeasurements& geo) {
  return "Measurements";
}

// TODO

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

#endif // MMP_DEMO_H_
