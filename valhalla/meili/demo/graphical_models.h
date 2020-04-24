#ifndef MMP_DEMO_GRAPHICAL_MODELS_H_
#define MMP_DEMO_GRAPHICAL_MODELS_H_

#include <cmath>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <glog/logging.h>

#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>

#include <valhalla/meili/demo/configuration.h>
#include <valhalla/meili/demo/macros.h>
#include <valhalla/meili/demo/spatial.h>

using namespace valhalla::midgard;

namespace valhalla {

namespace matching {

// Graphical Models: Likelihood models

class EmissionLikelihood {

public:
  EmissionLikelihood(const std::shared_ptr<RoadNetworkIndex>& road_network, double sigma_z = kSigmaZ)
      : road_network_(road_network), sigma_z_(sigma_z) {
  }

  bool IsInitialized() const {
    return road_network_ != nullptr;
  }

  float operator()(const Measurement& point, const RoadCandidate& candidate) const {
    CHECK(IsInitialized());
    CHECK(sigma_z_ > 0);
    // Handle projection (see NOTE in RoadNetworkindex::GetNearestEdges)
    Shape7Decoder<PointLL> geometry = road_network_->GetGeometryLazy(candidate);
    if (geometry.empty()) {
      return -std::numeric_limits<float>::infinity();
    }
    projector_t projector(point.data.lnglat);
    const auto& snapping_result = meili::helpers::Project(projector, geometry);
    float l2_distance = std::sqrt(std::get<1>(snapping_result));
    return -0.5 * std::pow(l2_distance / sigma_z_, 2) -
           0.5 * std::log(2 * kPi * std::pow(sigma_z_, 2));
  }

private:
  double sigma_z_;
  std::shared_ptr<RoadNetworkIndex> road_network_;
};

class TransmissionLikelihood {

public:
  TransmissionLikelihood(const std::shared_ptr<RoadNetworkIndex>& road_network, double beta = kBeta)
      : road_network_(road_network), beta_(beta) {
  }

  bool IsInitialized() const {
    return road_network_ != nullptr;
  }

  double operator()(Measurement src,
                    const RoadCandidate& src_edge,
                    Measurement dst,
                    const RoadCandidate& dst_edge) const {
    CHECK(IsInitialized());
    CHECK(false) << "Not implemented";
    CHECK(beta_ > 0);
    return 0;
  }

private:
  double beta_;
  std::shared_ptr<RoadNetworkIndex> road_network_;
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
    CHECK(false) << "Not implemented";
    return 0;
  }

  double GetWeight() const {
    CHECK(false) << "Not implemented";
    return 0;
  }

  double GetDistanceMeters() const {
    CHECK(false) << "Not implemented";
    return 0;
  }

  double GetDurationSec() const {
    CHECK(false) << "Not implemented";
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
    CHECK(false) << "Not implemented";
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

  // TransmissionLikelihood trasmission_likelihood_;
  // EmissionLikelihood emission_likelihood_;
};

} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_GRAPHICAL_MODELS_H_
