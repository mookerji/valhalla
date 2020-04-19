#ifndef MMP_DEMO_GRAPHICAL_MODELS_H_
#define MMP_DEMO_GRAPHICAL_MODELS_H_

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <glog/logging.h>

#include <valhalla/midgard/pointll.h>

#include <valhalla/meili/demo/configuration.h>
#include <valhalla/meili/demo/macros.h>
#include <valhalla/meili/demo/spatial.h>

using namespace valhalla::midgard;

namespace valhalla {

namespace matching {

// Graphical Models: Likelihood models

class EmissionLikelihood {

public:
  EmissionLikelihood(double sigma_z = kSigmaZ) : sigma_z_(sigma_z) {
  }

  double operator()(const PointLL& point) const {
    CHECK(false) << "Not implemented";
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
    CHECK(false) << "Not implemented";
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

  TransmissionLikelihood trasmission_likelihood_;
  EmissionLikelihood emission_likelihood_;
};

} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_GRAPHICAL_MODELS_H_
