#ifndef MMP_DEMO_GRAPHICAL_MODELS_H_
#define MMP_DEMO_GRAPHICAL_MODELS_H_

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include <glog/logging.h>

#include <valhalla/midgard/constants.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/util.h>

#include <valhalla/meili/demo/configuration.h>
#include <valhalla/meili/demo/macros.h>
#include <valhalla/meili/demo/measurement.h>
#include <valhalla/meili/demo/spatial.h>
#include <valhalla/meili/demo/utils.h>

using namespace valhalla::midgard;

namespace valhalla {

namespace matching {

// Graphical Models: Likelihood models

class EmissionLikelihood {

public:
  EmissionLikelihood() = default;

  EmissionLikelihood(std::shared_ptr<RoadNetworkIndex> roads, double sigma_z = kSigmaZ)
      : roads_(roads), sigma_z_(sigma_z) {
  }

  bool IsInitialized() const {
    return roads_ != nullptr;
  }

  float operator()(const Measurement& point, const RoadCandidate& candidate) const {
    CHECK(IsInitialized());
    CHECK(sigma_z_ > 0);
    // NOTE: Handle projection (see NOTE in RoadNetworkindex::GetNearestEdges; we may want to keep
    // these pre-computed in the feature).
    Shape7Decoder<PointLL> geometry = roads_->GetGeometryLazy(candidate);
    if (geometry.empty()) {
      return -std::numeric_limits<float>::infinity();
    }
    projector_t projector(point.data.lnglat);
    const Projection& snapping_result = meili::helpers::Project(projector, geometry);
    const float l2_distance_meters = std::sqrt(std::get<1>(snapping_result));
    return -0.5 * std::pow(l2_distance_meters / sigma_z_, 2) -
           0.5 * std::log(2 * kPi * std::pow(sigma_z_, 2));
  }

private:
  double sigma_z_;
  std::shared_ptr<RoadNetworkIndex> roads_;
};

class TransitionLikelihood {

public:
  TransitionLikelihood() = default;

  TransitionLikelihood(std::shared_ptr<RoadNetworkIndex> roads, double beta = kBeta)
      : roads_(roads), beta_(beta) {
  }

  bool IsInitialized() const {
    return roads_ != nullptr;
  }

  double operator()(const Measurement src,
                    const RoadCandidate& src_edge,
                    const Measurement dst,
                    const RoadCandidate& dst_edge) const {
    CHECK(IsInitialized());
    CHECK(beta_ > 0);
    const float l2_distance_meters = src.data.lnglat.Distance(dst.data.lnglat);
    CHECK(l2_distance_meters > 0);
    const float l1_distance_meters = roads_->GetNetworkDistanceMeters(src, src_edge, dst, dst_edge);
    CHECK(l1_distance_meters > 0);
    const float d = std::fabs(l2_distance_meters - l1_distance_meters);
    return -d / beta_ - std::log(beta_);
  }

private:
  double beta_;
  std::shared_ptr<RoadNetworkIndex> roads_;
};

// Graphical Models

class Trellis {

public:
  Trellis() = default;

  struct Node {
    Measurement obs;
    RoadCandidate candidate;
    bool is_virtual{false};
  };

  struct Edge {
    Node left;
    Node right;
  };

  bool HasNode(const Node& node) {
    return adjacency_list_.find(node) != adjacency_list_.end();
  }

  void AddNode(const Node& node) {
    if (HasNode(node)) {
      return;
    }
    adjacency_list_[node] = {};
  }

  bool AddEdge(const Edge& edge) {
    if (!HasNode(edge.left)) {
      AddNode(edge.left);
    }
    if (!HasNode(edge.right)) {
      AddNode(edge.right);
    }
    return adjacency_list_[edge.left].emplace(edge.right).second;
  }

  std::vector<Node> GetShortestPath(const Node& start, const Node& end) {
    CHECK(false) << "Not implemented";
    return {};
  }

private:
  VL_DISALLOW_COPY_AND_ASSIGN(Trellis);

  struct NodeHash {
    size_t operator()(const Node& node) const {
      size_t seed = 0;
      hash_combine(seed, std::hash<size_t>{}(node.obs.measurement_id));
      hash_combine(seed, std::hash<size_t>{}(node.candidate.edge_id));
      hash_combine(seed, std::hash<bool>{}(node.is_virtual));
      return seed;
    }
  };

  struct NodeEq {
    size_t operator()(const Node& n1, const Node& n2) const {
      return n1.obs.measurement_id == n2.obs.measurement_id

             && n1.candidate.edge_id == n2.candidate.edge_id && n1.is_virtual == n2.is_virtual;
    }
  };

  // TODO(mookerji): maybe this should be a vector?
  using EdgeSet = std::unordered_set<Node, NodeHash, NodeEq>;

  std::unordered_map<Node, EdgeSet, NodeHash, NodeEq> adjacency_list_;
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

  Trellis::Node& operator[](unsigned i) {
    return nodes_.at(i);
  }

  const Trellis::Node& operator[](unsigned i) const {
    return nodes_.at(i);
  }

  size_t size() const {
    return nodes_.size();
  }

private:
  // VL_DISALLOW_COPY_AND_ASSIGN(ViterbiPath);
  std::vector<Trellis::Node> nodes_;
};

// TODO(mookerji): implement trellis structure and search

class HiddenMarkovModel {
public:
  HiddenMarkovModel(const Config& config, std::shared_ptr<RoadNetworkIndex> roads)
      : config_(config), roads_(roads) {
    transition_likelihood_ = TransitionLikelihood(roads_, config_.transition.beta);
    emission_likelihood_ = EmissionLikelihood(roads_, config_.emission.sigma_z);
  }

  bool IsInitialized() const {
    return roads_ && transition_likelihood_.IsInitialized() && emission_likelihood_.IsInitialized();
  }

  void InitModel(const Trajectory& traj) {
    CHECK(IsInitialized());
    for (size_t i = 0; i < traj.size(); ++i) {
      const Measurement& point = traj[i];
      RoadCandidateList candidates = roads_->GetNearestEdges(point);
      candidates.set_measurement_id(i);
      for (size_t j = 0; j < candidates.size(); ++j) {
        continue;
      }
    }
  }

  ViterbiPath Decode() {
    return {};
  }

  float GetEdgeLikelihood(Trellis::Node src, Trellis::Node dst) {
    CHECK(IsInitialized());
    CHECK(false) << "Not implemented";
    if (src.is_virtual) {
      return 1;
    }
    if (dst.is_virtual) {
      // return -emission_likelihood()
    }
    const float transition_weight = 0;
    const float emission_weight = 0;
    return -transition_weight - emission_weight;
  }

private:
  VL_DISALLOW_COPY_AND_ASSIGN(HiddenMarkovModel);

  Config config_;
  std::shared_ptr<RoadNetworkIndex> roads_;

  TransitionLikelihood transition_likelihood_;
  EmissionLikelihood emission_likelihood_;
  Trellis::Node start_node_{};
  Trellis::Node end_node_{};
  Trellis trellis_{};
};

} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_GRAPHICAL_MODELS_H_
