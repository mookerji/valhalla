#ifndef MMP_DEMO_GRAPHICAL_MODELS_H_
#define MMP_DEMO_GRAPHICAL_MODELS_H_

#include <cmath>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
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

class DirectedGraph {

public:
  DirectedGraph() = default;

  struct Node {
    Measurement obs;
    RoadCandidate candidate;
    bool is_virtual{false};
  };

  struct Edge {
    Node left;
    Node right;
  };

  using WeightFunction = std::function<float(const Node& src, const Node& dst)>;

  bool HasNode(const Node& node) {
    return adjacency_list_.find(node) != adjacency_list_.end();
  }

  void AddNode(const Node& node) {
    if (HasNode(node)) {
      return;
    }
    adjacency_list_[node] = {};
  }

  void AddEdge(const Edge& edge) {
    if (!HasNode(edge.left)) {
      AddNode(edge.left);
    }
    if (!HasNode(edge.right)) {
      AddNode(edge.right);
    }
    adjacency_list_[edge.left].emplace_back(edge.right);
  }

  std::vector<Node> GetShortestPath(const Node& start, const Node& end, WeightFunction edge_weight) {
    CHECK(false) << "Not implemented";
    return {};
  }

  size_t size() const {
    return adjacency_list_.size();
  }

  bool empty() const {
    return adjacency_list_.empty();
  }

private:
  VL_DISALLOW_COPY_AND_ASSIGN(DirectedGraph);

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
  // using EdgeSet = std::unordered_set<Node, NodeHash, NodeEq>;

  std::unordered_map<Node, std::vector<Node>, NodeHash, NodeEq> adjacency_list_;
};

// TODO(mookerji): replace with a typedef?
class StateSequence {

public:
  StateSequence() = default;

  StateSequence(const std::vector<DirectedGraph::Node> nodes) : nodes_(nodes) {
  }

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

  DirectedGraph::Node& operator[](unsigned i) {
    return nodes_.at(i);
  }

  const DirectedGraph::Node& operator[](unsigned i) const {
    return nodes_.at(i);
  }

  size_t size() const {
    return nodes_.size();
  }

  bool empty() const {
    return nodes_.empty();
  }

private:
  // VL_DISALLOW_COPY_AND_ASSIGN(StateSequence);
  std::vector<DirectedGraph::Node> nodes_;
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

  void InitModel(const ObservationSet& traj) {
    CHECK(IsInitialized());
    start_node_ = DirectedGraph::Node{{}, {}, true};
    trellis_.AddNode(start_node_);
    std::vector<DirectedGraph::Node> prev_layer = {start_node_};
    std::vector<DirectedGraph::Node> current_layer = {};
    for (size_t i = 0; i < traj.size(); ++i) {
      const Measurement& obs = traj[i];
      RoadCandidateList candidates = roads_->GetNearestEdges(obs);
      candidates.set_measurement_id(i);
      current_layer.clear();
      current_layer.resize(candidates.size());
      for (size_t j = 0; j < candidates.size(); ++j) {
        const RoadCandidate& candidate = candidates[j];
        DirectedGraph::Node node{obs, candidate, false};
        for (const auto& prev : prev_layer) {
          trellis_.AddEdge(DirectedGraph::Edge{prev, node});
        }
        current_layer.push_back(node);
      }
      prev_layer = current_layer;
    }
    end_node_ = DirectedGraph::Node{{}, {}, true};
    for (const auto& prev : prev_layer) {
      trellis_.AddEdge(DirectedGraph::Edge{prev, end_node_});
    }
  }

  StateSequence Decode() {
    CHECK(IsInitialized());
    if (trellis_.empty()) {
      return {};
    }
    // TODO(mookerji/yz): Is there a better way to do this?
    DirectedGraph::WeightFunction weight_function = [this](const DirectedGraph::Node& src,
                                                           const DirectedGraph::Node& dst) {
      return this->GetEdgeLikelihood(src, dst);
    };
    return StateSequence(trellis_.GetShortestPath(start_node_, end_node_, weight_function));
  }

  float GetEdgeLikelihood(const DirectedGraph::Node& src, const DirectedGraph::Node& dst) {
    CHECK(IsInitialized());
    if (src.is_virtual) {
      return 1;
    }
    if (dst.is_virtual) {
      return -emission_likelihood_(src.obs, src.candidate);
    }
    const float transition_weight =
        -transition_likelihood_(src.obs, src.candidate, dst.obs, dst.candidate);
    const float emission_weight = -emission_likelihood_(src.obs, src.candidate);
    return transition_weight + emission_weight;
  }

private:
  VL_DISALLOW_COPY_AND_ASSIGN(HiddenMarkovModel);

  Config config_;
  std::shared_ptr<RoadNetworkIndex> roads_;

  TransitionLikelihood transition_likelihood_;
  EmissionLikelihood emission_likelihood_;
  DirectedGraph::Node start_node_{};
  DirectedGraph::Node end_node_{};
  DirectedGraph trellis_{};
};

} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_GRAPHICAL_MODELS_H_
