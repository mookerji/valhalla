#ifndef MMP_DEMO_SPATIAL_H_
#define MMP_DEMO_SPATIAL_H_

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <glog/logging.h>

#include <valhalla/baldr/directededge.h>
#include <valhalla/baldr/edgeinfo.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/location.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/meili/candidate_search.h>
#include <valhalla/meili/geometry_helpers.h>
#include <valhalla/meili/grid_range_query.h>
#include <valhalla/meili/routing.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/linesegment2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/tiles.h>
#include <valhalla/midgard/util.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/thor/bidirectional_astar.h>
#include <valhalla/thor/pathinfo.h>

#include <valhalla/meili/demo/configuration.h>
#include <valhalla/meili/demo/macros.h>

// FIX(mookerji): Don't use globbing namespace imports
using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::sif;
using namespace valhalla::thor;

namespace valhalla {

namespace matching {

// Internal Trajectory measurements

struct Measurement {
  struct Data {
    PointLL lnglat;
  };
  Data data{};
  size_t measurement_id = -1;
};

std::ostream& operator<<(std::ostream& os, const Measurement& meas) {
  os << "Point[lat=" << meas.data.lnglat.lat() << ", lng=" << meas.data.lnglat.lng()
     << ", measurement_id=" << meas.measurement_id << "]";
  return os;
}

struct RoadCandidate {
  GraphId edge_id;
};

std::ostream& operator<<(std::ostream& os, const RoadCandidate& meas) {
  os << "RoadCandidate[edge_id=" << meas.edge_id << "]";
  return os;
}

Measurement ReadMeasurement(std::string line) {
  DLOG(INFO) << "measurement line: " << line;
  std::vector<float> records;
  std::istringstream ss(line);
  size_t num_cols = 0;
  for (std::string col; std::getline(ss, col, ','); ++num_cols) {
    CHECK(num_cols < 2) << "Only 'lng,lat' per line.";
    records.emplace_back(std::stof(col));
  }
  return Measurement{PointLL(records[1], records[0])};
}

struct Tracepoint {
  PointLL location;
  float distance;
  std::string name;

  size_t waypoint_index;
  size_t matchings_index;
  size_t alternatives_count;
};

class Trajectory {

public:
  Trajectory() = default;

  Trajectory(const std::vector<Measurement>& meas) : measurements_(meas) {
  }

  Measurement& operator[](size_t i) {
    return measurements_.at(i);
  }

  const Measurement& operator[](size_t i) const {
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
  VL_DISALLOW_COPY_AND_ASSIGN(Trajectory);
  std::vector<Measurement> measurements_;
};

std::vector<Measurement> ReadMeasurements(std::string filename) {
  DLOG(INFO) << "Loading from ..." << filename;
  std::vector<Measurement> measurements;
  std::fstream fs(filename, std::fstream::in);
  std::string line;
  for (size_t line_count = 0; !fs.eof(); ++line_count) {
    std::getline(fs, line);
    if (line.empty()) {
      continue;
    }
    Measurement meas = ReadMeasurement(line);
    meas.measurement_id = line_count;
    measurements.emplace_back(meas);
  }
  return measurements;
}

class RoadCandidateList {
public:
  RoadCandidateList(const std::vector<RoadCandidate>& candidates) : candidates_(candidates) {
  }

  RoadCandidate& operator[](size_t i) {
    return candidates_.at(i);
  }

  const RoadCandidate& operator[](size_t i) const {
    return candidates_.at(i);
  }

  size_t size() const {
    return candidates_.size();
  }

  bool empty() const {
    return candidates_.empty();
  }

  size_t measurement_id() const {
    return measurement_id_;
  }

  void set_measurement_id(size_t index) {
    CHECK(index >= 0);
    measurement_id_ = index;
  }

private:
  // VL_DISALLOW_COPY_AND_ASSIGN(RoadCandidateList);
  std::vector<RoadCandidate> candidates_;
  size_t measurement_id_ = -1;
};

std::ostream& operator<<(std::ostream& os, const RoadCandidateList& list) {
  os << "RoadCandidateList[measurement_id=" << list.measurement_id() << ", ";
  for (size_t i = 0; i < list.size(); ++i) {
    os << list[i] << ", ";
  }
  os << "]";
  return os;
}

// Seaching the road network

float GetLocalTileSize() {
  // NOTE(mookerji): copied from map_matcher.h
  return (baldr::TileHierarchy::levels().rbegin()->second.tiles).TileSize();
}

valhalla::Location PointLLToLocation(PointLL ll) {
  valhalla::Location loc;
  loc.mutable_ll()->set_lng(ll.lng());
  loc.mutable_ll()->set_lat(ll.lat());
  return loc;
}

class RoadNetworkIndex {

public:
  // Projection result from meili/geometry_helpers.h
  // Tuple items: snapped point, sqaured distance, segment index, offset
  using Projection = std::tuple<PointLL, float, typename std::vector<PointLL>::size_type, float>;
  using Paths = std::vector<std::vector<PathInfo>>;

  RoadNetworkIndex() = default;

  RoadNetworkIndex(std::shared_ptr<GraphReader> graph_reader,
                   Config::CandidateSearch search_conf,
                   Config::Routing routing_conf,
                   const cost_ptr_t* mode_costing,
                   TravelMode travelmode)
      : graph_reader_(graph_reader), search_conf_(search_conf), routing_conf_(routing_conf),
        mode_costing_(mode_costing), travelmode_(travelmode) {
    // TODO(mookerji): refactor into a separate initialization step. this is probably do much work
    // to do in the constructor.
    const float local_tile_size = GetLocalTileSize();
    CHECK(local_tile_size > 0);
    const float cell_width = local_tile_size / search_conf_.grid_size;
    const float cell_height = local_tile_size / search_conf_.grid_size;
    candidate_index_ =
        std::make_shared<meili::CandidateGridQuery>(*graph_reader_, cell_width, cell_height);
  }

  bool IsInitialized() const {
    return graph_reader_ && candidate_index_ && mode_costing_;
  }

  RoadCandidateList GetNearestEdges(const Measurement& point) const {
    DLOG(INFO) << "GetNearestEdges arg=" << point;
    CHECK(IsInitialized());
    CHECK(point.data.lnglat.IsValid());
    const AABB2<PointLL>& range =
        midgard::ExpandMeters(point.data.lnglat, search_conf_.search_radius_meters);
    const std::unordered_set<GraphId>& edge_ids = candidate_index_->RangeQuery(range);
    std::vector<RoadCandidate> candidates;
    candidates.reserve(edge_ids.size());
    const GraphTile* tile = nullptr;
    const EdgeFilter& edgefilter = costing()->GetEdgeFilter();
    for (const auto& edge_id : edge_ids) {
      if (!edge_id.Is_Valid()) {
        continue;
      }
      const DirectedEdge* edge = graph_reader_->directededge(edge_id, tile);
      if (!edge) {
        continue;
      }
      const bool ignore_edge = edgefilter(edge) == 0;
      if (ignore_edge) {
        continue;
      }
      candidates.push_back(RoadCandidate{edge_id});
    }
    // NOTE: Note that CandidateQuery::Query precomputes projections of points to edges
    // that we may want to add back as a performance-optimization
    return RoadCandidateList(candidates);
  }

  // TODO/QUESTION(mookerji): Why doesn't this use standard A* search? This function likely will
  // need to be heavily revisited in the future.
  float GetNetworkDistanceMetersBroken(Measurement src,
                                       const RoadCandidate& src_edge,
                                       Measurement dst,
                                       const RoadCandidate& dst_edge) {
    DLOG(INFO) << "GetNetworkDistanceMeters arg="
               << "src=" << src << " src_edge=" << src_edge << " dst=" << dst
               << " dst_edge=" << dst_edge;
    CHECK(IsInitialized());
    // NOTE: See NOTE in RoadNetworkindex::GetNearestEdges; we may want to keep these
    // pre-computed in the feature).
    // NOTE: Duplicating!
    std::vector<baldr::PathLocation> locations = {
        ToPathLocation(src, src_edge),
        ToPathLocation(dst, dst_edge),
    };
    bool no_path = locations[0].edges.empty() || locations[1].edges.empty();
    if (no_path) {
      DLOG(INFO) << "GetNetworkDistanceMeters no_path!";
      return std::numeric_limits<float>::infinity();
    }
    const DistanceApproximator approximator(src.data.lnglat);
    meili::labelset_ptr_t label_set =
        std::make_shared<meili::LabelSet>(routing_conf_.max_intercandidate_distance_meters);
    const uint16_t origin_index = 0;
    const std::unordered_map<uint16_t, uint32_t>& results =
        meili::find_shortest_path(*graph_reader_, locations, origin_index, label_set, approximator,
                                  routing_conf_.search_radius_meters, costing(), nullptr,
                                  turn_cost_table_, routing_conf_.max_intercandidate_distance_meters,
                                  routing_conf_.max_intercandidate_time_seconds);
    CHECK(results.size() == 2) << "find_shortest_path results.size() = " << results.size();
    const auto& it = results.find(1);
    CHECK(it != results.end()) << "Something is wrong here";
    const float distance_meters = label_set->label(it->second).cost().cost;
    CHECK(distance_meters >= 0);
    return distance_meters;
  }

  // Like GetNetworkDistanceMetersBroken, but uses AStar
  float GetNetworkDistanceMeters(Measurement src,
                                 const RoadCandidate& src_edge,
                                 Measurement dst,
                                 const RoadCandidate& dst_edge) {
    DLOG(INFO) << "GetNetworkDistanceMeters arg="
               << "src=" << src << " src_edge=" << src_edge << " dst=" << dst
               << " dst_edge=" << dst_edge;
    CHECK(IsInitialized());
    // NOTE: See NOTE in RoadNetworkindex::GetNearestEdges; we may want to keep these
    // pre-computed in the feature).
    const PathLocation& src_snapped = ToPathLocation(src, src_edge);
    const PathLocation& dst_snapped = ToPathLocation(dst, dst_edge);
    bool no_path = src_snapped.edges.empty() || dst_snapped.edges.empty();
    if (no_path) {
      return std::numeric_limits<float>::infinity();
    }
    valhalla::Location origin;
    PathLocation::toPBF(src_snapped, &origin, *graph_reader_);
    valhalla::Location destination;
    PathLocation::toPBF(dst_snapped, &destination, *graph_reader_);
    cost_ptr_t mode_costing = costing();
    BidirectionalAStar astar;
    const Paths& paths =
        astar.GetBestPath(origin, destination, *graph_reader_, &mode_costing, travelmode_);
    if (paths.empty()) {
      return std::numeric_limits<float>::infinity();
    }
    CHECK(paths.size() == 1) << "GetBestPath path size " << path.size();
    float route_distance_meters = 0;
    for (const auto& leg : path.at(0)) {
      const GraphTile* tile = nullptr;
      const DirectedEdge* edge = graph_reader_->directededge(leg.edgeid, tile);
      if (!edge) {
        continue;
      }
      route_distance_meters += edge->length();
    }
    // TODO: something needs to be done here w.r.t. trimming of distances
    return route_distance_meters;
  }

  std::vector<PointLL> GetGeometry(const RoadCandidate& edge) const {
    const GraphTile* tile = nullptr;
    // TODO: handle exception here
    const EdgeInfo edge_info = graph_reader_->edgeinfo(edge.edge_id, tile);
    return edge_info.shape();
  }

  Shape7Decoder<PointLL> GetGeometryLazy(const RoadCandidate& edge) const {
    const GraphTile* tile = nullptr;
    // TODO: handle exception here
    const EdgeInfo edge_info = graph_reader_->edgeinfo(edge.edge_id, tile);
    return edge_info.lazy_shape();
  }

private:
  // Given a Measurement (PointLL) and a Candidate (graph reader edge_id), get a PathLocation, which
  // correlates that measured point with a projected point on a specific road edge.
  PathLocation ToPathLocation(const Measurement& meas, const RoadCandidate& road) {
    DLOG(INFO) << "ToPathLocation arg="
               << "meas=" << meas << " road=" << road;
    Shape7Decoder<PointLL> geometry = GetGeometryLazy(road);
    if (geometry.empty()) {
      DLOG(INFO) << "ToPathLocation geometry=empty";
      PathLocation invalid_path({PointLL{}});
      return invalid_path;
    }
    projector_t projector(meas.data.lnglat);
    const Projection& snapping = meili::helpers::Project(projector, geometry);
    const GraphTile* tile = nullptr;
    const DirectedEdge* edge = graph_reader_->directededge(road.edge_id, tile);
    CHECK(edge != nullptr);
    PathLocation pl({meas.data.lnglat.x(), meas.data.lnglat.y()});
    pl.edges.reserve(1);
    const float distance_along_fraction =
        edge->forward() ? std::get<3>(snapping) : 1.f - std::get<3>(snapping);
    CHECK(0 <= distance_along_fraction <= 1) << "distance_along_fraction must be a <= 1";
    const PointLL& projected = std::get<0>(snapping);
    const float distance_score = std::get<1>(snapping);
    DLOG(INFO) << "ToPathLocation distance_along=" << distance_along_fraction
               << " distance_score=" << distance_score;
    CHECK(distance_score >= 0);
    pl.edges.push_back(
        PathLocation::PathEdge(road.edge_id, distance_along_fraction, projected, distance_score));
    return pl;
  }

  cost_ptr_t costing() const {
    return mode_costing_[static_cast<size_t>(travelmode_)];
  }

  VL_DISALLOW_COPY_AND_ASSIGN(RoadNetworkIndex);
  std::shared_ptr<GraphReader> graph_reader_;
  std::shared_ptr<meili::CandidateGridQuery> candidate_index_;
  Config::CandidateSearch search_conf_;
  Config::Routing routing_conf_;
  // TODO(mookerji): Figure this stuff out
  // NOTE: std::shared_ptr<cost_t> mode_costing_;
  const cost_ptr_t* mode_costing_;
  TravelMode travelmode_;
  // Turn costs for 0 to pi radians, default to zero, but generally configured by
  // meili.turn_penalty_factor.
  const float turn_cost_table_[181] = {0};
};

} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_SPATIAL_H_
