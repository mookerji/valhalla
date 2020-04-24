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
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/distanceapproximator.h>
#include <valhalla/midgard/linesegment2.h>
#include <valhalla/midgard/pointll.h>
#include <valhalla/midgard/tiles.h>
#include <valhalla/midgard/util.h>
#include <valhalla/sif/costfactory.h>
#include <valhalla/sif/dynamiccost.h>

#include <valhalla/meili/demo/configuration.h>
#include <valhalla/meili/demo/macros.h>

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::sif;

namespace valhalla {

namespace matching {

// Internal Trajectory measurements

struct Measurement {
  struct Data {
    PointLL lnglat;
  };

  Data data{};
};

std::ostream& operator<<(std::ostream& os, const Measurement& meas) {
  os << "Point[" << meas.data.lnglat.lat() << "," << meas.data.lnglat.lng() << "]";
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
  while (!fs.eof()) {
    std::getline(fs, line);
    if (line.empty()) {
      continue;
    }
    measurements.emplace_back(ReadMeasurement(line));
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

private:
  // VL_DISALLOW_COPY_AND_ASSIGN(RoadCandidateList);
  std::vector<RoadCandidate> candidates_;
};

std::ostream& operator<<(std::ostream& os, const RoadCandidateList& list) {
  os << "RoadCandidateList[";
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

class RoadNetworkIndex {

public:
  RoadNetworkIndex() = default;

  RoadNetworkIndex(const std::shared_ptr<GraphReader>& graph_reader,
                   const Config::CandidateSearch& search_conf,
                   const cost_ptr_t* mode_costing,
                   TravelMode travelmode)
      : graph_reader_(graph_reader), search_conf_(search_conf), mode_costing_(mode_costing),
        travelmode_(travelmode) {
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
    DLOG(INFO) << "GetNearestEdges" << point;
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

  // TODO: src, src_edge, dst, dst_edge
  // TODO(mookerji): Type here should be matching::Measurement, etc.
  float GetNetworkDistanceMeters(Measurement src,
                                 const RoadCandidate& src_edge,
                                 Measurement dst,
                                 const RoadCandidate& dst_edge) const {
    CHECK(IsInitialized());
    CHECK(false) << "Not implemented";
    return 0;
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

  // TODO:
  // Get edge by ID
  //
private:
  cost_ptr_t costing() const {
    return mode_costing_[static_cast<size_t>(travelmode_)];
  }

  VL_DISALLOW_COPY_AND_ASSIGN(RoadNetworkIndex);
  std::shared_ptr<GraphReader> graph_reader_;
  std::shared_ptr<meili::CandidateGridQuery> candidate_index_;
  Config::CandidateSearch search_conf_;
  // TODO(mookerji): Figure this stuff out
  // NOTE: std::shared_ptr<cost_t> mode_costing_;
  const cost_ptr_t* mode_costing_;
  TravelMode travelmode_;
};

} // namespace matching
} // namespace valhalla

#endif // MMP_DEMO_SPATIAL_H_
