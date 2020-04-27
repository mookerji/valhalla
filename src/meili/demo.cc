#include <boost/property_tree/ptree.hpp>
#include <glog/logging.h>

#include <iostream>
#include <memory>

#include "baldr/graphreader.h"
#include "baldr/rapidjson_utils.h"
#include "meili/demo.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::sif;

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  if (argc < 3) {
    std::cout << "usage: map_matching CONFIG DATA" << std::endl;
    return 1;
  }
  // Load configuration, map matching, costing configuration
  boost::property_tree::ptree root_config;
  rapidjson::read_json(argv[1], root_config);
  const matching::Config matching_config = matching::ReadConfig(root_config.get_child("meili"));
  cost_ptr_t costing = matching::MakeCosting(matching_config.routing);
  TravelMode travelmode = costing->travel_mode();

  // Load road networking graph
  std::shared_ptr<baldr::GraphReader> graph_reader =
      std::make_shared<baldr::GraphReader>(root_config.get_child("mjolnir"));

  const auto& tiles = graph_reader->GetTileSet();
  DLOG(INFO) << "tiles size: " << tiles.size();

  // Load measurements
  const matching::Trajectory traj(matching::ReadMeasurements(argv[2]));

  // Initiate map matcher
  const auto& roads =
      std::make_shared<matching::RoadNetworkIndex>(graph_reader, matching_config.candidate_search,
                                                   matching_config.routing, &costing, travelmode);

  matching::EmissionLikelihood emission_model(roads);
  for (size_t i = 0; i < traj.size(); ++i) {
    const matching::Measurement& point = traj[i];
    matching::RoadCandidateList candidates = roads->GetNearestEdges(point);
    candidates.set_measurement_id(i);
    for (size_t j = 0; j < candidates.size(); ++j) {
      DLOG(INFO) << "emission_model: " << emission_model(point, candidates[j]);
    }
    DLOG(INFO) << "result size: " << candidates.size();
    DLOG(INFO) << "results: " << candidates;

    if (i < traj.size() - 2) {
      const float distance =
          roads->GetNetworkDistanceMeters(traj[i], candidates[0], traj[i + 1], candidates[0]);
      DLOG(INFO) << "distance: " << distance;
    }
  }

  return 0;
}
